#include <cerrno>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <cmath>
#include <memory>
#include <mutex>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>

#include "CLI11.hpp"
#include "picojson.h"
#include "validation.hpp"

extern "C" {
#include <kipr/accel/accel.h>
#include <kipr/analog/analog.h>
#include <kipr/button/button.h>
#include <kipr/digital/digital.h>
#include <kipr/gyro/gyro.h>
#include <kipr/magneto/magneto.h>
#include <kipr/motor/motor.h>
#include <kipr/servo/servo.h>
}

namespace {

constexpr const char *WOMBAT_SPI_DEVICE = "/dev/spidev0.0";
constexpr const char *REQUEST_ID_RULE_MESSAGE =
    "request ID must be 1-128 characters and contain only ASCII letters, "
    "digits, '.', '_', ':', and '-'";
constexpr std::size_t MAX_DAEMON_REQUEST_LINE = 64 * 1024;
constexpr int DAEMON_CLIENT_RESPONSE_TIMEOUT_SECONDS = 2;

// Set only from async signal handlers; the main daemon loop observes this and
// moves shutdown back into normal C++ control flow.
volatile sig_atomic_t daemon_shutdown_requested = 0;

struct OutputContext {
    bool json = false;
    std::string id;
};

struct RawGlobalOptions {
    bool json = false;
    bool has_id = false;
    bool id_missing_value = false;
    std::string id;
};

struct DaemonRequest {
    // Assigned at queue insertion time. This is the daemon's ordering point for
    // valid hardware requests from all client connections.
    unsigned long long sequence = 0;
    std::string id;
    std::string command;
    picojson::object params;
};

struct DaemonWorkItem {
    DaemonRequest request;
    // Filled by the dispatcher while holding DaemonRuntime::mutex, then read by
    // the socket handler that enqueued this item.
    picojson::object response;
    bool completed = false;
};

struct DaemonRuntime {
    // Socket handlers are producers and the motor dispatcher is the only
    // consumer. Keep this as the only shared daemon state until another
    // hardware subsystem needs to join the daemon.
    std::mutex mutex;
    std::condition_variable queue_ready;
    std::condition_variable response_ready;
    std::deque<std::shared_ptr<DaemonWorkItem>> queue;
    bool stopping = false;
    unsigned long long next_sequence = 1;
    // Zero means no motor timeout is configured. Positive values are
    // milliseconds and are resolved once at daemon startup.
    int motor_timeout_ms = 0;
};

void request_daemon_shutdown(int)
{
    daemon_shutdown_requested = 1;
}

picojson::value json_int(int value)
{
    return picojson::value(static_cast<double>(value));
}

CLI::Validator analog_port_validator()
{
    CLI::Range port_range(0, 5);

    return CLI::Validator(
        [port_range](std::string &input) {
            std::string checked_input = input;
            if (!port_range(checked_input).empty()) {
                return std::string("analog port must be between 0 and 5");
            }

            return std::string();
        },
        "0-5",
        "ANALOG_PORT");
}

CLI::Validator digital_port_validator()
{
    CLI::Range port_range(0, 9);

    return CLI::Validator(
        [port_range](std::string &input) {
            std::string checked_input = input;
            if (!port_range(checked_input).empty()) {
                return std::string("digital port must be between 0 and 9");
            }

            return std::string();
        },
        "0-9",
        "DIGITAL_PORT");
}

CLI::Validator servo_port_validator()
{
    CLI::Range port_range(0, 3);

    return CLI::Validator(
        [port_range](std::string &input) {
            std::string checked_input = input;
            if (!port_range(checked_input).empty()) {
                return std::string("servo port must be between 0 and 3");
            }

            return std::string();
        },
        "0-3",
        "SERVO_PORT");
}

CLI::Validator servo_enabled_validator()
{
    CLI::Range enabled_range(0, 1);

    return CLI::Validator(
        [enabled_range](std::string &input) {
            std::string checked_input = input;
            if (!enabled_range(checked_input).empty()) {
                return std::string("servo enabled value must be 0 or 1");
            }

            return std::string();
        },
        "0|1",
        "SERVO_ENABLED");
}

CLI::Validator servo_position_validator()
{
    CLI::Range position_range(0, 2047);

    return CLI::Validator(
        [position_range](std::string &input) {
            std::string checked_input = input;
            if (!position_range(checked_input).empty()) {
                return std::string("servo position must be between 0 and 2047");
            }

            return std::string();
        },
        "0-2047",
        "SERVO_POSITION");
}

CLI::Validator motor_port_validator()
{
    CLI::Range port_range(0, 3);

    return CLI::Validator(
        [port_range](std::string &input) {
            std::string checked_input = input;
            if (!port_range(checked_input).empty()) {
                return std::string("motor port must be between 0 and 3");
            }

            return std::string();
        },
        "0-3",
        "MOTOR_PORT");
}

CLI::Validator motor_velocity_validator()
{
    CLI::Range velocity_range(-1500, 1500);

    return CLI::Validator(
        [velocity_range](std::string &input) {
            std::string checked_input = input;
            if (!velocity_range(checked_input).empty()) {
                return std::string(
                    "motor velocity must be between -1500 and 1500");
            }

            return std::string();
        },
        "-1500-1500",
        "MOTOR_VELOCITY");
}

CLI::Validator positive_milliseconds_validator(const std::string &name)
{
    CLI::Range positive_range(1, std::numeric_limits<int>::max());

    return CLI::Validator(
        [positive_range, name](std::string &input) {
            std::string checked_input = input;
            if (!positive_range(checked_input).empty()) {
                return name + " must be a positive integer";
            }

            return std::string();
        },
        "1+",
        name);
}

bool parse_positive_milliseconds(const std::string &input, int &value)
{
    if (input.empty()) {
        return false;
    }

    long long parsed = 0;
    for (const char ch : input) {
        if (ch < '0' || ch > '9') {
            return false;
        }

        parsed = parsed * 10 + (ch - '0');
        if (parsed > std::numeric_limits<int>::max()) {
            return false;
        }
    }

    if (parsed <= 0) {
        return false;
    }

    value = static_cast<int>(parsed);
    return true;
}

RawGlobalOptions inspect_raw_global_options(int argc, char **argv)
{
    RawGlobalOptions options;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--json") {
            options.json = true;
            continue;
        }

        if (arg == "--id") {
            options.has_id = true;
            if (i + 1 >= argc) {
                options.id_missing_value = true;
                continue;
            }
            options.id = argv[++i];
            continue;
        }

        constexpr const char *id_prefix = "--id=";
        constexpr std::size_t id_prefix_len = 5;
        if (arg.compare(0, id_prefix_len, id_prefix) == 0) {
            options.has_id = true;
            options.id = arg.substr(id_prefix_len);
        }
    }

    return options;
}

void add_optional_id(picojson::object &object, const OutputContext &output)
{
    if (!output.id.empty()) {
        object["id"] = picojson::value(output.id);
    }
}

void write_json(const picojson::object &object)
{
    std::cout << picojson::value(object).serialize() << '\n';
}

std::string make_json_line(const picojson::object &object)
{
    return picojson::value(object).serialize() + "\n";
}

int emit_error(const OutputContext &output,
               const std::string &code,
               const std::string &message,
               int exit_code = 1)
{
    if (!output.json) {
        std::cerr << "error: " << message << '\n';
        return exit_code;
    }

    picojson::object error;
    error["code"] = picojson::value(code);
    error["message"] = picojson::value(message);

    picojson::object response;
    response["ok"] = picojson::value(false);
    add_optional_id(response, output);
    response["error"] = picojson::value(error);

    write_json(response);
    return exit_code;
}

int emit_json_result(const OutputContext &output,
                     const std::string &command,
                     const picojson::object &result)
{
    picojson::object response;
    response["ok"] = picojson::value(true);
    add_optional_id(response, output);
    response["command"] = picojson::value(command);
    response["result"] = picojson::value(result);

    write_json(response);
    return 0;
}

picojson::object make_daemon_error(const std::string &id,
                                   const std::string &code,
                                   const std::string &message)
{
    picojson::object error;
    error["code"] = picojson::value(code);
    error["message"] = picojson::value(message);

    picojson::object response;
    response["ok"] = picojson::value(false);
    if (!id.empty()) {
        response["id"] = picojson::value(id);
    }
    response["error"] = picojson::value(error);
    return response;
}

picojson::object make_daemon_echo_response(const DaemonRequest &request)
{
    picojson::object result = request.params;
    if (request.command == "motor.off" &&
        request.params.find("port") == request.params.end()) {
        picojson::array ports;
        for (int port = 0; port < 4; ++port) {
            ports.push_back(json_int(port));
        }
        result["ports"] = picojson::value(ports);
    }

    picojson::object response;
    response["ok"] = picojson::value(true);
    if (!request.id.empty()) {
        response["id"] = picojson::value(request.id);
    }
    response["command"] = picojson::value(request.command);
    response["result"] = picojson::value(result);
    return response;
}

picojson::object make_daemon_success_response(const DaemonRequest &request,
                                              const picojson::object &result)
{
    picojson::object response;
    response["ok"] = picojson::value(true);
    if (!request.id.empty()) {
        response["id"] = picojson::value(request.id);
    }
    response["command"] = picojson::value(request.command);
    response["result"] = picojson::value(result);
    return response;
}

int emit_int_result(const OutputContext &output,
                    const std::string &command,
                    int value)
{
    if (!output.json) {
        std::cout << value << '\n';
        return 0;
    }

    picojson::object result;
    result["value"] = json_int(value);
    return emit_json_result(output, command, result);
}

int emit_port_value_result(const OutputContext &output,
                           const std::string &command,
                           int port,
                           int value)
{
    if (!output.json) {
        std::cout << value << '\n';
        return 0;
    }

    picojson::object result;
    result["port"] = json_int(port);
    result["value"] = json_int(value);
    return emit_json_result(output, command, result);
}

int emit_analog_values_result(const OutputContext &output, const int values[6])
{
    if (!output.json) {
        for (int port = 0; port < 6; ++port) {
            if (port != 0) {
                std::cout << " ";
            }
            std::cout << values[port];
        }
        std::cout << '\n';
        return 0;
    }

    picojson::array json_values;
    for (int port = 0; port < 6; ++port) {
        json_values.push_back(json_int(values[port]));
    }

    picojson::object result;
    result["values"] = picojson::value(json_values);
    return emit_json_result(output, "analog", result);
}

int emit_digital_values_result(const OutputContext &output, const int values[10])
{
    if (!output.json) {
        for (int port = 0; port < 10; ++port) {
            if (port != 0) {
                std::cout << " ";
            }
            std::cout << values[port];
        }
        std::cout << '\n';
        return 0;
    }

    picojson::array json_values;
    for (int port = 0; port < 10; ++port) {
        json_values.push_back(json_int(values[port]));
    }

    picojson::object result;
    result["values"] = picojson::value(json_values);
    return emit_json_result(output, "digital", result);
}

int emit_servo_values_result(const OutputContext &output,
                             const std::string &command,
                             const int values[4])
{
    if (!output.json) {
        for (int port = 0; port < 4; ++port) {
            if (port != 0) {
                std::cout << " ";
            }
            std::cout << values[port];
        }
        std::cout << '\n';
        return 0;
    }

    picojson::array json_values;
    for (int port = 0; port < 4; ++port) {
        json_values.push_back(json_int(values[port]));
    }

    picojson::object result;
    result["values"] = picojson::value(json_values);
    return emit_json_result(output, command, result);
}

int emit_servo_set_enabled_result(const OutputContext &output,
                                  int port,
                                  int enabled)
{
    if (!output.json) {
        std::cout << "ok\n";
        return 0;
    }

    picojson::object result;
    result["port"] = json_int(port);
    result["enabled"] = json_int(enabled);
    return emit_json_result(output, "servo.set_enabled", result);
}

int emit_servo_set_position_result(const OutputContext &output,
                                   int port,
                                   int position)
{
    if (!output.json) {
        std::cout << "ok\n";
        return 0;
    }

    picojson::object result;
    result["port"] = json_int(port);
    result["position"] = json_int(position);
    return emit_json_result(output, "servo.set", result);
}

bool is_axis(const std::string &axis)
{
    return axis == "x" || axis == "y" || axis == "z";
}

int read_accel_axis(const std::string &axis)
{
    if (axis == "x") {
        return accel_x();
    }
    if (axis == "y") {
        return accel_y();
    }
    return accel_z();
}

int read_gyro_axis(const std::string &axis)
{
    if (axis == "x") {
        return gyro_x();
    }
    if (axis == "y") {
        return gyro_y();
    }
    return gyro_z();
}

int read_magneto_axis(const std::string &axis)
{
    if (axis == "x") {
        return magneto_x();
    }
    if (axis == "y") {
        return magneto_y();
    }
    return magneto_z();
}

int emit_axis_result(const OutputContext &output,
                     const std::string &command,
                     const std::string &axis,
                     int value)
{
    if (!output.json) {
        std::cout << value << '\n';
        return 0;
    }

    picojson::object result;
    result["axis"] = picojson::value(axis);
    result["value"] = json_int(value);

    return emit_json_result(output, command, result);
}

int emit_xyz_result(const OutputContext &output,
                    const std::string &command,
                    int x,
                    int y,
                    int z)
{
    if (!output.json) {
        std::cout << x << " " << y << " " << z << '\n';
        return 0;
    }

    picojson::object result;
    result["x"] = json_int(x);
    result["y"] = json_int(y);
    result["z"] = json_int(z);

    return emit_json_result(output, command, result);
}

std::string parse_error_code(const CLI::ParseError &err)
{
    const std::string message = err.what();
    if (dynamic_cast<const CLI::ValidationError *>(&err) != nullptr &&
        (message.find("analog port") != std::string::npos ||
         message.find("digital port") != std::string::npos ||
         message.find("servo port") != std::string::npos ||
         message.find("motor port") != std::string::npos)) {
        return "invalid_port";
    }

    if (dynamic_cast<const CLI::ValidationError *>(&err) != nullptr &&
        message.find("motor velocity") != std::string::npos) {
        return "invalid_velocity";
    }

    if (dynamic_cast<const CLI::ValidationError *>(&err) != nullptr &&
        message.find("servo enabled") != std::string::npos) {
        return "invalid_enabled";
    }

    if (dynamic_cast<const CLI::ValidationError *>(&err) != nullptr &&
        message.find("servo position") != std::string::npos) {
        return "invalid_position";
    }

    if (dynamic_cast<const CLI::ValidationError *>(&err) != nullptr &&
        message.find("motor timeout") != std::string::npos) {
        return "invalid_motor_timeout";
    }

    if (dynamic_cast<const CLI::RequiredError *>(&err) != nullptr ||
        dynamic_cast<const CLI::ArgumentMismatch *>(&err) != nullptr) {
        return "missing_argument";
    }

    if (dynamic_cast<const CLI::ExtrasError *>(&err) != nullptr) {
        return "unknown_command";
    }

    return "parse_error";
}

std::string parse_error_message(const CLI::ParseError &err)
{
    const std::string message = err.what();
    const std::string prefixes[] = {
        "port: ", "enabled: ", "position: ", "velocity: ",
        "--motor-timeout-ms: "};

    for (const std::string &prefix : prefixes) {
        if (message.compare(0, prefix.size(), prefix) == 0) {
            return message.substr(prefix.size());
        }
    }

    return message;
}

bool wombat_spi_available(std::string &message)
{
    if (access(WOMBAT_SPI_DEVICE, R_OK | W_OK) == 0) {
        return true;
    }

    message = std::string(WOMBAT_SPI_DEVICE) + " is not available: " +
              std::strerror(errno);
    return false;
}

std::string default_socket_path()
{
    const char *runtime_dir = std::getenv("XDG_RUNTIME_DIR");
    if (runtime_dir != nullptr && runtime_dir[0] != '\0') {
        return std::string(runtime_dir) + "/botcli.sock";
    }

    return "/tmp/botcli-" + std::to_string(static_cast<long long>(getuid())) +
           ".sock";
}

std::string resolve_socket_path(const std::string &cli_socket_path)
{
    // The client and daemon deliberately share the same precedence rule so a
    // caller can point both sides at an alternate socket without changing the
    // command shape.
    if (!cli_socket_path.empty()) {
        return cli_socket_path;
    }

    const char *env_socket_path = std::getenv("BOTCLI_SOCKET");
    if (env_socket_path != nullptr && env_socket_path[0] != '\0') {
        return env_socket_path;
    }

    return default_socket_path();
}

bool resolve_motor_timeout_ms(bool cli_timeout_set,
                              int cli_timeout_ms,
                              int &motor_timeout_ms,
                              std::string &error_message)
{
    if (cli_timeout_set) {
        motor_timeout_ms = cli_timeout_ms;
        return true;
    }

    const char *env_timeout = std::getenv("BOTCLI_MOTOR_TIMEOUT");
    if (env_timeout == nullptr || env_timeout[0] == '\0') {
        motor_timeout_ms = 0;
        return true;
    }

    if (!parse_positive_milliseconds(env_timeout, motor_timeout_ms)) {
        error_message =
            "BOTCLI_MOTOR_TIMEOUT must be a positive integer milliseconds value";
        return false;
    }

    return true;
}

bool object_has_only_fields(const picojson::object &object,
                            const std::string fields[],
                            std::size_t field_count,
                            std::string &unknown_field)
{
    for (const auto &entry : object) {
        bool found = false;
        for (std::size_t i = 0; i < field_count; ++i) {
            if (entry.first == fields[i]) {
                found = true;
                break;
            }
        }

        if (!found) {
            unknown_field = entry.first;
            return false;
        }
    }

    return true;
}

bool params_have_only_fields(const picojson::object &params,
                             const std::string &command,
                             std::string &unknown_field)
{
    if (command == "motor.set") {
        const std::string fields[] = {"port", "velocity"};
        return object_has_only_fields(params, fields, 2, unknown_field);
    }

    if (command == "motor.off") {
        const std::string fields[] = {"port"};
        return object_has_only_fields(params, fields, 1, unknown_field);
    }

    const std::string fields[] = {"port"};
    return object_has_only_fields(params, fields, 1, unknown_field);
}

bool is_json_whitespace(char ch)
{
    return ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n';
}

bool read_integral_param(const picojson::object &params,
                         const std::string &name,
                         bool required,
                         int &value,
                         std::string &code,
                         std::string &message)
{
    const auto found = params.find(name);
    if (found == params.end()) {
        if (!required) {
            return true;
        }

        code = "missing_argument";
        message = name + " is required";
        return false;
    }

    if (!found->second.is<double>()) {
        code = "invalid_argument";
        message = name + " must be an integer";
        return false;
    }

    const double number = found->second.get<double>();
    if (!std::isfinite(number) || std::floor(number) != number ||
        number < std::numeric_limits<int>::min() ||
        number > std::numeric_limits<int>::max()) {
        code = "invalid_argument";
        message = name + " must be an integer";
        return false;
    }

    value = static_cast<int>(number);
    return true;
}

bool validate_daemon_motor_params(const std::string &command,
                                  const picojson::object &params,
                                  std::string &code,
                                  std::string &message)
{
    // The daemon is a trust boundary, so repeat validation here even though the
    // built-in CLI client validates its own arguments before sending a request.
    std::string unknown_field;
    if (!params_have_only_fields(params, command, unknown_field)) {
        code = "unknown_field";
        message = "unknown params field: " + unknown_field;
        return false;
    }

    int port = -1;
    const bool port_required = command != "motor.off";
    if (!read_integral_param(
            params, "port", port_required, port, code, message)) {
        return false;
    }

    if ((port_required || params.find("port") != params.end()) &&
        (port < 0 || port > 3)) {
        code = "invalid_port";
        message = "motor port must be between 0 and 3";
        return false;
    }

    if (command != "motor.set") {
        return true;
    }

    int velocity = 0;
    if (!read_integral_param(
            params, "velocity", true, velocity, code, message)) {
        return false;
    }

    if (velocity < -1500 || velocity > 1500) {
        code = "invalid_velocity";
        message = "motor velocity must be between -1500 and 1500";
        return false;
    }

    return true;
}

bool parse_daemon_request_line(const std::string &line,
                               unsigned long long sequence,
                               DaemonRequest &request,
                               picojson::object &error_response)
{
    // A daemon protocol message is exactly one JSON object per line. Protocol
    // errors are answered by the socket handler and never enter the hardware
    // queue, which keeps the dispatcher focused on validated motor requests.
    picojson::value value;
    std::string parse_error;
    std::string::const_iterator parsed_to =
        picojson::parse(value, line.begin(), line.end(), &parse_error);
    if (!parse_error.empty()) {
        error_response =
            make_daemon_error("", "daemon_protocol_error", parse_error);
        return false;
    }

    while (parsed_to != line.end() && is_json_whitespace(*parsed_to)) {
        ++parsed_to;
    }
    if (parsed_to != line.end()) {
        error_response = make_daemon_error(
            "", "daemon_protocol_error", "unexpected trailing JSON content");
        return false;
    }

    if (!value.is<picojson::object>()) {
        error_response = make_daemon_error(
            "", "daemon_protocol_error", "daemon request must be a JSON object");
        return false;
    }

    const picojson::object &object = value.get<picojson::object>();
    std::string unknown_field;
    const std::string top_level_fields[] = {"id", "command", "params"};
    if (!object_has_only_fields(object, top_level_fields, 3, unknown_field)) {
        error_response = make_daemon_error(
            "", "unknown_field", "unknown top-level field: " + unknown_field);
        return false;
    }

    std::string id;
    const auto id_field = object.find("id");
    if (id_field != object.end()) {
        if (!id_field->second.is<std::string>()) {
            error_response =
                make_daemon_error("", "invalid_id", REQUEST_ID_RULE_MESSAGE);
            return false;
        }

        id = id_field->second.get<std::string>();
        if (!is_valid_request_id(id)) {
            error_response =
                make_daemon_error("", "invalid_id", REQUEST_ID_RULE_MESSAGE);
            return false;
        }
    }

    const auto command_field = object.find("command");
    if (command_field == object.end() ||
        !command_field->second.is<std::string>()) {
        error_response = make_daemon_error(
            id, "missing_argument", "command is required and must be a string");
        return false;
    }

    const std::string command = command_field->second.get<std::string>();
    if (command != "motor.set" && command != "motor.off" &&
        command != "motor.get" && command != "motor.clear") {
        error_response =
            make_daemon_error(id, "unknown_command", "unknown daemon command");
        return false;
    }

    const auto params_field = object.find("params");
    if (params_field == object.end() ||
        !params_field->second.is<picojson::object>()) {
        error_response = make_daemon_error(
            id, "missing_argument", "params is required and must be an object");
        return false;
    }

    const picojson::object &params = params_field->second.get<picojson::object>();
    std::string code;
    std::string message;
    if (!validate_daemon_motor_params(command, params, code, message)) {
        error_response = make_daemon_error(id, code, message);
        return false;
    }

    request.sequence = sequence;
    request.id = id;
    request.command = command;
    request.params = params;
    return true;
}

bool write_all(int fd, const std::string &data)
{
    std::size_t written = 0;
    while (written < data.size()) {
        const ssize_t rc =
            write(fd, data.data() + written, data.size() - written);
        if (rc < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }

        if (rc == 0) {
            return false;
        }

        written += static_cast<std::size_t>(rc);
    }

    return true;
}

bool read_daemon_response_line(int fd, std::string &line, std::string &code)
{
    timeval timeout;
    timeout.tv_sec = DAEMON_CLIENT_RESPONSE_TIMEOUT_SECONDS;
    timeout.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    line.clear();
    char ch = '\0';
    while (true) {
        const ssize_t rc = read(fd, &ch, 1);
        if (rc == 0) {
            code = "daemon_protocol_error";
            return false;
        }

        if (rc < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                code = "daemon_timeout";
                return false;
            }
            code = "daemon_protocol_error";
            return false;
        }

        if (ch == '\n') {
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            return true;
        }

        line.push_back(ch);
        if (line.size() > MAX_DAEMON_REQUEST_LINE) {
            code = "daemon_protocol_error";
            return false;
        }
    }
}

bool parse_daemon_response_line(const std::string &line,
                                picojson::object &response)
{
    picojson::value value;
    std::string parse_error;
    std::string::const_iterator parsed_to =
        picojson::parse(value, line.begin(), line.end(), &parse_error);
    if (!parse_error.empty()) {
        return false;
    }

    while (parsed_to != line.end() && is_json_whitespace(*parsed_to)) {
        ++parsed_to;
    }
    if (parsed_to != line.end() || !value.is<picojson::object>()) {
        return false;
    }

    response = value.get<picojson::object>();
    return true;
}

bool read_result_int(const picojson::object &result,
                     const std::string &name,
                     int &value)
{
    const auto found = result.find(name);
    if (found == result.end() || !found->second.is<double>()) {
        return false;
    }

    const double number = found->second.get<double>();
    if (!std::isfinite(number) || std::floor(number) != number ||
        number < std::numeric_limits<int>::min() ||
        number > std::numeric_limits<int>::max()) {
        return false;
    }

    value = static_cast<int>(number);
    return true;
}

int read_validated_int_param(const picojson::object &params,
                             const std::string &name)
{
    return static_cast<int>(params.find(name)->second.get<double>());
}

picojson::object dispatch_daemon_request(const DaemonRequest &request)
{
    // This function runs only on the dispatcher thread. Any direct libwallaby
    // motor call added here stays serialized with every other daemon-backed
    // motor command.
    if (request.command == "motor.get") {
        const int port = read_validated_int_param(request.params, "port");
        const int value = gmpc(port);

        picojson::object result;
        result["port"] = json_int(port);
        result["value"] = json_int(value);
        return make_daemon_success_response(request, result);
    }

    if (request.command == "motor.set") {
        const int port = read_validated_int_param(request.params, "port");
        const int velocity =
            read_validated_int_param(request.params, "velocity");
        mav(port, velocity);

        picojson::object result;
        result["port"] = json_int(port);
        result["velocity"] = json_int(velocity);
        return make_daemon_success_response(request, result);
    }

    if (request.command == "motor.off") {
        picojson::object result;

        const auto port_field = request.params.find("port");
        if (port_field != request.params.end()) {
            const int port = read_validated_int_param(request.params, "port");
            off(port);
            result["port"] = json_int(port);
        } else {
            ao();

            picojson::array ports;
            for (int port = 0; port < 4; ++port) {
                ports.push_back(json_int(port));
            }
            result["ports"] = picojson::value(ports);
        }

        return make_daemon_success_response(request, result);
    }

    if (request.command == "motor.clear") {
        const int port = read_validated_int_param(request.params, "port");
        cmpc(port);

        picojson::object result;
        result["port"] = json_int(port);
        return make_daemon_success_response(request, result);
    }

    return make_daemon_echo_response(request);
}

void run_motor_dispatcher(DaemonRuntime &runtime)
{
    while (true) {
        std::shared_ptr<DaemonWorkItem> item;
        {
            std::unique_lock<std::mutex> lock(runtime.mutex);
            // Wait for either work or shutdown. Future motor-timeout wakeups
            // should be added to this same wait so timeout stops still run on
            // the serialized hardware path.
            runtime.queue_ready.wait(lock, [&runtime] {
                return runtime.stopping || !runtime.queue.empty();
            });

            if (runtime.queue.empty()) {
                if (runtime.stopping) {
                    break;
                }
                continue;
            }

            item = runtime.queue.front();
            runtime.queue.pop_front();
        }

        // Do not hold the runtime mutex while touching hardware. The queue
        // already establishes command order, and releasing the lock lets socket
        // handlers enqueue later requests or observe shutdown while libwallaby
        // work is in progress.
        const picojson::object response =
            dispatch_daemon_request(item->request);

        {
            std::lock_guard<std::mutex> lock(runtime.mutex);
            item->response = response;
            item->completed = true;
        }
        runtime.response_ready.notify_all();
    }
}

void stop_daemon_runtime(DaemonRuntime &runtime)
{
    {
        std::lock_guard<std::mutex> lock(runtime.mutex);
        // Wakes both the dispatcher and any socket handler waiting for a
        // response so daemon shutdown cannot leave threads blocked forever.
        runtime.stopping = true;
    }
    runtime.queue_ready.notify_all();
    runtime.response_ready.notify_all();
}

picojson::object queue_daemon_request_and_wait(DaemonRuntime &runtime,
                                               DaemonRequest &request)
{
    std::shared_ptr<DaemonWorkItem> item = std::make_shared<DaemonWorkItem>();

    {
        std::lock_guard<std::mutex> lock(runtime.mutex);
        if (runtime.stopping) {
            return make_daemon_error(request.id,
                                     "daemon_shutting_down",
                                     "daemon is shutting down");
        }

        // Assigning the sequence while holding the queue mutex makes insertion
        // order explicit and independent of socket accept order or partial
        // request arrival timing.
        request.sequence = runtime.next_sequence++;
        item->request = request;
        runtime.queue.push_back(item);
    }
    runtime.queue_ready.notify_one();

    std::unique_lock<std::mutex> lock(runtime.mutex);
    runtime.response_ready.wait(lock, [&runtime, &item] {
        return runtime.stopping || item->completed;
    });

    if (!item->completed) {
        return make_daemon_error(request.id,
                                 "daemon_shutting_down",
                                 "daemon is shutting down");
    }

    return item->response;
}

int emit_daemon_response(const OutputContext &output,
                         const std::string &command,
                         const picojson::object &response)
{
    // The public CLI keeps its normal output contract even though motors are
    // daemon-backed: JSON mode forwards the daemon object, while default mode
    // adapts it back to raw values or "ok".
    const auto ok_field = response.find("ok");
    if (ok_field == response.end() || !ok_field->second.is<bool>()) {
        return emit_error(output,
                          "daemon_protocol_error",
                          "daemon response is missing ok");
    }

    const bool ok = ok_field->second.get<bool>();
    if (!ok) {
        const auto error_field = response.find("error");
        if (error_field == response.end() ||
            !error_field->second.is<picojson::object>()) {
            return emit_error(output,
                              "daemon_protocol_error",
                              "daemon error response is malformed");
        }

        const picojson::object &error =
            error_field->second.get<picojson::object>();
        const auto code_field = error.find("code");
        const auto message_field = error.find("message");
        if (code_field == error.end() || message_field == error.end() ||
            !code_field->second.is<std::string>() ||
            !message_field->second.is<std::string>()) {
            return emit_error(output,
                              "daemon_protocol_error",
                              "daemon error response is malformed");
        }

        return emit_error(output,
                          code_field->second.get<std::string>(),
                          message_field->second.get<std::string>());
    }

    const auto result_field = response.find("result");
    if (result_field == response.end() ||
        !result_field->second.is<picojson::object>()) {
        return emit_error(output,
                          "daemon_protocol_error",
                          "daemon success response is missing result");
    }

    if (output.json) {
        write_json(response);
        return 0;
    }

    if (command == "motor.get") {
        int value = 0;
        const picojson::object &result =
            result_field->second.get<picojson::object>();
        if (!read_result_int(result, "value", value)) {
            return emit_error(output,
                              "daemon_protocol_error",
                              "daemon motor.get response is missing value");
        }

        std::cout << value << '\n';
        return 0;
    }

    std::cout << "ok\n";
    return 0;
}

int run_daemon_motor_command(const OutputContext &output,
                             const std::string &socket_path,
                             const std::string &command,
                             const picojson::object &params)
{
    // Motor subcommands are CLI adapters: validate and shape the request in
    // main(), send exactly one protocol message here, then translate the daemon
    // response back to the user's selected output mode.
    const std::string resolved_socket_path = resolve_socket_path(socket_path);
    if (resolved_socket_path.empty() ||
        resolved_socket_path.size() >= sizeof(sockaddr_un::sun_path)) {
        return emit_error(output,
                          "daemon_unavailable",
                          "daemon socket path is too long");
    }

    const int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        return emit_error(output,
                          "daemon_unavailable",
                          "could not create daemon socket");
    }

    sockaddr_un address;
    std::memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    std::strncpy(address.sun_path,
                 resolved_socket_path.c_str(),
                 sizeof(address.sun_path) - 1);

    if (connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) !=
        0) {
        close(fd);
        return emit_error(output,
                          "daemon_unavailable",
                          "botcli daemon is not available");
    }

    picojson::object request;
    if (!output.id.empty()) {
        request["id"] = picojson::value(output.id);
    }
    request["command"] = picojson::value(command);
    request["params"] = picojson::value(params);

    if (!write_all(fd, make_json_line(request))) {
        close(fd);
        return emit_error(output,
                          "daemon_protocol_error",
                          "could not send daemon request");
    }

    std::string response_line;
    std::string transport_code;
    if (!read_daemon_response_line(fd, response_line, transport_code)) {
        close(fd);
        if (transport_code == "daemon_timeout") {
            return emit_error(output,
                              "daemon_timeout",
                              "timed out waiting for daemon response");
        }
        return emit_error(output,
                          "daemon_protocol_error",
                          "could not read daemon response");
    }

    close(fd);

    picojson::object response;
    if (!parse_daemon_response_line(response_line, response)) {
        return emit_error(output,
                          "daemon_protocol_error",
                          "daemon response is not valid JSON");
    }

    return emit_daemon_response(output, command, response);
}

bool daemon_socket_has_listener(const std::string &socket_path)
{
    // A leftover pathname is common after an unclean exit. Only refuse startup
    // when another process is actually accepting connections there.
    if (access(socket_path.c_str(), F_OK) != 0) {
        return false;
    }

    const int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        return false;
    }

    sockaddr_un address;
    std::memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    std::strncpy(address.sun_path, socket_path.c_str(),
                 sizeof(address.sun_path) - 1);

    const bool connected =
        connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) ==
        0;
    close(fd);
    return connected;
}

void handle_daemon_client(int client_fd, DaemonRuntime &runtime)
{
    // Keep the wire format simple and bounded: read one newline-delimited JSON
    // object, answer one newline-delimited JSON object, and reject oversized
    // lines before they can consume unbounded memory.
    std::string line;
    char ch = '\0';

    while (!daemon_shutdown_requested) {
        const ssize_t rc = read(client_fd, &ch, 1);
        if (rc == 0) {
            return;
        }
        if (rc < 0) {
            if (errno == EINTR) {
                continue;
            }
            return;
        }

        if (ch == '\n') {
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }

            picojson::object response;
            DaemonRequest request;
            if (parse_daemon_request_line(
                    line, 0, request, response)) {
                response = queue_daemon_request_and_wait(runtime, request);
            }

            if (!write_all(client_fd, make_json_line(response))) {
                return;
            }

            line.clear();
            continue;
        }

        line.push_back(ch);
        if (line.size() > MAX_DAEMON_REQUEST_LINE) {
            picojson::object response = make_daemon_error(
                "", "daemon_protocol_error", "daemon request line is too long");
            write_all(client_fd, make_json_line(response));
            return;
        }
    }
}

int run_daemon(const std::string &cli_socket_path, int motor_timeout_ms)
{
    daemon_shutdown_requested = 0;

    const std::string socket_path = resolve_socket_path(cli_socket_path);
    if (socket_path.empty() ||
        socket_path.size() >= sizeof(sockaddr_un::sun_path)) {
        std::cerr << "error: daemon socket path is too long\n";
        return 1;
    }

    struct sigaction action;
    std::memset(&action, 0, sizeof(action));
    action.sa_handler = request_daemon_shutdown;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, nullptr);
    sigaction(SIGTERM, &action, nullptr);

    const int server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_fd < 0) {
        std::cerr << "error: could not create daemon socket: "
                  << std::strerror(errno) << '\n';
        return 1;
    }

    sockaddr_un address;
    std::memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    std::strncpy(address.sun_path, socket_path.c_str(),
                 sizeof(address.sun_path) - 1);

    if (daemon_socket_has_listener(socket_path)) {
        std::cerr << "error: daemon socket is already in use\n";
        close(server_fd);
        return 1;
    }

    unlink(socket_path.c_str());
    if (bind(server_fd,
             reinterpret_cast<sockaddr *>(&address),
             sizeof(address)) != 0) {
        std::cerr << "error: could not bind daemon socket: "
                  << std::strerror(errno) << '\n';
        close(server_fd);
        return 1;
    }

    chmod(socket_path.c_str(), S_IRUSR | S_IWUSR);

    if (listen(server_fd, 16) != 0) {
        std::cerr << "error: could not listen on daemon socket: "
                  << std::strerror(errno) << '\n';
        close(server_fd);
        unlink(socket_path.c_str());
        return 1;
    }

    DaemonRuntime runtime;
    runtime.motor_timeout_ms = motor_timeout_ms;
    // All daemon-backed motor calls flow through this single thread. The accept
    // loop only owns sockets and lifecycle, which prevents accidental hardware
    // access from client handling or signal/shutdown code.
    std::thread dispatcher(run_motor_dispatcher, std::ref(runtime));

    while (!daemon_shutdown_requested) {
        const int client_fd = accept(server_fd, nullptr, nullptr);
        if (client_fd < 0) {
            if (errno == EINTR) {
                continue;
            }

            std::cerr << "error: could not accept daemon client: "
                      << std::strerror(errno) << '\n';
            break;
        }

        handle_daemon_client(client_fd, runtime);
        close(client_fd);
    }

    stop_daemon_runtime(runtime);
    dispatcher.join();

    close(server_fd);
    unlink(socket_path.c_str());
    return 0;
}

int run_side_btn(const OutputContext &output)
{
    std::string error_message;
    if (!wombat_spi_available(error_message)) {
        return emit_error(output, "wallaby_error", error_message);
    }

    return emit_int_result(output, "side_btn", side_button());
}

int run_analog(const OutputContext &output, bool has_port, int port)
{
    std::string error_message;
    if (!wombat_spi_available(error_message)) {
        return emit_error(output, "wallaby_error", error_message);
    }

    if (has_port) {
        return emit_port_value_result(output, "analog", port, analog(port));
    }

    int values[6];
    for (int analog_port = 0; analog_port < 6; ++analog_port) {
        values[analog_port] = analog(analog_port);
    }

    return emit_analog_values_result(output, values);
}

int run_digital(const OutputContext &output, bool has_port, int port)
{
    std::string error_message;
    if (!wombat_spi_available(error_message)) {
        return emit_error(output, "wallaby_error", error_message);
    }

    if (has_port) {
        return emit_port_value_result(output, "digital", port, digital(port));
    }

    int values[10];
    for (int digital_port = 0; digital_port < 10; ++digital_port) {
        values[digital_port] = digital(digital_port);
    }

    return emit_digital_values_result(output, values);
}

int run_servo_get_enabled(const OutputContext &output, bool has_port, int port)
{
    std::string error_message;
    if (!wombat_spi_available(error_message)) {
        return emit_error(output, "wallaby_error", error_message);
    }

    if (has_port) {
        return emit_port_value_result(
            output, "servo.get_enabled", port, get_servo_enabled(port));
    }

    int values[4];
    for (int servo_port = 0; servo_port < 4; ++servo_port) {
        values[servo_port] = get_servo_enabled(servo_port);
    }

    return emit_servo_values_result(output, "servo.get_enabled", values);
}

int run_servo_get_position(const OutputContext &output, bool has_port, int port)
{
    std::string error_message;
    if (!wombat_spi_available(error_message)) {
        return emit_error(output, "wallaby_error", error_message);
    }

    if (has_port) {
        return emit_port_value_result(
            output, "servo.get", port, get_servo_position(port));
    }

    int values[4];
    for (int servo_port = 0; servo_port < 4; ++servo_port) {
        values[servo_port] = get_servo_position(servo_port);
    }

    return emit_servo_values_result(output, "servo.get", values);
}

int run_servo_set_enabled(const OutputContext &output, int port, int enabled)
{
    std::string error_message;
    if (!wombat_spi_available(error_message)) {
        return emit_error(output, "wallaby_error", error_message);
    }

    set_servo_enabled(port, enabled);
    return emit_servo_set_enabled_result(output, port, enabled);
}

int run_servo_set_position(const OutputContext &output, int port, int position)
{
    std::string error_message;
    if (!wombat_spi_available(error_message)) {
        return emit_error(output, "wallaby_error", error_message);
    }

    set_servo_position(port, position);
    return emit_servo_set_position_result(output, port, position);
}

int run_accel(const OutputContext &output, const std::string &axis)
{
    if (!axis.empty() && !is_axis(axis)) {
        return emit_error(output,
                          "invalid_axis",
                          "axis must be one of x, y, or z");
    }

    std::string error_message;
    if (!wombat_spi_available(error_message)) {
        return emit_error(output, "wallaby_error", error_message);
    }

    if (!axis.empty()) {
        return emit_axis_result(output, "accel", axis, read_accel_axis(axis));
    }

    return emit_xyz_result(output, "accel", accel_x(), accel_y(), accel_z());
}

int run_gyro(const OutputContext &output, const std::string &axis)
{
    if (!axis.empty() && !is_axis(axis)) {
        return emit_error(output,
                          "invalid_axis",
                          "axis must be one of x, y, or z");
    }

    std::string error_message;
    if (!wombat_spi_available(error_message)) {
        return emit_error(output, "wallaby_error", error_message);
    }

    if (!axis.empty()) {
        return emit_axis_result(output, "gyro", axis, read_gyro_axis(axis));
    }

    return emit_xyz_result(output, "gyro", gyro_x(), gyro_y(), gyro_z());
}

int run_magneto(const OutputContext &output, const std::string &axis)
{
    if (!axis.empty() && !is_axis(axis)) {
        return emit_error(output,
                          "invalid_axis",
                          "axis must be one of x, y, or z");
    }

    std::string error_message;
    if (!wombat_spi_available(error_message)) {
        return emit_error(output, "wallaby_error", error_message);
    }

    if (!axis.empty()) {
        return emit_axis_result(
            output, "magneto", axis, read_magneto_axis(axis));
    }

    return emit_xyz_result(
        output, "magneto", magneto_x(), magneto_y(), magneto_z());
}

}  // namespace

int main(int argc, char **argv)
{
    const RawGlobalOptions raw_options = inspect_raw_global_options(argc, argv);

    CLI::App app{"botcli"};
    app.require_subcommand(1);

    OutputContext output;
    CLI::Option *json_option =
        app.add_flag("--json", output.json, "Write one JSON object per response");
    CLI::Option *id_option =
        app.add_option("--id", output.id, "Request ID for JSON responses");
    std::string socket_path;
    app.add_option("--socket", socket_path, "Daemon socket path");

    int motor_timeout_ms = 0;
    auto *daemon = app.add_subcommand("daemon", "Run the botcli daemon");
    CLI::Option *motor_timeout_option = daemon
        ->add_option("--motor-timeout-ms",
                     motor_timeout_ms,
                     "Stop a motor after this many milliseconds without a refresh")
        ->check(positive_milliseconds_validator("motor timeout"));

    std::string accel_axis;
    auto *accel = app.add_subcommand("accel", "Read the accelerometer");
    accel
        ->add_option("axis", accel_axis, "Axis to read: x, y, or z")
        ->expected(0, 1);

    std::string gyro_axis;
    auto *gyro = app.add_subcommand("gyro", "Read the gyroscope");
    gyro
        ->add_option("axis", gyro_axis, "Axis to read: x, y, or z")
        ->expected(0, 1);

    std::string magneto_axis;
    auto *magneto = app.add_subcommand("magneto", "Read the magnetometer");
    magneto
        ->add_option("axis", magneto_axis, "Axis to read: x, y, or z")
        ->expected(0, 1);

    auto *side_btn = app.add_subcommand("side_btn", "Read the side button");

    int analog_port = -1;
    auto *analog_cmd = app.add_subcommand("analog", "Read analog sensor ports");
    CLI::Option *analog_port_option =
        analog_cmd
            ->add_option("port", analog_port, "Analog port to read: 0-5")
            ->expected(0, 1)
            ->check(analog_port_validator());

    int digital_port = -1;
    auto *digital_cmd =
        app.add_subcommand("digital", "Read digital sensor ports");
    CLI::Option *digital_port_option =
        digital_cmd
            ->add_option("port", digital_port, "Digital port to read: 0-9")
            ->expected(0, 1)
            ->check(digital_port_validator());

    auto *motor = app.add_subcommand("motor", "Read and write motor ports");
    motor->require_subcommand(1);

    int motor_set_port = -1;
    int motor_set_velocity = 0;
    auto *motor_set = motor->add_subcommand("set", "Set motor velocity");
    motor_set
        ->add_option("port", motor_set_port, "Motor port to write: 0-3")
        ->required()
        ->check(motor_port_validator());
    motor_set
        ->add_option("velocity",
                     motor_set_velocity,
                     "Motor velocity: -1500-1500")
        ->required()
        ->check(motor_velocity_validator());

    int motor_off_port = -1;
    auto *motor_off =
        motor->add_subcommand("off", "Stop one motor or all motors");
    CLI::Option *motor_off_port_option =
        motor_off
            ->add_option("port", motor_off_port, "Motor port to stop: 0-3")
            ->expected(0, 1)
            ->check(motor_port_validator());

    int motor_get_port = -1;
    auto *motor_get =
        motor->add_subcommand("get", "Read motor position counter");
    motor_get
        ->add_option("port", motor_get_port, "Motor port to read: 0-3")
        ->required()
        ->check(motor_port_validator());

    int motor_clear_port = -1;
    auto *motor_clear =
        motor->add_subcommand("clear", "Clear motor position counter");
    motor_clear
        ->add_option("port", motor_clear_port, "Motor port to clear: 0-3")
        ->required()
        ->check(motor_port_validator());

    auto *servo = app.add_subcommand("servo", "Read and write servo ports");
    servo->require_subcommand(1);

    int servo_get_enabled_port = -1;
    auto *servo_get_enabled =
        servo->add_subcommand("get_enabled", "Read servo enable state");
    CLI::Option *servo_get_enabled_port_option =
        servo_get_enabled
            ->add_option(
                "port", servo_get_enabled_port, "Servo port to read: 0-3")
            ->expected(0, 1)
            ->check(servo_port_validator());

    int servo_get_position_port = -1;
    auto *servo_get = servo->add_subcommand("get", "Read servo position");
    CLI::Option *servo_get_position_port_option =
        servo_get
            ->add_option(
                "port", servo_get_position_port, "Servo port to read: 0-3")
            ->expected(0, 1)
            ->check(servo_port_validator());

    int servo_set_enabled_port = -1;
    int servo_set_enabled_value = -1;
    auto *servo_set_enabled =
        servo->add_subcommand("set_enabled", "Set servo enable state");
    servo_set_enabled
        ->add_option(
            "port", servo_set_enabled_port, "Servo port to write: 0-3")
        ->required()
        ->check(servo_port_validator());
    servo_set_enabled
        ->add_option("enabled",
                     servo_set_enabled_value,
                     "Servo enabled value: 0 or 1")
        ->required()
        ->check(servo_enabled_validator());

    int servo_set_position_port = -1;
    int servo_set_position_value = -1;
    auto *servo_set = servo->add_subcommand("set", "Set servo position");
    servo_set
        ->add_option(
            "port", servo_set_position_port, "Servo port to write: 0-3")
        ->required()
        ->check(servo_port_validator());
    servo_set
        ->add_option(
            "position", servo_set_position_value, "Servo position: 0-2047")
        ->required()
        ->check(servo_position_validator());

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &err) {
        if (err.get_exit_code() == 0) {
            return app.exit(err);
        }

        OutputContext error_output;
        error_output.json = raw_options.json;
        if (raw_options.json && raw_options.has_id &&
            !raw_options.id_missing_value && is_valid_request_id(raw_options.id)) {
            error_output.id = raw_options.id;
        }

        if (raw_options.json && raw_options.has_id &&
            !raw_options.id_missing_value &&
            !is_valid_request_id(raw_options.id)) {
            return emit_error(error_output,
                              "invalid_id",
                              REQUEST_ID_RULE_MESSAGE);
        }

        return emit_error(
            error_output,
            parse_error_code(err),
            parse_error_message(err),
            err.get_exit_code());
    }

    if (id_option->count() > 0 && json_option->count() == 0) {
        return emit_error(output, "invalid_id", "--id is only valid with --json");
    }

    if (id_option->count() > 0 && !is_valid_request_id(output.id)) {
        OutputContext error_output;
        error_output.json = output.json;
        return emit_error(error_output,
                          "invalid_id",
                          REQUEST_ID_RULE_MESSAGE);
    }

    if (daemon->parsed()) {
        int resolved_motor_timeout_ms = 0;
        std::string motor_timeout_error;
        if (!resolve_motor_timeout_ms(motor_timeout_option->count() > 0,
                                      motor_timeout_ms,
                                      resolved_motor_timeout_ms,
                                      motor_timeout_error)) {
            return emit_error(output,
                              "invalid_motor_timeout",
                              motor_timeout_error);
        }

        return run_daemon(socket_path, resolved_motor_timeout_ms);
    }

    if (side_btn->parsed()) {
        return run_side_btn(output);
    }

    if (analog_cmd->parsed()) {
        return run_analog(output, analog_port_option->count() > 0, analog_port);
    }

    if (digital_cmd->parsed()) {
        return run_digital(
            output, digital_port_option->count() > 0, digital_port);
    }

    if (motor_set->parsed()) {
        picojson::object params;
        params["port"] = json_int(motor_set_port);
        params["velocity"] = json_int(motor_set_velocity);
        return run_daemon_motor_command(
            output, socket_path, "motor.set", params);
    }

    if (motor_off->parsed()) {
        picojson::object params;
        if (motor_off_port_option->count() > 0) {
            params["port"] = json_int(motor_off_port);
        }
        return run_daemon_motor_command(
            output, socket_path, "motor.off", params);
    }

    if (motor_get->parsed()) {
        picojson::object params;
        params["port"] = json_int(motor_get_port);
        return run_daemon_motor_command(
            output, socket_path, "motor.get", params);
    }

    if (motor_clear->parsed()) {
        picojson::object params;
        params["port"] = json_int(motor_clear_port);
        return run_daemon_motor_command(
            output, socket_path, "motor.clear", params);
    }

    if (servo_get_enabled->parsed()) {
        return run_servo_get_enabled(output,
                                     servo_get_enabled_port_option->count() > 0,
                                     servo_get_enabled_port);
    }

    if (servo_get->parsed()) {
        return run_servo_get_position(output,
                                      servo_get_position_port_option->count() > 0,
                                      servo_get_position_port);
    }

    if (servo_set_enabled->parsed()) {
        return run_servo_set_enabled(
            output, servo_set_enabled_port, servo_set_enabled_value);
    }

    if (servo_set->parsed()) {
        return run_servo_set_position(
            output, servo_set_position_port, servo_set_position_value);
    }

    if (accel->parsed()) {
        return run_accel(output, accel_axis);
    }

    if (gyro->parsed()) {
        return run_gyro(output, gyro_axis);
    }

    if (magneto->parsed()) {
        return run_magneto(output, magneto_axis);
    }

    return emit_error(output, "unknown_command", "unknown command");
}
