#include <iostream>
#include <cerrno>
#include <cstring>
#include <string>
#include <unistd.h>

#include "CLI11.hpp"
#include "picojson.h"

extern "C" {
#include <kipr/accel/accel.h>
#include <kipr/analog/analog.h>
#include <kipr/button/button.h>
#include <kipr/digital/digital.h>
#include <kipr/gyro/gyro.h>
#include <kipr/magneto/magneto.h>
}

namespace {

constexpr const char *WOMBAT_SPI_DEVICE = "/dev/spidev0.0";
constexpr const char *REQUEST_ID_RULE_MESSAGE =
    "request ID must be 1-128 characters and contain only ASCII letters, "
    "digits, '.', '_', ':', and '-'";

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

picojson::value json_int(int value)
{
    return picojson::value(static_cast<double>(value));
}

bool is_request_id_char(char ch)
{
    return (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
           (ch >= '0' && ch <= '9') || ch == '.' || ch == '_' ||
           ch == ':' || ch == '-';
}

bool is_valid_request_id(const std::string &id)
{
    if (id.empty() || id.size() > 128) {
        return false;
    }

    for (const char ch : id) {
        if (!is_request_id_char(ch)) {
            return false;
        }
    }

    return true;
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
         message.find("digital port") != std::string::npos)) {
        return "invalid_port";
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
    constexpr const char *port_prefix = "port: ";
    constexpr std::size_t port_prefix_len = 6;
    if (message.compare(0, port_prefix_len, port_prefix) == 0) {
        return message.substr(port_prefix_len);
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
