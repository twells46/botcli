#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <pthread.h>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/un.h>
#include <thread>
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
#include <kipr/motor/motor.h>
#include <kipr/servo/servo.h>
}

namespace
{

constexpr const char *REQUEST_ID_RULE_MESSAGE =
	"request ID must be 1-128 characters and contain only ASCII letters, "
	"digits, '.', '_', ':', and '-'";
constexpr std::size_t MAX_DAEMON_REQUEST_LINE = 64 * 1024;
constexpr int DAEMON_REQUEST_READ_TIMEOUT_SECONDS = 2;
constexpr int DAEMON_CLIENT_RESPONSE_TIMEOUT_SECONDS = 2;
const std::array<std::string, 14> DAEMON_ALLOWED_COMMANDS = {
	"accel",
	"analog",
	"digital",
	"gyro",
	"magneto",
	"motor.clear",
	"motor.get",
	"motor.off",
	"motor.set",
	"servo.get",
	"servo.get_enabled",
	"servo.set",
	"servo.set_enabled",
	"side_btn"
};

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
	// Socket handlers are producers and the hardware dispatcher is the only
	// consumer.
	std::mutex mutex;
	std::condition_variable queue_ready;
	std::condition_variable response_ready;
	std::deque<std::shared_ptr<DaemonWorkItem> > queue;
	bool stopping = false;
	// Zero means no motor timeout is configured. Positive values are
	// milliseconds and are resolved once at daemon startup.
	int motor_timeout_ms = 0;
};

using MotorClock = std::chrono::steady_clock;

struct MotorTimeoutState {
	std::array<bool, 4> active = {};
	std::array<MotorClock::time_point, 4> deadline = {};
};

void request_daemon_shutdown(int)
{
	daemon_shutdown_requested = 1;
}

picojson::value json_int(int value)
{
	return picojson::value(static_cast<double>(value));
}

CLI::Validator range_validator(const std::string &name, int minimum,
			       int maximum, const std::string &description,
			       const std::string &type_name)
{
	CLI::Range range(minimum, maximum);

	return CLI::Validator(
		[range, name, minimum, maximum](std::string &input) {
			std::string checked_input = input;
			if (!range(checked_input).empty()) {
				return name + " must be between " +
				       std::to_string(minimum) + " and " +
				       std::to_string(maximum);
			}

			return std::string();
		},
		description, type_name);
}

CLI::Validator servo_port_validator()
{
	return range_validator("servo port", 0, 3, "0-3", "SERVO_PORT");
}

CLI::Validator motor_port_validator()
{
	return range_validator("motor port", 0, 3, "0-3", "MOTOR_PORT");
}

bool is_axis(const std::string &axis)
{
	return axis == "x" || axis == "y" || axis == "z";
}

CLI::Validator axis_validator(const std::string &description,
			      const std::string &type_name)
{
	return CLI::Validator(
		[](std::string &input) {
			if (is_axis(input)) {
				return "";
			}
			return "Invalid axis -- axis must be one of x y z";
		},
		description, type_name);
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

CLI::Validator id_validator(const std::string &description,
			    const std::string &type_name)
{
	return CLI::Validator(
		[](std::string &input) {
			if (is_valid_request_id(input)) {
				return "";
			}
			return REQUEST_ID_RULE_MESSAGE;
		},
		description, type_name);
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

int emit_error(const OutputContext &output, const std::string &code,
	       const std::string &message, int exit_code = 1)
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

using AxisFunc = short int (*)();

int read_axis(const std::string &axis, AxisFunc x, AxisFunc y, AxisFunc z)
{
	if (axis == "x") {
		return x();
	} else if (axis == "y") {
		return y();
	}
	return z();
}

bool object_has_only_fields(const picojson::object &object,
			    const std::string fields[], std::size_t field_count,
			    std::string &unknown_field)
{
	for (const auto &entry : object) {
		bool found = false;
		for (std::size_t i = 0; i < field_count && !found; ++i) {
			if (entry.first == fields[i]) {
				found = true;
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
	if (command == "side_btn") {
		return object_has_only_fields(params, nullptr, 0,
					      unknown_field);
	}

	if (command == "accel" || command == "gyro" || command == "magneto") {
		const std::string fields[] = { "axis" };
		return object_has_only_fields(params, fields, 1, unknown_field);
	}

	if (command == "motor.set") {
		const std::string fields[] = { "port", "velocity" };
		return object_has_only_fields(params, fields, 2, unknown_field);
	}

	if (command == "motor.off") {
		const std::string fields[] = { "port" };
		return object_has_only_fields(params, fields, 1, unknown_field);
	}

	if (command == "servo.set_enabled") {
		const std::string fields[] = { "port", "enabled" };
		return object_has_only_fields(params, fields, 2, unknown_field);
	}

	if (command == "servo.set") {
		const std::string fields[] = { "port", "position" };
		return object_has_only_fields(params, fields, 2, unknown_field);
	}

	const std::string fields[] = { "port" };
	return object_has_only_fields(params, fields, 1, unknown_field);
}

bool is_daemon_axis_command(const std::string &command)
{
	return command == "accel" || command == "gyro" || command == "magneto";
}

bool daemon_command_has_port(const std::string &command)
{
	return command == "analog" || command == "digital" ||
	       command == "servo.get_enabled" || command == "servo.get" ||
	       command == "servo.set_enabled" || command == "servo.set" ||
	       command == "motor.set" || command == "motor.off" ||
	       command == "motor.get" || command == "motor.clear";
}

bool daemon_port_is_required(const std::string &command)
{
	return command == "motor.set" || command == "motor.clear" ||
	       command == "servo.set_enabled" || command == "servo.set";
}

int daemon_port_max(const std::string &command)
{
	if (command == "analog") {
		return 5;
	}
	if (command == "digital") {
		return 9;
	}
	return 3;
}

std::string daemon_port_name(const std::string &command)
{
	if (command == "analog") {
		return "analog port";
	}
	if (command == "digital") {
		return "digital port";
	}
	if (command == "servo.get_enabled" || command == "servo.get" ||
	    command == "servo.set_enabled" || command == "servo.set") {
		return "servo port";
	}
	return "motor port";
}

bool is_json_whitespace(char ch)
{
	return ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n';
}

bool parse_json_object_line(const std::string &line, picojson::object &object,
			    std::string &message,
			    const std::string &object_message)
{
	picojson::value value;
	std::string parse_error;
	std::string::const_iterator parsed_to =
		picojson::parse(value, line.begin(), line.end(), &parse_error);
	if (!parse_error.empty()) {
		message = parse_error;
		return false;
	}

	while (parsed_to != line.end() && is_json_whitespace(*parsed_to)) {
		++parsed_to;
	}
	if (parsed_to != line.end()) {
		message = "unexpected trailing JSON content";
		return false;
	}

	if (!value.is<picojson::object>()) {
		message = object_message;
		return false;
	}

	object = value.get<picojson::object>();
	return true;
}

bool json_int_value(const picojson::value &json_value, int &value)
{
	if (!json_value.is<double>()) {
		return false;
	}

	const double number = json_value.get<double>();
	if (!std::isfinite(number) || std::floor(number) != number ||
	    number < std::numeric_limits<int>::min() ||
	    number > std::numeric_limits<int>::max()) {
		return false;
	}

	value = static_cast<int>(number);
	return true;
}

bool read_integral_param(const picojson::object &params,
			 const std::string &name, bool required, int &value,
			 std::string &code, std::string &message)
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

	if (!json_int_value(found->second, value)) {
		code = "invalid_argument";
		message = name + " must be an integer";
		return false;
	}

	return true;
}

bool validate_daemon_params(const std::string &command,
			    const picojson::object &params, std::string &code,
			    std::string &message)
{
	// The daemon is a trust boundary, so repeat validation here even though the
	// built-in CLI client validates its own arguments before sending a request.
	std::string unknown_field;
	if (!params_have_only_fields(params, command, unknown_field)) {
		code = "daemon_protocol_error";
		message = "unknown params field: " + unknown_field;
		return false;
	}

	if (command == "side_btn") {
		return true;
	}

	if (is_daemon_axis_command(command)) {
		const auto axis = params.find("axis");
		if (axis == params.end()) {
			return true;
		}

		if (!axis->second.is<std::string>() ||
		    !is_axis(axis->second.get<std::string>())) {
			code = "invalid_axis";
			message = "axis must be one of x, y, or z";
			return false;
		}

		return true;
	}

	if (daemon_command_has_port(command)) {
		int port = -1;
		const bool port_required = daemon_port_is_required(command);
		if (!read_integral_param(params, "port", port_required, port,
					 code, message)) {
			return false;
		}

		if ((port_required || params.find("port") != params.end()) &&
		    (port < 0 || port > daemon_port_max(command))) {
			code = "invalid_port";
			message = daemon_port_name(command) +
				  " must be between 0 and " +
				  std::to_string(daemon_port_max(command));
			return false;
		}
	}

	if (command == "servo.set_enabled") {
		int enabled = 0;
		if (!read_integral_param(params, "enabled", true, enabled, code,
					 message)) {
			return false;
		}

		if (enabled < 0 || enabled > 1) {
			code = "invalid_argument";
			message = "servo enabled value must be between 0 and 1";
			return false;
		}

		return true;
	}

	if (command == "servo.set") {
		int position = 0;
		if (!read_integral_param(params, "position", true, position,
					 code, message)) {
			return false;
		}

		if (position < 0 || position > 2047) {
			code = "invalid_position";
			message = "servo position must be between 0 and 2047";
			return false;
		}

		return true;
	}

	if (command != "motor.set") {
		return true;
	}

	int velocity = 0;
	if (!read_integral_param(params, "velocity", true, velocity, code,
				 message)) {
		return false;
	}

	if (velocity < -1500 || velocity > 1500) {
		code = "invalid_velocity";
		message = "motor velocity must be between -1500 and 1500";
		return false;
	}

	return true;
}

// A daemon protocol message is exactly one JSON object per line. Protocol
// errors are answered by the socket handler and never enter the hardware
// queue, which keeps the dispatcher focused on validated motor requests.
bool parse_daemon_request_line(const std::string &line, DaemonRequest &request,
			       picojson::object &error_response)
{
	picojson::object object;
	std::string parse_message;
	if (!parse_json_object_line(line, object, parse_message,
				    "daemon request must be a JSON object")) {
		error_response = make_daemon_error("", "daemon_protocol_error",
						   parse_message);
		return false;
	}

	std::string unknown_field;
	const std::string top_level_fields[] = { "id", "command", "params" };
	if (!object_has_only_fields(object, top_level_fields, 3,
				    unknown_field)) {
		error_response = make_daemon_error("", "daemon_protocol_error",
						   "unknown top-level field: " +
							   unknown_field);
		return false;
	}

	std::string id;
	// Note: `object.find` returns an iterator at the requested element.
	// If element doesn't exist, it returns the past-the-end sentinel
	// iterator `end()`, hence the `object.end()` comparison below.
	const auto id_field = object.find("id");
	if (id_field != object.end()) {
		if (!id_field->second.is<std::string>()) {
			error_response = make_daemon_error(
				"", "invalid_id", REQUEST_ID_RULE_MESSAGE);
			return false;
		}

		id = id_field->second.get<std::string>();
		if (!is_valid_request_id(id)) {
			error_response = make_daemon_error(
				"", "invalid_id", REQUEST_ID_RULE_MESSAGE);
			return false;
		}
	}

	const auto command_field = object.find("command");
	if (command_field == object.end() ||
	    !command_field->second.is<std::string>()) {
		error_response = make_daemon_error(
			id, "missing_argument",
			"command is required and must be a string");
		return false;
	}

	const std::string command = command_field->second.get<std::string>();
	if (std::find(DAEMON_ALLOWED_COMMANDS.begin(),
		      DAEMON_ALLOWED_COMMANDS.end(),
		      command) == DAEMON_ALLOWED_COMMANDS.end()) {
		error_response = make_daemon_error(id, "unknown_command",
						   "unknown daemon command");
		return false;
	}

	const auto params_field = object.find("params");
	if (params_field == object.end() ||
	    !params_field->second.is<picojson::object>()) {
		error_response = make_daemon_error(
			id, "missing_argument",
			"params is required and must be an object");
		return false;
	}

	const picojson::object &params =
		params_field->second.get<picojson::object>();
	std::string code;
	std::string message;
	if (!validate_daemon_params(command, params, code, message)) {
		error_response = make_daemon_error(id, code, message);
		return false;
	}

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
	std::string parse_message;
	return parse_json_object_line(line, response, parse_message,
				      "daemon response must be a JSON object");
}

bool read_result_int(const picojson::object &result, const std::string &name,
		     int &value)
{
	const auto found = result.find(name);
	if (found == result.end()) {
		return false;
	}

	return json_int_value(found->second, value);
}

int read_validated_int_param(const picojson::object &params,
			     const std::string &name)
{
	return static_cast<int>(params.find(name)->second.get<double>());
}

std::string read_validated_string_param(const picojson::object &params,
					const std::string &name)
{
	return params.find(name)->second.get<std::string>();
}

bool next_motor_timeout_deadline(const MotorTimeoutState &timeouts,
				 MotorClock::time_point &deadline)
{
	bool found = false;
	for (int port = 0; port < 4; ++port) {
		if (!timeouts.active[port]) {
			continue;
		}
		if (!found || timeouts.deadline[port] < deadline) {
			deadline = timeouts.deadline[port];
			found = true;
		}
	}
	return found;
}

int expired_motor_timeout_port(const MotorTimeoutState &timeouts,
			       MotorClock::time_point now)
{
	for (int port = 0; port < 4; ++port) {
		if (timeouts.active[port] && timeouts.deadline[port] <= now) {
			return port;
		}
	}
	return -1;
}

picojson::object dispatch_daemon_request(const DaemonRequest &request,
					 MotorTimeoutState &timeouts,
					 int motor_timeout_ms)
{
	// This function runs only on the dispatcher thread. Any direct libwallaby
	// call added here stays serialized with every other daemon-backed command.
	if (request.command == "side_btn") {
		picojson::object result;
		result["value"] = json_int(side_button());
		return make_daemon_success_response(request, result);
	}

	if (request.command == "analog") {
		picojson::object result;
		const auto port_field = request.params.find("port");
		if (port_field != request.params.end()) {
			const int port = read_validated_int_param(
				request.params, "port");
			result["port"] = json_int(port);
			result["value"] = json_int(analog(port));
		} else {
			picojson::array values;
			for (int port = 0; port < 6; ++port) {
				values.push_back(json_int(analog(port)));
			}
			result["values"] = picojson::value(values);
		}
		return make_daemon_success_response(request, result);
	}

	if (request.command == "digital") {
		picojson::object result;
		const auto port_field = request.params.find("port");
		if (port_field != request.params.end()) {
			const int port = read_validated_int_param(
				request.params, "port");
			result["port"] = json_int(port);
			result["value"] = json_int(digital(port));
		} else {
			picojson::array values;
			for (int port = 0; port < 10; ++port) {
				values.push_back(json_int(digital(port)));
			}
			result["values"] = picojson::value(values);
		}
		return make_daemon_success_response(request, result);
	}

	if (request.command == "accel" || request.command == "gyro" ||
	    request.command == "magneto") {
		AxisFunc x = &accel_x;
		AxisFunc y = &accel_y;
		AxisFunc z = &accel_z;
		if (request.command == "gyro") {
			x = &gyro_x;
			y = &gyro_y;
			z = &gyro_z;
		} else if (request.command == "magneto") {
			x = &magneto_x;
			y = &magneto_y;
			z = &magneto_z;
		}

		picojson::object result;
		const auto axis_field = request.params.find("axis");
		if (axis_field != request.params.end()) {
			const std::string axis = read_validated_string_param(
				request.params, "axis");
			result["axis"] = picojson::value(axis);
			result["value"] = json_int(read_axis(axis, x, y, z));
		} else {
			result["x"] = json_int(x());
			result["y"] = json_int(y());
			result["z"] = json_int(z());
		}
		return make_daemon_success_response(request, result);
	}

	if (request.command == "servo.get_enabled" ||
	    request.command == "servo.get") {
		const bool read_enabled = request.command ==
					  "servo.get_enabled";
		picojson::object result;
		const auto port_field = request.params.find("port");
		if (port_field != request.params.end()) {
			const int port = read_validated_int_param(
				request.params, "port");
			result["port"] = json_int(port);
			result["value"] = json_int(
				read_enabled ? get_servo_enabled(port) :
					       get_servo_position(port));
		} else {
			picojson::array values;
			for (int port = 0; port < 4; ++port) {
				values.push_back(json_int(
					read_enabled ?
						get_servo_enabled(port) :
						get_servo_position(port)));
			}
			result["values"] = picojson::value(values);
		}
		return make_daemon_success_response(request, result);
	}

	if (request.command == "servo.set_enabled") {
		const int port =
			read_validated_int_param(request.params, "port");
		const int enabled =
			read_validated_int_param(request.params, "enabled");
		set_servo_enabled(port, enabled);

		picojson::object result;
		result["port"] = json_int(port);
		result["enabled"] = json_int(enabled);
		return make_daemon_success_response(request, result);
	}

	if (request.command == "servo.set") {
		const int port =
			read_validated_int_param(request.params, "port");
		const int position =
			read_validated_int_param(request.params, "position");
		set_servo_position(port, position);

		picojson::object result;
		result["port"] = json_int(port);
		result["position"] = json_int(position);
		return make_daemon_success_response(request, result);
	}

	if (request.command == "motor.get") {
		picojson::object result;
		const auto port_field = request.params.find("port");
		if (port_field != request.params.end()) {
			const int port = read_validated_int_param(
				request.params, "port");
			result["port"] = json_int(port);
			result["value"] = json_int(gmpc(port));
		} else {
			picojson::array values;
			for (int port = 0; port < 4; ++port) {
				values.push_back(json_int(gmpc(port)));
			}
			result["values"] = picojson::value(values);
		}
		return make_daemon_success_response(request, result);
	}

	if (request.command == "motor.set") {
		const int port =
			read_validated_int_param(request.params, "port");
		const int velocity =
			read_validated_int_param(request.params, "velocity");
		mav(port, velocity);
		if (motor_timeout_ms > 0) {
			timeouts.active[port] = true;
			timeouts.deadline[port] =
				MotorClock::now() +
				std::chrono::milliseconds(motor_timeout_ms);
		}

		picojson::object result;
		result["port"] = json_int(port);
		result["velocity"] = json_int(velocity);
		return make_daemon_success_response(request, result);
	}

	if (request.command == "motor.off") {
		picojson::object result;

		const auto port_field = request.params.find("port");
		if (port_field != request.params.end()) {
			const int port = read_validated_int_param(
				request.params, "port");
			off(port);
			timeouts.active[port] = false;
			result["port"] = json_int(port);
		} else {
			ao();
			timeouts.active.fill(false);

			picojson::array ports;
			for (int port = 0; port < 4; ++port) {
				ports.push_back(json_int(port));
			}
			result["ports"] = picojson::value(ports);
		}

		return make_daemon_success_response(request, result);
	}

	if (request.command == "motor.clear") {
		const int port =
			read_validated_int_param(request.params, "port");
		cmpc(port);

		picojson::object result;
		result["port"] = json_int(port);
		return make_daemon_success_response(request, result);
	}

	return make_daemon_error(request.id, "daemon_protocol_error",
				 "unknown daemon command");
}

void run_hardware_dispatcher(DaemonRuntime &runtime)
{
	MotorTimeoutState timeouts;

	while (true) {
		std::shared_ptr<DaemonWorkItem> item;
		int expired_port = -1;
		{
			std::unique_lock<std::mutex> mtx_lock(runtime.mutex);
			while (true) {
				if (runtime.stopping) {
					break;
				}

				expired_port = expired_motor_timeout_port(
					timeouts, MotorClock::now());
				if (expired_port >= 0 ||
				    !runtime.queue.empty()) {
					break;
				}

				MotorClock::time_point deadline;
				if (next_motor_timeout_deadline(timeouts,
								deadline)) {
					runtime.queue_ready.wait_until(
						mtx_lock, deadline);
				} else {
					runtime.queue_ready.wait(mtx_lock);
				}
			}

			if (runtime.stopping) {
				break;
			}
			if (expired_port >= 0) {
				timeouts.active[expired_port] = false;
			} else if (!runtime.queue.empty()) {
				item = runtime.queue.front();
				runtime.queue.pop_front();
			}

			if (!item && expired_port < 0) {
				continue;
			}
		}

		if (expired_port >= 0) {
			off(expired_port);
			continue;
		}

		// Do not hold the runtime mutex while touching hardware. The queue
		// already establishes command order, and releasing the lock lets socket
		// handlers enqueue later requests or observe shutdown while libwallaby
		// work is in progress.
		const picojson::object response = dispatch_daemon_request(
			item->request, timeouts, runtime.motor_timeout_ms);

		{
			std::lock_guard<std::mutex> lock(runtime.mutex);
			item->response = response;
			item->completed = true;
		}
		runtime.response_ready.notify_all();
	}

	ao();
}

void stop_daemon_runtime(DaemonRuntime &runtime)
{
	{
		std::lock_guard<std::mutex> lock(runtime.mutex);
		// Wakes both the dispatcher and any socket handler waiting for a
		// response so daemon shutdown cannot leave threads blocked forever.
		runtime.stopping = true;
		while (!runtime.queue.empty()) {
			std::shared_ptr<DaemonWorkItem> item =
				runtime.queue.front();
			runtime.queue.pop_front();
			item->response = make_daemon_error(
				item->request.id, "daemon_shutting_down",
				"daemon is shutting down");
			item->completed = true;
		}
	}
	runtime.queue_ready.notify_all();
	runtime.response_ready.notify_all();
}

picojson::object queue_daemon_request_and_wait(DaemonRuntime &runtime,
					       DaemonRequest &request)
{
	std::shared_ptr<DaemonWorkItem> item =
		std::make_shared<DaemonWorkItem>();

	{
		std::lock_guard<std::mutex> lock(runtime.mutex);
		if (runtime.stopping) {
			return make_daemon_error(request.id,
						 "daemon_shutting_down",
						 "daemon is shutting down");
		}

		item->request = request;
		runtime.queue.push_back(item);
	}
	runtime.queue_ready.notify_one();

	std::unique_lock<std::mutex> lock(runtime.mutex);
	runtime.response_ready.wait(lock, [&runtime, &item] {
		return runtime.stopping || item->completed;
	});

	if (!item->completed) {
		return make_daemon_error(request.id, "daemon_shutting_down",
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
		return emit_error(output, "daemon_protocol_error",
				  "daemon response is missing ok");
	}

	const bool ok = ok_field->second.get<bool>();
	if (!ok) {
		const auto error_field = response.find("error");
		if (error_field == response.end() ||
		    !error_field->second.is<picojson::object>()) {
			return emit_error(output, "daemon_protocol_error",
					  "daemon error response is malformed");
		}

		const picojson::object &error =
			error_field->second.get<picojson::object>();
		const auto code_field = error.find("code");
		const auto message_field = error.find("message");
		if (code_field == error.end() || message_field == error.end() ||
		    !code_field->second.is<std::string>() ||
		    !message_field->second.is<std::string>()) {
			return emit_error(output, "daemon_protocol_error",
					  "daemon error response is malformed");
		}

		return emit_error(output, code_field->second.get<std::string>(),
				  message_field->second.get<std::string>());
	}

	const auto result_field = response.find("result");
	if (result_field == response.end() ||
	    !result_field->second.is<picojson::object>()) {
		return emit_error(output, "daemon_protocol_error",
				  "daemon success response is missing result");
	}

	if (output.json) {
		write_json(response);
		return 0;
	}

	const picojson::object &result =
		result_field->second.get<picojson::object>();

	if (result.find("value") != result.end()) {
		int value = 0;
		if (!read_result_int(result, "value", value)) {
			return emit_error(output, "daemon_protocol_error",
					  "daemon response value is malformed");
		}

		std::cout << value << '\n';
		return 0;
	}

	const auto values_field = result.find("values");
	if (values_field != result.end()) {
		if (!values_field->second.is<picojson::array>()) {
			return emit_error(
				output, "daemon_protocol_error",
				"daemon response values are malformed");
		}

		const picojson::array &values =
			values_field->second.get<picojson::array>();
		for (std::size_t i = 0; i < values.size(); ++i) {
			int value = 0;
			if (!json_int_value(values[i], value)) {
				return emit_error(
					output, "daemon_protocol_error",
					"daemon response values are malformed");
			}
			if (i != 0) {
				std::cout << " ";
			}
			std::cout << value;
		}
		std::cout << '\n';
		return 0;
	}

	if (result.find("x") != result.end() ||
	    result.find("y") != result.end() ||
	    result.find("z") != result.end()) {
		int x = 0;
		int y = 0;
		int z = 0;
		if (!read_result_int(result, "x", x) ||
		    !read_result_int(result, "y", y) ||
		    !read_result_int(result, "z", z)) {
			return emit_error(output, "daemon_protocol_error",
					  "daemon response axes are malformed");
		}

		std::cout << x << " " << y << " " << z << '\n';
		return 0;
	}

	std::cout << "ok\n";
	return 0;
}

int run_daemon_command(const OutputContext &output,
		       const std::string &socket_path,
		       const std::string &command,
		       const picojson::object &params)
{
	// Daemon-backed subcommands validate and shape the request in
	// main(), send exactly one protocol message here, then translate the daemon
	// response back to the user's selected output mode.

	const int fd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (fd < 0) {
		return emit_error(output, "daemon_unavailable",
				  "could not create daemon socket");
	}

	sockaddr_un address;
	std::memset(&address, 0, sizeof(address));
	address.sun_family = AF_UNIX;
	std::strncpy(address.sun_path, socket_path.c_str(),
		     sizeof(address.sun_path) - 1);

	if (connect(fd, reinterpret_cast<sockaddr *>(&address),
		    sizeof(address)) != 0) {
		close(fd);
		return emit_error(output, "daemon_unavailable",
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
		return emit_error(output, "daemon_protocol_error",
				  "could not send daemon request");
	}

	std::string response_line;
	std::string transport_code;
	if (!read_daemon_response_line(fd, response_line, transport_code)) {
		close(fd);
		if (transport_code == "daemon_timeout") {
			return emit_error(
				output, "daemon_timeout",
				"timed out waiting for daemon response");
		}
		return emit_error(output, "daemon_protocol_error",
				  "could not read daemon response");
	}

	close(fd);

	picojson::object response;
	if (!parse_daemon_response_line(response_line, response)) {
		return emit_error(output, "daemon_protocol_error",
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

	const bool connected = connect(fd,
				       reinterpret_cast<sockaddr *>(&address),
				       sizeof(address)) == 0;
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
	MotorClock::time_point read_deadline =
		MotorClock::now() +
		std::chrono::seconds(DAEMON_REQUEST_READ_TIMEOUT_SECONDS);

	while (!daemon_shutdown_requested) {
		const MotorClock::time_point now = MotorClock::now();
		if (now >= read_deadline) {
			picojson::object response = make_daemon_error(
				"", "daemon_timeout",
				"timed out reading daemon request");
			write_all(client_fd, make_json_line(response));
			return;
		}

		const std::chrono::microseconds remaining =
			std::chrono::duration_cast<std::chrono::microseconds>(
				read_deadline - now);
		const long long remaining_us =
			std::max<long long>(1, remaining.count());
		timeval timeout;
		timeout.tv_sec = static_cast<long>(remaining_us / 1000000);
		timeout.tv_usec = static_cast<long>(remaining_us % 1000000);
		setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout,
			   sizeof(timeout));

		const ssize_t rc = read(client_fd, &ch, 1);
		if (rc == 0) {
			return;
		}
		if (rc < 0) {
			if (errno == EINTR) {
				continue;
			}
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				picojson::object response = make_daemon_error(
					"", "daemon_timeout",
					"timed out reading daemon request");
				write_all(client_fd, make_json_line(response));
			}
			return;
		}

		if (ch == '\n') {
			if (!line.empty() && line.back() == '\r') {
				line.pop_back();
			}

			picojson::object response;
			DaemonRequest request;
			if (parse_daemon_request_line(line, request,
						      response)) {
				response = queue_daemon_request_and_wait(
					runtime, request);
			}

			if (!write_all(client_fd, make_json_line(response))) {
				return;
			}

			line.clear();
			read_deadline =
				MotorClock::now() +
				std::chrono::seconds(
					DAEMON_REQUEST_READ_TIMEOUT_SECONDS);
			continue;
		}

		line.push_back(ch);
		if (line.size() > MAX_DAEMON_REQUEST_LINE) {
			picojson::object response = make_daemon_error(
				"", "daemon_protocol_error",
				"daemon request line is too long");
			write_all(client_fd, make_json_line(response));
			return;
		}
	}
}

int run_daemon(const std::string &socket_path, int motor_timeout_ms)
{
	daemon_shutdown_requested = 0;

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
	if (bind(server_fd, reinterpret_cast<sockaddr *>(&address),
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
	// All daemon-backed hardware calls flow through this single thread. The accept
	// loop only owns sockets and lifecycle, which prevents accidental hardware
	// access from client handling or signal/shutdown code.
	sigset_t daemon_signal_set;
	sigemptyset(&daemon_signal_set);
	sigaddset(&daemon_signal_set, SIGINT);
	sigaddset(&daemon_signal_set, SIGTERM);
	sigset_t old_signal_set;
	pthread_sigmask(SIG_BLOCK, &daemon_signal_set, &old_signal_set);
	std::thread dispatcher(run_hardware_dispatcher, std::ref(runtime));
	pthread_sigmask(SIG_SETMASK, &old_signal_set, nullptr);

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

} // namespace

int main(int argc, char **argv)
{
	const RawGlobalOptions raw_options =
		inspect_raw_global_options(argc, argv);

	CLI::App app{ "botcli" };
	app.require_subcommand(1);

	// Global options
	OutputContext output;
	CLI::Option *json_option = app.add_flag(
		"--json", output.json, "Write one JSON object per response");
	CLI::Option *id_option =
		app.add_option("--id", output.id,
			       "Request ID for JSON responses")
			->needs(json_option)
			->check(id_validator("Valid ID", "ID"));

	std::string socket_path = "/tmp/botcli.sock";
	if (const char *env_runtime_dir = std::getenv("XDG_RUNTIME_DIR")) {
		socket_path = std::string(env_runtime_dir) + "/botcli.sock";
	}
	constexpr auto SOCK_LEN = sizeof(sockaddr_un::sun_path) - 1;
	app.add_option("--socket", socket_path, "Daemon socket path")
		->capture_default_str()
		->check(CLI::Validator(
			[](std::string &input) {
				if (input.size() >= SOCK_LEN) {
					return "Socket address too long. Must be less than " +
					       std::to_string(SOCK_LEN);
				}
				return std::string();
			},
			"Socket length", "SOCK_LEN"));

	// -------- daemon subcommand --------
	int motor_timeout_ms = 0;
	auto *daemon = app.add_subcommand("daemon", "Run the botcli daemon");
	daemon->add_option(
		      "--motor-timeout-ms", motor_timeout_ms,
		      "Stop a motor after this many milliseconds without a refresh")
		->check(range_validator("motor timeout", 1,
					std::numeric_limits<int>::max(), "1+",
					"MOTOR_TIMEOUT"));

	// -------- accel subcommand --------
	std::string accel_axis;
	auto *accel = app.add_subcommand("accel", "Read the accelerometer");
	accel->add_option("axis", accel_axis, "Axis to read: x, y, or z")
		->expected(0, 1)
		->check(axis_validator("Axis in [xyz]", "AXIS"));

	// -------- gyro subcommand --------
	std::string gyro_axis;
	auto *gyro = app.add_subcommand("gyro", "Read the gyroscope");
	gyro->add_option("axis", gyro_axis, "Axis to read: x, y, or z")
		->expected(0, 1)
		->check(axis_validator("Axis in [xyz]", "AXIS"));

	// -------- magneto subcommand --------
	std::string magneto_axis;
	auto *magneto = app.add_subcommand("magneto", "Read the magnetometer");
	magneto->add_option("axis", magneto_axis, "Axis to read: x, y, or z")
		->expected(0, 1)
		->check(axis_validator("Axis in [xyz]", "AXIS"));

	// -------- side_btn subcommand --------
	auto *side_btn = app.add_subcommand("side_btn", "Read the side button");

	// -------- analog subcommand --------
	int analog_port = -1;
	auto *analog_cmd =
		app.add_subcommand("analog", "Read analog sensor ports");
	CLI::Option *analog_port_option =
		analog_cmd
			->add_option("port", analog_port,
				     "Analog port to read: 0-5")
			->expected(0, 1)
			->check(range_validator("analog port", 0, 5, "0-5",
						"ANALOG_PORT"));

	// -------- digital subcommand --------
	int digital_port = -1;
	auto *digital_cmd =
		app.add_subcommand("digital", "Read digital sensor ports");
	CLI::Option *digital_port_option =
		digital_cmd
			->add_option("port", digital_port,
				     "Digital port to read: 0-9")
			->expected(0, 1)
			->check(range_validator("digital port", 0, 9, "0-9",
						"DIGITAL_PORT"));

	// -------- Motor subcommand --------
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
		->add_option("velocity", motor_set_velocity,
			     "Motor velocity: -1500-1500")
		->required()
		->check(range_validator("motor velocity", -1500, 1500,
					"-1500-1500", "MOTOR_VELOCITY"));

	int motor_off_port = -1;
	auto *motor_off =
		motor->add_subcommand("off", "Stop one motor or all motors");
	CLI::Option *motor_off_port_option =
		motor_off
			->add_option("port", motor_off_port,
				     "Motor port to stop: 0-3")
			->expected(0, 1)
			->check(motor_port_validator());

	int motor_get_port = -1;
	auto *motor_get =
		motor->add_subcommand("get", "Read motor position counter");
	CLI::Option *motor_get_port_option =
		motor_get
			->add_option("port", motor_get_port,
				     "Motor port to read: 0-3")
			->expected(0, 1)
			->check(motor_port_validator());

	int motor_clear_port = -1;
	auto *motor_clear =
		motor->add_subcommand("clear", "Clear motor position counter");
	motor_clear
		->add_option("port", motor_clear_port,
			     "Motor port to clear: 0-3")
		->required()
		->check(motor_port_validator());

	// -------- Servo subcommand --------
	auto *servo = app.add_subcommand("servo", "Read and write servo ports");
	servo->require_subcommand(1);

	int servo_get_enabled_port = -1;
	auto *servo_get_enabled =
		servo->add_subcommand("get_enabled", "Read servo enable state");
	CLI::Option *servo_get_enabled_port_option =
		servo_get_enabled
			->add_option("port", servo_get_enabled_port,
				     "Servo port to read: 0-3")
			->expected(0, 1)
			->check(servo_port_validator());

	int servo_get_position_port = -1;
	auto *servo_get = servo->add_subcommand("get", "Read servo position");
	CLI::Option *servo_get_position_port_option =
		servo_get
			->add_option("port", servo_get_position_port,
				     "Servo port to read: 0-3")
			->expected(0, 1)
			->check(servo_port_validator());

	int servo_set_enabled_port = -1;
	int servo_set_enabled_value = -1;
	auto *servo_set_enabled =
		servo->add_subcommand("set_enabled", "Set servo enable state");
	servo_set_enabled
		->add_option("port", servo_set_enabled_port,
			     "Servo port to write: 0-3")
		->required()
		->check(servo_port_validator());
	servo_set_enabled
		->add_option("enabled", servo_set_enabled_value,
			     "Servo enabled value: 0 or 1")
		->required()
		->check(range_validator("servo enabled value", 0, 1, "0|1",
					"SERVO_ENABLED"));

	int servo_set_position_port = -1;
	int servo_set_position_value = -1;
	auto *servo_set = servo->add_subcommand("set", "Set servo position");
	servo_set
		->add_option("port", servo_set_position_port,
			     "Servo port to write: 0-3")
		->required()
		->check(servo_port_validator());
	servo_set
		->add_option("position", servo_set_position_value,
			     "Servo position: 0-2047")
		->required()
		->check(range_validator("servo position", 0, 2047, "0-2047",
					"SERVO_POSITION"));

	// Parse and run
	try {
		CLI11_PARSE(app, argc, argv);
	} catch (const CLI::ParseError &err) {
		return app.exit(err);
	}

	if (daemon->parsed()) {
		return run_daemon(socket_path, motor_timeout_ms);
	}

	if (side_btn->parsed()) {
		picojson::object params;
		return run_daemon_command(output, socket_path, "side_btn",
					  params);
	}

	if (analog_cmd->parsed()) {
		picojson::object params;
		if (analog_port_option->count() > 0) {
			params["port"] = json_int(analog_port);
		}
		return run_daemon_command(output, socket_path, "analog",
					  params);
	}

	if (digital_cmd->parsed()) {
		picojson::object params;
		if (digital_port_option->count() > 0) {
			params["port"] = json_int(digital_port);
		}
		return run_daemon_command(output, socket_path, "digital",
					  params);
	}

	if (motor_set->parsed()) {
		picojson::object params;
		params["port"] = json_int(motor_set_port);
		params["velocity"] = json_int(motor_set_velocity);
		return run_daemon_command(output, socket_path, "motor.set",
					  params);
	}

	if (motor_off->parsed()) {
		picojson::object params;
		if (motor_off_port_option->count() > 0) {
			params["port"] = json_int(motor_off_port);
		}
		return run_daemon_command(output, socket_path, "motor.off",
					  params);
	}

	if (motor_get->parsed()) {
		picojson::object params;
		if (motor_get_port_option->count() > 0) {
			params["port"] = json_int(motor_get_port);
		}
		return run_daemon_command(output, socket_path, "motor.get",
					  params);
	}

	if (motor_clear->parsed()) {
		picojson::object params;
		params["port"] = json_int(motor_clear_port);
		return run_daemon_command(output, socket_path, "motor.clear",
					  params);
	}

	if (servo_get_enabled->parsed()) {
		picojson::object params;
		if (servo_get_enabled_port_option->count() > 0) {
			params["port"] = json_int(servo_get_enabled_port);
		}
		return run_daemon_command(output, socket_path,
					  "servo.get_enabled", params);
	}

	if (servo_get->parsed()) {
		picojson::object params;
		if (servo_get_position_port_option->count() > 0) {
			params["port"] = json_int(servo_get_position_port);
		}
		return run_daemon_command(output, socket_path, "servo.get",
					  params);
	}

	if (servo_set_enabled->parsed()) {
		picojson::object params;
		params["port"] = json_int(servo_set_enabled_port);
		params["enabled"] = json_int(servo_set_enabled_value);
		return run_daemon_command(output, socket_path,
					  "servo.set_enabled", params);
	}

	if (servo_set->parsed()) {
		picojson::object params;
		params["port"] = json_int(servo_set_position_port);
		params["position"] = json_int(servo_set_position_value);
		return run_daemon_command(output, socket_path, "servo.set",
					  params);
	}

	if (accel->parsed()) {
		picojson::object params;
		if (!accel_axis.empty()) {
			params["axis"] = picojson::value(accel_axis);
		}
		return run_daemon_command(output, socket_path, "accel", params);
	}

	if (gyro->parsed()) {
		picojson::object params;
		if (!gyro_axis.empty()) {
			params["axis"] = picojson::value(gyro_axis);
		}
		return run_daemon_command(output, socket_path, "gyro", params);
	}

	if (magneto->parsed()) {
		picojson::object params;
		if (!magneto_axis.empty()) {
			params["axis"] = picojson::value(magneto_axis);
		}
		return run_daemon_command(output, socket_path, "magneto",
					  params);
	}

	return emit_error(output, "unknown_command", "unknown command");
}
