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

bool is_axis(const std::string &axis)
{
	return axis == "x" || axis == "y" || axis == "z";
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

std::string make_json_line(const picojson::object &object)
{
	return picojson::value(object).serialize() + "\n";
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

bool daemon_socket_has_listener(const std::string &socket_path)
{
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

std::string default_socket_path()
{
	if (const char *env_runtime_dir = std::getenv("XDG_RUNTIME_DIR")) {
		return std::string(env_runtime_dir) + "/botcli.sock";
	}
	return "/tmp/botcli.sock";
}

void print_usage(std::ostream &out)
{
	out << "Usage: botcli [--socket <path>] [--motor-timeout-ms <ms>]\n";
}

bool parse_positive_int(const std::string &input, int &value)
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

} // namespace

int main(int argc, char **argv)
{
	std::string socket_path = default_socket_path();
	int motor_timeout_ms = 0;

	for (int i = 1; i < argc; ++i) {
		const std::string arg = argv[i];

		if (arg == "--help" || arg == "-h") {
			print_usage(std::cout);
			return 0;
		}

		if (arg == "--socket") {
			if (i + 1 >= argc) {
				std::cerr << "error: --socket requires a path\n";
				return 1;
			}
			socket_path = argv[++i];
			continue;
		}

		constexpr const char *socket_prefix = "--socket=";
		if (arg.compare(0, 9, socket_prefix) == 0) {
			socket_path = arg.substr(9);
			if (socket_path.empty()) {
				std::cerr << "error: --socket requires a path\n";
				return 1;
			}
			continue;
		}

		if (arg == "--motor-timeout-ms") {
			if (i + 1 >= argc) {
				std::cerr
					<< "error: --motor-timeout-ms requires a value\n";
				return 1;
			}
			if (!parse_positive_int(argv[++i], motor_timeout_ms)) {
				std::cerr << "error: motor timeout must be a "
					     "positive integer\n";
				return 1;
			}
			continue;
		}

		constexpr const char *timeout_prefix = "--motor-timeout-ms=";
		if (arg.compare(0, 20, timeout_prefix) == 0) {
			if (!parse_positive_int(arg.substr(20),
						motor_timeout_ms)) {
				std::cerr << "error: motor timeout must be a "
					     "positive integer\n";
				return 1;
			}
			continue;
		}

		std::cerr << "error: unknown argument: " << arg << '\n';
		return 1;
	}

	constexpr auto SOCK_LEN = sizeof(sockaddr_un::sun_path) - 1;
	if (socket_path.size() >= SOCK_LEN) {
		std::cerr << "error: socket path is too long\n";
		return 1;
	}

	return run_daemon(socket_path, motor_timeout_ms);
}
