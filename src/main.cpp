#include <iostream>
#include <cerrno>
#include <cstring>
#include <string>
#include <unistd.h>

#include "CLI11.hpp"
#include "picojson.h"

extern "C" {
#include <kipr/button/button.h>
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

int emit_result(const OutputContext &output,
                const std::string &command,
                const std::string &plain_value,
                const picojson::object &result)
{
    if (!output.json) {
        std::cout << plain_value << '\n';
        return 0;
    }

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
    picojson::object result;
    result["value"] = json_int(value);
    return emit_result(output, command, std::to_string(value), result);
}

std::string parse_error_code(const CLI::ParseError &err)
{
    if (dynamic_cast<const CLI::RequiredError *>(&err) != nullptr ||
        dynamic_cast<const CLI::ArgumentMismatch *>(&err) != nullptr) {
        return "missing_argument";
    }

    if (dynamic_cast<const CLI::ExtrasError *>(&err) != nullptr) {
        return "unknown_command";
    }

    return "parse_error";
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

    auto *side_btn = app.add_subcommand("side_btn", "Read the side button");

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
            error_output, parse_error_code(err), err.what(), err.get_exit_code());
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

    return emit_error(output, "unknown_command", "unknown command");
}
