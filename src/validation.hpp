#pragma once

#include <string>

inline bool is_request_id_char(char ch)
{
    return (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
           (ch >= '0' && ch <= '9') || ch == '.' || ch == '_' ||
           ch == ':' || ch == '-';
}

inline bool is_valid_request_id(const std::string &id)
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
