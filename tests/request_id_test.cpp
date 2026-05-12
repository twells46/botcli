#include "lest.hpp"
#include "validation.hpp"

#include <string>

const lest::test specification[] = {
    CASE("request IDs accept the documented characters and reject invalid lengths")
    {
        EXPECT(is_valid_request_id("req-42"));
        EXPECT(is_valid_request_id("client.alpha_1:run-2"));

        EXPECT(!is_valid_request_id(""));
        EXPECT(!is_valid_request_id("contains space"));
        EXPECT(!is_valid_request_id(std::string(129, 'a')));
    },
};

int main(int argc, char *argv[])
{
    return lest::run(specification, argc, argv);
}
