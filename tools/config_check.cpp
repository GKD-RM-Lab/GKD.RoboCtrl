#include "config/runtime.hpp"
#include <iostream>

// This executable never constructs device/IO instances or runs the event loop.
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: config-check <config.yaml|config.json> [...]\n";
        return 2;
    }
    int status = 0;
    for (int index = 1; index < argc; ++index) {
        const auto configuration = roboctrl::config::load_configuration(argv[index]);
        if (!configuration) {
            std::cerr << argv[index] << ": " << configuration.error() << '\n';
            status = 1;
        } else std::cout << argv[index] << ": valid\n";
    }
    return status;
}
