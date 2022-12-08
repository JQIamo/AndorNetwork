#include <iostream>
#include <sstream>
#include <string>
#include <Ice/Ice.h>
#include "AndorI.h"

#define STR(x) #x
 
using namespace AndorNetwork;

static void show_usage(std::string name)
{
    std::cerr << "ZeroC-ICE Wrapper of Andor SDK\n"
#ifdef GIT_HASH
        << "(build rev. " << STR(GIT_HASH) << ")\n"
#endif
        << "\n"
        << "Usage: " << name << " <option(s)>"
        << "Options:\n"
        << "\t-h,--help\t\tShow this help message\n"
        << "\t-b,--bind BIND\tSpecify the interface to bind to. Default: `*`, i.e. listen to all interface.\n"
        << "\t-p,--port PORT\tSpecify the port to bind to. Default: `5566`."
        << std::endl;
}
 
int main(int argc, char* argv[]) {
    std::string host = "*";
    int port = 5566;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if(arg == "-h" || arg == "--help") {
            show_usage(argv[0]);
            return 0;
        } else if(arg == "-b" || arg == "--bind") {
            if (i + 1 < argc) {
                host = argv[++i];
            } else {
                std::cerr << "-b/--bind option requires one argument." <<std::endl;
                return 1;
            }
        } else if(arg == "-p" || arg == "--port") {
            if (i + 1 < argc) {
                port = std::stoi(argv[++i]);
            } else {
                std::cerr << "-p/--port option requires one argument." <<std::endl;
                return 1;
            }
        }
    }

    std::ostringstream endpoint;
    endpoint << "tcp -h " << host << " -p " << port << " -z";

    try {
        Ice::CommunicatorHolder ich(argc, argv);
        std::cerr << "Bind to " << host << ":" << port << "." << std::endl;
        auto adapter = ich->createObjectAdapterWithEndpoints("AndorNetworkAdapter", endpoint.str());
        auto object = std::make_shared<AndorI>();
        adapter->add(object, Ice::stringToIdentity("AndorNetwork"));
        adapter->activate();
        ich->waitForShutdown();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
