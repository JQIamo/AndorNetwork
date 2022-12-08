#include <stdio.h>
#include <Ice/Ice.h>
#include "AndorI.h"
 
using namespace AndorNetwork;
 
int main(int argc, char* argv[]) {
    try {
        Ice::CommunicatorHolder ich;
        auto adapter = ich->createObjectAdapterWithEndpoints("AndorNetworkAdapter", "default -p 10000");
        auto object = std::make_shared<AndorI>();
        adapter->add(object, ich->stringToIdentity("AndorNetwork"));
        adapter->activate();
        ich->waitForShutdown();
    } catch (const std::exception& e) {
        cerr << e.what() << endl;
        return 1;
    }
    return 0;
}
