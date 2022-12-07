#include <stdio.h>
#include <Ice/Ice.h>
#include "AndorI.h"
 
using namespace std;
using namespace AndorNetwork;
 
int main(int argc, char* argv[]) {
    int status = 0;

    Ice::CommunicatorPtr ic;
    try {
        ic = Ice::initialize(argc, argv);
        Ice::ObjectAdapterPtr adapter = ic->createObjectAdapterWithEndpoints("AndorNetworkAdapter", "default -p 10000");
        Ice::ObjectPtr object = new AndorI;
        adapter->add(object, ic->stringToIdentity("AndorNetwork"));
        adapter->activate();
        ic->waitForShutdown();
    } catch (const Ice::Exception& e) {
        cerr << e << endl;
        status = 1;
    } catch (const char* msg) {
        cerr << msg << endl;
        status = 1;
    }
    if (ic) {
        try {
            ic->destroy();
        } catch (const Ice::Exception& e) {
            cerr << e << endl;
            status = 1;
        }
    }
    return status;
}
