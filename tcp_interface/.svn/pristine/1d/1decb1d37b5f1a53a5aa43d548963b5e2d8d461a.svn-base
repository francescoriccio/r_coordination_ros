#include <iostream>
#include "networkedmodule.h"

using namespace std;
using namespace GNetwork;

struct MyNetworkedSupplier: public NetworkedSupplier{
	virtual int parseSupplierMessage(unsigned int size, const char* buf);
};

int MyNetworkedSupplier::parseSupplierMessage(unsigned int size, const char* buf){
	cout << "received " << size << " bytes" << endl;
	cout << "peers# " << peers.size() << endl;
	return size;
}

int main(int argc, char** argv){
	MyNetworkedSupplier supplier;
	supplier.clientSuppressionTime=10000;
	if (supplier.bind(Address::getServerAddress(1976))==-1){
		cout << "cannot bind a socket" << endl;
		return 0;
	}
	supplier.start();
	while(1){
		supplier.dispatch(10,"123456789");
		supplier.suppressPending();
		sleep(1);
		cout << "Alive" << endl;
	}
}

