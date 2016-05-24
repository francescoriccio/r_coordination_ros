#include <iostream>
#include "networkedmodule.h"

using namespace std;
struct MySimpleClient: public GNetwork::NetworkedClient{
	virtual int parseMessage(unsigned int size, const char* buf);
	virtual void elapsedTimeoutReceived();
};

int MySimpleClient::parseMessage(unsigned int size, const char* buf){
	std::cout << "received " << size << " bytes" << endl;
	return size;
}
void MySimpleClient::elapsedTimeoutReceived(){
	std::cout << "timeout elapsed" << endl;
	NetworkedClient::elapsedTimeoutReceived();
}

int main(int c, const char* const * args){
	using namespace GNetwork;
	MySimpleClient client;
	PeerInfo pinfo;
	client.peers.insert(make_pair(Address::getAddressByName("localhost", 9106), pinfo));
	if (client.bind(Address::getAddressByName("localhost", 0))==-1){
		cerr << "cannot bind client socket " << endl;	
		return 0;
	}
	while(1){
		client.sendToAll(5,"hello");
		usleep(1000000);
//		client.receive(1000);
	}
}

