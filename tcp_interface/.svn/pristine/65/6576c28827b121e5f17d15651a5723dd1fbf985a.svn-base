#include "binaryclient.h"
#include <sstream>

namespace GNetwork{
using namespace std;

UDPBinaryClient::UDPBinaryClient():lastMessage(""){
	ownAddress=Address();
	serverAddress=Address::getAddressByName("localhost", 10000);
	if  (bind(ownAddress)==-1)
		throw "could not bind a soket";
	lastMessage_p=0;
}

void UDPBinaryClient::setServerAddress(const Address& address){
	serverAddress=address;
	peers.clear();
	PeerInfo sinfo;
	peers.insert(make_pair(serverAddress, sinfo));
}

int UDPBinaryClient::parseMessage(unsigned int size, const char* buf){
	if (size>0){
		if (buf[0]=='<'){
			int ctag=0;
			for (unsigned int i=0; i<size && ctag<2; i++, buf++){
				if (*buf=='>')
					ctag++;
			}
		}
		lastMessage.fromNetworkByteArray(buf);
		lastMessage_p=&lastMessage;;
	} else {
		lastMessage_p=0;
	}
//	std::cout << "InnerParsing" << size << " bytes" << endl;
	return size;
}

void UDPBinaryClient::elapsedTimeoutReceived(){
	std::cout << "timeout elapsed" << endl;
	NetworkedClient::elapsedTimeoutReceived();
}


int UDPBinaryClient::run(int timeout){
	return receive(timeout);
}


const BinaryMessage* UDPBinaryClient::getMessage(){
	return lastMessage_p;
}

};

