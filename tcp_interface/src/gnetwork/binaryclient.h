#ifndef UDPBINARYCLINET_H
#define UDPBINARYCLINET_H

#include <iostream>
#include <stdio.h>
#include <vector>
#include <string>
#include <errno.h>
#include "networkedmodule.h"
#include "binarymessage.h"

namespace GNetwork{

struct UDPBinaryClient: public NetworkedClient{
	UDPBinaryClient();
	Address serverAddress, ownAddress;
	BinaryMessage lastMessage, *lastMessage_p;
	virtual int parseMessage(unsigned int size, const char* buf);
	virtual void elapsedTimeoutReceived();
	void setServerAddress(const Address& address);
	int run(int timeout=1000);
	const BinaryMessage* getMessage();
};

};

#endif
