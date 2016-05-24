#include "string.h"
#include "binarymessage.h"
#include "gformatters.h"

namespace GNetwork{
BinaryMessage::BinaryMessage(const char* id, int sec, int usec){
	strncpy(identifier, id, BINARYMESSAGE_IDENTIFIER_SIZE);
	timestamp.tv_sec=sec;
	timestamp.tv_usec=usec;
}

char* BinaryMessage::toNetworkByteArray(char* b) const{
	memcpy(b,identifier,BINARYMESSAGE_IDENTIFIER_SIZE);
	b+=BINARYMESSAGE_IDENTIFIER_SIZE;
	b=GNetwork::toNetworkByteArray(timestamp.tv_sec,b);
	b=GNetwork::toNetworkByteArray(timestamp.tv_usec,b);
	return b;
}

const char* BinaryMessage::fromNetworkByteArray(const char * b){
	memcpy(identifier,b,BINARYMESSAGE_IDENTIFIER_SIZE);
	b+=BINARYMESSAGE_IDENTIFIER_SIZE;
	b=GNetwork::fromNetworkByteArray(timestamp.tv_sec,b);
	b=GNetwork::fromNetworkByteArray(timestamp.tv_usec,b);
	return b;
}

BinaryMessage::~BinaryMessage(){
}

};

