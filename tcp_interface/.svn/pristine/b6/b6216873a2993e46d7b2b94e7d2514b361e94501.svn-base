#ifndef BINARYMESSAGE_H
#define BINARYMESSAGE_H

#include <sys/time.h>

#define BINARYMESSAGE_IDENTIFIER_SIZE 32
namespace GNetwork{

struct BinaryMessage {
	BinaryMessage(const char* identifier, int sec=0, int usec=0);
	char identifier[BINARYMESSAGE_IDENTIFIER_SIZE];
	struct timeval timestamp;
	inline double getDoubleTime() const {
		return timestamp.tv_sec*1000.+0.001*timestamp.tv_usec;
	}
	inline double getDoubleTimeAtHour() const {
		int x=(timestamp.tv_sec/3600)*3600;
		return x*1000.;
	}
	virtual char* toNetworkByteArray(char* ) const;
	virtual const char* fromNetworkByteArray(const char *);
	virtual ~BinaryMessage();
};


};

#endif
