#ifndef NETWORKEDMODULE_H
#define NETWORKEDMODULE_H

#include <string>
#include <map>
#include <list>
#include <sys/time.h>

#include "network.h"
#define NETWORKED_MODULE_RECEIVE_BUFFER_SIZE 8196

namespace GNetwork{

struct PeerInfo{
	std::string name;
	struct timeval lastPingTime;
};


typedef std::map<Address, PeerInfo, AddressComparator> PeerInfoMap;

struct NetworkedModule{
	PeerInfoMap peers;
	virtual int parseMessage(unsigned int size, const char* buf);
	virtual int sendToAll(unsigned int size, const char* buf);
	virtual ~NetworkedModule();
	
	Address myAddress;
	UDPSocket socket;
	int bind(const Address& address);
};


struct NetworkedClient : public NetworkedModule{
	virtual int receive(int timeout);
	virtual void elapsedTimeoutReceived();
};


struct NetworkedSupplier : public NetworkedModule{
	NetworkedSupplier();
	virtual ~NetworkedSupplier();
	virtual void dispatch(unsigned int size, const char* packet);
	
	void  start();
	void stop();
	virtual int receive(int timeout);
	virtual int parseSupplierMessage(unsigned int size, const char* buf);
	static void* listenerThread(NetworkedSupplier* );
	void lockListener();
	void unlockListener();
	unsigned int suppressPending();
	
	bool running;
	pthread_t listenerThreadHandle;
	pthread_mutex_t listenerMutex;
	unsigned int clientSuppressionTime;
};

};

#endif
