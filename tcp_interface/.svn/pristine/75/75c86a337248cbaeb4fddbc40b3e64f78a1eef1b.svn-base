#include <limits.h>
#include <iostream>
#include "networkedmodule.h"
namespace GNetwork{


NetworkedModule::~NetworkedModule(){
}

int NetworkedModule::bind(const Address& address){
	myAddress=address;
	return socket.bind(address);
}

int NetworkedModule::parseMessage(unsigned int size, const char* buf){
	return 0;
}

int NetworkedModule::sendToAll(unsigned int size, const char* buf){
	int retVal=0;
	for (PeerInfoMap::const_iterator it=peers.begin(); it!=peers.end(); it++){
		retVal+=socket.sendTo(it->first, buf, size);
        // usleep(200000);  // 200 ms
		//std::cout <<"retVal" <<retVal << std::endl;
	}
	return retVal;
}

int NetworkedClient::receive(int timeout){
	char buf[NETWORKED_MODULE_RECEIVE_BUFFER_SIZE];
	Address remote;
	int size=socket.recvFrom(remote, buf, NETWORKED_MODULE_RECEIVE_BUFFER_SIZE, timeout);
//	std::cout << __PRETTY_FUNCTION__ << "begin" << std::endl;
	if (remote.getHost()==0xffffffff){
//		std::cout << __PRETTY_FUNCTION__ << " broadcast HOST" << std::endl;
		elapsedTimeoutReceived();
		return 0;
	} else {
		PeerInfoMap::iterator peer=peers.find(remote);
		if (peer==peers.end()){
			PeerInfo pinfo;
			peers.insert(std::make_pair(remote,pinfo));
			peer=peers.find(remote);
			std::cout << "registering supplier HOST= " <<remote.getHost() << " port=" << remote.getPort() <<std::endl;
		}
		gettimeofday(&peer->second.lastPingTime, 0);
		socket.sendTo(remote,"",0);
//		std::cout << "Parsing" << std::endl;
		if (size)
			parseMessage(size, buf);
		return size;
	}
}
#ifdef MACOSX
void NetworkedClient::elapsedTimeoutReceived(){
    usleep(50000);
	sendToAll(0,"");
}
#else
void NetworkedClient::elapsedTimeoutReceived(){
	sendToAll(0,"");
}
#endif

NetworkedSupplier::NetworkedSupplier(){
	pthread_mutex_init(&listenerMutex,0);
	running=false;
	clientSuppressionTime=INT_MAX;
}

NetworkedSupplier::~NetworkedSupplier(){
	pthread_mutex_destroy(&listenerMutex);
}

void NetworkedSupplier::lockListener(){
	pthread_mutex_lock(&listenerMutex);
}

void NetworkedSupplier::unlockListener(){
	pthread_mutex_unlock(&listenerMutex);
}

void NetworkedSupplier::dispatch(unsigned int size, const char* packet){
	lockListener();
	sendToAll(size, packet);
	unlockListener();
}

int NetworkedSupplier::receive(int timeout){
	char buf[NETWORKED_MODULE_RECEIVE_BUFFER_SIZE];
	Address remote;
	int size=socket.recvFrom(remote, buf, NETWORKED_MODULE_RECEIVE_BUFFER_SIZE, timeout);
	if (remote.getHost()==0xffffffff)
		return 0;
	else {
		lockListener();
		PeerInfoMap::iterator peer=peers.find(remote);
		if (peer==peers.end()){
            struct PeerInfo pi;
            gettimeofday(&pi.lastPingTime,0);
            if (strstr(buf,"hello")!=0) {
                char *q = strrchr(buf,' ');
                pi.name= std::string(q+1);
            }
            else
                pi.name="NONAME";
			peers.insert(std::make_pair(remote,pi));
			std::cout << "inserting peer" << std::endl;
		} else {
			gettimeofday(&peer->second.lastPingTime, 0);
		}
		if (size)
			parseSupplierMessage(size, buf);
		unlockListener();
		return size;
	}
}

int NetworkedSupplier::parseSupplierMessage(unsigned int size, const char* buf){
	return 0;
}

unsigned int NetworkedSupplier::suppressPending(){
	lockListener();
	struct timeval tv;
	gettimeofday(&tv,0);
	std::list<PeerInfoMap::iterator> removable;
	uint alive=0;
	for (PeerInfoMap::iterator it=peers.begin(); it!=peers.end(); it++){
		uint dt= tv.tv_sec*1000+tv.tv_usec/1000-
			it->second.lastPingTime.tv_sec*1000-it->second.lastPingTime.tv_usec/1000;
		if (dt>clientSuppressionTime){
			removable.push_back(it);
		} else
			alive++;
	}
	for(std::list<PeerInfoMap::iterator>::iterator it=removable.begin(); it!=removable.end(); it++){
		peers.erase(*it);
		std::cout << "suppressing peer" << std::endl;
	}
	unlockListener();
	return alive;
}

void* NetworkedSupplier::listenerThread(NetworkedSupplier* s){
	while (s->running){
		s->receive(-1);
	}
	return 0;
}

void  NetworkedSupplier::start(){
	if (running)
		return;
	running=true;
	pthread_create(&listenerThreadHandle, 0, (void *(*)(void*)) listenerThread, (void *) this);
}

void NetworkedSupplier::stop(){
	void* rval;
	running=false;
	pthread_join(listenerThreadHandle, &rval);
}


};
