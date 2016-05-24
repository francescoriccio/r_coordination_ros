#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include "network.h"
#include <netdb.h>
#include <iostream>
#include <errno.h>

namespace GNetwork{

Address::Address(){
	addr.sin_family = AF_INET;
	addr.sin_port = 0;
	addr.sin_addr.s_addr = INADDR_ANY;
	bzero( &addr.sin_zero, 8 );
}

bool Address::operator == (const Address& a){
	return !memcmp(&addr, &a.addr, sizeof(struct sockaddr_in));
}

Address Address::getAddressByName(const char* hostname, uint16_t port){
	struct hostent* host =gethostbyname(hostname);
	if (!host)
		throw "unexisting address";
	Address a;
	if( host ) {
		memcpy(&((struct sockaddr_in&)a.addr).sin_addr.s_addr, host->h_addr_list[0], host->h_length);
	}
	a.addr.sin_family=AF_INET;
	a.setPort(port);
	return a;
}

Address Address::getAddressByInterface(const char* ifname, uint16_t port){
	struct ifreq ifr;
	int fd;
	Address a;
	fd = ::socket(AF_INET,SOCK_DGRAM, 0);
	if (fd <= 0){
		std::cout <<  "cant open socket" << std::endl;
		throw "cant open socket";
	}
	strcpy(ifr.ifr_name, ifname);
	if (ioctl(fd, SIOCGIFADDR, &ifr) != 0){
		::shutdown(fd,2);
		close(fd);
		std::cout <<  "no address bound to interface" << std::endl;
		throw "no address bound to interface";
	}
	struct sockaddr_in* addr=(struct sockaddr_in*)&(ifr.ifr_ifru.ifru_addr);
	memcpy(&a.addr, addr, sizeof(struct sockaddr_in));
	close(fd);
	a.setPort(port);
	return a;
}

Address Address::getServerAddress(uint16_t port){
	Address a;
	struct sockaddr_in & addr =(struct sockaddr_in &)a.addr;
	addr.sin_family=AF_INET;
	addr.sin_port=htons(port);
	addr.sin_addr.s_addr=INADDR_ANY;
	memset(&addr.sin_zero,0,8);
	return a;
}

Socket::Socket(){
	fd=-1;
}

Socket::Socket(int style){
	fd=::socket(PF_INET,style,0);
	if (fd<0)
		throw "cant open socket";
}

Socket::~Socket(){
	close();
}

int Socket::close(){
	if (fd>0){
		::shutdown(fd,2);
		::close(fd);
		return 0;
	}
	fd=-1;
	return -1;
};

const Address Socket::getAddress() const {
	struct sockaddr addr;
	uint32_t size;
	Address a;
	if (getsockname(fd,&addr,&size)){
		memcpy(&a.addr,&addr,sizeof(struct sockaddr_in));
	} else {
		throw "socket not bound";
	}
	return a;
}

int Socket::bind(const Address& address){
	int r = ::bind(fd,(struct sockaddr*) (&address.addr), sizeof(struct sockaddr_in));
	return r;
}


UDPSocket::UDPSocket(): Socket(SOCK_DGRAM){
}

int UDPSocket::sendTo(const Address& address, const char* msg, unsigned int size){
	if (!isValid())
		return -1;
	int v=::sendto(fd,msg,size, 0, (struct sockaddr*) (&address.addr), sizeof(struct sockaddr_in));
	if (v<0) {
		std::cerr << "invalid sendto sok=" << fd << std::endl;
		std::cerr << strerror(errno) << std::endl;
        }
	return v;
}

int UDPSocket::recvFrom(Address& address, char* msg, unsigned int size, int wait){
	//int flags=0;
	unsigned int l=sizeof(struct sockaddr_in);
	int r;
	address.setHost(0xffffffff);
	if (wait!=-1){
		fd_set readSet;
		FD_ZERO(&readSet);
		FD_SET(fd, &readSet);
		struct timeval tv;
		tv.tv_sec=wait/1000;
		tv.tv_usec=(1000*wait)%1000000;
		//		std::cout << fd << " " << tv.tv_sec << " " << tv.tv_usec;
		r=select(fd+1, &readSet, 0, 0, &tv);
		if (r == -1){
			//std::cerr << "select()" << std::endl;
		} else if (r && FD_ISSET(fd,&readSet)){
			//std::cerr << "DATA SELECT=" << r << std::endl;
			r= recvfrom (fd, msg, size, 0, (struct sockaddr*) (&address.addr), &l);
			//std::cerr << "FD=" << fd << " BYTES=" << r << std::endl;
			if (r==-1)
 				std::cerr << strerror(errno) << " fd=" << fd << std::endl;
		} else{
			//std::cerr << "NO DATA r=" << r << std::endl;
		}
		return r;
	} else {
		r=recvfrom (fd, msg, size, 0, (struct sockaddr*) (&address.addr), &l);
		if (r==-1)
			std::cerr << strerror(errno) << " fd=" << fd << std::endl;
		return r;
	}
}


TCPSocket::TCPSocket(): Socket(SOCK_STREAM){
	remoteAddressValid=false;
}

TCPSocket::TCPSocket(unsigned int s, const Address& ra): Socket(){
	remoteAddress=ra;
	fd=s;
	remoteAddressValid=true;
}

TCPSocket* TCPSocket::accept(){
	struct sockaddr addr;
	socklen_t asize;
	int v=::accept(fd,&addr,&asize);
	if (v<=0)
		return 0;
	Address a;
	memcpy(&a.addr,&addr,sizeof(struct sockaddr_in));
	return new TCPSocket(v,a);
}

int TCPSocket::listen(int backlog){
	return ::listen(fd, backlog);
}

int TCPSocket::connect(const Address& ra){
	int v=::connect(fd,(sockaddr*)(&ra.addr),sizeof(struct sockaddr_in));
	if (v)
		return v;
	remoteAddress=ra;
	remoteAddressValid=true;
	return v;
}

int TCPSocket::read(char* buf, unsigned int size,  int wait){
	if (wait!=-1){
		fd_set readSet;
		FD_ZERO(&readSet);
		FD_SET(fd, &readSet);
		struct timeval tv;
		tv.tv_sec=0;
		tv.tv_usec=1000*wait;
		int r=select(1, &readSet, 0, 0, &tv);
		FD_ZERO(&readSet);
		if (r)
			return ::recv(fd, buf, size, 0);
		else
			return -1;
	}
	return ::read(fd, buf, size);
}

int TCPSocket::write(const char* buf, unsigned int size){
	return ::send(fd, buf, size, 0);
}

};

