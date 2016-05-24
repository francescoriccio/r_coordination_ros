#ifndef _NETWORK_HXX_
#define _NETWORK_HXX_

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>

namespace GNetwork{

struct Address{
	struct sockaddr_in addr;
	Address();
	inline uint32_t getHost() const     {return ntohl(addr.sin_addr.s_addr);}
	inline void     setHost(uint32_t h) {addr.sin_addr.s_addr=htonl(h);}
	inline uint16_t getPort() const     {return ntohs(addr.sin_port);}
	inline void     setPort(uint16_t p) {addr.sin_port=htons(p);}
	bool operator == (const Address& a);
	static Address getAddressByName(const char* hostname, uint16_t port);
	static Address getAddressByInterface(const char* ifname, uint16_t port);
	static Address getServerAddress(uint16_t port);
};

struct AddressComparator{
	inline bool operator () (const Address& a1, const Address& a2){
		return memcmp(&a1.addr, &a2.addr, sizeof(struct sockaddr_in))<0;
	}
};

struct Socket{
	const Address getAddress() const ;
	Socket();
	Socket(int style);
	~Socket();
	int bind(const Address& address);
	int close();
	int fd;
	inline bool isValid() const {return fd>0;}
};


struct UDPSocket: public Socket{
	UDPSocket();
	int sendTo(const Address& address, const char* msg, unsigned int size);
	int recvFrom(Address& address, char* msg, unsigned int size, int wait=-1);
};


struct TCPSocket: public Socket{
	TCPSocket();
	TCPSocket(unsigned int, const Address& remoteAddress);
	int listen(int backlog);
	TCPSocket* accept();
	int connect(const Address& ra);
	int read(char* buf, unsigned int size,  int wait=-1);
	int write(const char* buf, unsigned int size);
	Address remoteAddress;
	bool remoteAddressValid;
};

};

#endif
