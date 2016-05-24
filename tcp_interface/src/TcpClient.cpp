#include <cstring>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <sstream>

#include "TcpClient.h"


using namespace std;

TcpClient::TcpClient(std::string n) : name(n)
{
#ifdef WIN32
	// Note: the following routine is required for windows
	//		 but WILL NOT WORK on linux computers
	WORD version;
	WSADATA data;
	version = MAKEWORD(2, 2);
	if(WSAStartup(version, &data) != 0)
	{
		std::cout << "WSAStartup error." << std::endl;
	}
	// End of windows-specific code.
#endif
	connected=false;
}


TcpClient::~TcpClient()
{
	close();
}

bool TcpClient::connect(string host, int port)
{
    return connect(host.c_str(),port);
}

bool TcpClient::connect(const char* host, int port)
{
  
	this->host = string(host); this->port = port;
	
	connected = false;
	printf("Connecting to %s:%d ... ",host,port); fflush(stdout);
	     //set up the socket file descriptor

  	/* get a datagram socket */
#ifdef WIN32
	sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
#else
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
#endif
	if (sockfd<0) {
		printf("errno=%d sockfd=%d \n",errno, sockfd);
		// perror("socket() failed");
		return false;
	}

  /* assign our destination address */
	memset ( &serv_addr, 0, sizeof(serv_addr) );
	serv_addr.sin_family	 =	AF_INET;
	serv_addr.sin_addr.s_addr  =	inet_addr(host);
	serv_addr.sin_port	 =	htons(port);

  /* connection request */
	if ( ::connect(sockfd, (struct sockaddr*) &serv_addr, sizeof(serv_addr))<0)  {
		printf("errno=%d sockfd=%d \n",errno, sockfd);
		// perror("connect() failed");
		return false;
	}

	printf("OK\n"); fflush(stdout);

	 connected = true;
	 return true;
}

string TcpClient::getHostPort_string()
{
    stringstream ss;
    ss << host << ":" << port;
    return ss.str();
}
string TcpClient::getHost_string()
{
    stringstream ss;
    ss << host;
    return ss.str();
}

string TcpClient::getName_string()
{
    return name;
}


void TcpClient::close()
{
  if (connected) {
     //Close the socket
#ifdef WIN32
	::closesocket(sockfd);
#else
	::close(sockfd);
#endif
  }
  connected = false;
}

bool TcpClient::send(const void* buffer, int len)
{
 	if (!connected)
		return false;
	//write to the server
	
	int send_flags=0;
#ifdef LINUX
	send_flags = MSG_NOSIGNAL ;
#endif
	::send(sockfd, (const char*)buffer, len, send_flags);
/*
#ifdef WIN32
	::send(sockfd, (const char*)buffer, len, 0);
#else
	write(sockfd, (const char*)buffer, len);
#endif
*/
	// printf("*"); fflush(stdout);
	return true;
}

bool TcpClient::send(const char* buffer)
{
	return send(buffer, strlen(buffer));
}

int TcpClient::receive(char *buffer, int len)
{
	if (!connected)
    	return -1;

	int r;
#ifdef WIN32
	r = ::recv(sockfd, (char *)buffer, len,0);
#else
	r = read(sockfd, (char *)buffer, len);
#endif

	// if (r<0) printf("receive error: %d\n",errno);

	// if (r>0) printf("TcpClient: Received %d bytes.\n",r);
	return r;
}
