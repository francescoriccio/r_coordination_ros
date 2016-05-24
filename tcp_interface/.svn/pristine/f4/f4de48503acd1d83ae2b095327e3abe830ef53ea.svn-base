#include <string.h>
#include <cmath>
#include <iostream>
#include <signal.h>
#include "TcpServer.h"

// please avoid following line
// using namespace std;
using std::cerr;
using std::endl;
using std::cout;
// thx

void  * tcpServer_run (void * arg);
void  * tcpConnection_run (void * arg);


TcpServer::TcpServer()
{
	for (int k=0; k<10; k++)
		connection[k] = NULL;
}

TcpServer::~TcpServer()
{
}

bool TcpServer::open(int p)
{
	listenPort = p;

#ifdef WIN32
	// Note: the following routine is required for windows
	//		 but WILL NOT WORK on linux computers
	WORD version;
	WSADATA data;
	version = MAKEWORD(2, 2);
	if(WSAStartup(version, &data) != 0)
	{
		cout << "WSAStartup error." << endl;
		return false;
	}
	// End of windows-specific code.
#endif

	// Create socket for listening for client connection requests.
	listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
#ifndef INVALID_SOCKET
#define INVALID_SOCKET -1
#endif
	if (listenSocket == INVALID_SOCKET) {
	    cerr << "cannot create listen socket" << endl;
	    // exit(1);
	    return false;
	}
    int yes=1;
    if (setsockopt(listenSocket,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(int)) == -1) {
        perror("setsockopt");
        exit(1);
    }

	// Bind listen socket to listen port.  First set various fields in
	// the serverAddress structure, then call bind().
	// htonl() and htons() convert long integers and short integers
	// (respectively) from host byte order (on x86 this is Least
	// Significant Byte first) to network byte order (Most Significant
	// Byte first).
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
	serverAddress.sin_port = htons(listenPort);

	if (bind(listenSocket,
	    (struct sockaddr *) &serverAddress,
	    sizeof(serverAddress)) < 0) {
	  cerr << "cannot bind socket" << endl;
	  return false;
	}

	// Wait for connections from clients.
	// This is a non-blocking call; i.e., it registers this program with
	// the system as expecting connections on this socket, and then
	// this thread of execution continues on.
	listen(listenSocket, 5);

	running = true;

	return true;
}



void TcpServer::_run()
{
  while (running) {
	   // cout << "TcpServer: waiting for TCP connection on port " << listenPort << " ... " << endl;

		// Accept a connection with a client that is requesting one.  The
		// accept() call is a blocking call; i.e., this thread of
		// execution stops until a connection comes in.
		// connectSocket is a new socket that the system provides,
		// separate from listenSocket.  We *could* accept more
		// connections on listenSocket, before connectSocket is closed,
		// but this program doesn't do that.
		
		socklen_t clientAddressSize = sizeof(clientAddress);
        int csocket = accept(listenSocket, (struct sockaddr *) &clientAddress,  &clientAddressSize);

		if (csocket < 0) {
		  cerr << "TcpServer: cannot accept connection " << endl;
		  break;
		}

		nclient=-1;
		for (int k=0; k<10; k++) {
			if (!connection[k] || connection[k]->clientSocket==0) {
				nclient=k;	break;
			}
		}

	    if (nclient<0) {
	    	printf("No more client available. Use lists!!!\n");
			continue;
	    }

		if (connection[nclient]) delete connection[nclient];
		connection[nclient] = newConnection();
		connection[nclient]->clientSocket = csocket;
		connection[nclient]->clientAddress = clientAddress;
		strcpy(connection[nclient]->host,inet_ntoa(connection[nclient]->clientAddress.sin_addr));
		connection[nclient]->port = ntohs(connection[nclient]->clientAddress.sin_port);


#if 1
		// Show the IP address of the client.
		// inet_ntoa() converts an IP address from binary form to the
		// standard "numbers and dots" notation.
        cout <<endl<<endl << name << ": client " << nclient << " connected from " << inet_ntoa(connection[nclient]->clientAddress.sin_addr);

		// Show the client's port number.
		// ntohs() converts a short int from network byte order (which is
		// Most Significant Byte first) to host byte order (which on x86,
		// for example, is Least Significant Byte first).
        cout << ":" << ntohs(connection[nclient]->clientAddress.sin_port) << endl<<endl;
#endif

		printf("%s: Client %d connected: %s:%d\n",name,nclient,connection[nclient]->host,connection[nclient]->port);
		connection[nclient]->start();

		nclient=-1;
		for (int k=0; k<10; k++) {
			if (!connection[k] || connection[k]->clientSocket==0) {
				nclient=k;
			}
		}
	    nclient++;

  } // while listen



}

void TcpServer::start()
{
	running = true;
	int r = pthread_create (&thr, NULL, tcpServer_run, (void *)this);
	if (r!=0) {
		printf("\e[33m Error in creating CmdServer thread [%d].\n \e[37m",r);
		perror(":");
		running = false;
	}
	printf("%s started on port %d...\n",name,listenPort);
}


void TcpServer::stop()
{
	if (running) {
	running=false;

	for (int k=0; k<10; k++) {
		if (connection[k]) {
			connection[k]->stop();
			delete connection[k];
			connection[k]=0;
		}
	}

#ifdef WIN32
	pthread_kill(thr,SIGBREAK);  // kill perch� bloccato su accept
#elif LINUX
	pthread_kill(thr,SIGKILL);
#else
#error: Unsupported platform (or missing -D<platform>)
#endif
	// pthread_join(thr,NULL);
	}
}



void TcpServer::close() {
//  if (!running) return;
     //Close the socket
#ifdef WIN32
	::closesocket(listenSocket);
#else
	::close(listenSocket);
#endif
}


int TcpServer::countConnections()
{
	int nc=0;
	for (int k=0; k<10; k++) {
		if (connection[k] && connection[k]->clientSocket>0) {
			nc++;
		}
	}
	return nc;
}

TcpConnection *TcpServer::getConnection(int k) {
	if (connection[k] && connection[k]->clientSocket>0)
		return connection[k];
	else
		return NULL;
}

TcpConnection *TcpServer::getFirstConnection() {
	int ic=-1;
	for (int k=0; k<10; k++) {
		if (connection[k] && connection[k]->clientSocket>0) {
			ic=k; break;
		}
	}
	if (ic>=0)
		return connection[ic];
	else
		return NULL;
}


TcpConnection::TcpConnection()
{}

TcpConnection::~TcpConnection()
{
	running=false;
	pthread_join(thr,NULL);
}

void TcpConnection::start()
{
	running = true;
	int r = pthread_create (&thr, NULL, tcpConnection_run, (void *)this);
	if (r!=0) {
		printf("\e[33m Error in creating CmdConnection thread [%d].\n \e[37m",r);
		perror(":");
		running = false;
	}

}

void TcpConnection::stop()
{
	running = false;
}

bool TcpConnection::send(const void* buffer, int len)
{
	int send_flags=0;
#ifdef LINUX
	send_flags = MSG_NOSIGNAL ;
#endif
	if (clientSocket>0 && ::send(clientSocket, (const char*) buffer, len, send_flags) < 0) {
		cerr << "Connection closed by client." << endl;
#ifdef _WIN32
		closesocket(clientSocket);
#elif LINUX
		close(clientSocket);
#else
#error: Unsupported platform (or missing -D<platform>)
#endif
		clientSocket = 0;
		return false;
	}
	return true;
}

bool TcpConnection::send(const char *buffer)
{
	return send(buffer, strlen(buffer) + 1);
}

int TcpConnection::getline(char *buffer, int len)
{
	memset(buffer,0,len);
	char *p = buffer; int s=0;
	while (*p!='\n' && *p!='\r') {
		int r=receive(p,len-s);
		if (r<=0) return r;
		s+=r;
		if (s==len) return s;
		for (int k=0; k<r; k++) {
			if (*p=='\n' || *p=='\r') break;
			p++;
		}
	}
	*p='\0';
	return s;
}

int TcpConnection::receive(void *buffer, int len)
{
	if (!running || clientSocket<=0)
    	return -1;

	int r;
#ifdef WIN32
	r = ::recv(clientSocket, (char *)buffer, len, 0);
#else
	r = read(clientSocket, buffer, len);
#endif

	return r;
}



void  * tcpServer_run (void * arg)
{
	TcpServer *s = (TcpServer *)arg;
	if (s)
	    s->_run();
	return NULL;
}


void  * tcpConnection_run (void * arg)
{
	TcpConnection *c = (TcpConnection *)arg;
	c->_run();
	return NULL;
}
