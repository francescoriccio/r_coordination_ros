#include "CmdServer.h"
// #include "../common.h"

#ifndef WIN32
#define Sleep(n) usleep((n)*1000);
#endif

#include <iostream>

CmdServer::CmdServer()
{
	strcpy(name,"Command Server");
}

CmdServer::~CmdServer()
{
}

void CmdServer::setParseFn(FCmdParse f)
{
	parsefn = f;
}


TcpConnection *CmdServer::newConnection()
{
	CmdConnection *c = new CmdConnection();
	c->setParseFn(parsefn);
	return c;
}


CmdConnection::CmdConnection()
{
	parsefn=NULL;
}

CmdConnection::~CmdConnection()
{
}

void CmdConnection::setParseFn(FCmdParse f)
{
	parsefn = f;
}

void CmdConnection::_run()
{
	if (clientSocket==0) return;
		
	sprintf(bufout,"\n\rCommand server\n\r> "); send(bufout);
	
	while (running) {
	
		// receive from socket
		int r=getline(bufin,CMDBUFLEN);
		if (r>0) {
			strcpy(bufout,"Command unknown.");
			if (parsefn) {
                parsefn(bufin,bufout, host,port);
			}
			if (bufout[0]!='\0') {
				send(bufout);
			}
		}
		else running = false;
		Sleep(100);
	}
	printf("CmdServer: Connection closed by client\n");
	clientSocket=0;
}


