#ifndef _CMDSERVER_H_
#define _CMDSERVER_H_


#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include "TcpServer.h"

typedef bool (*FCmdParse) (const char *, char *, const char *, int) ;

#define CMDBUFLEN 20000

class CmdConnection : public TcpConnection
{
private:
	char bufin[CMDBUFLEN], bufout[CMDBUFLEN];
	FCmdParse parsefn;
public:
	CmdConnection();
	virtual ~CmdConnection();
	virtual void _run();
	virtual void setParseFn(FCmdParse f);
};

class CmdServer : public TcpServer
{
private:
	FCmdParse parsefn;
public:
	CmdServer();
	virtual ~CmdServer();
	virtual TcpConnection *newConnection();
	virtual void setParseFn(FCmdParse f);
};

#endif /*CMDSERVER_H_*/
