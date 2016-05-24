#ifndef TCPCLIENTLIST_H
#define TCPCLIENTLIST_H

#include "TcpClient.h"
#include <vector>

using std::string;

#define BUFINLEN 2000

class TcpClientList
{
public:

      std::vector<TcpClient*> clients;
      string robotname;
      string host;
      int TCP_server_port;

      char buffin[BUFINLEN];

      TcpClientList();

      ~TcpClientList();

      void addClient(string clientstr);

      TcpClient *findClientByHostPort(string hostport);

      TcpClient *findClientByName(string name);

      string to_string(string host, int port);

      bool connect(string name, string host, int port);
      bool suppressPending();

      bool send(string name, string buffer);

      bool send(string name, const char* buffer);

      bool send(string name, string host, int port, const char* buffer);

      bool send(TcpClient *c, const char* buffer);

      bool sendToAll( const char* buffer);
};

#endif // TCPCLIENTLIST_H
