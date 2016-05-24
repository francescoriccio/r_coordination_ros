#include <iostream>
#include "TcpClientList.h"

#include <sstream>
#include <boost/algorithm/string.hpp>

using namespace std;



TcpClientList::TcpClientList() {}

TcpClientList::~TcpClientList() {
    for (size_t i=0; i<clients.size(); i++) {
        clients[i]->close();
        delete clients[i];
    }
}

TcpClient *TcpClientList::findClientByHostPort(string hostport) {
    TcpClient *r =NULL;
    for (size_t i=0; i<clients.size(); i++)  {
        if (clients[i]->getHostPort_string()==hostport)
    return clients[i];
    }
    return r;
}

TcpClient *TcpClientList::findClientByName(string name) {
    TcpClient *r =NULL;
    for (size_t i=0; i<clients.size(); i++)  {
        if (clients[i]->getName_string()==name)
    return clients[i];
    }
    return r;
}

string TcpClientList::to_string(string host, int port)
{
    stringstream ss;
    ss << host << ":" << port;
    return ss.str();
}
bool TcpClientList::suppressPending() {
    for (std::vector<TcpClient*>::iterator it=clients.begin(); it!=clients.end(); ++it)  {
        //cout<<"-------------"<< (*it)->getName_string()<<" "<<  (*it)->sockfd<<" "<<(*it)->connected<<endl;
        if ((*it)->connected){
        (*it)->close();}
    }
    return true;

}

bool TcpClientList::connect(string name, string host, int port) {
    bool b = true;

    TcpClient *c = findClientByHostPort(to_string(host,port));
    if (c==NULL) {

      c = new TcpClient(name);
      clients.push_back(c);
    }
    if (!c->connected){

      b = c->connect(host,port);
      usleep(100);
      stringstream ss;

      ss << "hello " << robotname << " " << TCP_server_port << "\n\r";
      c->send(ss.str().c_str());
    }
   /* if (b && c->connected) {
        stringstream ss;

        ss << "hello " << robotname << " " << TCP_server_port << "\n\r";
        c->send(ss.str().c_str());
        usleep(100);
        c->receive(buffin,BUFINLEN-1);
    }*/
    return b;
}


bool TcpClientList::send(string name, string buffer) {
    return send(name,buffer.c_str());
}

bool TcpClientList::send(string name, const char* buffer) {
    if (name=="All"||name=="all"){
        return sendToAll(buffer);
    }else{
    TcpClient *c = findClientByName(name);

    if (c==NULL)  return false;
    else send(c,buffer);
    }
    return true;
}


bool TcpClientList::send(string name, string host, int port, const char* buffer) {
    bool b = true;
    TcpClient *c = findClientByHostPort(to_string(host,port));
    if (c==NULL) {
        b = connect(name,host,port);
        c = findClientByHostPort(to_string(host,port));
        if (c==NULL) b = false;
    }
    if (b && !c->connected) {
        b = connect(name,host,port);
    }
    b = b && send(c,buffer);
    //std::cout <<"---------------send" <<buffer <<b<<endl;
    return b;
}


bool TcpClientList::send(TcpClient *c, const char* buffer) {
    if (c==NULL) return false;
    if (!c->connected) return false;
    c->send(buffer);
    //std::cout << "send " << buffer << endl;
    usleep(100);
   // c->receive(buffin,BUFINLEN-1);  cout << buffin << endl;
    return true;
}

bool TcpClientList::sendToAll( const char* buffer) {
    for (size_t i=0; i<clients.size(); i++)  {
        //std::cout <<"-------------------client ip: "<<clients[i]->getName_string().c_str()<<"my ip"<<robotname.c_str()<<endl;
        if (strcmp(clients[i]->getName_string().c_str(),robotname.c_str())!=0) {
            send(clients[i],buffer); // usleep(250000);  // 250 ms
        }
    }
    return true;
}
