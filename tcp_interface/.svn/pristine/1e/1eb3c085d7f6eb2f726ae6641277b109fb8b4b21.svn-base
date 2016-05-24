#include <iostream>
#include <cstdio>
#include <map>
#include <fstream> 
#include <vector>
#include <sstream>
#include <cstdlib>


#include "gnetwork/networkedmodule.h"
#include "UDPManager.h"

using namespace GNetwork;
using namespace std;

struct UDPNetworkedSupplier: public NetworkedSupplier {
    virtual int parseSupplierMessage(unsigned int size, const char* buf);
};

struct UDPNetworkClient: public NetworkedClient{
    virtual int parseMessage(unsigned int size, const char* buf);
    virtual void elapsedTimeoutReceived();
};



int UDP_server_port=0;
int UDPMinPort=0, UDPMaxPort=0;

extern string config_file,robotname,robothost;
extern int TCP_server_port;

static	UDPNetworkedSupplier UDPsupplier;
static	UDPNetworkClient UDPclient;

static pthread_t udpth;
//static pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;


static int ping_delay = 10; // sec.


void doParseMessage(unsigned int size, const char* buf){
    //cout << UDP_server_port << ": received [" << size << "] : " << buf << endl;
    if (strstr(buf,"hello")!=0) {
        stringstream temp,temp2;
        string ss,name,host,TcpPort,UdpPort;
        temp << string(buf);
        getline(temp, ss, ' ');
       getline(temp, ss, ' ');
       getline(temp, ss, ' ');
       temp2 << ss;
       getline(temp2,name, ':');
       getline(temp2,host, ':');
       getline(temp2,TcpPort, ':');
       getline(temp2, UdpPort, ':');
       //cout << "---------received:"<<name << ":"<<host<< ":"<<TcpPort<< ":"<<atoi(TcpPort.c_str())<<endl;
       clients.connect(name,host,atoi(TcpPort.c_str()));
    }
}

int UDPNetworkedSupplier::parseSupplierMessage(unsigned int size, const char* buf){
    doParseMessage(size, buf);

    //cout << "peers# " << peers.size() << endl;
    PeerInfoMap::iterator it = peers.begin();
    while (it!=peers.end()) {
        PeerInfo info = it->second;
        struct timeval tv;
        gettimeofday(&tv,0);
        uint delay= tv.tv_sec*1000+tv.tv_usec/1000-
                it->second.lastPingTime.tv_sec*1000-it->second.lastPingTime.tv_usec/1000;
      //  cout << "     " << info.name<<" delay "<<delay << endl; // "  delay: " << delay << " sec   " << ((delay>2*ping_delay)?"OUT":"") << endl;
        it++;
    }

    return size;
}

int UDPNetworkClient::parseMessage(unsigned int size, const char* buf){
    doParseMessage(size, buf);
    return size;
}

void UDPNetworkClient::elapsedTimeoutReceived(){
    std::cout << "timeout elapsed" << endl;
    NetworkedClient::elapsedTimeoutReceived();
}

void* UDP_discover_run(void *)
{PeerInfo pinfo;
    // Read config file
    cout<<"config file: "<<config_file<<endl;
   if(config_file!=""){
	ifstream infile(config_file.c_str());
    string item;
    std::vector<string> elems;
		if(infile) {
		  string s="";
		  while(getline(infile, s)) {
			//cout<<"UDP peers "<<s<<endl;
			 stringstream ss;
			 ss<<s;
			 elems.clear();
			while (getline(ss, item, '\t')) {
				elems.push_back(item);
			}
			 for (int UDPClientPort=UDPMinPort; UDPClientPort<=UDPMaxPort; UDPClientPort++) {
				
                 if (UDPClientPort==UDP_server_port &&(strcmp(robothost.c_str(),elems[0].c_str())==0)) continue;
				Address remote=Address::getAddressByName(elems[0].c_str(), UDPClientPort);
				UDPclient.peers.insert(make_pair(remote, pinfo));
			}

		  }
		 // if (UDPclient.bind(Address::getAddressByName("localhost", 0))==-1){
			//cerr << "cannot bind UDP client socket" << endl;}
		}else{cerr << "error opening file"<< endl;}
		  infile.close();
   }else{
    //ifstream f("config/UDPpeers.cfg");

    //f.close();

    for (int UDPClientPort=UDPMinPort; UDPClientPort<=UDPMaxPort; UDPClientPort++) {
        if (UDPClientPort==UDP_server_port) continue;
        Address remote=Address::getAddressByName("localhost", UDPClientPort);
        UDPclient.peers.insert(make_pair(remote, pinfo));
    }
    if (UDPclient.bind(Address::getAddressByName("localhost", 0))==-1){
        cerr << "cannot bind UDP client socket" << endl;
    }
  }   
   int alive=0,old=0;
    char buf[64];
    while (true) {
        alive=UDPsupplier.suppressPending();
        if (alive<old){
         clients.suppressPending();

        }
        old=alive;
        usleep(ping_delay*1e6);
         
        sprintf(buf,"hello from %s:%s:%d:%d",robotname.c_str(),robothost.c_str(),TCP_server_port,UDP_server_port);
        //cout << "ping around..." << buf << endl;
        UDPclient.sendToAll(strlen(buf),buf);

        
    }
    return NULL;
}

// public interface

void UDPStart()
{
    if (UDP_server_port==0) return;

    // start server
    UDPsupplier.clientSuppressionTime=10000;
    if (UDPsupplier.bind(Address::getServerAddress(UDP_server_port))==-1){
        cerr << "cannot bind UDP server socket on port " << UDP_server_port << endl;
        return;
    }
    UDPsupplier.start();
    cout << "UDP server running on port " << UDP_server_port << endl;

    // start thread for discovery clients
    pthread_create( &udpth, NULL, UDP_discover_run, NULL);

    return;

}


void UDPWho() {
    PeerInfoMap::iterator it = UDPsupplier.peers.begin();
    while (it!=UDPsupplier.peers.end()) {
        PeerInfo info = it->second;
        cout << info.name << endl;
        it++;
    }
}
