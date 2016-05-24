#include <iostream>
#include <sstream>
#include <fstream>
#include <pthread.h>
#include <stdio.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <set>
#include <string>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>



#include <tcp_interface/RCOMMessage.h>


#include "CmdServer.h"
#include "TcpClient.h"
#include "TcpClientList.h"
#include "UDPManager.h"


using namespace std;

ros::Publisher pubmsg;
ros::NodeHandle *_n, *_np;

const char *topic_message     = "/RCOMMessage";


//static int counter=0;

string config_file,robotname, robothost, network_interface;
int TCP_server_port;

//#define BUFLEN 2000
//static char bufout[BUFLEN];


//static TcpClientList clients;
TcpClientList clients;
static map<string,string> clientnames;



void getMyIpAddress()
{
    struct ifaddrs * ifAddrStruct=NULL;
    struct ifaddrs * ifa=NULL;
    void * tmpAddrPtr=NULL;

    getifaddrs(&ifAddrStruct);

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa ->ifa_addr->sa_family==AF_INET) { // check it is IP4
            // is a valid IP4 Address
            tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer);
            if (string(ifa->ifa_name)==network_interface) robothost = string(addressBuffer);
        } /*else if (ifa->ifa_addr->sa_family==AF_INET6) { // check it is IP6
        // is a valid IP6 Address
        tmpAddrPtr=&((struct sockaddr_in6 *)ifa->ifa_addr)->sin6_addr;
        char addressBuffer[INET6_ADDRSTRLEN];
        inet_ntop(AF_INET6, tmpAddrPtr, addressBuffer, INET6_ADDRSTRLEN);
        printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer);
        }*/
    }
    if (ifAddrStruct!=NULL) freeifaddrs(ifAddrStruct);
    return;
}

void deliver(const char *mess, const char *host, int port)
{


    tcp_interface::RCOMMessage msg;
    msg.header.stamp = ros::Time::now();
    stringstream ss;
    ss << host << ":" << port;


    //TcpClient *c=clients.findClientByHostPort(ss.str());
    //msg.robotsender = c->getName_string();

    string str(mess);
    string buf; // Have a buffer string
    stringstream sms(str); // Insert the string into a stream

    vector<string> tokens; // Create vector to hold our words

    while (sms >> buf)
        tokens.push_back(buf);
    if(tokens[0]=="hello"){
        if(clientnames.find(ss.str())==clientnames.end()){
            clientnames[ss.str()]=tokens[1];
        }
    }else{
        msg.robotsender=clientnames[ss.str()]+":"+ss.str();
        msg.robotreceiver = robotname;
        msg.value = string(mess[0]==' '?(mess+1):mess);
        pubmsg.publish(msg);
    }
}
/*
bool send(const char * mess)
{
    stringstream ss(mess);
    string name;
    ss >> name;
    char buf[strlen(mess)];
    ss.read(buf,strlen(mess)-1);
    size_t r = ss.gcount();
    buf[r]='\0';
    //cout << "remaining: " << buf << endl;

    sprintf(bufout,"message %s\n\r",buf[0]==' '?(buf+1):buf);
    return clients.send(name,bufout);
}

void hello(const char * mess, const char *host, int port)
{  
    stringstream ss(mess);
    string name; int sport;
    ss >> name;
    ss >> sport;

    cout << "Received hello from " << name << "|" << host << ":" << sport << " ("<< host << ":" << port << ")" << endl;

    stringstream ks; ks << host << ":" << port;
    stringstream ns; ns << name << "|" << host << ":" << sport;

    cout << "map[" << ks.str() <<"] = " << ns.str() << endl;
    clientnames[ks.str()] = ns.str();

    cout << "map value now = "<< clientnames[ks.str()] << endl;

    //clients.connect(name,host,sport);
}
*/


void RCOMMessageCallback(const tcp_interface::RCOMMessage::ConstPtr& msg)
{
    /*
    cout << "Message to send: " << endl <<
        msg->header <<
        msg->robotsender << endl <<
        msg->robotreceiver << endl <<
        msg->value << endl << endl;*/
    if(msg->robotsender==clients.robotname){
        if(clients.send(msg->robotreceiver,msg->value+"\n\r")){
            // cout<<"MESSAGGIO INVIATO------------"<<endl;
        }else{
            // cout<<"MESSAGGIO  NON   INVIATO------------"<<endl;
        }
    }
}

bool cmdparse (const char * in, char * out, const char *host, int port) 
{
    //std::cout << "*** received from " << host << ":" << port << " [" << in << "] ***" << endl;
    sprintf(out,"OK");
    deliver(in,host,port);

    strcat(out,"\n\r");
    return true;
}

int main(int argc, char** argv)
{
    int refresh_rate;

    ros::init(argc, argv, "tcp_interface");
    ros::NodeHandle n, np("~");
    _n = &n; _np = &np;
    config_file="";


    // param("refresh_rate", refresh_rate, 10);

    if(!np.getParam("refresh_rate", refresh_rate))
    {
        refresh_rate=10; // Hz
    }
    if(!np.getParam("server_port", TCP_server_port))
    {
        TCP_server_port=9000;
    }

    np.param("TCP_server_port", TCP_server_port,9000);
    np.param("UDP_server_port", UDP_server_port, 0);
    np.param("network_interface", network_interface, string("eth0"));

    if(!np.getParam("UDP_team_min_port", UDPMinPort))
    {
        UDPMinPort=0;
    }
    if(!np.getParam("UDP_team_max_port", UDPMaxPort))
    {
        UDPMaxPort=0;
    }
    if(!np.getParam("config_file", config_file))
    {
        config_file="";
    }


    string tf_prefix="NONE";
    if(!n.getParam("tf_prefix", tf_prefix)){
        if(!np.getParam("robot_name",robotname))
        { robotname =tf_prefix;}
    }else{
        robotname = tf_prefix;
    }
    std::cout << "robotname = " << robotname << endl;

    getMyIpAddress();
    clients.robotname = robotname;
    clients.host= robothost;
    clients.TCP_server_port = TCP_server_port;


    UDPStart();

    CmdServer server;
    server.setParseFn(cmdparse);
    server.open(TCP_server_port);
    server.start();
    ROS_INFO_STREAM("TCP Listening on interface "<<network_interface<<endl);
    ROS_INFO_STREAM("TCP Listening on ip "<<robothost<<endl);
    ROS_INFO_STREAM("TCP Listening on port "<<TCP_server_port<<endl);



    ros::Subscriber RCOMMessageSubscriber = n.subscribe(topic_message, 10, RCOMMessageCallback);

    const int queue_size=100;


    pubmsg = n.advertise<tcp_interface::RCOMMessage>(topic_message,queue_size);

    ros::Rate rate(refresh_rate);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}
