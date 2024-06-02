

#ifndef HAND_H
#define HAND_H

#include <cstring>
#include <iostream>
#include <fstream>
#include <stack>
#include <utility> // for std::pair
#include "../main.h"
#include "crc16.h"
#define MAXBUF 100
using namespace std;

#include "Poco/Net/Net.h"
#include "Poco/Net/StreamSocket.h"
#include "Poco/Net/SocketAddress.h"
#include "Poco/Dynamic/Var.h"
#include "Poco/Exception.h"
#include "Poco/Timer.h"
#include "Poco/Stopwatch.h"
#include "Poco/Thread.h"
#include "Poco/DateTime.h"
#include "Poco/Net/ServerSocket.h"
#include "Poco/Net/SocketAddress.h"
#include "Poco/Timespan.h"
#include "Poco/NumericString.h"

using namespace Poco;
using namespace Poco::Dynamic;
using Poco::Net::SocketAddress;
using Poco::Net::StreamSocket;
using Poco::Net::Socket;
using Poco::Timer;
using Poco::TimerCallback;
using Poco::Thread;
using Poco::Stopwatch;

enum {
    GETQ = 1,
    SETTORQ = 2,
};

class Hand {
public:
    Hand();
    Hand(const std::string hostname,const Poco::UInt16 Port);  // Constructor
    HScrewList Slist;
    HScrewList Blist;
    SE3 M;
    StreamSocket sock;
	vector<Matrix6d> Glist;	
	vector<SE3> Mlist;	
    Vector3d g;    
    std::vector<HJVec> get_q();
    std::vector<SE3> Mlist1,Mlist2,Mlist3;
    std::vector<Matrix6d> Glist1,Glist2,Glist3;
    HScrewList Slist1,Slist2,Slist3;
    HScrewList Blist1,Blist2,Blist3;
    SE3 M1,M2,M3;
    void set_tau(const std::vector<HJVec> tau_list);
    std::vector<MassMat> MassMatrix(std::vector<HJVec>  q_list);
    std::vector<MassMat> MassMatrixInverse(std::vector<HJVec>  q_list);
    std::vector<HJVec> GravityForces(std::vector<HJVec>  q_list);
    int state;
    pinocchio::Model finger1;    
    pinocchio::Model finger2;    
    pinocchio::Model finger3;    
    
};


#endif // HAND
