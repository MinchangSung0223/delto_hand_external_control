#include "control_run.h"
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <sstream>
#include <cmath>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

#define TORQ2DUTY 100

void* control_run(void* arg) {
    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);
    double dt = 0.005;
    Vector3d g;
    g<<0,0,-9.8;
    std::vector<HJVec> prev_q_list;
    std::vector<HJVec> q_dot_list;
    q_dot_list.push_back(HJVec::Zero());
    q_dot_list.push_back(HJVec::Zero());
    q_dot_list.push_back(HJVec::Zero());
    prev_q_list.push_back(HJVec::Zero());
    prev_q_list.push_back(HJVec::Zero());
    prev_q_list.push_back(HJVec::Zero());
    std::vector<HJVec> tau_list;
    tau_list.push_back(HJVec::Zero());
    tau_list.push_back(HJVec::Zero());
    tau_list.push_back(HJVec::Zero());


    HJVec q_des1,q_des2,q_des3;
    HJVec q_dot_des1,q_dot_des2,q_dot_des3;
    HJVec q_ddot_des1,q_ddot_des2,q_ddot_des3;
    std::vector<HJVec> q_list=delto.get_q();
    q_list=delto.get_q();
    usleep(10000);
    q_list=delto.get_q();
    usleep(10000);
    q_list=delto.get_q();
    usleep(10000);

    HJVec q1_0,q2_0,q3_0;
    q1_0 = q_list.at(0);
    q2_0 = q_list.at(1);
    q3_0 = q_list.at(2);
    prev_q_list.at(0) = q_list.at(0);
    prev_q_list.at(1) = q_list.at(1);
    prev_q_list.at(2) = q_list.at(2);

    HJVec q1_T,q2_T,q3_T=HJVec::Zero();
    HJVec eint1,eint2,eint3=HJVec::Zero();

    q1_T<<0,0,3.141592/3.0,3.141592/3.0;
    q2_T<<-3.141592/3.0,0,3.141592/3.0,3.141592/3.0;
    q3_T<<3.141592/3.0,0,3.141592/3.0,3.141592/3.0;


    for(int i =0;i<10;i++){
        delto.set_tau(tau_list);
        usleep(10000);
        delto.set_tau(tau_list);
        usleep(10000);
        delto.set_tau(tau_list);
        usleep(10000);
        delto.set_tau(tau_list);
        usleep(10000);
    }
    
        
    double Tf = 1;
    while (is_run) {
        next_period.tv_nsec += CYCLE_NS;
        if (next_period.tv_nsec >= 1000000000) {
            next_period.tv_nsec -= 1000000000;
            next_period.tv_sec++;
        }
		q_list=delto.get_q();

        lr_hand::JointTrajectory(q1_0, q1_T, Tf, gt ,5 ,q_des1, q_dot_des1, q_ddot_des1);
        lr_hand::JointTrajectory(q2_0, q2_T, Tf, gt ,5 ,q_des2, q_dot_des2, q_ddot_des2);
        lr_hand::JointTrajectory(q3_0, q3_T, Tf, gt ,5 ,q_des3, q_dot_des3, q_ddot_des3);




        q_dot_list.at(0) = 0.95*q_dot_list.at(0)+0.05*(q_list.at(0)-prev_q_list.at(0))/dt;
        q_dot_list.at(1) = 0.95*q_dot_list.at(1)+0.05*(q_list.at(1)-prev_q_list.at(1))/dt;
        q_dot_list.at(2) = 0.95*q_dot_list.at(2)+0.05*(q_list.at(2)-prev_q_list.at(2))/dt;

        prev_q_list.at(0) = q_list.at(0);
        prev_q_list.at(1) = q_list.at(1);
        prev_q_list.at(2) = q_list.at(2);





        HJVec q1 = q_list.at(0);
        HJVec q2 = q_list.at(1);
        HJVec q3 = q_list.at(2);

        HJVec q_dot1 = q_dot_list.at(0);
        HJVec q_dot2 = q_dot_list.at(1);
        HJVec q_dot3 = q_dot_list.at(2);

        HJVec e1 = q_des1-q1;
        HJVec e2 = q_des2-q2;
        HJVec e3 = q_des3-q3;
        
        HJVec edot1 = q_dot_des1-q_dot1;
        HJVec edot2 = q_dot_des2-q_dot2;
        HJVec edot3 = q_dot_des3-q_dot3;

        eint1 += e1*dt;
        eint2 += e2*dt;
        eint3 += e3*dt;

        HJacobian Jb1 = lr_hand::JacobianBody(delto.Blist1,q1);
        HJacobian Jb2 = lr_hand::JacobianBody(delto.Blist2,q2);
        HJacobian Jb3 = lr_hand::JacobianBody(delto.Blist3,q3);




        HMassMat M1 = lr_hand::MassMatrix(q_list.at(0),delto.Mlist1,delto.Glist,delto.Slist1);
        HMassMat M2 = lr_hand::MassMatrix(q_list.at(1),delto.Mlist2,delto.Glist,delto.Slist2);
        HMassMat M3 = lr_hand::MassMatrix(q_list.at(2),delto.Mlist3,delto.Glist,delto.Slist3);

        HJVec cvec1 = lr_hand::VelQuadraticForces(q_list.at(0),q_dot_list.at(0) ,delto.Mlist1,delto.Glist,delto.Slist1);
        HJVec cvec2 = lr_hand::VelQuadraticForces(q_list.at(1),q_dot_list.at(1) ,delto.Mlist2,delto.Glist,delto.Slist2);
        HJVec cvec3 = lr_hand::VelQuadraticForces(q_list.at(2),q_dot_list.at(2) ,delto.Mlist3,delto.Glist,delto.Slist3);
        

        HJVec gvec1 = lr_hand::GravityForces(q_list.at(0),g,delto.Mlist1,delto.Glist,delto.Slist1);
        HJVec gvec2 = lr_hand::GravityForces(q_list.at(1),g,delto.Mlist2,delto.Glist,delto.Slist2);
        HJVec gvec3 = lr_hand::GravityForces(q_list.at(2),g,delto.Mlist3,delto.Glist,delto.Slist3);
        


        HJVec Hinf_K;
        Hinf_K<<0.7,0.7,0.5,0.5;

        HJVec gamma;
        gamma<<100,100,100,100;
        HMatrixNd Hinf_Kp = HMatrixNd::Identity() * 100.0;
        HMatrixNd Hinf_Kv = HMatrixNd::Identity() * 20.0;
        HMatrixNd Hinf_K_gamma = HMatrixNd::Identity();
        for (int i = 0; i < 4; i++)
        {
            Hinf_K_gamma(i, i) = Hinf_K(i) + 1.0 / gamma(i);
        }



        HJVec q_ddot_ref1 = q_ddot_des1+Hinf_Kv*edot1+Hinf_Kp*e1;
        HJVec q_ddot_ref2 = q_ddot_des2+Hinf_Kv*edot2+Hinf_Kp*e2;
        HJVec q_ddot_ref3 = q_ddot_des3+Hinf_Kv*edot3+Hinf_Kp*e3;

        
        Vector6d F1,F2,F3;
        F1<<0,0,50,0,0,0;
        F2<<0,0,50,0,0,0;
        F3<<0,0,50,0,0,0;

        HJVec tau1 = M1*q_ddot_ref1 + cvec1+gvec1+ (Hinf_K_gamma) * (edot1 + Hinf_Kv * e1 + Hinf_Kp * eint1);
        HJVec tau2 = M2*q_ddot_ref2 + cvec2+gvec2+ (Hinf_K_gamma) * (edot2 + Hinf_Kv * e2 + Hinf_Kp * eint2);
        HJVec tau3 = M3*q_ddot_ref3 + cvec3+gvec3+ (Hinf_K_gamma) * (edot3 + Hinf_Kv * e3 + Hinf_Kp * eint3);
        if(gt>Tf){
            tau1= Jb1.transpose()*F1;
            tau2= Jb2.transpose()*F2;
            tau3= Jb3.transpose()*F3;
        }


        tau_list.at(0)=(tau1*TORQ2DUTY);
        tau_list.at(1)=(tau2*TORQ2DUTY);
        tau_list.at(2)=(tau3*TORQ2DUTY);

        delto.set_tau(tau_list);


        std::cout<<"tau1:" << tau1.transpose()<<std::endl;
        std::cout<<"tau2:" << tau2.transpose()<<std::endl;
        std::cout<<"tau3:" << tau3.transpose()<<std::endl;
        info.act.q_list = q_list;
        
        gt += dt;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
    }
    return NULL;
}
