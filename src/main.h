#ifndef MAIN_H
#define  MAIN_H
#include <signal.h>
#include <sched.h>

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <LR/include/LR_Control.h>
#include <LR/include/LR_Trajectory.h>
#include <LR/include/liegroup_robotics_hand.h>
#include <SimRobot/Robot.h>
#include <RobotSimulator/b3RobotSimulatorClientAPI.h>
#undef debug
#include "qt/qt_run.h"
#include "control/control_run.h"
#include "delto_hand/Hand.h"





typedef struct STATE{

    std::vector<HJVec> q_list;
	JVec q;
	JVec q_dot;
	JVec q_ddot;
	JVec tau;
	JVec tau_ext;
	JVec G;

	Vector6d x;                           //Task space
	Vector6d x_dot;
	Vector6d x_ddot;
	Vector6d F;
	Vector6d F_CB;

	Vector6d V;
	Vector6d Vdot;
	Vector6d Vddot;
	Vector6d lambda;
	Vector6d lambdadot;
	Vector6d lambdaddot;
	Vector6d lambdadddot;

    double s_time;
}state;

typedef struct JOINT_INFO{
	int Position;
	// int aq_inc[NUM_AXIS];
	// int atoq_per[NUM_AXIS];
	// short dtor_per[NUM_AXIS];
	// int statusword[NUM_AXIS];

	JVec q_target;
	JVec qdot_target;
	JVec qddot_target;
	JVec traj_time;

	STATE act;
	STATE des;
	STATE nom;

}JointInfo;



#define MS_TO_US 1000000
#define CYCLE_NS 5000000

extern JOINT_INFO info;
extern double gt;
extern int is_run;
extern Hand delto;

#endif // MAIN_H
