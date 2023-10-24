/*
 * dependency_test.h
 *
 *  Created on: 2023. 10. 24.
 *      Author: Sunhong Kim
 */

#ifndef DEPENDENCYTEST_H_
#define DEPENDENCYTEST_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <assert.h>
#include <sys/mman.h>
#include <string.h>		// string function definitions
#include <fcntl.h>		// File control definitions
#include <errno.h>		// Error number definitions
#include <termios.h>	// POSIX terminal control definitions
#include <time.h>		// time calls
#include <sys/ioctl.h>
#include <math.h>
#include <string>
#include "iostream"
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <Eigen/Dense>

#include "CS_Indy7.h"
#include "PropertyDefinition.h"
#include "ServoAxis.h"  // For Indy7 interface

using namespace std;

#define NUM_AXIS		6

#define WAKEUP_TIME				(5)	// wake up timeout before really run, in second
#define NSEC_PER_SEC 			1000000000

// Cycle time in nanosecond
unsigned int cycle_ns = 1000000; /* 1 ms */

typedef unsigned int UINT32;
typedef int32_t INT32;
typedef int16_t INT16;
typedef uint16_t UINT16;
typedef uint8_t UINT8;
typedef int8_t INT8;

// For RT thread management
static int run = 1;

unsigned long periodCycle = 0, worstCycle = 0;
unsigned long periodCompute = 0, worstCompute = 0;
unsigned long periodEcat = 0, worstEcat = 0;
unsigned long periodBuffer = 0, worstBuffer = 0;
unsigned int overruns = 0;

unsigned long periodPoco = 0;
unsigned long periodIndysim = 0;

// Global time (beginning from zero)
double gt=0;

const int 	 zeroPos[NUM_AXIS] = {ZERO_POS_1,ZERO_POS_2,ZERO_POS_3,ZERO_POS_4,ZERO_POS_5,ZERO_POS_6};
const int 	 gearRatio[NUM_AXIS] = {GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_101,GEAR_RATIO_101,GEAR_RATIO_101};
const int 	 TauADC[NUM_AXIS] = {TORQUE_ADC_500,TORQUE_ADC_500,TORQUE_ADC_200,TORQUE_ADC_100,TORQUE_ADC_100,TORQUE_ADC_100};
const double TauK[NUM_AXIS] = {TORQUE_CONST_500,TORQUE_CONST_500,TORQUE_CONST_200,TORQUE_CONST_100,TORQUE_CONST_100,TORQUE_CONST_100};
const int 	 dirQ[NUM_AXIS] = {-1,-1,1,-1,-1,1};
const int 	 dirTau[NUM_AXIS] = {-1,-1,1,-1,-1,1};

// Interface to physical axes
NRMKHelper::ServoAxis Axis[NUM_AXIS];

struct LOGGING_PACK
{
	double Time;
	INT32 	ActualPos[JOINTNUM];
	INT32 	ActualVel[JOINTNUM];
};

typedef struct STATE{
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

    double s_time;
}state;

typedef struct JOINT_INFO{
	int Position;
	int aq_inc[NUM_AXIS];
	int atoq_per[NUM_AXIS];
	short dtor_per[NUM_AXIS];
	int statusword[NUM_AXIS];

	JVec q_target;
	JVec qdot_target;
	JVec qddot_target;
	JVec traj_time;

	STATE act;
	STATE des;
	STATE nom;

}JointInfo;



#endif /* DEPENDENCYTEST_H_ */
