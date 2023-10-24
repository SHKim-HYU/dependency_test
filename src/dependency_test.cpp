/*
 * dependency_test.cpp
 *
 *  Created on: 2023. 10. 24.
 *      Author: Sunhong Kim
 */

#include "dependency_test.h"


JointInfo info;

CS_Indy7 cs_indy7;


void casadi_run()
{
	info.act.q = JVec::Zero();   
    info.act.q_dot = JVec::Zero();   

    Jacobian J_b;
    MassMat M, C;
    JVec G;

    // update robot & compute dynamics
    cs_indy7.updateRobot(info.act.q, info.act.q_dot);
    
    M = cs_indy7.getM();
    C = cs_indy7.getC();
    G = cs_indy7.getG();
    J_b = cs_indy7.getJ_b();

    cout<<"M: "<<M<<endl;
    cout<<"C: "<<C<<endl;
    cout<<"G: "<<G<<endl;
    cout<<"J_b: "<<J_b<<endl;
    

}



int main(int argc, char *argv[])
{

    // CasADi Test
    cs_indy7=CS_Indy7();
	cs_indy7.CSSetup("../lib/URDF2CASADI/indy7/indy7.json");
    casadi_run();

    return 0;
}