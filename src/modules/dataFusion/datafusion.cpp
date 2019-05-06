//
//  datafusion.cpp
//  DroneReferenceMonitor
//
//  Created by Raul on 1/17/19.
//  Copyright © 2019 Raul. All rights reserved.
//
/*Variables
y  (double) angle calculation
u  (double) gyroscope reading on one axis
R  (double) covariance of sensors. We can measure this one
Q  (double[2][2]) Covariance of states. We can not measure it
dt (double)sampleRate
*/
#include "functions.h"

AngleInfo angleDataFusionRoll(double y, double u, double R, double Q [2][2], double dt){
    //static variables
    static double angle_hat=0;
    static double wb =0;
    static double angle=0;
    static double P[2][2]={0.1,0,
        0,0.1};
    
     AngleInfo output;
    
    double A[2][2]={1,-dt,
        0,1};
    
    double C[1][2]={1,0};
    
    
    //intermediary variables
    double P1[2][2]={0,0,
        0,0};
    double P2[2][2]={0,0,
        0,0};
    double P3[2][2]={0,0,
        0,0};
    double P_[2][2]={0,0,
        0,0};
    double Atrans[2][2]={0,0,
        0,0};
    double Ctrans[2][1]={1,
        0};
    double S[1][1]={0};
    double Sinv[1][1]={0};
    double S1[1][2]={0,0};
    double S2 [1][1]={0};
    double K[2][1]={0,
        0};
    double K1[2][1]={0,
        0};
    double KC[2][2]={0,0,
        0,0};
    double identity2x2[2][2]={1,0,
        0,1};
    
    
    
    //mesurement update
    //P_roll=A*P*AT+Q
    multiplyMatrices<2>(A, P, P1);
    transposeMatrix<2>(A, Atrans);
    multiplyMatrices<2>(P1, Atrans, P2);
    addMatrices<2>(P2, Q, P_);
    //P_checked
    
    //time update
    //S=C*P_roll*C.transpose()+R_roll;
    multiplyMatrices<2>(C, P_, S1);
    transposeMatrix(C, Ctrans);
    multiplyMatrices<1>(S1, Ctrans, S2);
    S[0][0]= S2[0][0]+R;
    //S checked
    
    //K=P_roll*C.transpose()*S.inverse()
    multiplyMatrices<2>(P_, Ctrans, K1);
    Sinv[0][0]=1.0/S[0][0];
    multiplyMatrices<2>(K1, Sinv, K);
    //K checked
    
    angle_hat=angle_hat + dt*(u-wb);
    wb=wb+K[1][0]*(y-angle_hat);
    angle_hat=angle_hat + K[0][0]*(y-angle_hat);
    
    
    //P_roll=(MatrixXd::Identity(n,n)-K*C)*P_roll;
    multiplyMatrices<2>(K, C, KC);
    subtractMatrices<2>(identity2x2, KC, P3);
    initializeMatrix<2>(P);//clean P
    multiplyMatrices<2>(P3, P_, P);
    
    angle = angle+(u-wb)*dt;
    output.angle=angle;
    output.angularSpeed=u-wb;
    
    return output;
}

AngleInfo angleDataFusionPitch(double y, double u, double R, double Q [2][2], double dt){
    //static variables
    static double angle_hat=0;
    static double wb =0;
    static double angle=0;
    static double P[2][2]={0.1,0,
        0,0.1};
    
    
    double A[2][2]={1,-dt,
        0,1};
    
    double C[1][2]={1,0};
    struct AngleInfo output;
    
    //intermediary variables
    double P1[2][2]={0,0,
        0,0};
    double P2[2][2]={0,0,
        0,0};
    double P3[2][2]={0,0,
        0,0};
    double P_[2][2]={0,0,
        0,0};
    double Atrans[2][2]={0,0,
        0,0};
    double Ctrans[2][1]={1,
        0};
    double S[1][1]={0};
    double Sinv[1][1]={0};
    double S1[1][2]={0,0};
    double S2 [1][1]={0};
    double K[2][1]={0,
        0};
    double K1[2][1]={0,
        0};
    double KC[2][2]={0,0,
        0,0};
    double identity2x2[2][2]={1,0,
        0,1};
    
    
    
    //mesurement update
    //P_roll=A*P*AT+Q
    multiplyMatrices<2>(A, P, P1);
    transposeMatrix<2>(A, Atrans);
    multiplyMatrices<2>(P1, Atrans, P2);
    addMatrices<2>(P2, Q, P_);
    //P_checked
    
    //time update
    //S=C*P_roll*C.transpose()+R_roll;
    multiplyMatrices<2>(C, P_, S1);
    transposeMatrix(C, Ctrans);
    multiplyMatrices<1>(S1, Ctrans, S2);
    S[0][0]= S2[0][0]+R;
    //S checked
    
    //K=P_roll*C.transpose()*S.inverse()
    multiplyMatrices<2>(P_, Ctrans, K1);
    Sinv[0][0]=1.0/S[0][0];
    multiplyMatrices<2>(K1, Sinv, K);
    //K checked
    
    angle_hat=angle_hat + dt*(u-wb);
    wb=wb+K[1][0]*(y-angle_hat);
    angle_hat=angle_hat + K[0][0]*(y-angle_hat);
    
    
    //P_roll=(MatrixXd::Identity(n,n)-K*C)*P_roll;
    multiplyMatrices<2>(K, C, KC);
    subtractMatrices<2>(identity2x2, KC, P3);
    initializeMatrix<2>(P);//clean P
    multiplyMatrices<2>(P3, P_, P);
    
    angle = angle+(u-wb)*dt;
    
    output.angle=angle;
    output.angularSpeed=u-wb;
    
    return output;
}

AngleInfo angleDataFusionYaw(double y, double u, double R, double Q [2][2], double dt){
    //static variables
    static double angle_hat=0;
    static double wb =0;
    static double angle=0;
    static double P[2][2]={0.1,0,
        0,0.1};
    
    
    double A[2][2]={1,-dt,
        0,1};
    
    double C[1][2]={1,0};
    
    AngleInfo output;
    
    //intermediary variables
    double P1[2][2]={0,0,
        0,0};
    double P2[2][2]={0,0,
        0,0};
    double P3[2][2]={0,0,
        0,0};
    double P_[2][2]={0,0,
        0,0};
    double Atrans[2][2]={0,0,
        0,0};
    double Ctrans[2][1]={1,
        0};
    double S[1][1]={0};
    double Sinv[1][1]={0};
    double S1[1][2]={0,0};
    double S2 [1][1]={0};
    double K[2][1]={0,
        0};
    double K1[2][1]={0,
        0};
    double KC[2][2]={0,0,
        0,0};
    double identity2x2[2][2]={1,0,
        0,1};
    
    
    
    //mesurement update
    //P_roll=A*P*AT+Q
    multiplyMatrices<2>(A, P, P1);
    transposeMatrix<2>(A, Atrans);
    multiplyMatrices<2>(P1, Atrans, P2);
    addMatrices<2>(P2, Q, P_);
    //P_checked
    
    //time update
    //S=C*P_roll*C.transpose()+R_roll;
    multiplyMatrices<2>(C, P_, S1);
    transposeMatrix(C, Ctrans);
    multiplyMatrices<1>(S1, Ctrans, S2);
    S[0][0]= S2[0][0]+R;
    //S checked
    
    //K=P_roll*C.transpose()*S.inverse()
    multiplyMatrices<2>(P_, Ctrans, K1);
    Sinv[0][0]=1.0/S[0][0];
    multiplyMatrices<2>(K1, Sinv, K);
    //K checked
    
    angle_hat=angle_hat + dt*(u-wb);
    wb=wb+K[1][0]*(y-angle_hat);
    angle_hat=angle_hat + K[0][0]*(y-angle_hat);
    
    
    //P_roll=(MatrixXd::Identity(n,n)-K*C)*P_roll;
    multiplyMatrices<2>(K, C, KC);
    subtractMatrices<2>(identity2x2, KC, P3);
    initializeMatrix<2>(P);//clean P
    multiplyMatrices<2>(P3, P_, P);
    
    angle = angle+(u-wb)*dt;
    output.angle=angle;
    output.angularSpeed=u-wb;
    
    
    
    return output;
}
