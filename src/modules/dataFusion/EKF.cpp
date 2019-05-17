//
//  EFK_functions.cpp
//  DroneReferenceMonitor
//
//  Created by Raul on 1/17/19.
//  Copyright © 2019 Raul. All rights reserved.
//

// jacobian of the drone
// Ix,Iy,Iz,m,kd are parameters of the model of the drone, x is the
#include "functions.h"
#include <cmath>
#include <stdio.h>
#include <px4_log.h>
#include <drivers/drv_hrt.h>
bool systemCompromised = false;

void llaTOxyz(double lla[3],double ll0[2],double alt, double * array){
    double PI= 3.14159265;
    double R =6378137;
    double f =1/298.257223563;

    double dLat = lla[0]-ll0[0];
    double dLon = lla[1]-ll0[1];

    double rll0[2];
    rll0[0]=ll0[0]*PI/180;
    rll0[1]=ll0[1]*PI/180;
    double Rn = R/sqrt(1-(2*f-f*f)*sin(rll0[0])*sin(rll0[0]));
    double Rm = Rn*((1-(2*f-f*f))/(1-(2*f-f*f)*sin(rll0[0])*sin(rll0[0])));

    double dNorth = dLat*PI/180/atan2(1, Rm);
    double dEast = dLon*PI/180/atan2(1, Rn*cos(rll0[0]));

    array[0] =dNorth;
    array[1] =dEast;
    array[2] =-lla[2]-alt;

}

void EKF_Fk(double x[12][1],   double Ix, double Iy, double Iz,  double T, double m,  double kd, double Fk[12][12]){

    double Fk_temp[12][12]={
        x[4][0]*cos(x[0][0])*tan(x[1][0])-x[5][0]*sin(x[0][0])*tan(x[1][0]),x[5][0]*cos(x[0][0])*(pow(tan(x[1][0]),2) + 1)+x[4][0]*sin(x[0][0])*(pow(tan(x[1][0]),2)+ 1),0,1,sin(x[0][0])*tan(x[1][0]),cos(x[0][0])*tan(x[1][0]),0,0,0,0,0,0,
        
        -x[5][0]*cos(x[0][0])-x[4][0]*sin(x[0][0]),0,0,0,cos(x[0][0]),-sin(x[0][0]),0,0,0,0,0,0,
        
        (x[4][0]*cos(x[0][0]))/cos(x[1][0])-(x[5][0]*sin(x[0][0]))/cos(x[1][0]),(x[5][0]*cos(x[0][0])*sin(x[1][0]))/pow(cos(x[1][0]),2) + (x[4][0]*sin(x[0][0])*sin(x[1][0]))/pow(cos(x[1][0]),2),0,0,sin(x[0][0])/cos(x[1][0]),cos(x[0][0])/cos(x[1][0]),0,0,0,0,0,0,
        
        0,0,0,0,(x[5][0]*(Iy - Iz))/Ix,(x[4][0]*(Iy - Iz))/Ix,0,0,0,0,0,0,
        
        0,0,0,-(x[5][0]*(Ix - Iz))/Iy,0, -(x[3][0]*(Ix - Iz))/Iy,0,0,0,0,0,0,
        
        0,0,0,(x[4][0]*(Ix - Iy))/Iz, (x[3][0]*(Ix - Iy))/Iz,0,0,0,0,0,0,0,
        
        0,0,0,0,0,0,0,0,0,1,0,0,
        
        0,0,0,0,0,0,0,0,0,0,1,0,
        
        0,0,0,0,0,0,0,0,0,0,0,1,
        (T*(cos(x[0][0])*sin(x[2][0]) - cos(x[2][0])*sin(x[0][0])*sin(x[1][0])))/m,(T*cos(x[0][0])*cos(x[1][0])*cos(x[2][0]))/m, (T*(cos(x[2][0])*sin(x[0][0]) - cos(x[0][0])*sin(x[1][0])*sin(x[2][0])))/m,0,0,0,0,0,0,-kd/m,0,0,
        
        -(T*(cos(x[0][0])*cos(x[2][0]) + sin(x[0][0])*sin(x[1][0])*sin(x[2][0])))/m,(T*cos(x[0][0])*cos(x[1][0])*sin(x[2][0]))/m,(T*(sin(x[0][0])*sin(x[2][0])+cos(x[0][0])*cos(x[2][0])*sin(x[1][0])))/m,0,0,0,0,0,0,0,-kd/m,0,
        
        -(T*cos(x[1][0])*sin(x[0][0]))/m,-(T*cos(x[0][0])*sin(x[1][0]))/m,0,0,0,0,0,0,0,0,0,-kd/m};

    for(int i=0;i<12;i++)
        for(int j=0;j<12;j++)
            Fk[i][j]=Fk_temp[i][j];

}
void EKF_Hk(double Hk[9][12]){
    for(int i=0;i<9;i++)
        for(int j=0;j<12;j++)
        {
            if(i==j)
                Hk[i][j]=1;
            
            else
                Hk[i][j]=0;
        }
}

// Rotation matrix
void eul2rotmat(double angles[3],double rotmatrix[3][3]){
    double R2[3][3]={cos(angles[1])*cos(angles[2]), sin(angles[0])*sin(angles[1])*cos(angles[2])-cos(angles[0])*sin(angles[2]), cos(angles[0])*sin(angles[1])*cos(angles[2])+sin(angles[0])*sin(angles[2]),
        cos(angles[1])*sin(angles[2]),sin(angles[0])*sin(angles[1])*sin(angles[2])+cos(angles[0])*cos(angles[2]), cos(angles[0])*sin(angles[1])*sin(angles[2])-sin(angles[0])*cos(angles[2]),
        -sin(angles[1]),sin(angles[0])*cos(angles[1]),cos(angles[0])*cos(angles[1])};
    
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            rotmatrix[i][j]=R2[i][j];
}

// Transformation matrix inverse
void transforMinv(double angles[3],double Minv_out[3][3]){
    double  Minv[3][3]={1 ,sin(angles[0])*tan(angles[1]), cos(angles[0])*tan(angles[1]),
                        0,cos(angles[0]),-sin(angles[0]),
                        0,sin(angles[0])/cos(angles[1]), cos(angles[0])/cos(angles[1])};
    
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            Minv_out[i][j]=Minv[i][j];
}

void update_estimated_states(double estimated_states[12][1],double dxangles[3][1],double dxangularspeeds[3],double dxposition[3],double dxvelocity[3],double KRes[12][1],double dt){
    double dx[12]={0,0,0,0,0,0,0,0,0,0,0,0};
    dx[0]=dxangles[0][0]+KRes[0][0];
    dx[1]=dxangles[1][0]+KRes[1][0];
    dx[2]=dxangles[2][0]+KRes[2][0];
    
    dx[3]=dxangularspeeds[0]+KRes[3][0];
    dx[4]=dxangularspeeds[1]+KRes[4][0];
    dx[5]=dxangularspeeds[2]+KRes[5][0];
    
    dx[6]=dxposition[0]+KRes[6][0];
    dx[7]=dxposition[1]+KRes[7][0];
    dx[8]=dxposition[2]+KRes[8][0];
    
    dx[9]=dxvelocity[0]+KRes[9][0];
    dx[10]=dxvelocity[1]+KRes[10][0];
    dx[11]=dxvelocity[2]+KRes[11][0];
    
    for(int i=0;i<12;i++)
        estimated_states[i][0]=estimated_states[i][0]+dt*dx[i];
    
    
}

void updateP(double Pdot[12][12],double P[12][12],double dt){
    for(int i=0;i<12;i++)
        for(int j=0;j<12;j++)
            P[i][j]=P[i][j]+dt*Pdot[i][j];
}


void EKF(double sensors[9][1], double controls[4],double dt){

    //hrt_abstime startTime= hrt_absolute_time();
    //static variables
    static double P[12][12];
    static double estimated_states[12][1];
    
    //constants
    double Ix=0.005;
    double Iy=0.005;
    double Iz=0.009;
    double m=1;
    double kd=0.025;
    double dragf=1;
    double thrustf=1;
    double control_bias=1;
    double L=0.15;
    double g=9.8;
    
    double T=0;

    double F[12][12];
    double H[9][12];
    double K1[12][9];
    double Htrans[12][9];
    double Rinv[9][9];//inverse of R
    double K[12][9];
    double FP[12][12];
    double Ftrans[12][12];
    double PFt[12][12];
    double KH[12][12];
    double KHP[12][12];
    double Pdot[12][12];
    double Pdot1[12][12];
    double Pdot2[12][12];
    double Q[12][12];
    double R[9][9];
    double residuals[9][1];
    double KRes[12][1];
    double dxAngles[3][1];              //dynamics of the drone
    double dxAngularSpeed[3]={0,0,0};
    double dxPosition[3]={0,0,0};
    double dxVelocity[3]={0,0,0};

    initializeMatrix<12>(F);
    initializeMatrix<9>(H);
    initializeMatrix<12>(K1);
    initializeMatrix<12>(Htrans);
    initializeMatrix<9>(Rinv);
    initializeMatrix<12>(K);
    initializeMatrix<12>(FP);
    initializeMatrix<12>(Ftrans);
    initializeMatrix<12>(PFt);
    initializeMatrix<12>(KH);
    initializeMatrix<12>(KHP);
    initializeMatrix<12>(Pdot);
    initializeMatrix<12>(Pdot2);
    initializeMatrix<12>(Pdot1);
    initializeMatrix<9>(residuals);
    initializeMatrix<12>(KRes);
    initializeMatrix<3>(dxAngles);

    //initializes R to identity matrix
    for(int i=0;i<9;i++)
        for(int j=0;j<9;j++){
            if(i==j)
                R[i][j]=1;
            else
                R[i][j]=0;
            
        }
    
    inverseMatrix(R, Rinv);
    
    for(int i=0;i<12;i++)
        for(int j=0;j<12;j++){
            if(i==j)
                Q[i][j]=1;
            else
                Q[i][j]=0;
            
        }

    //variable initialization
    double tau[3]={0,0,0};
    
    double Minv[3][3]={0,0,0,
        0,0,0,
        0,0,0};
    
    double rotm[3][3]={0,0,0,
        0,0,0,
        0,0,0};
    
    double TB[3][1]={0,
        0,
        0};

    double angles[3]={estimated_states[0][0],estimated_states[1][0],estimated_states[2][0]};
    double angularSpeed[3][1]={estimated_states[3][0],
        estimated_states[4][0],
        estimated_states[5][0]};

    transforMinv(angles,Minv);
    eul2rotmat(angles, rotm);
    
    T=thrustf*(controls[0]+controls[1]+controls[2]+controls[3]-4*control_bias);
    tau[0] = L*thrustf*((controls[1]+controls[2]) - (controls[0]+controls[3]));
    tau[1] = L*thrustf*((controls[0]+controls[1]) - (controls[2]+controls[3]));
    tau[2] = L*dragf*((controls[0]+controls[2]) - (controls[1]+controls[3]));
    
    double T2[3][1]={0,
                     0,
                     T};
    
    multiplyMatrices<3>(rotm, T2, TB);
    multiplyMatrices<3>(Minv, angularSpeed, dxAngles);
    dxAngularSpeed[0]=tau[0]/Ix+(Iy-Iz)/Ix*angularSpeed[1][0]*angularSpeed[2][0];
    dxAngularSpeed[1]=tau[1]/Iy+(Iz-Ix)/Iy*angularSpeed[0][0]*angularSpeed[2][0];
    dxAngularSpeed[2]=tau[2]/Iz+(Ix-Iy)/Iz*angularSpeed[0][0]*angularSpeed[1][0];
    
    dxPosition[0]=estimated_states[9][0];
    dxPosition[1]=estimated_states[10][0];
    dxPosition[2]=estimated_states[11][0];
    

    dxVelocity[0]= TB[0][0]/m-kd*estimated_states[9][0];
    dxVelocity[1]= TB[1][0]/m-kd*estimated_states[10][0];
    dxVelocity[2]= -g+TB[2][0]/m-kd*estimated_states[11][0];

    EKF_Fk(estimated_states,Ix,Iy,Iz,T,m,kd,F);

    EKF_Hk(H);

    transposeMatrix(H, Htrans);
    
    transposeMatrix(F, Ftrans);

    multiplyMatrices<12>(P, Htrans, K1);
    multiplyMatrices<12>(K1, Rinv, K);
    multiplyMatrices<12>(F, P, FP);
    multiplyMatrices<12>(P, Ftrans, PFt);
    multiplyMatrices<12>(K, H, KH);
    multiplyMatrices<12>(KH, P, KHP);
    addMatrices<12>(FP, PFt, Pdot1);
    subtractMatrices<12>(Pdot1, KHP, Pdot2);
    addMatrices<12>(Pdot2, Q, Pdot);
    
    subtractMatrices<9>(sensors, estimated_states, residuals);
    multiplyMatrices<9>(K, residuals, KRes);
    

    update_estimated_states(estimated_states,dxAngles,dxAngularSpeed,dxPosition,dxVelocity,KRes,dt);

    updateP(Pdot, P,dt);
    
    if((estimated_states[8][0]<=0))
        estimated_states[8][0]=0;
    if((estimated_states[8][0]<=0) && (estimated_states[11][0]<=0))
        estimated_states[11][0]=0;



    //hrt_abstime endTime= hrt_absolute_time();
    //PX4_ERR("EKF time: %llu\n", endTime-startTime);


}//end of function

bool attackDetected(){
    return false;
}



