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
#include <fstream>
#include <iomanip>
std::fstream dataFile("residual.txt",std::ios::out);
bool systemCompromised = false;
//                currentGPS, GPSInitial, initialAlt, correctedGPS
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
        for(int j=0;j<3;j++){
            Minv_out[i][j]=Minv[i][j];
            //PX4_ERR("tMIN %d %d : %f ",i,j,Minv[i][j]);
        }
}

void update_estimated_states(double estimated_states[12][1],double dxangles[3][1],double dxangularspeeds[3],double dxposition[3],double dxvelocity[3],double KRes[12][1],double dt){
    double dx[12]={0,0,0,0,0,0,0,0,0,0,0,0};

    dx[0]=dxangles[0][0]+KRes[0][0];
    dx[1]=dxangles[1][0]+KRes[1][0];
    dx[2]=dxangles[2][0]+KRes[2][0];
    
    dx[3]=dxangularspeeds[0]+KRes[3][0];
    dx[4]=dxangularspeeds[1]+KRes[4][0];
    dx[5]=dxangularspeeds[2]+KRes[5][0];
    
    dx[6]=dxposition[0]+ KRes[6][0];
    dx[7]=dxposition[1]+ KRes[7][0];
    dx[8]=dxposition[2]+ KRes[8][0];
    
    dx[9]=dxvelocity[0]+ KRes[9][0];
    dx[10]=dxvelocity[1]+ KRes[10][0];
    dx[11]=dxvelocity[2]+ KRes[11][0];
    
    for(int i=0;i<12;i++)
        estimated_states[i][0]=estimated_states[i][0]+dt*dx[i];
}

void updateP(double Pdot[12][12],double P[12][12],double dt){
    for(int i=0;i<12;i++)
        for(int j=0;j<12;j++)
            P[i][j]=P[i][j]+dt*Pdot[i][j];
}


void EKF(double sensors[9][1], double controls[4],double dt){

    static int counter =0;
    static double CUSUM[9];
    PX4_ERR("ITERATION: \%d**************************************************************************",++counter);
    //static variables
    //estimated_states[0][0] roll angle
    //estimated_states[1][0] pitch angle
    //estimated_states[2][0] yaw angle
    //estimated_states[3][0] roll speed
    //estimated_states[4][0] pitch speed
    //estimated_states[5][0] yaw speed
    //estimated_states[6][0] x
    //estimated_states[7][0] y
    //estimated_states[8][0] z
    //estimated_states[9][0]  vx
    //estimated_states[10][0] vy
    //estimated_states[11][0] vz
    static double estimated_states[12][1]={0,0,0,0,0,0,0,0,0,0,0,0};

    double K1[12][9]={0,0,0,0,0,0,0,0,0,
                      0,0,0,0,0,0,0,0,0,
                      0,0,0,0,0,0,0,0,0,
                      0,0,0,0,0,0,0,0,0,
                      0,0,0,0,0,0,0,0,0,
                      0,0,0,0,0,0,0,0,0,
                      0,0,0,0,0,0,0,0,0,
                      0,0,0,0,0,0,0,0,0,
                      0,0,0,0,0,0,0,0,0,
                      0,0,0,0,0,0,0,0,0,
                      0,0,0,0,0,0,0,0,0,
                      0,0,0,0,0,0,0,0,0};

    double Htrans[12][9]={0,0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,0};

    double Rinv[9][9]={0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0};

    double K[12][9]={0,0,0,0,0,0,0,0,0,
                     0,0,0,0,0,0,0,0,0,
                     0,0,0,0,0,0,0,0,0,
                     0,0,0,0,0,0,0,0,0,
                     0,0,0,0,0,0,0,0,0,
                     0,0,0,0,0,0,0,0,0,
                     0,0,0,0,0,0,0,0,0,
                     0,0,0,0,0,0,0,0,0,
                     0,0,0,0,0,0,0,0,0,
                     0,0,0,0,0,0,0,0,0,
                     0,0,0,0,0,0,0,0,0,
                     0,0,0,0,0,0,0,0,0};

    double FP[12][12]={0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0};

    double Ftrans[12][12]={0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0};

    double PFt[12][12]={0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0};

    double KH[12][12]={0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0,
                       0,0,0,0,0,0,0,0,0,0,0,0};

    double KHP[12][12]={0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0};

    double Pdot[12][12]={0,0,0,0,0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,0,0,0,0,0};

    double Pdot1[12][12]= {0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0};

    double Pdot2[12][12] ={0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,0,0,0,0};

    double residuals[9][1]={0,0,0,0,0,0,0,0,0};
    double KRes[12][1]= {0,0,0,0,0,0,0,0,0,0,0,0};
    double dxAngles[3][1]={0,0,0};              //dynamics of the drone
    double dxAngularSpeed[3]={0,0,0};
    double dxPosition[3]={0,0,0};
    double dxVelocity[3]={0,0,0};
    double T;
    double H[9][12]={1,0,0,0,0,0,0,0,0,0,0,0,
                     0,1,0,0,0,0,0,0,0,0,0,0,
                     0,0,1,0,0,0,0,0,0,0,0,0,
                     0,0,0,1,0,0,0,0,0,0,0,0,
                     0,0,0,0,1,0,0,0,0,0,0,0,
                     0,0,0,0,0,1,0,0,0,0,0,0,
                     0,0,0,0,0,0,1,0,0,0,0,0,
                     0,0,0,0,0,0,0,1,0,0,0,0,
                     0,0,0,0,0,0,0,0,1,0,0,0};

    double r=1;
    double R[9][9]={r,0,0,0,0,0,0,0,0,
                    0,r,0,0,0,0,0,0,0,
                    0,0,r,0,0,0,0,0,0,
                    0,0,0,r,0,0,0,0,0,
                    0,0,0,0,r,0,0,0,0,
                    0,0,0,0,0,r,0,0,0,
                    0,0,0,0,0,0,r,0,0,
                    0,0,0,0,0,0,0,r,0,
                    0,0,0,0,0,0,0,0,r};

    double q=10;
    double Q[12][12]={q,0,0,0,0,0,0,0,0,0,0,0,
                      0,q,0,0,0,0,0,0,0,0,0,0,
                      0,0,q,0,0,0,0,0,0,0,0,0,
                      0,0,0,q,0,0,0,0,0,0,0,0,
                      0,0,0,0,q,0,0,0,0,0,0,0,
                      0,0,0,0,0,q,0,0,0,0,0,0,
                      0,0,0,0,0,0,q,0,0,0,0,0,
                      0,0,0,0,0,0,0,q,0,0,0,0,
                      0,0,0,0,0,0,0,0,q,0,0,0,
                      0,0,0,0,0,0,0,0,0,q,0,0,
                      0,0,0,0,0,0,0,0,0,0,q,0,
                      0,0,0,0,0,0,0,0,0,0,0,q};



    double pq=.0001;
    static double P[12][12]={ pq,0,0,0,0,0,0,0,0,0,0,0,
                              0, pq,0,0,0,0,0,0,0,0,0,0,
                              0,0, pq,0,0,0,0,0,0,0,0,0,
                              0,0,0, pq,0,0,0,0,0,0,0,0,
                              0,0,0,0, pq,0,0,0,0,0,0,0,
                              0,0,0,0,0, pq,0,0,0,0,0,0,
                              0,0,0,0,0,0, pq,0,0,0,0,0,
                              0,0,0,0,0,0,0, pq,0,0,0,0,
                              0,0,0,0,0,0,0,0, pq,0,0,0,
                              0,0,0,0,0,0,0,0,0, pq,0,0,
                              0,0,0,0,0,0,0,0,0,0, pq,0,
                              0,0,0,0,0,0,0,0,0,0,0, pq};

    //variable initialization
    //constants
    double Ix=0.005;
    double Iy=0.005;
    double Iz=0.009;
    double m= 0.8; //1.285; drone
    double kd=0;//0.027;
    double dragf = 1e-6;//0.15;
    double thrustf = 1e-6;//0.2;
    double control_bias=0;
    double L=0.1905;
    double g=9.8;
    T=4*(controls[0]+controls[1]+controls[2]+controls[3]-4*control_bias);//thrustf*...


    double F[12][12]={
        estimated_states[4][0]*cos(estimated_states[0][0])*tan(estimated_states[1][0])-estimated_states[5][0]*sin(estimated_states[0][0])*tan(estimated_states[1][0]), estimated_states[5][0]*cos(estimated_states[0][0])*(pow(tan(estimated_states[1][0]),2) + 1)+estimated_states[4][0]*sin(estimated_states[0][0])*(pow(tan(estimated_states[1][0]),2)+ 1), 0, 1, sin(estimated_states[0][0])*tan(estimated_states[1][0]), cos(estimated_states[0][0])*tan(estimated_states[1][0]), 0, 0, 0, 0, 0, 0,

        -estimated_states[5][0]*cos(estimated_states[0][0])-estimated_states[4][0]*sin(estimated_states[0][0]),0,0,0,cos(estimated_states[0][0]),-sin(estimated_states[0][0]),0,0,0,0,0,0,

        (estimated_states[4][0]*cos(estimated_states[0][0]))/cos(estimated_states[1][0])-(estimated_states[5][0]*sin(estimated_states[0][0]))/cos(estimated_states[1][0]),(estimated_states[5][0]*cos(estimated_states[0][0])*sin(estimated_states[1][0]))/pow(cos(estimated_states[1][0]),2) + (estimated_states[4][0]*sin(estimated_states[0][0])*sin(estimated_states[1][0]))/pow(cos(estimated_states[1][0]),2),0,0,sin(estimated_states[0][0])/cos(estimated_states[1][0]),cos(estimated_states[0][0])/cos(estimated_states[1][0]),0,0,0,0,0,0,

        0,0,0,0,(estimated_states[5][0]*(Iy - Iz))/Ix,(estimated_states[4][0]*(Iy - Iz))/Ix,0,0,0,0,0,0,

        0,0,0,-(estimated_states[5][0]*(Ix - Iz))/Iy,0, -(estimated_states[3][0]*(Ix - Iz))/Iy,0,0,0,0,0,0,

        0,0,0,(estimated_states[4][0]*(Ix - Iy))/Iz, (estimated_states[3][0]*(Ix - Iy))/Iz,0,0,0,0,0,0,0,

        0,0,0,0,0,0,0,0,0,1,0,0,

        0,0,0,0,0,0,0,0,0,0,1,0,

        0,0,0,0,0,0,0,0,0,0,0,1,
        (T*(cos(estimated_states[0][0])*sin(estimated_states[2][0]) - cos(estimated_states[2][0])*sin(estimated_states[0][0])*sin(estimated_states[1][0])))/m,(T*cos(estimated_states[0][0])*cos(estimated_states[1][0])*cos(estimated_states[2][0]))/m, (T*(cos(estimated_states[2][0])*sin(estimated_states[0][0]) - cos(estimated_states[0][0])*sin(estimated_states[1][0])*sin(estimated_states[2][0])))/m,0,0,0,0,0,0,-kd/m,0,0,

        -(T*(cos(estimated_states[0][0])*cos(estimated_states[2][0]) + sin(estimated_states[0][0])*sin(estimated_states[1][0])*sin(estimated_states[2][0])))/m,(T*cos(estimated_states[0][0])*cos(estimated_states[1][0])*sin(estimated_states[2][0]))/m,(T*(sin(estimated_states[0][0])*sin(estimated_states[2][0])+cos(estimated_states[0][0])*cos(estimated_states[2][0])*sin(estimated_states[1][0])))/m,0,0,0,0,0,0,0,-kd/m,0,

        -(T*cos(estimated_states[1][0])*sin(estimated_states[0][0]))/m,-(T*cos(estimated_states[0][0])*sin(estimated_states[1][0]))/m,0,0,0,0,0,0,0,0,0,-kd/m};


    double tau[3]={0,0,0};
    double Minv[3][3]={0,0,0,0,0,0,0,0,0};
    double rotm[3][3]={0,0,0,0,0,0,0,0,0};
    double TB[3][1]={0,0,0};

    inverseMatrix(R, Rinv);
    double angles[3]={estimated_states[0][0],estimated_states[1][0],estimated_states[2][0]};
    double angularSpeed[3][1]={estimated_states[3][0],
                               estimated_states[4][0],
                               estimated_states[5][0]};

    transforMinv(angles,Minv);
    eul2rotmat(angles, rotm);


    tau[0] = L*thrustf*((controls[2]+controls[1]) - (controls[0]+controls[3]));//(controls[1] - controls[3]);
    tau[1] = L*thrustf*((controls[0]+controls[2]) - (controls[1]+controls[3]));//(controls[2]-controls[0]);
    tau[2] = L*dragf*((controls[0]+controls[1]) - (controls[2]+controls[3]));//((controls[0]+controls[1]) - (controls[2]+controls[3]));

    double T2[3][1]={0,0,T};

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
    multiplyMatrices<12>(K, residuals, KRes);
    
    update_estimated_states(estimated_states,dxAngles,dxAngularSpeed,dxPosition,dxVelocity,KRes,dt);

    updateP(Pdot, P,dt);

    //for(int i =0;i<12;i++)
    //  for(int j =0;j<9;j++)
    //    PX4_ERR("P[%d][%d]: %f",i,j,P[i][j]);
    
    if((estimated_states[8][0]<=0))
        estimated_states[8][0]=0;

    if((estimated_states[8][0]<=0) && (estimated_states[11][0]<=0))
        estimated_states[11][0]=0;




    CUSUM[0]= std::abs(estimated_states[0][0]-sensors[0][0]);
    CUSUM[1]= std::abs(estimated_states[1][0]-sensors[1][0]);
    CUSUM[2]= std::abs(estimated_states[2][0]-sensors[2][0]);
    CUSUM[3]= std::abs(estimated_states[3][0]-sensors[3][0]);
    CUSUM[4]= std::abs(estimated_states[4][0]-sensors[4][0]);
    CUSUM[5]= std::abs(estimated_states[5][0]-sensors[5][0]);
    CUSUM[6]= std::abs(estimated_states[6][0]-sensors[6][0]);
    CUSUM[7]= std::abs(estimated_states[7][0]-sensors[7][0]);
    CUSUM[8]= std::abs(estimated_states[8][0]-sensors[8][0]);


/*
    PX4_ERR("DT: \%f",dt);
    PX4_ERR("Controls: \%f",controls[0]);
    PX4_ERR("Controls: \%f",controls[1]);
    PX4_ERR("Controls: \%f",controls[2]);
    PX4_ERR("Controls: \%f\n",controls[3]);

    PX4_ERR("Sensors: \%f",sensors[0][0]);
    PX4_ERR("Sensors: \%f",sensors[1][0]);
    PX4_ERR("Sensors: \%f",sensors[2][0]);
    PX4_ERR("Sensors: \%f",sensors[3][0]);
    PX4_ERR("Sensors: \%f",sensors[4][0]);
    PX4_ERR("Sensors: \%f",sensors[5][0]);
    PX4_ERR("Sensors: \%f",sensors[6][0]);
    PX4_ERR("Sensors: \%f",sensors[7][0]);
    PX4_ERR("Sensors: \%f\n",sensors[8][0]);

    PX4_ERR("Estimated states: \%f",estimated_states[0][0]);
    PX4_ERR("Estimated states: \%f",estimated_states[1][0]);
    PX4_ERR("Estimated states: \%f",estimated_states[2][0]);
    PX4_ERR("Estimated states: \%f",estimated_states[3][0]);
    PX4_ERR("Estimated states: \%f",estimated_states[4][0]);
    PX4_ERR("Estimated states: \%f",estimated_states[5][0]);
    PX4_ERR("Estimated states: \%f",estimated_states[6][0]);
    PX4_ERR("Estimated states: \%f",estimated_states[7][0]);
    PX4_ERR("Estimated states: \%f",estimated_states[8][0]);
    PX4_ERR("Estimated states: \%f",estimated_states[9][0]);
    PX4_ERR("Estimated states: \%f",estimated_states[10][0]);
    PX4_ERR("Estimated states: \%f\n",estimated_states[11][0]);


    */

dataFile<<std::fixed<<std::setprecision(9)<<CUSUM[0]<<"\n";
dataFile<<std::fixed<<std::setprecision(9)<<CUSUM[1]<<"\n";
dataFile<<std::fixed<<std::setprecision(9)<<CUSUM[2]<<"\n";
dataFile<<std::fixed<<std::setprecision(9)<<CUSUM[3]<<"\n";
dataFile<<std::fixed<<std::setprecision(9)<<CUSUM[4]<<"\n";
dataFile<<std::fixed<<std::setprecision(9)<<CUSUM[5]<<"\n";
dataFile<<std::fixed<<std::setprecision(9)<<CUSUM[6]<<"\n";
dataFile<<std::fixed<<std::setprecision(9)<<CUSUM[7]<<"\n";
dataFile<<std::fixed<<std::setprecision(9)<<CUSUM[8]<<"\n\n";

    PX4_ERR("Residual \%f",CUSUM[0]);
    PX4_ERR("Residual \%f",CUSUM[1]);
    PX4_ERR("Residual \%f",CUSUM[2]);
    PX4_ERR("Residual \%f",CUSUM[3]);
    PX4_ERR("Residual \%f",CUSUM[4]);
    PX4_ERR("Residual \%f",CUSUM[5]);
    PX4_ERR("Residual \%f",CUSUM[6]);
    PX4_ERR("Residual \%f",CUSUM[7]);
    PX4_ERR("Residual \%f\n",CUSUM[8]);









    PX4_ERR("***********************************************************************************");
}//end of function

bool attackDetected(){
    return false;
}



