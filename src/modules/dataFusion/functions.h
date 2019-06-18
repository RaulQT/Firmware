//
//  MatricesFunctions.h
//  DroneReferenceMonitor
//
//  Created by Raul on 1/13/19.
//  Copyright © 2019 Raul. All rights reserved.
//

#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include <stddef.h>
struct Imu{
    //accelerometer
    double ay;
    double ax;
    double az;
    
    //Gyroscope
    double gx;
    double gy;
    double gz;
    
    //magnetometer
    double mx;
    double my;
    double mz;
};

struct AngleInfo{
    double angle;
    double angularSpeed;
};

struct F{
    double Fk[3][3];
};
bool attackDetected();
void llaTOxyz(double lla[3],double ll0[2],double alt, double * array);//GPS convertion
//data fusion
AngleInfo angleDataFusionRoll(double y, double u, double R, double Q [2][2], double dt);
AngleInfo angleDataFusionPitch(double y, double u, double R, double Q [2][2], double dt);
AngleInfo angleDataFusionYaw(double y, double u, double R, double Q [2][2], double dt);

//EKF
bool attackDetected();
void EKF_CALL(double contr[4]);
void EKF(double sensors[9][1], double controls[4],double dt);
void EKF_Fk(double x[12][1],   double Ix, double Iy, double Iz,  double T, double m,  double kd, double Fk[12][12]);
void EKF_Hk(double Hk[9][12]);
void eul2rotmat(double angles[3],double rotmatrix[3][3]);
void transforMinv(double angles[3],double Minv_out[3][3]);
void update_estimated_states(double estimated_states[12][1],double dxangles[3][1],double dxangularspeeds[3],double dxposition[3],double dxvelocity[3],double KRes[12][1],double dt);
void updateP(double Pdot[12][12],double P[12][12],double dt);

//initialize
template<size_t row,size_t col> void initializeMatrix(double array[row][col]){
    for(size_t i =0;i<row;i++)
        for(size_t j =0;j<col;j++)
            array[i][j]=0;
}

//addition
template<size_t row, size_t col> void addMatrices(double arrayA[row][col],double arrayB[row][col], double arrayC[row][col]){
    for (size_t i=0;i<row;i++)
        for(size_t j =0;j<col;j++)
            arrayC[i][j]=arrayA[i][j]+arrayB[i][j];
}

//subtraction
template<size_t rows, size_t cols> void subtractMatrices(double arrayA[rows][cols],double arrayB[rows][cols], double arrayC[rows][cols]){
    for (size_t i=0;i<rows;i++)
        for(size_t j =0;j<cols;j++)
            arrayC[i][j]=arrayA[i][j]-arrayB[i][j];
}

//multiplication
template<size_t row, size_t col,size_t col2> void multiplyMatrices(double arrayA[row][col],double arrayB[col][col2], double arrayC[row][col2]){
    for (size_t i=0;i<row;i++)
        for(size_t j =0;j<col2;j++)
            for(size_t k=0;k<col;k++)
                arrayC[i][j]+=arrayA[i][k]*arrayB[k][j];
}

//transpose
template<size_t rows, size_t cols> void transposeMatrix(double arrayA[rows][cols],double result[cols][rows]){
    for (size_t i=0;i<rows;i++)
        for(size_t j =0;j<cols;j++)
            result[j][i]=arrayA[i][j];
}

//inverse
template<size_t size> void inverseMatrix(double array[size][size],double arrayReturn[size][size]){

    double arrayA[size][size*2];
    for(size_t i =0;i<size;i++)
        for(size_t j=0;j<size;j++)
            arrayA[i][j]=array[i][j];
    
    double a=0;
    double ratio =0;
    size_t n=size;
    
    for(size_t i = 0; i < n; i++){
        for(size_t j = n; j < 2*n; j++){
            if(i==(j-n))
                arrayA[i][j] = 1.0;
            else
                arrayA[i][j] = 0.0;
        }
    }
    for(size_t i = 0; i < n; i++){
        for(size_t j = 0; j < n; j++){
            if(i!=j){
                ratio = arrayA[j][i]/arrayA[i][i];
                for(size_t k = 0; k < 2*n; k++){
                    arrayA[j][k] -= ratio * arrayA[i][k];
                }
            }
        }
    }
    for(size_t i = 0; i < n; i++){
        a = arrayA[i][i];
        for(size_t j = 0; j < 2*n; j++){
            arrayA[i][j] /= a;
        }
    }
    
    for(size_t i=0;i<size;i++){
        for(size_t j=size;j<size*2;j++){
            arrayReturn[i][j-n]=arrayA[i][j];
        }
    }

}
#endif /* MatricesFunctions_h */
