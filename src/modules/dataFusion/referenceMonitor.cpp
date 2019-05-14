/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <cmath>
#include <uORB/topics/parameter_update.h>
#include "referenceMonitor.h"
#include "functions.h"
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <drivers/drv_hrt.h>

/*global variables needed for EKF*/
double controls[4]={0,0,0,0};
double sensors[9][1]={0,0,0,0,0,0,0,0,0};
double dt=0;


int ReferenceMonitor::print_usage(const char *reason){
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
                R"DESCR_STR(
                ### Description
                Section that describes the provided module functionality.

                module running as a task in the background with start/stop/status functionality.

                ### Implementation
                Section describing the high-level implementation of this module.

                ### Examples
                CLI usage example:
                $ referenceMonitor start -f -p 42

                )DESCR_STR");

    PRINT_MODULE_USAGE_NAME("referenceMonitor", "template");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
    PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int ReferenceMonitor::print_status(){
    PX4_INFO("Running");
    // TODO: print additional runtime information about the state of the module

    return 0;
}

int ReferenceMonitor::custom_command(int argc, char *argv[]){
    /*
        if (!is_running()) {
                print_usage("not running");
                return 1;
        }

        // additional custom commands can be handled like this:
        if (!strcmp(argv[0], "do-something")) {
                get_instance()->do_something();
                return 0;
        }
         */

    return print_usage("unknown command");
}

int ReferenceMonitor::task_spawn(int argc, char *argv[]){
    _task_id = px4_task_spawn_cmd("referenceMonitor",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  5000,                             //stack size
                                  (px4_main_t)&run_trampoline,      //entry
                                  (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

ReferenceMonitor *ReferenceMonitor::instantiate(int argc, char *argv[]){
    int example_param = 0;
    bool example_flag = false;
    bool error_flag = false;

    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    // parse CLI arguments
    while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
        case 'p':
            example_param = (int)strtol(myoptarg, nullptr, 10);
            break;

        case 'f':
            example_flag = true;
            break;

        case '?':
            error_flag = true;
            break;

        default:
            PX4_WARN("unrecognized flag");
            error_flag = true;
            break;
        }
    }

    if (error_flag) {
        return nullptr;
    }

    ReferenceMonitor *instance = new ReferenceMonitor(example_param, example_flag);

    if (instance == nullptr) {
        PX4_ERR("alloc failed");
    }

    return instance;
}

ReferenceMonitor::ReferenceMonitor(int example_param, bool example_flag) : ModuleParams(nullptr){
}

void ReferenceMonitor::run(){
   // PX4_INFO("Here 1");
    //subscribe to topic needed
    int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
    int vehicle_magnetometer_sub = orb_subscribe(ORB_ID(vehicle_magnetometer));
    int vehicle_gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));

    //initialize parameters
    int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
    parameters_update(parameter_update_sub, true);

    //structures needed
    struct sensor_combined_s raw;
    struct vehicle_magnetometer_s magnetometer;
    struct vehicle_gps_position_s gps;

    //struct vehicle_air_data_s airdata;
    //struct actuator_outputs_s act;

    struct Imu imuRaw;
    struct AngleInfo roll_angle;
    struct AngleInfo pitch_angle;
    struct AngleInfo yaw_angle;

    /*parameters needed for data fusion function*/
    double m1;          //intermediate variables for yaw calculation
    double m2;
    double roll_acc;    //estimated roll angle from accelerometer
    double pitch_acc;   //estimated pitch angle from accelerometer
    double yaw_mag;     //estimated yaw angle from magnetometer
    double y;
    double u;
    double R;
    double Q[2][2]={0.01,0,
                    0,0.01};


    double currentGPS[3]={0,0,0};
    double correctedGPS[3]={0,0,0};
    double initialGPS[2]={0,0};
    double initialAlt=0;

    double currentTime=0;
    double previousTime =0;
    int executeOnce = 0;
    /*constants values adjusted*/
    dt =.02;
    R=.1;

    //time measurements
    hrt_abstime startTimeDF;
    hrt_abstime endTimeDF;

    /* limit the update rate to 5 Hz */
    orb_set_interval(sensor_combined_sub, 200);
    orb_set_interval(vehicle_magnetometer_sub, 200);
    orb_set_interval(vehicle_gps_position_sub, 200);

    //std::fstream dataFile("timeDataFustion.txt",std::ios::out);
    //std::chrono::high_resolution_clock::time_point t1;
    //std::chrono::high_resolution_clock::time_point t2;
    //long long duration =0;

    //initialized one by one to avoid issues in the drone
    px4_pollfd_struct_t fds[3];
    fds[0].fd= sensor_combined_sub;
    fds[0].events= POLLIN;
    fds[0].revents= 0;
    fds[0].sem= NULL;
    fds[0].priv= NULL;


    fds[1].fd= vehicle_magnetometer_sub;
    fds[1].events= POLLIN;
    fds[1].revents= 0;
    fds[1].sem= NULL;
    fds[1].priv= NULL;

    fds[2].fd= vehicle_gps_position_sub;
    fds[2].events= POLLIN;
    fds[2].revents= 0;
    fds[2].sem= NULL;
    fds[2].priv= NULL;

   // PX4_INFO("Here 2");
    while (!should_exit()) {
       //PX4_INFO("ReferenceMonitor");

       usleep(200000);



        // wait for up to 1000ms for data
        int poll_ret = px4_poll(fds, 3, 1000);
startTimeDF= hrt_absolute_time();

        if (poll_ret == 0) {
           // PX4_INFO("Here timeout");
            // Timeout: let the loop run anyway, don't do `continue` here
            continue;

        } else if (poll_ret < 0) {
            //PX4_INFO("Here ERROR");
            // this is undesirable but not much we can do
            PX4_ERR("poll error %d, %d", poll_ret, errno);
            usleep(50000);
            continue;

        } else{


            //PX4_INFO("Here else");
            //usleep(10000);

            //dataFile<<"Available topics "<<poll_ret<<"\n";
            if (fds[0].revents & POLLIN) {
                //PX4_INFO("sensors combined executes: %llu us",hrt_absolute_time());
                //dataFile<<"sensors combined"<<"\n";
                // copy sensors raw data into local buffer
                // accelerometer and gyroscope
                orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &raw);
                currentTime=raw.timestamp;
                dt=(currentTime-previousTime)/1000000.0;
                previousTime=currentTime;
            }


            if (fds[1].revents & POLLIN){
                //PX4_INFO("magnetometer executes: %llu us",hrt_absolute_time());
                //dataFile<<"magnetometer"<<"\n";
                // copy magnetometer data into local buffer
                // magnetometer
                orb_copy(ORB_ID(vehicle_magnetometer), vehicle_magnetometer_sub, &magnetometer);
                currentTime=magnetometer.timestamp;
            }

            if (fds[2].revents & POLLIN){
                //PX4_INFO("GPS executes: %llu us",hrt_absolute_time());
                //dataFile<<"gps"<<"\n";
                // copy GPS data into local buffer
                // gps
                orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_position_sub, &gps);
                currentTime=gps.timestamp;

                if(executeOnce==0){
                    initialGPS[0]= 32.9807568;          //(double)gps.lat/10000000;
                    initialGPS[1]= -96.7547627;         //(double)gps.lon/10000000;
                    initialAlt =  204;                  //(double)gps.alt/1000;
                    executeOnce++;
                    if(initialGPS[0]>3){
                        executeOnce++;
                        initialAlt++;
                    }

                }
            }



            //PX4_INFO("Here gathering sensor info, sleep 1s");

            // gathering sensor information
            imuRaw.ax = (double)raw.accelerometer_m_s2[0];
            imuRaw.ay = (double)raw.accelerometer_m_s2[1];
            imuRaw.az = (double)raw.accelerometer_m_s2[2];

            imuRaw.gx= (double)raw.gyro_rad[0];
            imuRaw.gy= (double)raw.gyro_rad[1];
            imuRaw.gz= (double)raw.gyro_rad[2];

            imuRaw.mx =(double)magnetometer.magnetometer_ga[0];
            imuRaw.my =(double)magnetometer.magnetometer_ga[1];
            imuRaw.mz =(double)magnetometer.magnetometer_ga[2];

            currentGPS[0]= 32.9807568;      //(double)gps.lat/10000000;
            currentGPS[1]= -96.7547627;     //(double)gps.lon/10000000;
            currentGPS[2] = 204;            //(double)gps.alt/1000;

            llaTOxyz(currentGPS,initialGPS, initialAlt, correctedGPS);

            //Roll angle
            roll_acc=atan(imuRaw.ay/imuRaw.az);
            u=imuRaw.gx;
            y=roll_acc;
            roll_angle= angleDataFusionRoll(y, u, R, Q, dt);

            //Pitch angle
            pitch_acc=atan(imuRaw.ax/imuRaw.az);
            u=imuRaw.gy;
            y=pitch_acc;
            pitch_angle= angleDataFusionPitch(y, u, R, Q, dt);

            //yaw angle
            m1=imuRaw.mx*cos(pitch_angle.angle)+imuRaw.my*sin(pitch_angle.angle)*sin(roll_angle.angle)- imuRaw.mz*cos(roll_angle.angle)*sin(pitch_angle.angle);
            m2=imuRaw.my*cos(roll_angle.angle)+imuRaw.mz*sin(roll_angle.angle);
            yaw_mag=-atan2(m2, m1);
            u=imuRaw.gz;
            y=yaw_mag;
            yaw_angle= angleDataFusionYaw(y, u, R, Q, dt);

            sensors[0][0]=roll_angle.angle;
            sensors[1][0]=roll_angle.angularSpeed;
            sensors[2][0]=pitch_angle.angle;
            sensors[3][0]=pitch_angle.angularSpeed;
            sensors[4][0]=yaw_angle.angle;
            sensors[5][0]=yaw_angle.angularSpeed;
            sensors[6][0]=correctedGPS[0];
            sensors[7][0]=correctedGPS[1];
            sensors[8][0]=correctedGPS[2];





            endTimeDF = hrt_absolute_time();
            PX4_ERR("DF time: %llu us", endTimeDF-startTimeDF);

            parameters_update(parameter_update_sub);

        }//end else


    }//end while


    orb_unsubscribe(sensor_combined_sub);
    orb_unsubscribe(vehicle_magnetometer_sub);
    orb_unsubscribe(vehicle_gps_position_sub);
    orb_unsubscribe(parameter_update_sub);

}




void EKF_CALL(double contr[4]){
    EKF(sensors,contr,dt);
}

void ReferenceMonitor::parameters_update(int parameter_update_sub, bool force){
    bool updated;
    struct parameter_update_s param_upd;

    orb_check(parameter_update_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
    }

    if (force || updated) {
        updateParams();
    }
}

int referenceMonitor_main(int argc, char *argv[]){
    return ReferenceMonitor::main(argc, argv);
}
