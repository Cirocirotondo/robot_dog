//-----------------------------------------------------
// MAIN.C: Project for the course "Real Time Systems"
// https://www.youtube.com/watch?v=6afhNot8dIo
//-----------------------------------------------------

#ifndef MAIN_H
#define MAIN_H

#include <stdlib.h>     // include standard lib first
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <allegro.h>
#include <math.h>
#include "ptask.h"      // a lib for periodic tasks


//-------------------------------------------------------------
// GLOBAL CONSTANTS
//-------------------------------------------------------------

#define XWIN        (double)(1280)      // Window x resolution
#define YWIN        (double)(720)       // Window y resolution
#define BKG         0                   // Background color
#define PIX_M       400                 // Pixel/m retio (how many pixels to make one meter)
#define PI          3.14159265358979323846
//-------------------------------------------------------------
#define MAXT        10      // Max number of tasks
#define ACT         1       // Active flag
#define NOTACT      0       // Not Active flag
#define SLEN        80      // length of the text strings
#define GRAPHIC_T   30      // Period of the graphic task (ms)
#define SIMULATOR_T 10      // Period of the simulator task (ms)
#define USER_T      30      // Period of the user input task (ms)
#define QLEARN_T    200     // Period of the Q_Learning --> Must be higher than the simulator!
#define TSK_I_GRAPH 0       // Graphic task Index
#define TSK_I_SIM   1       // Simulation task Index
#define TSK_I_USR   2       // User_input task Index
#define TSK_I_Q     3       // Q_learning task index

//-------------------------------------------------------------
#define ROB_WIDTH           (double)(0.20)           // Robot body width (in meters)
#define ROB_HEIGHT          (double)(0.10)         // Robot body height (in meters)
#define ROB_WIDTH_PIX       ROB_WIDTH*PIX_M         // Robot body width (in pixels)
#define ROB_HEIGHT_PIX      ROB_HEIGHT*PIX_M        // Robot body height (in pixels)
#define ROB_WIDTH_sq        ROB_WIDTH*ROB_WIDTH     // Robot body width squadred
#define ROB_HEIGHT_sq       ROB_HEIGHT*ROB_HEIGHT   // Robot body height squadred
#define ROB_SEMIDIAG        0.5*sqrt(ROB_WIDTH_sq+ROB_HEIGHT_sq)
//#define ROB_GAMMA           atan(ROB_HEIGHT/ROB_WIDTH)            //angle from the diagonal to the base
#define ROB_COL             5                       // Robot colour     
#define ROB_L1              (double)(0.12)          // Length of the first segment of the arm (meters)
#define ROB_L2              (double)(0.10)          // Length of the second segment of the arm (meters)
#define ROB_L1_PIX          ROB_L1*PIX_M            // Length of the first segment of the arm (pixels)
#define ROB_L2_PIX          ROB_L2*PIX_M            // Length of the second segment of the arm (pixels)
#define THETA1_MIN          (double)(PI/3)
#define THETA1_MAX          (double)(2*PI/3)
#define NUM_SHOULDER_ANGLES 6
#define THETA1_INC          (double)((THETA1_MAX - THETA1_MIN)/(double)(NUM_SHOULDER_ANGLES))                     // Discretizatioin step of the servo motors
#define THETA2_MIN          (double)(PI/6)
#define THETA2_MAX          (double)(5*PI/6)
#define NUM_ELBOW_ANGLES    8
#define THETA2_INC          (double)((THETA2_MAX - THETA2_MIN)/(double)(NUM_ELBOW_ANGLES))                     // Discretizatioin step of the servo motors

#define ROB_MASS            2                       // Robot mass, in kg
#define ROB_INERTIA         ROB_MASS*(ROB_WIDTH_sq + ROB_HEIGHT_sq)/12      // Moment of inertia of the robot body, about the center of mass
#define ROB_INERTIA_V       ROB_INERTIA+ROB_MASS*ROB_SEMIDIAG               // Moment of inertia of the robot body, about one of the vertexes
#define ROB_START_X         (double)(0.1)
#define ROB_START_Y         (double)(FLOOR_H + ROB_HEIGHT/2)
#define ROB_START_THETA1    (double)(104.5*PI/180)
#define ROB_START_THETA2    (double)(111*PI/180)
#define NUM_ACTIONS         4       // Decrease/increas shoulder angle, dec/inc elbow angle



#define FLOOR_H_PIX     (double)(50)                      // Level of the floor
#define FLOOR_H         FLOOR_H_PIX/PIX_M       // Level of the floor

#define APPROX          (double)(0.001)                    // approximation bound



//-------------------------------------------------------------
// GLOBAL VARIABLES
//-------------------------------------------------------------
extern int     end;         // end flag
extern int qlearn_active;   // Variable indicating whether the Q_learning is running or not. = 0: qlearn inactive; = 1: train; = 2: run
extern int epoch;           // Q-Learning epoch index
extern double epsilon;      // Q-Learning epsilon
extern double episode_rewards;
extern int selected_task;   // Selected Task Index
extern double q_table[NUM_SHOULDER_ANGLES][NUM_ELBOW_ANGLES][NUM_ACTIONS];



struct state_variables{
    double x;        // x of the center of the body, in meters
    double y;        // y of the center of the body, in meters
    double vx;
    double vy;
    double alpha;    // angle of the body (radians)
    double w;        // omega: angular velocity of the body
    double theta1;   // angle of the first joint (radians)
    double w1;       // angular velocity of the first joint
    double theta2;   // angle of the second joint (radians)
    double w2;       // angular velocity of the second joint

    double one_contact_num_dt;
    int one_contact_first;

    double prev_ee_x;
    double prev_ee_y;

};
extern struct state_variables rob_st;       // Robot state
extern struct state_variables rob_st_prev;  // Robot state at the previous robot move


struct rob_vertexes{      // STRUCT with the coordinates of the vertexes of the robot
    double v[8];           // vertex of the robot body in the form {x1,y1,x2,y2,...} where (x1,y1) is the upper-right vertex and the other vertexes follow in counter-clockwise order
    double elb_x, elb_y;   // coordinates of the elbow
    double ee_x, ee_y;     // coordinates of the end effector 
};
extern struct rob_vertexes rob_v;



pthread_mutex_t mux_rob_st;     // mutex semaphore to protect the variable "rob_st"
pthread_mutex_t mux_rob_v;      // mutex semaphore to protect the variable "rob_v"


//-------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------
void init(void);
void init_rob_st();



//-------------------------------------------------------------
// DEBUG
//-------------------------------------------------------------
#define DEBUG       1
extern double rot_cent_x, rot_cent_y;
typedef struct {
    double x;
    double y;
} Point;

#endif