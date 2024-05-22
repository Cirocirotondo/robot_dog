#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <math.h>
#include "main.h"
#include "ptask.h"



//-------------------------------------------------------------
// CONSTANTS
//-------------------------------------------------------------
#define GRAVITY     (double)(-1)//-9.81
#define PERIOD_S    SIMULATOR_T/1000    // task period in seconds
#define DUMP        0.5                 // Dump constant

//-----------------------------------------------------


//-----------------------------------------------------
// GLOBAL VARIABLES
//-----------------------------------------------------
extern int end;
extern pthread_mutex_t mux_rob_st;
extern struct state_variables rob_st;
extern struct state_variables rob_st_prev;


//-----------------------------------------------------
// PUBLIC FUNCTIONS
//-----------------------------------------------------
void *simulator(void* arg);
void print_rob_st();
void compute_rob_v();
void handle_gravity();
void handle_ground_contact();
void handle_ee_collision();
void rotate_point_around_vertex(double *x, double *y, double xc, double yc, double alpha);
int find_num_contacts();
int move_robot(int action);


//-------------------------------------------------------------
// DEBUG
//-------------------------------------------------------------
extern double px,py;
extern double rot_cent_x, rot_cent_y;


#endif