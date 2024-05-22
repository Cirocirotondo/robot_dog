#ifndef USER_H
#define USER_H

#include "main.h"
#include "ptask.h"
#include "q_learning.h"
#include "simulator.h"


//-----------------------------------------------------
// GLOBAL VARIABLES
//-----------------------------------------------------
extern int end;
extern int qlearn_active;               // Variable indicating whether the Q_learning is running or not
extern pthread_mutex_t mux_rob_st;
extern struct state_variables rob_st;
extern int selected_task;

//-----------------------------------------------------
// PUBLIC FUNCTIONS
//-----------------------------------------------------
void *user_input(void* arg);
char get_scancode();


#endif