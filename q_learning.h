#ifndef Q_LEARNING_H
#define Q_LEARNING_H

#include "main.h"
#include "simulator.h"


//-------------------------------------------------------------
// CONSTANTS
//-------------------------------------------------------------
/* ACTIONS:
    - 0: Decrease shoulder angle    (v)
    - 1: Increase shoulder angle    (^)
    - 2: Decrease elbow angle       (<)
    - 3: Increase elbow angle       (>)
*/
#define LR                  0.1     // Learning Rate of the Q-Learning
#define DISCOUNT_FACTOR     0.9
#define EPISODES            2500
#define MAX_STEPS           100
#define INITIAL_EPSILON     1.0
#define MIN_EPSILON         0.1
#define EPSILON_DECAY       0.999   // Decay factor per episode
#define PENALITY_OOF        -50.0   // Penality for Out Of Bound movement
#define PENALITY_NM         -0.1    // Pensality for No Move   
#define REWARD_FACTOR       100.0    // Reward factor
#define REWARD_POS          30.0
#define SAVE_PATH           "q_table.dat"


extern int end;
extern pthread_mutex_t mux_rob_st;
extern struct state_variables rob_st;
extern struct state_variables rob_st_prev;

extern int epoch;
extern double epsilon;     // Exploration vs. exploitation trade-off
extern double episode_rewards;
extern int qlearn_active;
extern double q_table[NUM_SHOULDER_ANGLES][NUM_ELBOW_ANGLES][NUM_SHOULDER_ANGLES][NUM_ELBOW_ANGLES][NUM_ACTIONS];


//-----------------------------------------------------
// PUBLIC FUNCTIONS
//-----------------------------------------------------
void *q_learning_train(void* arg);
void *q_learning_run(void* arg);
void q_init();
int choose_action(int hipA_angle_i, int kneeA_angle_i, int hipB_angle_i, int kneeB_angle_i, double epsilon);
int choose_best_action(int hipA_angle_i, int kneeA_angle_i, int hipB_angle_i, int kneeB_angle_i);
int from_theta1_to_index(double theta1);
int from_theta2_to_index(double theta2);
double calculate_reward(int abort);
void save_q_table(const char *filename);
void load_q_table(const char *filename);
void printf_qtable();

#endif