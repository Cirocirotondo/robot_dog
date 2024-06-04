#include "main.h"
#include "graphic.h"
#include "simulator.h"
#include "user_input.h"

//-------------------------------------------------------------
// GLOBAL VARIABLES DECLARATIONS
//-------------------------------------------------------------
int end = 0;
int qlearn_active = 0;
struct state_variables rob_st;
struct state_variables rob_st_prev;
struct rob_vertexes rob_v;
pthread_mutex_t mux_rob_st = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mux_rob_v = PTHREAD_MUTEX_INITIALIZER;
int epoch = 0;
double epsilon = 0;
double episode_rewards = 0;
int selected_task = 0;
double q_table[NUM_SHOULDER_ANGLES][NUM_ELBOW_ANGLES][NUM_SHOULDER_ANGLES][NUM_ELBOW_ANGLES][NUM_ACTIONS];


int main(void) {
    int i = 0;
    char scan;
    init_rob_st();
    init();

    task_create(graphic_task, TSK_I_GRAPH, GRAPHIC_T, GRAPHIC_T, 50, ACT);      // graphic task is task 0
    task_create(simulator, TSK_I_SIM, SIMULATOR_T, SIMULATOR_T, 50, ACT);       // simulator task is task 1
    task_create(user_input, TSK_I_USR, USER_T, USER_T, 50, ACT);                // user_input task is task 2

    for (i=0; i<MAXT; i++) wait_for_task_end(i);

    allegro_exit();
    return 0;
    
}


//-------------------------------------------------------------
// INIT: initializes allegro and ptask
//-------------------------------------------------------------
void init(void) {
    char s[SLEN];
    allegro_init();
    set_gfx_mode(GFX_AUTODETECT_WINDOWED, XWIN, YWIN, 0, 0);
    clear_to_color(screen, BKG);
    install_keyboard();
    ptask_init(SCHED_OTHER);

    sprintf(s, "Simulation of the robot!");
    textout_ex(screen, font, s, 10, 10, 14, BKG);
}

//-------------------------------------------------------------
// INIT_ROB_ST: initializes the robot state
//-------------------------------------------------------------
void init_rob_st() {
    rob_st.x = ROB_START_X;
    rob_st.y = ROB_START_Y;
    rob_st.thetaA1 = ROB_START_THETA_A1;
    rob_st.thetaA2 = ROB_START_THETA_A2;
    rob_st.thetaB1 = ROB_START_THETA_B1;
    rob_st.thetaB2 = ROB_START_THETA_B2;
    rob_st.prev_foot_2x = ROB_START_X - ROB_WIDTH/2;

    rob_st.one_contact_first = 1;

    rob_st_prev = rob_st;
}


