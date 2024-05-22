#include "ptask.h"

//------------------------------------------------------------------------
// TIME_COPY: copies a source time variable ts in a destination variable
// pointed by td
//------------------------------------------------------------------------

void time_copy(struct timespec *td, struct timespec ts) {

    td->tv_sec = ts.tv_sec;
    td->tv_nsec = ts.tv_nsec;
}

//---------------------------------------------------------------------------------------------
// TIME_ADD_MS: adds a value ms expressed in milliseconds to the time variable pointed by t
//---------------------------------------------------------------------------------------------

void time_add_ms(struct timespec *t, int ms) {

    t->tv_sec += ms / 1000;
    t->tv_nsec += (ms%1000) * 1000000;

    if (t->tv_nsec > 1000000000) {
        t->tv_nsec -= 1000000000;
        t->tv_sec += 1;
    }
}

//---------------------------------------------------------------------
// TIME_CMP: compares two time variables t1 and t2 and returns 0 if
// they are equal, 1 if t1>t2, -1 if t1<t2
//---------------------------------------------------------------------

int time_cmp(struct timespec t1, struct timespec t2) {

    if (t1.tv_sec > t2.tv_sec) return 1;
    if (t1.tv_sec < t2.tv_sec) return -1;
    if (t1.tv_nsec > t2.tv_nsec) return 1;
    if (t1.tv_nsec < t2.tv_nsec) return -1;
    return 0;
}


//---------------------------------------------------------------------------------------------
// PTASK_INIT: initializes the task, setting the system start time and the scheduler
//---------------------------------------------------------------------------------------------
void ptask_init(int policy){
    int i;
    ptask_policy = policy;
    clock_gettime(CLOCK_MONOTONIC, &ptask_t0);

    // initialize activation semaphores
    for (i=0; i<MAX_TASKS; i++)
        sem_init(&tp[i].asem, 0, 0);
}

//---------------------------------------------------------------------------------------------
// GET_SYSTIME: returns the current elapsed time since ptask_t0
//---------------------------------------------------------------------------------------------
long get_systime(int unit){
    struct timespec t;
    long tu, mul, div;
    switch (unit) {
        case MICRO: mul = 1000000; div = 1000; break;
        case MILLI: mul = 1000; div = 1000000; break;
        default:    mul = 1000; div = 1000000; break;
    }
    clock_gettime(CLOCK_MONOTONIC, &t);
    tu = (t.tv_sec - ptask_t0.tv_sec)*mul;
    tu += (t.tv_nsec - ptask_t0.tv_nsec)/div;
    return tu;
}

//---------------------------------------------------------------------------------------------
// TASK_CREATE: stores all the specified task parameters in the task_par structure, set the 
// attributes and call the pthread_create function.
//---------------------------------------------------------------------------------------------

int task_create(void* (*task)(void *), int i, int period, int drel, int prio, int aflag) {
    pthread_attr_t myatt;
    struct sched_param mypar;
    int tret;           // return of the pthread_create function
    if (i >= MAX_TASKS) return -1;
    tp[i].arg = i;
    tp[i].period = period;
    tp[i].deadline = drel;
    tp[i].priority = prio;
    tp[i].dmiss = 0;
    pthread_attr_init(&myatt);
    pthread_attr_setinheritsched(&myatt, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&myatt, ptask_policy);
    mypar.sched_priority = tp[i].priority;
    pthread_attr_setschedparam(&myatt, &mypar);
    tret = pthread_create(&tp[i].tid, &myatt, task, (void*)(&tp[i]));
    if (aflag == ACT) task_activate(i);
    return tret;

}
//---------------------------------------------------------------------------------------------
// GET_TASK_INDEX: Retrieves the task index stored in tp->arg
//---------------------------------------------------------------------------------------------

int get_task_index(void* arg){
    struct task_par *tpar;
    tpar = (struct task_par *) arg;
    return tpar->arg;
}

//---------------------------------------------------------------------------------------------
// SET_PERIOD: sets the variables at (activation time) and dl (deadline) of the 
// next cycle.
//---------------------------------------------------------------------------------------------
void set_period(int i){
    struct timespec t;
    sem_wait(&tp[i].asem);
    clock_gettime(CLOCK_MONOTONIC, &t);
    time_copy(&(tp[i].at), t);
    time_copy(&(tp[i].dl), t);
    time_add_ms(&(tp[i].at), tp[i].period);
    time_add_ms(&(tp[i].dl), tp[i].deadline);
}


void task_activate(int i){
    sem_post(&tp[i].asem);
}

//---------------------------------------------------------------------------------------------
// DEADLINE_MISS: It checks if there was a deadline miss and, in case, increments the counter
//---------------------------------------------------------------------------------------------
int deadline_miss(int i){
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (time_cmp(now, tp[i].dl) > 0) {
        tp[i].dmiss++;
        return 1;
    }
    return 0;
}

//---------------------------------------------------------------------------------------------
// WAIT_FOR_PERIOD: Suspends the calling thread until the next activation and, when
// awaken, updates activation time and deadline
//---------------------------------------------------------------------------------------------
void wait_for_period(int i){
    deadline_miss(i);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(tp[i].at), NULL);
    time_add_ms(&(tp[i].at), tp[i].period);
    time_add_ms(&(tp[i].dl), tp[i].period);
}

//---------------------------------------------------------------------------------------------
// TASK_SET_PERIOD: can be called to change the period of a task
//---------------------------------------------------------------------------------------------
void task_set_period(int i, int per){
    if (per > 0)
        tp[i].period = per;
}

//---------------------------------------------------------------------------------------------
// TASK_SET_DEADLINE: can be called to change the deadline of a task
//---------------------------------------------------------------------------------------------
void task_set_deadline(int i, int dline){
    if (dline > 0)
        tp[i].deadline = dline;
}

//---------------------------------------------------------------------------------------------
// GET_TASK_PERIOD: returns the period of a task
//---------------------------------------------------------------------------------------------
int task_get_period(int i){
    return tp[i].period;
}

//---------------------------------------------------------------------------------------------
// TASK_DEADLINE: returns the deadline of a task
//---------------------------------------------------------------------------------------------
int task_get_deadline(int i){
    return tp[i].deadline;
}

//---------------------------------------------------------------------------------------------
// Other functions to get the task parameters
//---------------------------------------------------------------------------------------------
int task_get_dmiss(int i){ 
    return tp[i].dmiss;
}
void task_get_atime(int i, struct timespec *at)
{ 
    at->tv_sec = tp[i].at.tv_sec;
    at->tv_nsec = tp[i].at.tv_nsec;
}
void task_get_adline(int i, struct timespec *dl){ 
    dl->tv_sec = tp[i].dl.tv_sec;
    dl->tv_nsec = tp[i].dl.tv_nsec;
}

//---------------------------------------------------------------------------------------------
// WAIT_FOR_TASK_END: function to make the program wait for the end of the tasks before closing
//---------------------------------------------------------------------------------------------
void wait_for_task_end(int i){
    pthread_join(tp[i].tid, NULL);
}