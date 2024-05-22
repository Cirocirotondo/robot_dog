#ifndef PTASK_H
#define PTASK_H

/* Includes */
#include <stdlib.h> // include standard lib first
#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>


/* Defines */
#define     MAX_TASKS   10
#define     MICRO       0
#define     MILLI       1
#define     ACT         1
#define     NOACT       0

struct timespec ptask_t0;   // system start time
int ptask_policy;           // scheduler

struct task_par{
    int     arg;        // task argument
    long    wcet;       // task WCET in us ( WCET = Worst Case Execution Time )  
    int period;         // task period in ms
    int deadline;       // relative deadline in ms
    int priority;       // task priority in [0,99]
    int dmiss;          // # of deadline misses
    struct timespec at; // next activation time
    struct timespec dl; // current abs. deadline
    pthread_t tid;      // thread id
    sem_t asem;         // activation semaphore
};
struct task_par tp[MAX_TASKS];



//-------------------------------------------------------------
// FUNCTION DECLARATIONS
//-------------------------------------------------------------
void time_copy(struct timespec *td, struct timespec ts);
void time_add_ms(struct timespec *t, int ms);
int time_cmp(struct timespec t1, struct timespec t2);
void ptask_init(int policy);
long get_systime(int unit);
int task_create(void* (*task)(void *), int i, int period, int drel, int prio, int aflag);
int get_task_index(void* arg);
void set_period(int i);
void task_activate(int i);
int deadline_miss(int i);
void wait_for_period(int i);
void task_set_period(int i, int per);
void task_set_deadline(int i, int dline);
int task_get_period(int i);
int task_get_deadline(int i);
int task_get_dmiss(int i);
void task_get_atime(int i, struct timespec *at);
void task_get_adline(int i, struct timespec *dl);
void wait_for_task_end(int i);

#endif