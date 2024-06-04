//-------------------------------------------------------------
// USER.C:   Task to manage the user inputs
//-------------------------------------------------------------
#include "user_input.h"


void *user_input(void* arg){

    int i;              // task index
    char scan;          // key pressed
    int per;
    i = get_task_index(arg);
    set_period(i);

    do{
        //printf("USER INPUT\n");
        scan = get_scancode();

        switch (scan) {
            case KEY_LEFT :
                move_robot(0);
                break;
                
            case KEY_RIGHT:
                move_robot(1);
                break;

            case KEY_UP:
                move_robot(2);
                break;

            case KEY_DOWN:
                move_robot(3);
                break;

            case KEY_A:
                move_robot(4);
                break;
                
            case KEY_D:
                move_robot(5);
                break;

            case KEY_W:
                move_robot(6);
                break;

            case KEY_S:
                move_robot(7);
                break;

            case KEY_SPACE:
                if (!qlearn_active) {
                    qlearn_active = 1;
                    task_create(q_learning_train, TSK_I_Q, QLEARN_T, QLEARN_T, 50, ACT);      // graphic task is task 0
                } else if(qlearn_active == 1) {
                    qlearn_active = 0;
                }
                break;

            case KEY_ENTER:
                if (!qlearn_active) {
                    qlearn_active = 2;
                    task_create(q_learning_run, TSK_I_Q, QLEARN_T, QLEARN_T, 50, ACT);      // graphic task is task 0
                } else if(qlearn_active == 2) {
                    qlearn_active = 0;
                }
                break;

            case KEY_1:
                if (selected_task > 0)
                    selected_task--;
                break;
            case KEY_2:
                if (selected_task < 3)
                    selected_task++;
                break;
            case KEY_Q:     // Decrease selected task period
                per = task_get_period(selected_task);
                task_set_period(selected_task, per - 1);
                task_set_deadline(selected_task, per - 1);
                break;
            case KEY_E:     // Increase selected task period
                per = task_get_period(selected_task);
                task_set_period(selected_task, per + 2);
                task_set_deadline(selected_task, per + 2);
                break;
            case KEY_R:     // Reset periods
                init_rob_st();

                task_set_period(TSK_I_GRAPH, GRAPHIC_T);
                task_set_period(TSK_I_SIM, SIMULATOR_T);
                task_set_period(TSK_I_USR, USER_T);
                task_set_period(TSK_I_Q, QLEARN_T);
                break;

            default: break;
        }

        wait_for_period(i);
    } while (!key[KEY_ESC] && end == 0);
    end = 1;

};


//----------------------------------------------------
// GET_SCANCODE: returns the scancode of a pressed key
//----------------------------------------------------
char get_scancode(){
    if (keypressed())
        return readkey() >> 8;
    else return 0;
}
