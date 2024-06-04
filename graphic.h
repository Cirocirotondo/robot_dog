#ifndef GRAPHIC_H
#define GRAPHIC_H

#include "main.h"
#include "q_learning.h"


//-----------------------------------------------------
// GRAPHICAL CONSTANTS
//-----------------------------------------------------
#define LINE_SPACING        20
#define LINE_SPACING_SMALL  12

#define BORDER_COL          LIGHT_RED  
#define TITLE_COL           12
#define TEXT_COL            WHITE
#define TEXT_SEL_COL        YELLOW

// MENU CONSTANTS--------------------------------------
#define MARGIN				(double)(7)
#define MENU_BOX_W		    (double)(400)
#define MENU_BOX_H		    (double)(210)
#define MENU_BOX_X1         MARGIN
#define MENU_BOX_X2         MENU_BOX_X1 + MENU_BOX_W
#define MENU_BOX_Y1         MARGIN
#define MENU_BOX_Y2         MENU_BOX_Y1 + MENU_BOX_H
#define MENU_TITLE_X        MARGIN
#define MENU_TITLE_Y	    MARGIN
// WORLD BOX-------------------------------------------
#define WORLD_BOX_X1        MARGIN
#define WORLD_BOX_X2        XWIN - MARGIN
#define WORLD_BOX_W         WORLD_BOX_X2 - WORLD_BOX_X1   // width 
#define WORLD_BOX_Y1        (double)(MENU_BOX_Y2 + MARGIN)
#define WORLD_BOX_Y2        (double)(YWIN-MARGIN)
#define WORLD_BOX_H         WORLD_BOX_Y2 - WORLD_BOX_Y1   // height
#define GROUND_COL          6
#define WORLD_BKG           BLUE
#define WORLD_SHIFT         (double)(0.2)               // X-Shift in meters of the "x=0m" from the right border 
#define ROB_LINE_THICKNESS  10
// STATISTIC BOX---------------------------------------
#define STAT_BOX_X1         MENU_BOX_X2 + MARGIN
#define STAT_BOX_W          (double)(300)
#define STAT_BOX_X2         STAT_BOX_X1 + STAT_BOX_W
#define STAT_BOX_H          MENU_BOX_H
#define STAT_BOX_Y1         MENU_BOX_Y1
#define STAT_BOX_Y2         MENU_BOX_Y2
#define STAT_TITLE_X        MARGIN
#define STAT_TITLE_Y        MENU_TITLE_Y
// TASKS BOX---------------------------------------
#define TASK_BOX_X1         STAT_BOX_X2 + MARGIN
#define TASK_BOX_W          (double)(245)
#define TASK_BOX_X2         TASK_BOX_X1 + TASK_BOX_W
#define TASK_BOX_H          MENU_BOX_H
#define TASK_BOX_Y1         MENU_BOX_Y1
#define TASK_BOX_Y2         MENU_BOX_Y2
#define TASK_TITLE_X        MARGIN
#define TASK_TITLE_Y        MENU_TITLE_Y
// QTABLE_BOX
#define QTABLE_BOX_W        (double)(300)   
#define QTABLE_BOX_H        MENU_BOX_H //(double)(200)
#define QTABLE_BOX_X1       TASK_BOX_X2 + MARGIN
#define QTABLE_BOX_Y1       MENU_BOX_Y1
#define CELL_WIDTH          20
#define CELL_HEIGHT         20
#define QTABLE_X1           (double)(40)
#define QTABLE_Y1           (double)(12)
#define QTABLE_TITLE_Y      MENU_TITLE_Y


//-----------------------------------------------------
// GLOBAL VARIABLES
//-----------------------------------------------------
extern int end;
extern pthread_mutex_t mux_rob_st;
extern struct state_variables rob_st;
extern struct rob_vertexes rob_v;
extern int epoch;   // epoch of the qlearning training
extern double epsilon;      // Q-Learning epsilon
extern double episode_rewards;
extern int selected_task;


//-----------------------------------------------------
// LOCAL VARIABLES
//-----------------------------------------------------
BITMAP *screen_buff;
BITMAP *bitmap_world; 
BITMAP *bitmap_menu;
BITMAP *bitmap_stat;
BITMAP *bitmap_task;
BITMAP *bitmap_qtable;

struct rob_vertexes_pix{        // STRUCT with the coordinates of the vertexes of the robot
    int x, y;                   // centre of the robot body
    int v[8];                   // vertex of the robot body in the form {x1,y1,x2,y2,...} where (x1,y1) is the upper-right vertex and the other vertexes follow in counter-clockwise order
    int knee1x, knee1y;         // coordinates of the knee 1
    int foot1x, foot1y;         // coordinates of the foot 1
    int knee2x, knee2y;         // coordinates of the knee 2
    int foot2x, foot2y;         // coordinates of the foot 2
    int knee3x, knee3y;         // coordinates of the knee 3
    int foot3x, foot3y;         // coordinates of the foot 3
    int knee4x, knee4y;         // coordinates of the knee 4
    int foot4x, foot4y;         // coordinates of the foot 4
};
struct rob_vertexes_pix rob_v_pix;


//-----------------------------------------------------
// PUBLIC FUNCTIONS
//-----------------------------------------------------
void *graphic_task(void* arg);
void draw_menu();
void draw_stat();
void draw_world();
void draw_robot();
void draw_rotated_rectangle();
void compute_rob_ver();
void detect_ee_collision();
void draw_units();
void draw_task();
int choose_color(int curr_task_i);
void draw_qtable();
void thick_line(BITMAP *bmp, int x1, int y1, int x2, int y2, int color, int thickness);




//-------------------------------------------------------------
// DEBUG
//-------------------------------------------------------------
extern double px, py;
double px_pix, py_pix;
extern double rot_cent_x, rot_cent_y;

#endif