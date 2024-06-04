//--------------------------------------------------------------------------------
// GRAPHIC.C:   Functions related to graphic
//--------------------------------------------------------------------------------
#include "graphic.h"

void *graphic_task(void* arg){
    int i;              // task index
    i = get_task_index(arg);

    // initializing screen bitmaps
	screen_buff = create_bitmap(XWIN, YWIN);    // bitmap on which all the subwindows are printed
    clear_bitmap(screen_buff);
    bitmap_world = create_bitmap(WORLD_BOX_W, WORLD_BOX_H);
    bitmap_menu = create_bitmap(MENU_BOX_W, MENU_BOX_H);
    draw_menu();
    bitmap_stat = create_bitmap(STAT_BOX_W, STAT_BOX_H);
    bitmap_task = create_bitmap(TASK_BOX_W,TASK_BOX_H);
    bitmap_qtable = create_bitmap(QTABLE_BOX_W,QTABLE_BOX_H);


    set_period(i);
    while (!end) {

        //printf("GRAPHIC -> world_box_y1 = %f, world_box_y2 = %f, world_box_h = %f\n", WORLD_BOX_Y1, WORLD_BOX_Y2, WORLD_BOX_H);
        draw_world();
        draw_stat();
        draw_task();
        draw_qtable();
        blit(screen_buff, screen, 0, 0, 0, 0, screen_buff->w, screen_buff->h);
 
        wait_for_period(i);
    }
};


//--------------------------------------------------------------------------------
// DRAW_MENU: Draws on the screen_buff the box representing the menu. The text
// created by this function is STATIC, so the function is called only once.
//--------------------------------------------------------------------------------
void draw_menu() {
    char str[SLEN]; // for text outputs
    int curr_y = MENU_TITLE_Y;

    rect(bitmap_menu, 0, MENU_BOX_H-1, MENU_BOX_W-1, 0, BORDER_COL);    // border

    sprintf(str, "THE CRAWLING ROBOT");
    textout_ex(bitmap_menu, font, str, MENU_TITLE_X, curr_y, TITLE_COL, -1);

    curr_y += LINE_SPACING;
    sprintf(str, "List of available commands:");
    textout_ex(bitmap_menu, font, str, MENU_TITLE_X, curr_y, TEXT_SEL_COL, -1);

    curr_y += LINE_SPACING;
    sprintf(str, "UP/DOWN:      Move first joint (shoulder)");
    textout_ex(bitmap_menu, font, str, MENU_TITLE_X, curr_y, TEXT_COL, -1);

    curr_y += LINE_SPACING;
    sprintf(str, "RIGHT/LEFT:   Move second joint (elbow)");
    textout_ex(bitmap_menu, font, str, MENU_TITLE_X, curr_y, TEXT_COL, -1);

    curr_y += LINE_SPACING;
    sprintf(str, "1/2:          Switch task selection");
    textout_ex(bitmap_menu, font, str, MENU_TITLE_X, curr_y, TEXT_COL, -1);

    curr_y += LINE_SPACING;
    sprintf(str, "Q/E:          Increase/Decrease task period");
    textout_ex(bitmap_menu, font, str, MENU_TITLE_X, curr_y, TEXT_COL, -1);

    curr_y += LINE_SPACING;
    sprintf(str, "SPACE:        Start/Stop Q_Learning training");
    textout_ex(bitmap_menu, font, str, MENU_TITLE_X, curr_y, TEXT_COL, -1);

    curr_y += LINE_SPACING;
    sprintf(str, "ENTER:        Start/Stop Q_Learning inference");
    textout_ex(bitmap_menu, font, str, MENU_TITLE_X, curr_y, TEXT_COL, -1);

    curr_y += LINE_SPACING;
    sprintf(str, "R:            Reset");
    textout_ex(bitmap_menu, font, str, MENU_TITLE_X, curr_y, TEXT_COL, -1);

    curr_y += LINE_SPACING;
    sprintf(str, "ESC:          Quit");
    textout_ex(bitmap_menu, font, str, MENU_TITLE_X, curr_y, TEXT_COL, -1);

    blit(bitmap_menu, screen_buff, 0, 0, MENU_BOX_X1, MENU_BOX_Y1, bitmap_menu->w, bitmap_menu->h);
}



//--------------------------------------------------------------------------------
// DRAW_WORLD: Draws on the screen_buff the box representing the world
//--------------------------------------------------------------------------------
void draw_world() {
    clear_to_color(bitmap_world, WORLD_BKG);
    rectfill(bitmap_world, 0, WORLD_BOX_H, WORLD_BOX_W, WORLD_BOX_H-FLOOR_H_PIX, GROUND_COL);   // ground
    rect(bitmap_world, 0, WORLD_BOX_H-1, WORLD_BOX_W-1, 0, BORDER_COL); // border

    draw_units();
    compute_rob_ver();
    draw_robot();
    

    if(DEBUG) {
    //     px_pix = px*PIX_M;
    //     py_pix = WORLD_BOX_H-py*PIX_M;
    //     circlefill(bitmap_world, px_pix, py_pix, 5, 8);
    //     circlefill(bitmap_world, rob_v.v[4]*PIX_M, WORLD_BOX_H-rob_v.v[5]*PIX_M, 3, 8);
    //     line(bitmap_world,rob_v_pix.v[4], rob_v_pix.v[5], px_pix, py_pix, 8);
    //     printf("px = %f, py = %f, px_pix = %f, py_pix = %f\n", px, py, px_pix, py_pix);
    //       circlefill(bitmap_world, rot_cent_x*PIX_M, WORLD_BOX_H-rot_cent_y*PIX_M, 3, 8);
    }

    // store the bitmap on the screen buffer
	blit(bitmap_world, screen_buff, 0, 0, WORLD_BOX_X1, WORLD_BOX_Y1, bitmap_world->w, bitmap_world->h);
}

//--------------------------------------------------------------------------------
// COMPUTE_ROB_VER: computes the positions in pixels of the robot vertexes,
// using as input the values of rob_st (which are in MKS)
//--------------------------------------------------------------------------------
void compute_rob_ver() {
    pthread_mutex_lock(&mux_rob_st);

    rob_v_pix.x = (rob_st.x+WORLD_SHIFT)*PIX_M;
    rob_v_pix.y = WORLD_BOX_H-rob_st.y*PIX_M;
    for(int i = 0; i < 4; i++){
        rob_v_pix.v[2*i] = (rob_v.v[2*i]+WORLD_SHIFT)*PIX_M;
        rob_v_pix.v[2*i+1] = WORLD_BOX_H-rob_v.v[2*i+1]*PIX_M;
        // printf("i = %d, (%1.2f,%1.2f) --> (%d,%d)\n", i, rob_v.v[2*i], rob_v.v[2*i+1], rob_v_pix.v[2*i], rob_v_pix.v[2*i+1] );
    }

    // KNEES
    rob_v_pix.knee1x = (rob_v.knee_1x+WORLD_SHIFT)*PIX_M;
    rob_v_pix.knee1y = WORLD_BOX_H-rob_v.knee_1y*PIX_M;
    rob_v_pix.knee2x = (rob_v.knee_2x+WORLD_SHIFT)*PIX_M;
    rob_v_pix.knee2y = WORLD_BOX_H-rob_v.knee_2y*PIX_M;
    rob_v_pix.knee3x = rob_v_pix.knee1x - ROB_WIDTH_PIX;
    rob_v_pix.knee3y = rob_v_pix.knee1y;
    rob_v_pix.knee4x = rob_v_pix.knee2x - ROB_WIDTH_PIX;
    rob_v_pix.knee4y = rob_v_pix.knee2y;

    // FEET
    rob_v_pix.foot1x = (rob_v.foot_1x+WORLD_SHIFT)*PIX_M;
    rob_v_pix.foot1y = WORLD_BOX_H-rob_v.foot_1y*PIX_M;
    rob_v_pix.foot2x = (rob_v.foot_2x+WORLD_SHIFT)*PIX_M;
    rob_v_pix.foot2y = WORLD_BOX_H-rob_v.foot_2y*PIX_M;
    rob_v_pix.foot3x = rob_v_pix.foot1x - ROB_WIDTH_PIX;
    rob_v_pix.foot3y = rob_v_pix.foot1y;
    rob_v_pix.foot4x = rob_v_pix.foot2x - ROB_WIDTH_PIX;
    rob_v_pix.foot4y = rob_v_pix.foot2y;
    // printf("\tv = [(%d;%d),(%d;%d),(%d;%d),(%d;%d)], ee_x: %d, ee_y = %d\n", 
    //     rob_v_pix.v[0],rob_v_pix.v[1],rob_v_pix.v[2],rob_v_pix.v[3],rob_v_pix.v[4],rob_v_pix.v[5],rob_v_pix.v[6],rob_v_pix.v[7], rob_v_pix.ee_x, rob_v_pix.ee_y);
    // printf("\tv = [(%1.2f;%1.2f),(%1.2f;%1.2f),(%1.2f;%1.2f),(%1.2f;%1.2f)], ee_x: %1.2f, ee_y = %1.2f\n", 
    //     rob_v.v[0],rob_v.v[1],rob_v.v[2],rob_v.v[3],rob_v.v[4],rob_v.v[5],rob_v.v[6],rob_v.v[7], rob_v.ee_x, rob_v.ee_y);

    pthread_mutex_unlock(&mux_rob_st);
}


void draw_robot() {
    int beta;
  
    // Body
    polygon(bitmap_world, 4, rob_v_pix.v, ROB_COL);

    // Leg 1
    thick_line(bitmap_world, rob_v_pix.v[6], rob_v_pix.v[7], rob_v_pix.knee1x, rob_v_pix.knee1y, LEG_A_COL,2);
    thick_line(bitmap_world, rob_v_pix.knee1x, rob_v_pix.knee1y, rob_v_pix.foot1x, rob_v_pix.foot1y, LEG_A_COL,2);

    // Leg 2
    thick_line(bitmap_world, rob_v_pix.v[6], rob_v_pix.v[7], rob_v_pix.knee2x, rob_v_pix.knee2y, LEG_B_COL,2);
    thick_line(bitmap_world, rob_v_pix.knee2x, rob_v_pix.knee2y, rob_v_pix.foot2x, rob_v_pix.foot2y, LEG_B_COL,2);

    // Leg 4
    thick_line(bitmap_world, rob_v_pix.v[4], rob_v_pix.v[5], rob_v_pix.knee4x, rob_v_pix.knee4y, LEG_B_COL,2);
    thick_line(bitmap_world, rob_v_pix.knee4x, rob_v_pix.knee4y, rob_v_pix.foot4x, rob_v_pix.foot4y, LEG_B_COL,2);

     // Leg 3
    thick_line(bitmap_world, rob_v_pix.v[4], rob_v_pix.v[5], rob_v_pix.knee3x, rob_v_pix.knee3y, LEG_A_COL,2);
    thick_line(bitmap_world, rob_v_pix.knee3x, rob_v_pix.knee3y, rob_v_pix.foot3x, rob_v_pix.foot3y, LEG_A_COL,2);
}

//--------------------------------------------------------------------------------
// THICK_LINE: draws a thick line
//--------------------------------------------------------------------------------
void thick_line(BITMAP *bmp, int x1, int y1, int x2, int y2, int color, int thickness) {
    float angle = atan2(y2 - y1, x2 - x1);
    float dx = sin(angle) * (thickness / 2.0);
    float dy = -cos(angle) * (thickness / 2.0);

    int vertices[8] = {
        x1 + dx, y1 + dy,
        x1 - dx, y1 - dy,
        x2 - dx, y2 - dy,
        x2 + dx, y2 + dy
    };

    polygon(bmp, 4, vertices, color);
}

void draw_units() {
    float i;
    char s[5];

    for(i = WORLD_SHIFT; i*PIX_M < WORLD_BOX_W+WORLD_SHIFT; i++){
        line(bitmap_world, i*PIX_M, WORLD_BOX_H-FLOOR_H_PIX, i*PIX_M, WORLD_BOX_H-FLOOR_H_PIX+10, 1);
        sprintf(s, "%dm", (int)(i));
        textout_centre_ex(bitmap_world, font, s, i*PIX_M, WORLD_BOX_H-FLOOR_H_PIX+20, 1, -1);
    }
    for(i = 0; i*PIX_M < WORLD_BOX_W; i += 0.1){
        line(bitmap_world, i*PIX_M, WORLD_BOX_H-FLOOR_H_PIX, i*PIX_M, WORLD_BOX_H-FLOOR_H_PIX+5, 1);
    }
}

//--------------------------------------------------------------------------------
// DRAW_STAT: Draws on the screen_buff the box with the robot statistics
//--------------------------------------------------------------------------------
void draw_stat() {
    char str[SLEN]; // for text outputs
    int curr_y = STAT_TITLE_Y;

    rect(bitmap_stat, 0, STAT_BOX_H-1, STAT_BOX_W-1, 0, BORDER_COL);    // border

    sprintf(str, "STATISTICS");
    textout_ex(bitmap_stat, font, str, STAT_TITLE_X, curr_y, TITLE_COL, BKG);

    curr_y += LINE_SPACING;
    sprintf(str, "Robot x:          %f", rob_st.x);
    textout_ex(bitmap_stat, font, str, STAT_TITLE_X, curr_y, TEXT_COL, BKG);

    curr_y += LINE_SPACING;
    sprintf(str, "Robot y:          %f", rob_st.y);
    textout_ex(bitmap_stat, font, str, STAT_TITLE_X, curr_y, TEXT_COL, BKG);

    curr_y += LINE_SPACING; 
    sprintf(str, "Robot thetaA1:    %f°  ", rob_st.thetaA1/PI*180);
    textout_ex(bitmap_stat, font, str, STAT_TITLE_X, curr_y, TEXT_COL, BKG);

    curr_y += LINE_SPACING;
    sprintf(str, "Robot thetaA2:    %f°  ", rob_st.thetaA2/PI*180);
    textout_ex(bitmap_stat, font, str, STAT_TITLE_X, curr_y, TEXT_COL, BKG);

    curr_y += LINE_SPACING; 
    sprintf(str, "Robot thetaB1:    %f°  ", rob_st.thetaB1/PI*180);
    textout_ex(bitmap_stat, font, str, STAT_TITLE_X, curr_y, TEXT_COL, BKG);

    curr_y += LINE_SPACING;
    sprintf(str, "Robot thetaB2:    %f°  ", rob_st.thetaB2/PI*180);
    textout_ex(bitmap_stat, font, str, STAT_TITLE_X, curr_y, TEXT_COL, BKG);

    curr_y += LINE_SPACING;
    sprintf(str, "Q-Learn Epoch:    %d/%d      ", epoch, EPISODES);
    textout_ex(bitmap_stat, font, str, STAT_TITLE_X, curr_y, TEXT_COL, BKG);

    curr_y += LINE_SPACING;
    sprintf(str, "Q-Learn Epsilon:  %f", epsilon);
    textout_ex(bitmap_stat, font, str, STAT_TITLE_X, curr_y, TEXT_COL, BKG);

    curr_y += LINE_SPACING;
    sprintf(str, "Q-Learn Reward:   %f", episode_rewards);
    textout_ex(bitmap_stat, font, str, STAT_TITLE_X, curr_y, TEXT_COL, BKG);

    blit(bitmap_stat, screen_buff, 0, 0, STAT_BOX_X1, STAT_BOX_Y1, bitmap_stat->w, bitmap_stat->h);
}

//--------------------------------------------------------------------------------
// DRAW_TASK: Draws on the screen_buff the box with the tasks info
//--------------------------------------------------------------------------------
void draw_task() {
    char str[SLEN]; // for text outputs
    int curr_y = TASK_TITLE_Y;
    int curr_task_i = 0;

    rect(bitmap_task, 0, TASK_BOX_H-1, TASK_BOX_W-1, 0, BORDER_COL);    // border

    sprintf(str, "TASKS INFO");
    textout_ex(bitmap_task, font, str, TASK_TITLE_X, curr_y, TITLE_COL, BKG);


    curr_y += LINE_SPACING;
    curr_task_i = TSK_I_GRAPH;
    sprintf(str, "Task Graphic:");
    textout_ex(bitmap_task, font, str, TASK_TITLE_X, curr_y, choose_color(curr_task_i), BKG);
    curr_y += LINE_SPACING_SMALL;
    sprintf(str, "    Period:     %d    ", task_get_period(curr_task_i));
    textout_ex(bitmap_task, font, str, TASK_TITLE_X, curr_y, choose_color(curr_task_i), BKG);
    curr_y += LINE_SPACING_SMALL;
    sprintf(str, "    DMiss:      %d    ", task_get_dmiss(curr_task_i));
    textout_ex(bitmap_task, font, str, TASK_TITLE_X, curr_y, choose_color(curr_task_i), BKG);

    curr_y += LINE_SPACING;
    curr_task_i = TSK_I_SIM;
    sprintf(str, "Task Simulation:");
    textout_ex(bitmap_task, font, str, TASK_TITLE_X, curr_y, choose_color(curr_task_i), BKG);
    curr_y += LINE_SPACING_SMALL;
    sprintf(str, "    Period:     %d    ", task_get_period(curr_task_i));
    textout_ex(bitmap_task, font, str, TASK_TITLE_X, curr_y, choose_color(curr_task_i), BKG);
    curr_y += LINE_SPACING_SMALL;
    sprintf(str, "    DMiss:      %d    ", task_get_dmiss(curr_task_i));
    textout_ex(bitmap_task, font, str, TASK_TITLE_X, curr_y, choose_color(curr_task_i), BKG);

    curr_y += LINE_SPACING;
    curr_task_i = TSK_I_USR;
    sprintf(str, "Task USER:");
    textout_ex(bitmap_task, font, str, TASK_TITLE_X, curr_y, choose_color(curr_task_i), BKG);
    curr_y += LINE_SPACING_SMALL;
    sprintf(str, "    Period:     %d    ", task_get_period(curr_task_i));
    textout_ex(bitmap_task, font, str, TASK_TITLE_X, curr_y, choose_color(curr_task_i), BKG);
    curr_y += LINE_SPACING_SMALL;
    sprintf(str, "    DMiss:      %d    ", task_get_dmiss(curr_task_i));
    textout_ex(bitmap_task, font, str, TASK_TITLE_X, curr_y, choose_color(curr_task_i), BKG);

    curr_y += LINE_SPACING;
    curr_task_i = TSK_I_Q;
    sprintf(str, "Task Q_LEARN:");
    textout_ex(bitmap_task, font, str, TASK_TITLE_X, curr_y, choose_color(curr_task_i), BKG);
    curr_y += LINE_SPACING_SMALL;
    sprintf(str, "    Period:     %d    ", task_get_period(curr_task_i));
    textout_ex(bitmap_task, font, str, TASK_TITLE_X, curr_y, choose_color(curr_task_i), BKG);
    curr_y += LINE_SPACING_SMALL;
    sprintf(str, "    DMiss:      %d    ", task_get_dmiss(curr_task_i));
    textout_ex(bitmap_task, font, str, TASK_TITLE_X, curr_y, choose_color(curr_task_i), BKG);

    blit(bitmap_task, screen_buff, 0, 0, TASK_BOX_X1, TASK_BOX_Y1, bitmap_task->w, bitmap_stat->h);
}


//--------------------------------------------------------------------------------
// CHOOSE_COLOR: Returns the color to use depending on whether the task is 
// selected or not
//--------------------------------------------------------------------------------
int choose_color(int curr_task_i){
    if (curr_task_i == selected_task)
        return TEXT_SEL_COL;
    return TEXT_COL;
}

//--------------------------------------------------------------------------------
// DRAW_QTABLE: Draws the content of the Q_Table
// Shoulder angles on the rows (from bottom to top), Elbow angles on the columns 
//--------------------------------------------------------------------------------
void draw_qtable(){
    clear_bitmap(bitmap_qtable);
    rect(bitmap_qtable, 0, QTABLE_BOX_H-1, QTABLE_BOX_W-1, 0, BORDER_COL);    // border
    int r, c;
    int hipA_angle_i, hipB_angle_i;
    int kneeA_angle_i, kneeB_angle_i;
    char str[SLEN];

    // Get angle indexes
    hipA_angle_i = from_theta1_to_index(rob_st.thetaA1);
    kneeA_angle_i = from_theta2_to_index(rob_st.thetaA2);
    hipB_angle_i = from_theta1_to_index(rob_st.thetaB1);
    kneeB_angle_i = from_theta2_to_index(rob_st.thetaB2);

    int curr_y = QTABLE_TITLE_Y;
    sprintf(str, "Q-TABLE");
    textout_ex(bitmap_qtable, font, str, MENU_TITLE_X, curr_y, TITLE_COL, -1);

    // // Get current angle indexes
    // shoulder_angle_i = from_theta1_to_index(rob_st.theta1);
    // elbow_angle_i = from_theta2_to_index(rob_st.theta2);

    // for (c = 0; c < NUM_ELBOW_ANGLES; ++c){
    //     sprintf(str, "%d", c);
    //     textout_centre_ex(bitmap_qtable, font, str, QTABLE_X1 + (c+1)*CELL_WIDTH + (double)(CELL_WIDTH)/2, QTABLE_Y1 + (double)(CELL_HEIGHT)/2, WHITE, BKG);
    // }

    // for (r = NUM_SHOULDER_ANGLES - 1; r >= 0 ; r--) {
    //     sprintf(str, "%d", r);
    //     textout_centre_ex(bitmap_qtable, font, str, QTABLE_X1 + (double)(CELL_WIDTH)/2, QTABLE_Y1 + (NUM_SHOULDER_ANGLES-r)*CELL_HEIGHT+(double)(CELL_HEIGHT)/2, WHITE, BKG);
    //     for (c = 0; c < NUM_ELBOW_ANGLES; ++c) {
    //         int x = (c+1) * CELL_WIDTH + QTABLE_X1;
    //         int y = (NUM_SHOULDER_ANGLES-1-r) * CELL_HEIGHT + QTABLE_Y1 + CELL_HEIGHT;
    //         if(r == shoulder_angle_i && c == elbow_angle_i)
    //             rectfill(bitmap_qtable, x, y, x + CELL_WIDTH, y + CELL_HEIGHT, YELLOW);
    //         else
    //             rectfill(bitmap_qtable, x, y, x + CELL_WIDTH, y + CELL_HEIGHT, BKG);

    //         rect(bitmap_qtable, x, y, x + CELL_WIDTH, y + CELL_HEIGHT, BORDER_COL);

    //         int best_action = choose_best_action(r, c);

    //         switch (best_action) {
    //             case 0: sprintf(str, "v"); break;
    //             case 1: sprintf(str, "^"); break;
    //             case 2: sprintf(str, "<"); break;
    //             case 3: sprintf(str, ">"); break;
    //             default: sprintf(str, "'"); break;
    //         }

    //         if(r == shoulder_angle_i && c == elbow_angle_i)
    //             textout_centre_ex(bitmap_qtable, font, str, x+(double)(CELL_WIDTH)/2, y+(double)(CELL_HEIGHT)/2, BKG, YELLOW);
    //         else 
    //             textout_centre_ex(bitmap_qtable, font, str, x+(double)(CELL_WIDTH)/2, y+(double)(CELL_HEIGHT)/2, WHITE, BKG);

    //     }
    // }

    // printf("\n GRAPHIC - QTABLE: Rob_theta1: %f, part1: %f, rob_index: %f\n",rob_st.theta1, (rob_st.theta1-THETA1_MIN), (rob_st.theta1-THETA1_MIN) / THETA1_INC);
    curr_y += LINE_SPACING;
    sprintf(str, "HIP A  : %d", hipA_angle_i);
    textout_ex(bitmap_qtable, font, str, 3*MARGIN, curr_y, WHITE, BKG);

    curr_y += LINE_SPACING;
    sprintf(str, "ELBOW A: %d", kneeA_angle_i);
    textout_ex(bitmap_qtable, font, str, 3*MARGIN, curr_y, WHITE, BKG);
    
    curr_y += LINE_SPACING;
    sprintf(str, "HIP B  : %d", hipB_angle_i);
    textout_ex(bitmap_qtable, font, str, 3*MARGIN, curr_y, WHITE, BKG);

    curr_y += LINE_SPACING;
    sprintf(str, "ELBOW B: %d", kneeB_angle_i);
    textout_ex(bitmap_qtable, font, str, 3*MARGIN, curr_y, WHITE, BKG);


    blit(bitmap_qtable, screen_buff, 0, 0, QTABLE_BOX_X1, QTABLE_BOX_Y1, bitmap_qtable->w, bitmap_qtable->h);
}
