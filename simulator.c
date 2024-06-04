//--------------------------------------------------------------------------------
// SIMULATOR.C:   2D Physics Engine
// NOTE: All the units here are in MKS unit system
//--------------------------------------------------------------------------------
#include "simulator.h"

double px = 20, py = 20;
double rot_cent_x = 0, rot_cent_y = 0;

void* simulator(void* arg) {
    int i;              // task index
    i = get_task_index(arg);
    set_period(i);


    while (!end) {
        //printf("SIMULATOR\n");
        pthread_mutex_lock(&mux_rob_st);

        compute_rob_v();
        //print_rob_st();

        handle_ground_contact();
      
        pthread_mutex_unlock(&mux_rob_st);
        wait_for_period(i);
    }
}

//--------------------------------------------------------------------------------
// PRINT_ROB_ST: Prints the state variables of the robot
//--------------------------------------------------------------------------------
void print_rob_st() {
    printf("Robot State: \tthetaA1: %1.2f, \tthetaA2 = %1.2f, \tthetaLEG = %1.2f, \tx: %1.2f, \ty: %1.2f\n", rob_st.thetaA1/PI, rob_st.thetaA2, rob_st.thetaA1 - rob_st.thetaA2, rob_st.x, rob_st.y);
    printf("\tv = [(%1.2f;%1.2f),(%1.2f;%1.2f),(%1.2f;%1.2f),(%1.2f;%1.2f)], knee1x: %1.2f, knee1y = %1.2f, foot1x: %1.2f, foot1y = %1.2f, \n", 
    rob_v.v[0],rob_v.v[1],rob_v.v[2],rob_v.v[3],rob_v.v[4],rob_v.v[5],rob_v.v[6],rob_v.v[7], rob_v.knee_1x, rob_v.knee_1y, rob_v.foot_1x, rob_v.foot_1y);
}

//--------------------------------------------------------------------------------
// COMPUTE_ROB_V: gets the position of the robot verteces using the robot state
//--------------------------------------------------------------------------------
void compute_rob_v() {
    // Calculate the coordinates of the four corners of the rectangle after rotation
    // (x1, y1) = upper-right corner
    rob_v.v[0] = rob_st.x + ROB_WIDTH / 2.0;
    rob_v.v[1] = rob_st.y + ROB_HEIGHT / 2.0;
    // (x2, y2) = upper-left corner
    rob_v.v[2] = rob_st.x - ROB_WIDTH / 2.0;
    rob_v.v[3] = rob_st.y + ROB_HEIGHT / 2.0;
    // (x3, y3) = lower-left corner
    rob_v.v[4] = rob_st.x - ROB_WIDTH / 2.0;
    rob_v.v[5] = rob_st.y - ROB_HEIGHT / 2.0 ;
    // (x4, y4) = lower-right corner
    rob_v.v[6] = rob_st.x + ROB_WIDTH / 2.0;
    rob_v.v[7] = rob_st.y - ROB_HEIGHT / 2.0;

    // KENNS 1&3
    double cos_theta = cos(rob_st.thetaA1);
    double sin_theta = sin(rob_st.thetaA1);
        // 1st leg
    rob_v.knee_1x = rob_v.v[6] - ROB_L1*cos_theta;
    rob_v.knee_1y = rob_v.v[7] - ROB_L1*sin_theta;
        // 3rd leg
    rob_v.knee_3x = rob_v.v[4] - ROB_L1*cos_theta;
    rob_v.knee_3y = rob_v.v[5] - ROB_L1*sin_theta;

    // FEET 1&3
    //printf("AAAAAAA: \tthetaA1: %1.2f, \tthetaA2 = %1.2f, \tthetaLEG = %1.2f\n", rob_st.thetaA1, rob_st.thetaA2, rob_st.thetaA1 - rob_st.thetaA2);
    cos_theta = cos(rob_st.thetaA1 - rob_st.thetaA2);
    sin_theta = sin(rob_st.thetaA1 - rob_st.thetaA2);
        // 1st leg
    rob_v.foot_1x = rob_v.knee_1x + ROB_L2*cos_theta;
    rob_v.foot_1y = rob_v.knee_1y + ROB_L2*sin_theta;
        // 3rd leg
    rob_v.foot_3x = rob_v.knee_3x + ROB_L2*cos_theta;
    rob_v.foot_3y = rob_v.knee_3y + ROB_L2*sin_theta;

    // LEGS 2&4
    cos_theta = cos(rob_st.thetaB1);
    sin_theta = sin(rob_st.thetaB1);
        // 2nd leg
    rob_v.knee_2x = rob_v.v[6] - ROB_L1*cos_theta;
    rob_v.knee_2y = rob_v.v[7] - ROB_L1*sin_theta;
        // 4th leg
    rob_v.knee_4x = rob_v.v[4] - ROB_L1*cos_theta;
    rob_v.knee_4y = rob_v.v[5] - ROB_L1*sin_theta;

    // FEET 2&4
    cos_theta = cos(rob_st.thetaB1 - rob_st.thetaB2);
    sin_theta = sin(rob_st.thetaB1 - rob_st.thetaB2);
        // 2nd leg
    rob_v.foot_2x = rob_v.knee_2x + ROB_L2*cos_theta;
    rob_v.foot_2y = rob_v.knee_2y + ROB_L2*sin_theta;
        // 4th leg
    rob_v.foot_4x = rob_v.knee_4x + ROB_L2*cos_theta;
    rob_v.foot_4y = rob_v.knee_4y + ROB_L2*sin_theta;


    //print_rob_st();
}

//--------------------------------------------------------------------------------
// HANDLE_GROUND_CONTACT: Handles the collisions of the robot with the ground
//--------------------------------------------------------------------------------
void handle_ground_contact() {
    int n = find_num_contacts();
    //printf("\n\nN = %d\n\n", n);

    if (n == 0) {
        double min_height;

        // bring to ground
        min_height = fmin(rob_v.foot_1y, rob_v.foot_2y ) - FLOOR_H;
        rob_st.y -= min_height;
    }
   
    if (n == 1) {
        double d_y;                 // Change in the y of the centre of mass 
        double coeff;
        double depth;

        if (rob_v.foot_1y <= FLOOR_H){
            depth = FLOOR_H - rob_v.foot_1y;
            rob_st.y += depth;
            rob_st.x += rob_st.prev_foot_1x - rob_v.foot_1x;
        }
        else { 
            //case "rob_v.foot_2y <= FLOOR_H"
            depth = FLOOR_H - rob_v.foot_2y;
            rob_st.y += depth;
            rob_st.x += rob_st.prev_foot_2x - rob_v.foot_2x;
        }

    }
    else if (n == 2) {
        double max_depth = FLOOR_H - fmin(rob_v.foot_1y, rob_v.foot_2y);
        rob_st.y += max_depth;

        if (rob_v.foot_1y < rob_v.foot_2y)       // if leg1 is touching the ground
            rob_st.x += rob_st.prev_foot_1x - rob_v.foot_1x;
        else                                    // if leg2 is touching the ground
            rob_st.x += rob_st.prev_foot_2x - rob_v.foot_2x;
    }
} 

//--------------------------------------------------------------------------------
// FIND_NUM_CONTACTS: Finds the number of feet touhcing the ground. Only the legs 
// 1&2 are considered (as legs 3&4 behave accordingly). So, the returned values 
// will always be 1 or 2
//--------------------------------------------------------------------------------
int find_num_contacts() {
    int i, count = 0;
    if (rob_v.foot_1y <= FLOOR_H)
        count ++;
        
    if (rob_v.foot_2y <= FLOOR_H)
        count ++;

    return count;
}


//--------------------------------------------------------------------------------
// ROTATE_POINT_AROUND_VERTEX: Rotates the point (x,y) counterclockwise around 
// the centre (xc, yc) by an angle of alpha radians
//--------------------------------------------------------------------------------
void rotate_point_around_vertex(double *x, double *y, double xc, double yc, double alpha) {
    double cos_alpha = cos(alpha);
    double sin_alpha = sin(alpha);
    double dx = *x - xc;
    double dy = *y - yc;

    // if(DEBUG){
    //     printf("Rot centre: (%1.2f, %1.2f); pos befor rot = (%1.2f, %1.2f)\n", xc, yc,*x, *y);
    //     rot_cent_x = xc;
    //     rot_cent_y = yc;
    // }

    *x = xc + dx * cos_alpha - dy * sin_alpha;
    *y = yc + dx * sin_alpha + dy * cos_alpha;
}

/*--------------------------------------------------------------------------------
    MOVE_ROBOT: Moves the robot to make it take the action given as argument
    Actions:
    - 0: Decrease hip angle of legs A         
    - 1: Increase hip angle of legs A
    - 2: Decrease knee angle of legs A
    - 3: Increase knee angle of legs A
    - 4: Decrease hip angle of legs B         
    - 5: Increase hip angle of legs B
    - 6: Decrease knee angle of legs B
    - 7: Increase knee angle of legs B
--------------------------------------------------------------------------------*/
int move_robot(int action) {
    int abort = 0;
    switch (action) {
        case 0:
           pthread_mutex_lock(&mux_rob_st);
            if (rob_st.thetaA1 - THETA1_INC >= THETA1_MIN){
                abort = 0;
                rob_st_prev = rob_st;
                rob_st.thetaA1 -= THETA1_INC;
                rob_st.prev_foot_1x = rob_v.foot_1x;
                rob_st.prev_foot_2x = rob_v.foot_2x;
            } else 
                abort = 1;
            pthread_mutex_unlock(&mux_rob_st);
            break;
        case 1:
            pthread_mutex_lock(&mux_rob_st);
            if (rob_st.thetaA1 + THETA1_INC <= THETA1_MAX){
                abort = 0;
                rob_st_prev = rob_st;
                rob_st.thetaA1 += THETA1_INC;
                rob_st.prev_foot_1x = rob_v.foot_1x;
                rob_st.prev_foot_2x = rob_v.foot_2x;
            } else
                abort = 1;
            pthread_mutex_unlock(&mux_rob_st);
            break;
        case 2:
            pthread_mutex_lock(&mux_rob_st);
            if (rob_st.thetaA2 - THETA2_INC >= THETA2_MIN){
                abort = 0;
                rob_st_prev = rob_st;
                rob_st.thetaA2 -= THETA2_INC;
                rob_st.prev_foot_1x = rob_v.foot_1x;
                rob_st.prev_foot_2x = rob_v.foot_2x;
            } else
                abort = 1;
            pthread_mutex_unlock(&mux_rob_st);
            break;
        case 3:
            pthread_mutex_lock(&mux_rob_st);
            if (rob_st.thetaA2 + THETA2_INC <= THETA2_MAX + APPROX_ANGLE){
                abort = 0;
                rob_st_prev = rob_st;
                rob_st.thetaA2 += THETA2_INC;
                rob_st.prev_foot_1x = rob_v.foot_1x;
                rob_st.prev_foot_2x = rob_v.foot_2x;
            } else
                abort = 1;
            pthread_mutex_unlock(&mux_rob_st);
            break;
        case 4:
           pthread_mutex_lock(&mux_rob_st);
            if (rob_st.thetaB1 - THETA1_INC >= THETA1_MIN){
                abort = 0;
                rob_st_prev = rob_st;
                rob_st.thetaB1 -= THETA1_INC;
                rob_st.prev_foot_1x = rob_v.foot_1x;
                rob_st.prev_foot_2x = rob_v.foot_2x;
            } else 
                abort = 1;
            pthread_mutex_unlock(&mux_rob_st);
            break;
        case 5:
            pthread_mutex_lock(&mux_rob_st);
            if (rob_st.thetaB1 + THETA1_INC <= THETA1_MAX){
                abort = 0;
                rob_st_prev = rob_st;
                rob_st.thetaB1 += THETA1_INC;
                rob_st.prev_foot_1x = rob_v.foot_1x;
                rob_st.prev_foot_2x = rob_v.foot_2x;
            } else
                abort = 1;
            pthread_mutex_unlock(&mux_rob_st);
            break;
        case 6:
            pthread_mutex_lock(&mux_rob_st);
            if (rob_st.thetaB2 - THETA2_INC >= THETA2_MIN){
                abort = 0;
                rob_st_prev = rob_st;
                rob_st.thetaB2 -= THETA2_INC;
                rob_st.prev_foot_1x = rob_v.foot_1x;
                rob_st.prev_foot_2x = rob_v.foot_2x;
            } else
                abort = 1;
            pthread_mutex_unlock(&mux_rob_st);
            break;
        case 7:
            pthread_mutex_lock(&mux_rob_st);
            if (rob_st.thetaB2 + THETA2_INC <= THETA2_MAX  + APPROX_ANGLE){
                abort = 0;
                rob_st_prev = rob_st;
                rob_st.thetaB2 += THETA2_INC;
                rob_st.prev_foot_1x = rob_v.foot_1x;
                rob_st.prev_foot_2x = rob_v.foot_2x;
            } else
                abort = 1;
            pthread_mutex_unlock(&mux_rob_st);
            break;
        default:
            break;
    }
    return abort;
}

//--------------------------------------------------------------------------------
// CHECK_KNEE: Checks whether the knee is under the ground
//--------------------------------------------------------------------------------
int check_knee(){
    if (rob_v.knee_1y < FLOOR_H || rob_v.knee_2y < FLOOR_H)
        return 0;
    return 1;
}