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
        //handle_ee_collision();

        // if(DEBUG){
        //     rotate_point_around_vertex(&px, &py, rob_v.v[4], rob_v.v[5], PI/4);
        //     rob_st.alpha += PI/4;
        //     rotate_point_around_vertex(&rob_st.x, &rob_st.y, 3.50, 17.50, PI/4);
        // }

        rob_st.x = rob_st.x + rob_st.vx*PERIOD_S;
        rob_st.y = rob_st.y + rob_st.vy*PERIOD_S;


        pthread_mutex_unlock(&mux_rob_st);
        wait_for_period(i);
    }
}

//--------------------------------------------------------------------------------
// PRINT_ROB_ST: Prints the state variables of the robot
//--------------------------------------------------------------------------------
void print_rob_st() {
    printf("Robot State: \talpha: %1.2f, \tx: %1.2f, \ty: %1.2f, \tvx: %f, \tvy: %f\n", rob_st.alpha/PI, rob_st.x, rob_st.y, rob_st.vx, rob_st.vy);
    printf("\tv = [(%1.2f;%1.2f),(%1.2f;%1.2f),(%1.2f;%1.2f),(%1.2f;%1.2f)], ee_x: %1.2f, ee_y = %1.2f\n", 
    rob_v.v[0],rob_v.v[1],rob_v.v[2],rob_v.v[3],rob_v.v[4],rob_v.v[5],rob_v.v[6],rob_v.v[7], rob_v.ee_x, rob_v.ee_y);
}

//--------------------------------------------------------------------------------
// COMPUTE_ROB_V: gets the position of the robot verteces using the robot state
//--------------------------------------------------------------------------------
void compute_rob_v() {
    // Calculate the coordinates of the corners after rotation
    double x1, y1, x2, y2, x3, y3, x4, y4;

    double cos_alpha = cos(rob_st.alpha);
    double sin_alpha = sin(rob_st.alpha);

    // Calculate the coordinates of the four corners of the rectangle after rotation
    // (x1, y1) = upper-right corner
    rob_v.v[0] = rob_st.x + ROB_WIDTH / 2.0 * cos_alpha - ROB_HEIGHT / 2.0 * sin_alpha;
    rob_v.v[1] = rob_st.y + ROB_WIDTH / 2.0 * sin_alpha + ROB_HEIGHT / 2.0 * cos_alpha;
    // (x2, y2) = upper-left corner
    rob_v.v[2] = rob_st.x - ROB_WIDTH / 2.0 * cos_alpha - ROB_HEIGHT / 2.0 * sin_alpha;
    rob_v.v[3] = rob_st.y - ROB_WIDTH / 2.0 * sin_alpha + ROB_HEIGHT / 2.0 * cos_alpha;
    // (x3, y3) = lower-left corner
    rob_v.v[4] = rob_st.x - ROB_WIDTH / 2.0 * cos_alpha + ROB_HEIGHT / 2.0 * sin_alpha;
    rob_v.v[5] = rob_st.y - ROB_WIDTH / 2.0 * sin_alpha - ROB_HEIGHT / 2.0 * cos_alpha;
    // (x4, y4) = lower-right corner
    rob_v.v[6] = rob_st.x + ROB_WIDTH / 2.0 * cos_alpha + ROB_HEIGHT / 2.0 * sin_alpha;
    rob_v.v[7] = rob_st.y + ROB_WIDTH / 2.0 * sin_alpha - ROB_HEIGHT / 2.0 * cos_alpha;

    // Calculate the coodrinates of the elbow --> start from the vertex (x1, y1)
    double cos_elb = cos(rob_st.alpha + rob_st.theta1 - PI/2);
    double sin_elb = sin(rob_st.alpha + rob_st.theta1 - PI/2);

    rob_v.elb_x = rob_v.v[0] + ROB_L1*cos_elb;
    rob_v.elb_y = rob_v.v[1] + ROB_L1*sin_elb;

    // Calculate the coodrinates of the end effector --> start from the elbow
    double cos_ee = cos(rob_st.alpha + rob_st.theta1 + rob_st.theta2 - 3*PI/2);
    double sin_ee = sin(rob_st.alpha + rob_st.theta1 + rob_st.theta2 - 3*PI/2);

    rob_v.ee_x = rob_v.elb_x + ROB_L2*cos_ee;
    rob_v.ee_y = rob_v.elb_y + ROB_L2*sin_ee;
}

//--------------------------------------------------------------------------------
// HANDLE_GROUND_CONTACT: Handles the collisions of the robot with the ground
//--------------------------------------------------------------------------------
void handle_ground_contact() {
    int n = find_num_contacts();
    //printf("\n\nN = %d\n\n", n);

    if (n == 0 && rob_st.one_contact_first != 0) {
        // Nothing happens, let the gravity do its job
        rob_st.one_contact_first = 1;
        handle_gravity();
    }       
    else if (n == 1){
        double d_y;                 // Change in the y of the centre of mass 
        double cos_alpha = cos(rob_st.alpha);
        double coeff;
        double depth;

        rob_st.vy = 0;   // We will compute the effect of the gravity differently

        if (rob_st.one_contact_first){
            // In the first iteration, zero the counter
            rob_st.one_contact_first = 0;
            rob_st.one_contact_num_dt = 0;
            // Make the robot stick to the ground (no higher, no lower)
            // depth = FLOOR_H - rob_v.v[5];
            // rob_st.y += depth;
        }

        coeff = -2*ROB_MASS*GRAVITY/(ROB_MASS+ROB_INERTIA/(ROB_SEMIDIAG*ROB_SEMIDIAG*cos_alpha*cos_alpha));
        d_y = coeff*(rob_st.one_contact_num_dt*SIMULATOR_T)*SIMULATOR_T;
        rob_st.y = rob_st.y + d_y;

        printf("asin: %f, arg: %f, atan: %f, denom: %f\n", asin((rob_st.y - FLOOR_H) / (double)(ROB_SEMIDIAG) ), (double)(rob_st.y - FLOOR_H) / (double)(ROB_SEMIDIAG), atan2((double)(ROB_HEIGHT), (double)(ROB_WIDTH)) , ROB_SEMIDIAG);
        rob_st.alpha = asin((double)(rob_st.y - FLOOR_H) / (double)(ROB_SEMIDIAG)) - atan((double)(ROB_HEIGHT) / (double)(ROB_WIDTH));
        
        rob_st.one_contact_num_dt ++; 
        end = 1;
        if (DEBUG){
            printf("Values: dy = %f, alpha = %f\n", d_y, rob_st.alpha);
            print_rob_st(); 
            //end = 1;
        }
        
    }
    else if (n >= 2){
        rob_st.one_contact_first = 1;
        rob_st.vy = 0;   // We will compute the effect of the gravity differently


        if (rob_v.ee_y <= FLOOR_H + APPROX){
            // two points of collision: lower-left vertex and end-effector
            handle_ee_collision();
        }
        else{
            // two points of collision: lower-left and lower-right vertexes
            rob_st.alpha = 0;
            rob_st.y = FLOOR_H + ROB_HEIGHT/2;
        }
    }


    // if (rob_st.y <= FLOOR_H+ROB_HEIGHT/2.0) {
    //     rob_st.y = FLOOR_H+ROB_HEIGHT/2.0;
    //     // rob_st.vy = -DUMP*rob_st.vy; // NO BOUNCING FOR NOW
    // }  
} 

//--------------------------------------------------------------------------------
// HANDLE_GRAVITY: Handles the effect of the gravity on the robot
//--------------------------------------------------------------------------------
void handle_gravity() {
    rob_st.vy = rob_st.vy + GRAVITY*PERIOD_S;
}

//--------------------------------------------------------------------------------
// FIND_NUM_CONTACTS: Finds the number of vertexes of the robot that are in 
// contact with the ground. If the number is:
// - 0: the robot is in the air.
// - 1: the robot is in an unstable situation and starts rotating around the 
//      contact point due to the gravity
// - 2: the robot is in a stable situation
// - other: there has been a problem
//--------------------------------------------------------------------------------
int find_num_contacts() {
    int i, count = 0;
    for (i = 1; i < 8; i += 2)
        if (rob_v.v[i] <= FLOOR_H)
            count++;
        
    if (rob_v.ee_y <= FLOOR_H)
        count++;

    if (count >= 1)
        count = 2;
    return count;
}

//--------------------------------------------------------------------------------
// HANDLE_EE_COLLISION: Handles the case when the end effector touches the ground
//--------------------------------------------------------------------------------
void handle_ee_collision() {
    double depth = FLOOR_H - rob_v.ee_y;    // sinking of the end effector in the ground
    double dy;
    double dx;                              // x-distance from lower-left vertex to ee
    double gamma;                           // angle of rotation of the body

    if (depth <= 0) return;                    // if there is no collision, return   

    // rotate the full robot around the end-effector
    dy = rob_v.v[5] - rob_v.ee_y;
    dx = (double)(rob_v.ee_x - rob_v.v[4]);
    gamma = atan(depth/dx);
    rob_st.alpha += gamma;
    rotate_point_around_vertex(&rob_st.x, &rob_st.y, rob_v.ee_x, rob_v.ee_y, gamma);
    rob_st.x += rob_st.prev_ee_x - rob_v.ee_x;
    rob_st.y += depth;
}

//--------------------------------------------------------------------------------
// HANDLE_EE_COLLISION_OLD: For rotation around the bottom-left vertex
//--------------------------------------------------------------------------------
void handle_ee_collision_old() {
    // if(DEBUG && rob_st.theta1 < PI/2){
    //     printf("Ciao\n");
    //     rotate_point_around_vertex(&rob_st.x, &rob_st.y, rob_v.v[4], rob_v.v[5], 0.1);
    // }
    double dy = FLOOR_H - rob_v.ee_y;    // sinking of the end effector in the ground
    double dx;                           // x-distance from lower-left vertex to ee
    double gamma;                        // angle of rotation of the body

    if (dy <= 0) return;                // if there is no collision, return   

    // rotate the full robot around the lower-left vertex (vertex 3, coordinates saved in rob_v.v[4] and rob_v.v[5] )
    dx = (double)(rob_v.ee_x - rob_v.v[4]);
    gamma = atan(dy/dx);
    rob_st.alpha += gamma;
    rotate_point_around_vertex(&rob_st.x, &rob_st.y, rob_v.v[4], rob_v.v[5], gamma);
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
    - 0: Decrease shoulder angle    (v)
    - 1: Increase shoulder angle    (^)
    - 2: Decrease elbow angle       (<)
    - 3: Increase elbow angle       (>)
--------------------------------------------------------------------------------*/
int move_robot(int action) {
    int abort = 0;
    switch (action) {
        case 0:
           pthread_mutex_lock(&mux_rob_st);
            if (rob_st.theta1 - THETA1_INC >= THETA1_MIN){
                abort = 0;
                rob_st_prev = rob_st;
                rob_st.theta1 -= THETA1_INC;
                rob_st.prev_ee_x = rob_v.ee_x;
                rob_st.prev_ee_y = rob_v.ee_y;
            } else 
                abort = 1;
            pthread_mutex_unlock(&mux_rob_st);
            break;
        case 1:
            pthread_mutex_lock(&mux_rob_st);
            if (rob_st.theta1 + THETA1_INC <= THETA1_MAX){
                abort = 0;
                rob_st_prev = rob_st;
                rob_st.theta1 += THETA1_INC;
                rob_st.prev_ee_x = rob_v.ee_x;
                rob_st.prev_ee_y = rob_v.ee_y;
            } else
                abort = 1;
            pthread_mutex_unlock(&mux_rob_st);
            break;
        case 2:
            pthread_mutex_lock(&mux_rob_st);
            if (rob_st.theta2 - THETA2_INC >= THETA2_MIN){
                abort = 0;
                rob_st_prev = rob_st;
                rob_st.theta2 -= THETA2_INC;
                rob_st.prev_ee_x = rob_v.ee_x;
                rob_st.prev_ee_y = rob_v.ee_y;
            } else
                abort = 1;
            pthread_mutex_unlock(&mux_rob_st);
            break;
        case 3:
            pthread_mutex_lock(&mux_rob_st);
            if (rob_st.theta2 + THETA2_INC <= THETA2_MAX){
                abort = 0;
                rob_st_prev = rob_st;
                rob_st.theta2 += THETA2_INC;
                rob_st.prev_ee_x = rob_v.ee_x;
                rob_st.prev_ee_y = rob_v.ee_y;
            } else
                abort = 1;
            pthread_mutex_unlock(&mux_rob_st);
            break;
        default:
            break;
    }
    return abort;
}