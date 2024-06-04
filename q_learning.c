//--------------------------------------------------------------------------------
// Q_LEARNING.C
//--------------------------------------------------------------------------------
#include "q_learning.h"

//--------------------------------------------------------------------------------
// Q_LEARNING_TRAIN: Task to train the q-learning algorithm
//--------------------------------------------------------------------------------
void* q_learning_train(void* arg) {
    int i;              // Task index
    int step;           // Step counter for the training
    int action;         // Action index
    double reward;      // Reward for a single move
    double total_episode_reward;
    int abort;          // Abort = 1, if the robot tries to move its joints over the limits
    int hipA_angle_i, new_hipA_angle_i, hipB_angle_i, new_hipB_angle_i;
    int kneeA_angle_i, new_kneeA_angle_i, kneeB_angle_i, new_kneeB_angle_i;

    i = get_task_index(arg);
    set_period(i);

    printf("Q_LEARNING - TRAINING\n");

    q_init();
    epsilon = INITIAL_EPSILON;

    // Uncomment the following line to load an existing Q-table
    // load_q_table("q_table.dat");

    for (epoch = 0; epoch < EPISODES && !end && qlearn_active == 1; epoch++) {
        // Reset Environment
        init_rob_st();
        total_episode_reward = 0;

        for (step = 0; step < MAX_STEPS && !end && qlearn_active == 1; step++) {

            // Get angle indexes
            hipA_angle_i = from_theta1_to_index(rob_st.thetaA1);
            kneeA_angle_i = from_theta2_to_index(rob_st.thetaA2);
            hipB_angle_i = from_theta1_to_index(rob_st.thetaB1);
            kneeB_angle_i = from_theta2_to_index(rob_st.thetaB2);
            
            // Choose an action
            action = choose_action(hipA_angle_i, kneeA_angle_i, hipB_angle_i, kneeB_angle_i, epsilon);

            // Take action and observe new state and reward
            abort = move_robot(action);
            // TODO: Wait for the simulator to update the position of the robot
            wait_for_period(i);
            reward = calculate_reward(abort);
            total_episode_reward = total_episode_reward + reward;

            // Update Q-value using Q-learning equation
            new_hipA_angle_i = from_theta1_to_index(rob_st.thetaA1);
            new_kneeA_angle_i = from_theta2_to_index(rob_st.thetaA2);
            new_hipB_angle_i = from_theta1_to_index(rob_st.thetaB1);
            new_kneeB_angle_i = from_theta2_to_index(rob_st.thetaB2);

            int best_next_action = choose_best_action(new_hipA_angle_i, new_kneeA_angle_i, new_hipB_angle_i, new_hipB_angle_i);
            q_table[hipA_angle_i][kneeA_angle_i][hipB_angle_i][kneeB_angle_i][action] = (1 - LR)*q_table[hipA_angle_i][kneeA_angle_i][hipB_angle_i][kneeB_angle_i][action] + LR*(reward+DISCOUNT_FACTOR*q_table[new_hipA_angle_i][new_kneeA_angle_i][new_hipB_angle_i][new_kneeB_angle_i][best_next_action]);
            
            // Check if the episode is done
            if (abort) {
                break;
            }
            wait_for_period(i);
        }

        epsilon = fmax(MIN_EPSILON, epsilon * EPSILON_DECAY);
        episode_rewards = total_episode_reward;
        
    }

    qlearn_active = 0;
    // Save only if the training was complete
    if(epoch == EPISODES){
        printf("Q_LEARNING - TRAINING - Completed!\n");
        save_q_table(SAVE_PATH);
    }
    else
        printf("Q_LEARNING - TRAINING - Aborted before completion\n");
}


//--------------------------------------------------------------------------------
// Q_LEARNING_RUN: Task to run the q-learning algorithm
//--------------------------------------------------------------------------------
void* q_learning_run(void* arg) {
    int i;              // Task index
    int action;         // Action index
    int hipA_angle_i, hipB_angle_i;
    int kneeA_angle_i, kneeB_angle_i;

    i = get_task_index(arg);
    set_period(i);

    load_q_table(SAVE_PATH);
    
    printf("Q_LEARNING - RUN\n");
    // printf_qtable();

    while (!end && qlearn_active == 2) {

        hipA_angle_i = from_theta1_to_index(rob_st.thetaA1);
        kneeA_angle_i = from_theta2_to_index(rob_st.thetaA2); 
        hipB_angle_i = from_theta1_to_index(rob_st.thetaB1);
        kneeB_angle_i = from_theta2_to_index(rob_st.thetaB2);

        action = choose_best_action(hipA_angle_i, kneeA_angle_i, hipB_angle_i, kneeB_angle_i);
        // if(DEBUG)
        //     printf("theta1: %1.2f; theta2: %1.2f; Shoulder: %d, Elbow: %d, action: %d\n", rob_st.theta1*180/PI, rob_st.theta2*180/PI, hipA_angle_i, kneeA_angle_i, action);

        move_robot(action);

        wait_for_period(i);
    }

    qlearn_active = 0;
    printf("Q_LEARNING - RUN - STOPPED!\n");

}



//--------------------------------------------------------------------------------
// Q_INIT: Initialize Q-Table to zeros
//--------------------------------------------------------------------------------
void q_init() {
    for (int i = 0; i < NUM_SHOULDER_ANGLES; ++i) 
        for (int j = 0; j < NUM_ELBOW_ANGLES; ++j) 
            for (int k = 0; k < NUM_SHOULDER_ANGLES; ++k)
                for (int l = 0; l < NUM_ELBOW_ANGLES; ++l)       
                    for (int m = 0; m < NUM_ACTIONS; ++m) {
                        q_table[i][j][k][l][m] = 0.0;
                    }

}

//--------------------------------------------------------------------------------
// CHOOSE_ACTION: With probability EPSILON, choose a random action. Else, choose 
// the action with the greatest expected reward.
//--------------------------------------------------------------------------------
int choose_action(int hipA_angle_i, int kneeA_angle_i, int hipB_angle_i, int kneeB_angle_i, double epsilon) {
    if ((double)rand() / RAND_MAX < epsilon) {
        return rand() % NUM_ACTIONS;
    } else {
        return choose_best_action(hipA_angle_i, kneeA_angle_i, hipB_angle_i, kneeB_angle_i);
    }
}

//--------------------------------------------------------------------------------
// CHOOSE_BEST_ACTION: Choose the action with the greatest expected reward.
//--------------------------------------------------------------------------------
int choose_best_action(int hipA_angle_i, int kneeA_angle_i, int hipB_angle_i, int kneeB_angle_i) {
    double max_q_value = -INFINITY;
    int best_action = 0;
    int action;
    double q_value;
    
    // find action with the greatest expected reward
    for (action = 0; action < NUM_ACTIONS; action++) {
        q_value = q_table[hipA_angle_i][kneeA_angle_i][hipB_angle_i][kneeB_angle_i][action];
        if (q_value > max_q_value) {
            max_q_value = q_value;
            best_action = action;
        }
    }

    return best_action;
}

//--------------------------------------------------------------------------------
// FROM_THETA1_TO_INDEX: Receives as input an angle, and gives as output the index
// indicating the position step of the joint
//--------------------------------------------------------------------------------
int from_theta1_to_index(double theta1) {
    return (int)((theta1-THETA1_MIN) / THETA1_INC + 0.01);     // The factor +0.01 is used to avoid problems of finite-precision representation (0.999999957 should be casted to 1)
}

int from_theta2_to_index(double theta2) {
    return (int)((theta2-THETA2_MIN) / THETA2_INC + 0.01);
}

//--------------------------------------------------------------------------------
// CALCULATE_REWARD: Calculates the reward based on how much did the robot move
//--------------------------------------------------------------------------------
double calculate_reward(int abort) {
    if (abort == 1)
        return PENALITY_OOF;
    if (rob_st.x == rob_st_prev.x) 
        return PENALITY_NM;
    if (rob_st.x > rob_st_prev.x)
        return (rob_st.x - rob_st_prev.x)*REWARD_FACTOR;
    return (rob_st.x - rob_st_prev.x)*REWARD_FACTOR*0.7;    // backward movement is penalized less
}

//--------------------------------------------------------------------------------
// PRINTF_QTABLE: Prints on the terminal the values of the q_table
//--------------------------------------------------------------------------------
void printf_qtable(){
    for(int r = 0; r < NUM_SHOULDER_ANGLES; r++){
        // for(int c = 0; c < NUM_ELBOW_ANGLES; c++)
        //     printf("[%1.1f; %1.1f; %1.1f; %1.1f] ; ",q_table[r][c][0], q_table[r][c][1], q_table[r][c][2], q_table[r][c][3]);
        // printf("\n");
    }
}

//--------------------------------------------------------------------------------
// SAVE_Q_TABLE: Saves the Q-Table
//--------------------------------------------------------------------------------
void save_q_table(const char *filename) {
    FILE *file = fopen(filename, "wb");
    if (!file) {
        perror("Failed to open file for writing");
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < NUM_SHOULDER_ANGLES; ++i) {
        for (int j = 0; j < NUM_ELBOW_ANGLES; ++j) {
            fwrite(q_table[i][j], sizeof(double), NUM_ACTIONS, file);
        }
    }
    fclose(file);
}


//--------------------------------------------------------------------------------
// LOAD_Q_TABLE: Loads a Q-Table from a file
//--------------------------------------------------------------------------------
void load_q_table(const char *filename) {
    FILE *file = fopen(filename, "rb");
    if (!file) {
        perror("Failed to open file for reading");
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < NUM_SHOULDER_ANGLES; ++i) {
        for (int j = 0; j < NUM_ELBOW_ANGLES; ++j) {
            fread(q_table[i][j], sizeof(double), NUM_ACTIONS, file);
        }
    }
    fclose(file);
}