
#include "StateMachine.h"



char* STATE_NAME[]={
    "DSP_INIT_RF",
    "DSP_INIT_LF",
    "DSP_RF",
    "SSP_RF",
    "DSP_LF",
    "SSP_LF",
    "DSP_FINAL",
    "EMPTY"
};



_window_element     window[WIN_NUM];
_footprint_info    target_foot[TARGET_FOOT_NUM];
_footprint_info    prev_foot;
int                target_foot_filled = 0;

// for short footprint for test
_footprint_info    short_foot[SHORT_FOOT_NUM];
_footprint_info    last_short_foot;
int                last_moving_leg = MOVING_RIGHT;

int ring_short_head;
int ring_short_tail;

WALKING_STATE current_walking_state;

int push_short_foot(_footprint_info fp){
    int next_head = (ring_short_head+1)%SHORT_FOOT_NUM;
    if(next_head == ring_short_tail){
        // buffer full
        printf("short_foot buffer full\n");
        return false;
    }

    memcpy(&short_foot[ring_short_head], &fp, sizeof(_footprint_info));

    ring_short_head = next_head;

    return true;
}

int pull_short_foot(_footprint_info &fp){
    if(ring_short_tail == ring_short_head){
        // nothing to read
//        printf("short_foot buffer nothing to read\n");
        return false;
    }
    memcpy(&fp, &short_foot[ring_short_tail], sizeof(_footprint_info));
    ring_short_tail = (ring_short_tail+1)%SHORT_FOOT_NUM;

    return true;
}



void zero_window(){
    memset(window, 0, sizeof(_window_element)*WIN_NUM);

}

void zero_localfoot(){
    memset(target_foot, 0, sizeof(_footprint_info)*TARGET_FOOT_NUM);
    target_foot_filled = 0;

//    memset(short_foot, 0, sizeof(_footprint_info)*TARGET_FOOT_NUM);

//    ring_short_head = ring_short_tail = 0;

}



void state_change(WALKING_STATE state){
    current_walking_state = state;
}



float cnt_2_time(long cnt){
    return cnt * TICK_TIME;
}

double foot_difference(double fp1[3], double fp2[3]){
    return sqrt((fp2[0]-fp1[0]) * (fp2[0]-fp1[0]) + (fp2[1]-fp1[1]) * (fp2[1]-fp1[1]));
}

MOVING_LEG check_moving_leg(int start_index){
    // start index = index of target_foot
    const double thresh = 0.0001;
    const double threshD = 0.001;
    double diff_R, diff_L, diff_RY, diff_LY;

    // if target_foot_filled is zero,
    // return MOVING_EMPTY
    if(target_foot_filled == 0)
        return MOVING_EMPTY;

    switch(start_index){
    case -1:
        // prev --> target_foot[0]
        diff_R = foot_difference(target_foot[0].footprint.rfoot, prev_foot.footprint.rfoot);
        diff_L = foot_difference(target_foot[0].footprint.lfoot, prev_foot.footprint.lfoot);
        diff_RY = fabs(target_foot[0].footprint.rori[0] - prev_foot.footprint.rori[0]);
        diff_LY = fabs(target_foot[0].footprint.lori[0] - prev_foot.footprint.lori[0]);
        break;
    case 0:
        // target_foot[0] --> target_foot[1]
        diff_R = foot_difference(target_foot[1].footprint.rfoot, target_foot[0].footprint.rfoot);
        diff_L = foot_difference(target_foot[1].footprint.lfoot, target_foot[0].footprint.lfoot);
        diff_RY = fabs(target_foot[1].footprint.rori[0] - target_foot[0].footprint.rori[0]);
        diff_LY = fabs(target_foot[1].footprint.lori[0] - target_foot[0].footprint.lori[0]);
        break;
    case 1:
        // target_foot[1] --> target_foot[2]
        diff_R = foot_difference(target_foot[2].footprint.rfoot, target_foot[1].footprint.rfoot);
        diff_L = foot_difference(target_foot[2].footprint.lfoot, target_foot[1].footprint.lfoot);
        diff_RY = fabs(target_foot[2].footprint.rori[0] - target_foot[1].footprint.rori[0]);
        diff_LY = fabs(target_foot[2].footprint.lori[0] - target_foot[1].footprint.lori[0]);
        break;
    default:
        printf("check_moving_leg error -- over index(%d)\n", start_index);
        return MOVING_JUMP;
        break;
    }

    if((diff_R > thresh || diff_RY > threshD) && (diff_L > thresh || diff_LY > threshD)){
        printf("JUMP!! %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", diff_R, diff_L, diff_RY, diff_LY,target_foot[0].footprint.rori,target_foot[0].footprint.lori);
        return MOVING_JUMP;
    }
    else if(diff_R > thresh || diff_RY > threshD)
        return MOVING_RIGHT;
    else if(diff_L > thresh || diff_LY > threshD)
        return MOVING_LEFT;
    else
        return MOVING_NOTHING;
}


void update_window_state(long w_cnt){
    double  tempTime;
    int     stateWritten = false;

    for(int i=0; i<WIN_NUM; i++){
        tempTime = 0.0;
        stateWritten = false;

        for(int j=0; j<target_foot_filled; j++){
            if(cnt_2_time(w_cnt) < (tempTime = tempTime + target_foot[j].time.dsp_time)){
                // In the DSP time ---
                switch(check_moving_leg(j-1)){
                case MOVING_RIGHT:
                    if(target_foot[j].info == FOOTINFO_FIRST_STEP)
                        window[i].state = DSP_INIT_RF;
                    else
                        window[i].state = DSP_RF;
                    break;
                case MOVING_LEFT:
                    if(target_foot[j].info == FOOTINFO_FIRST_STEP)
                        window[i].state = DSP_INIT_LF;
                    else
                        window[i].state = DSP_LF;
                    break;
                case MOVING_NOTHING:
                case MOVING_EMPTY:
                    window[i].state = STATE_EMPTY;
                    break;
                }
                if(target_foot[j].info == FOOTINFO_LAST_STEP)
                    window[i].state = DSP_FINAL;

                stateWritten = true;
                break;  // break the for loop
            }else if(cnt_2_time(w_cnt) <= (tempTime = tempTime + target_foot[j].time.ssp_time)){
                // In the SSP time ---
                switch(check_moving_leg(j-1)){
                case MOVING_RIGHT:
                    window[i].state = SSP_RF;
                    break;
                case MOVING_LEFT:
                    window[i].state = SSP_LF;
                    break;
                case MOVING_NOTHING:
                case MOVING_EMPTY:
                    window[i].state = STATE_EMPTY;
                    break;
                }

                stateWritten = true;
                break;  // break the for loop
            }
        }

        if(stateWritten == false){
            window[i].state = STATE_EMPTY;
        }

        w_cnt++;
    }
}



// Trajectory Functions =======================
double trajectory_5th(double cur_time, double tot_time, double cur_pos, double last_pos){
    double polyTemp1, polyTemp2, polyTemp3;
    double goalPosition, goalVelocity, goalAcceleration;
    double currentPosition, currentVelocity, currentAcceleration;
    double trajParam[6];
    double nTime = cur_time/tot_time;

    goalPosition = last_pos;
    currentPosition = cur_pos;

    goalVelocity = goalAcceleration = currentVelocity = currentAcceleration = 0.0;

    polyTemp1 = goalPosition - 0.5*currentAcceleration - currentVelocity - currentPosition;
    polyTemp2 = goalVelocity - currentAcceleration - currentVelocity;
    polyTemp3 = goalAcceleration - currentAcceleration;

    trajParam[0] = 0.5*(polyTemp3 - 6.0*polyTemp2 + 12.0*polyTemp1);
    trajParam[1] = polyTemp2 - 3.0*polyTemp1 - 2.0*trajParam[0];
    trajParam[2] = polyTemp1 - trajParam[0] - trajParam[1];
    trajParam[3] = 0.5*currentAcceleration;
    trajParam[4] = currentVelocity;
    trajParam[5] = currentPosition;


    currentPosition = trajParam[0]*nTime*nTime*nTime*nTime*nTime
                        + trajParam[1]*nTime*nTime*nTime*nTime
                        + trajParam[2]*nTime*nTime*nTime
                        + trajParam[3]*nTime*nTime
                        + trajParam[4]*nTime
                        + trajParam[5];
    currentVelocity = 5.0*trajParam[0]*nTime*nTime*nTime*nTime
                        + 4.0*trajParam[1]*nTime*nTime*nTime
                        + 3.0*trajParam[2]*nTime*nTime
                        + 2.0*trajParam[3]*nTime
                        + trajParam[4];
    currentAcceleration = 20.0*trajParam[0]*nTime*nTime*nTime
                        + 12.0*trajParam[1]*nTime*nTime
                        + 6.0*trajParam[2]*nTime
                        + 2.0*trajParam[3];

    return currentPosition;
}

double trajectory_onecos(double cur_time, double tot_time, double cur_pos, double last_pos){
    double goalPosition, goalVelocity, goalAcceleration;
    double currentPosition, currentVelocity, currentAcceleration;
    double trajParam[2];
    double nTime = cur_time/tot_time;

    goalPosition = last_pos;
    currentPosition = cur_pos;

    trajParam[0] = currentPosition;
    trajParam[1] = goalPosition - currentPosition;


    currentPosition = trajParam[0] + trajParam[1]*0.5*(1.0-cos(PI*nTime));
    currentVelocity = 0.5*trajParam[1]*PI*sin(PI*nTime);
    currentAcceleration = 0.5*trajParam[1]*PI*PI*cos(PI*nTime);

    return currentPosition;
}
// ============================================



void update_window_zmp(long w_cnt){
    double tempTime,curTime=0;
    static int last_foot_before_moving_leg = 0;
    int zmpWritten = false;

    _footprint_info *temp_pre;
    _footprint_info *temp_cur;


    for(int i=0; i<WIN_NUM; i++){
        tempTime = 0.0;
        curTime = cnt_2_time(w_cnt);

        for(int j=0; j<target_foot_filled; j++){

            // prev & cur foot selection
            if(j==0){
                temp_pre = &prev_foot;
                temp_cur = &target_foot[0];
            }else{
                temp_pre = &target_foot[j-1];
                temp_cur = &target_foot[j];
            }


            if(cnt_2_time(w_cnt) <= (tempTime = tempTime + target_foot[j].time.dsp_time)){
                // In the DSP time ---

                // calculate curTime
                curTime = cnt_2_time(w_cnt) - (tempTime - target_foot[j].time.dsp_time);

                switch(check_moving_leg(j-1)){

                case MOVING_RIGHT:
                    if(target_foot[j].info == FOOTINFO_FIRST_STEP){
                        // first step
                        if(cnt_2_time(w_cnt) <= (tempTime-0.1)){
                            // stay
                            window[i].zmp.x = trajectory_5th(curTime, temp_cur->time.dsp_time - 0.1,
                                                            (temp_pre->footprint.rfoot[0] + temp_pre->footprint.lfoot[0])/2.0,
                                                            (temp_pre->footprint.rfoot[0] + temp_pre->footprint.lfoot[0])/2.0);
                            window[i].zmp.y = trajectory_5th(curTime, temp_cur->time.dsp_time - 0.1,
                                                            (temp_pre->footprint.rfoot[1] + temp_pre->footprint.lfoot[1])/2.0,
                                                            (temp_pre->footprint.rfoot[1] + temp_pre->footprint.lfoot[1])/2.0);
                        }else{
                            // move
                            window[i].zmp.x = trajectory_5th(curTime - (tempTime-0.1), 0.1,
                                                            (temp_pre->footprint.rfoot[0] + temp_pre->footprint.lfoot[0])/2.0,
                                                            temp_cur->footprint.lfoot[0]);
                            window[i].zmp.y = trajectory_5th(curTime-(tempTime-0.1), 0.1,
                                                            (temp_pre->footprint.rfoot[1] + temp_pre->footprint.lfoot[1])/2.0,
                                                            temp_cur->footprint.lfoot[1]);
                        }
                    }else{
                        window[i].zmp.x = trajectory_5th(curTime, temp_cur->time.dsp_time,
                                                        temp_pre->footprint.rfoot[0],
                                                        temp_cur->footprint.lfoot[0]);
                        window[i].zmp.y = trajectory_5th(curTime, temp_cur->time.dsp_time,
                                                        temp_pre->footprint.rfoot[1],
                                                        temp_cur->footprint.lfoot[1]);
                    }
                    break;
                case MOVING_LEFT:
                    if(target_foot[j].info == FOOTINFO_FIRST_STEP){
                        // first step
                        if(cnt_2_time(w_cnt) <= (tempTime - 0.1)){
                            // stay
                            window[i].zmp.x = trajectory_5th(curTime, temp_cur->time.dsp_time - 0.1,
                                                            (temp_pre->footprint.rfoot[0] + temp_pre->footprint.lfoot[0])/2.0,
                                                            (temp_pre->footprint.lfoot[0] + temp_pre->footprint.rfoot[0])/2.0);
                            window[i].zmp.y = trajectory_5th(curTime, temp_cur->time.dsp_time - 0.1,
                                                            (temp_pre->footprint.rfoot[1] + temp_pre->footprint.lfoot[1])/2.0,
                                                            (temp_pre->footprint.rfoot[1] + temp_pre->footprint.lfoot[1])/2.0);
                        }else{
                            // move
                            window[i].zmp.x = trajectory_5th(curTime - (tempTime-0.1), 0.1,
                                                            (temp_pre->footprint.rfoot[0] + temp_pre->footprint.lfoot[0])/2.0,
                                                            temp_cur->footprint.rfoot[0]);
                            window[i].zmp.y = trajectory_5th(curTime - (tempTime-0.1), 0.1,
                                                            (temp_pre->footprint.rfoot[1] + temp_pre->footprint.lfoot[1])/2.0,
                                                            temp_cur->footprint.rfoot[1]);
                        }
                    }else{
                        window[i].zmp.x = trajectory_5th(curTime, temp_cur->time.dsp_time,
                                                        temp_pre->footprint.lfoot[0],
                                                        temp_cur->footprint.rfoot[0]);
                        window[i].zmp.y = trajectory_5th(curTime, temp_cur->time.dsp_time,
                                                        temp_pre->footprint.lfoot[1],
                                                        temp_cur->footprint.rfoot[1]);
                    }
                    break;
                case MOVING_NOTHING:
                case MOVING_EMPTY:
                    window[i].zmp.x = (temp_pre->footprint.rfoot[0] + temp_pre->footprint.lfoot[0])/2.0;
                    window[i].zmp.y = (temp_pre->footprint.rfoot[1] + temp_pre->footprint.lfoot[1])/2.0;
                    break;
                }

                if(target_foot[j].info != FOOTINFO_LAST_STEP){
                    last_foot_before_moving_leg = check_moving_leg(j-1);
                }

                if(target_foot[j].info == FOOTINFO_LAST_STEP){
                    // last step
//                                        printf("last step is right!!!!!!!!!!!!!!!!!!!!!!! \n");
                    if(last_foot_before_moving_leg == MOVING_RIGHT){
                        //
                        // last moving leg is right
                        if(curTime < 0.1){
                            if(curTime < 0.01)
//                              /  printf("%d  last step is right!!!!!!!!!!!!!!!!!!!!!!! \n",last_foot_before_moving_leg);
                            // move
                            window[i].zmp.x = trajectory_5th(curTime, 0.1,
                                                            temp_cur->footprint.lfoot[0],
                                                            (temp_cur->footprint.lfoot[0] + temp_cur->footprint.rfoot[0])/2.0);
                            window[i].zmp.y = trajectory_5th(curTime, 0.1,
                                                            temp_cur->footprint.lfoot[1],
                                                            (temp_cur->footprint.lfoot[1] + temp_cur->footprint.rfoot[1])/2.0);
                        }else{
                            // stay
                            window[i].zmp.x = trajectory_5th(curTime, temp_cur->time.dsp_time - 0.1,
                                                            (temp_cur->footprint.lfoot[0] + temp_cur->footprint.rfoot[0])/2.0,
                                                            (temp_cur->footprint.lfoot[0] + temp_cur->footprint.rfoot[0])/2.0);
                            window[i].zmp.y = trajectory_5th(curTime, temp_cur->time.dsp_time - 0.1,
                                                            (temp_cur->footprint.lfoot[1] + temp_cur->footprint.rfoot[1])/2.0,
                                                            (temp_cur->footprint.lfoot[1] + temp_cur->footprint.rfoot[1])/2.0);
                        }
                    }else if(last_foot_before_moving_leg == MOVING_LEFT){
                        //
                        // last moving leg is left
                        if(curTime < 0.1){
                            if(curTime < 0.01)
//                                printf("%d  last step is left!!!!!!!!!!!!!!!!!!!!!!! \n",last_foot_before_moving_leg);
                            // move
                            window[i].zmp.x = trajectory_5th(curTime, 0.1,
                                                            temp_cur->footprint.rfoot[0],
                                                            (temp_cur->footprint.lfoot[0] + temp_cur->footprint.rfoot[0])/2.0);
                            window[i].zmp.y = trajectory_5th(curTime, 0.1,
                                                            temp_cur->footprint.rfoot[1],
                                                            (temp_cur->footprint.lfoot[1] + temp_cur->footprint.rfoot[1])/2.0);
                        }else{
                            // stay
                            window[i].zmp.x = trajectory_5th(curTime, temp_cur->time.dsp_time - 0.1,
                                                            (temp_cur->footprint.lfoot[0] + temp_cur->footprint.rfoot[0])/2.0,
                                                            (temp_cur->footprint.lfoot[0] + temp_cur->footprint.rfoot[0])/2.0);
                            window[i].zmp.y = trajectory_5th(curTime, temp_cur->time.dsp_time - 0.1,
                                                            (temp_cur->footprint.lfoot[1] + temp_cur->footprint.rfoot[1])/2.0,
                                                            (temp_cur->footprint.lfoot[1] + temp_cur->footprint.rfoot[1])/2.0);
                        }
                    }else
                    {
//                        printf("last foot  %d   !!! \n",last_foot_before_moving_leg);
                        if(last_foot_before_moving_leg == MOVING_RIGHT){
//                        printf("%d Empty last step before is right!!!!!!!!!!!!!!!!!!!!!!! \n",la
//st_foot_before_moving_leg);
                            // last moving leg is right
                            if(curTime < 0.1){
                                // move
                                window[i].zmp.x = trajectory_5th(curTime, 0.1,
                                                                temp_cur->footprint.lfoot[0],
                                                                (temp_cur->footprint.lfoot[0] + temp_cur->footprint.rfoot[0])/2.0);
                                window[i].zmp.y = trajectory_5th(curTime, 0.1,
                                                                temp_cur->footprint.lfoot[1],
                                                                (temp_cur->footprint.lfoot[1] + temp_cur->footprint.rfoot[1])/2.0);
                            }else{
                                // stay
                                window[i].zmp.x = trajectory_5th(curTime, temp_cur->time.dsp_time - 0.1,
                                                                (temp_cur->footprint.lfoot[0] + temp_cur->footprint.rfoot[0])/2.0,
                                                                (temp_cur->footprint.lfoot[0] + temp_cur->footprint.rfoot[0])/2.0);
                                window[i].zmp.y = trajectory_5th(curTime, temp_cur->time.dsp_time - 0.1,
                                                                (temp_cur->footprint.lfoot[1] + temp_cur->footprint.rfoot[1])/2.0,
                                                                (temp_cur->footprint.lfoot[1] + temp_cur->footprint.rfoot[1])/2.0);
                            }
                        }else if(last_foot_before_moving_leg == MOVING_LEFT){
//                            printf("%d  Empty last step is before left!!!!!!!!!!!!!!!!!!!!!!! \n",last_foot_before_moving_leg);
                            // last moving leg is left
                            if(curTime < 0.1){
                                // move
                                window[i].zmp.x = trajectory_5th(curTime, 0.1,
                                                                temp_cur->footprint.rfoot[0],
                                                                (temp_cur->footprint.lfoot[0] + temp_cur->footprint.rfoot[0])/2.0);
                                window[i].zmp.y = trajectory_5th(curTime, 0.1,
                                                                temp_cur->footprint.rfoot[1],
                                                                (temp_cur->footprint.lfoot[1] + temp_cur->footprint.rfoot[1])/2.0);
                            }else{
                                // stay
                                window[i].zmp.x = trajectory_5th(curTime, temp_cur->time.dsp_time - 0.1,
                                                                (temp_cur->footprint.lfoot[0] + temp_cur->footprint.rfoot[0])/2.0,
                                                                (temp_cur->footprint.lfoot[0] + temp_cur->footprint.rfoot[0])/2.0);
                                window[i].zmp.y = trajectory_5th(curTime, temp_cur->time.dsp_time - 0.1,
                                                                (temp_cur->footprint.lfoot[1] + temp_cur->footprint.rfoot[1])/2.0,
                                                                (temp_cur->footprint.lfoot[1] + temp_cur->footprint.rfoot[1])/2.0);
                            }
                        }
                    }
                }

                zmpWritten = true;
                break;  // break the for loop
            }else if(cnt_2_time(w_cnt) <= (tempTime = tempTime + target_foot[j].time.ssp_time)){
                // In the SSP time ---

                // calculate curTime
                curTime = cnt_2_time(w_cnt) - (tempTime - target_foot[j].time.ssp_time);

                switch(check_moving_leg(j-1)){
                case MOVING_RIGHT:
                    window[i].zmp.x = temp_cur->footprint.lfoot[0];
                    window[i].zmp.y = temp_cur->footprint.lfoot[1];
                    break;
                case MOVING_LEFT:
                    window[i].zmp.x = temp_cur->footprint.rfoot[0];
                    window[i].zmp.y = temp_cur->footprint.rfoot[1];
                    break;
                case MOVING_NOTHING:
                case MOVING_EMPTY:
                    window[i].zmp.x = (temp_pre->footprint.rfoot[0] + temp_pre->footprint.lfoot[0])/2.0;
                    window[i].zmp.y = (temp_pre->footprint.rfoot[1] + temp_pre->footprint.lfoot[1])/2.0;
                    break;
                }

                zmpWritten = true;
                break;  // break the for loop
            }
        }

        if(zmpWritten == false){
            if(target_foot_filled == 0){
                temp_pre = &prev_foot;
            }else{
                temp_pre = &target_foot[target_foot_filled-1];
            }

            window[i].zmp.x = (temp_pre->footprint.rfoot[0] + temp_pre->footprint.lfoot[0])/2.0;
            window[i].zmp.y = (temp_pre->footprint.rfoot[1] + temp_pre->footprint.lfoot[1])/2.0;
        }

        w_cnt++;
    }
}



void update_window_foot_ref(long w_cnt){
    double height = 0.08,tempTime,curTime=0;
    int refWritten = false;

    _footprint_info *temp_pre;
    _footprint_info *temp_cur;

    for(int i=0; i<WIN_NUM; i++){
        tempTime = 0.0;
        curTime = cnt_2_time(w_cnt);

        for(int j=0; j<target_foot_filled; j++){

            // prev & cur foot selection
            if(j==0){
                temp_pre = &prev_foot;
                temp_cur = &target_foot[0];
            }else{
                temp_pre = &target_foot[j-1];
                temp_cur = &target_foot[j];
            }

            if(cnt_2_time(w_cnt) <= (tempTime = tempTime + target_foot[j].time.dsp_time)){
                // In the DSP time ---

                // calculate curTime
                curTime = cnt_2_time(w_cnt) - (tempTime - target_foot[j].time.dsp_time);

                switch(check_moving_leg(j-1)){
                case MOVING_RIGHT:
                    if(target_foot[j].info == FOOTINFO_FIRST_STEP){
                        // first step
                        if(cnt_2_time(w_cnt) <= (tempTime-0.1)){    // stay
                        }else{                                      // move
                        }
                    }else{
                    }
                case MOVING_LEFT:
                    if(target_foot[j].info == FOOTINFO_FIRST_STEP){
                        // first step
                        if(cnt_2_time(w_cnt) <= (tempTime-0.1)){    // stay
                        }else{                                      // move
                        }
                    }else{
                    }
                case MOVING_NOTHING:
                case MOVING_EMPTY:

                    window[i].left_foot_ref.x       = temp_pre->footprint.lfoot[0];
                    window[i].left_foot_ref.y       = temp_pre->footprint.lfoot[1];
                    window[i].left_foot_ref.z       = temp_pre->footprint.lfoot[2];
                    window[i].left_foot_ref.yaw     = temp_pre->footprint.lori[0];
                    window[i].left_foot_ref.roll    = temp_pre->footprint.lori[1];
                    window[i].left_foot_ref.pitch   = temp_pre->footprint.lori[2];

                    window[i].right_foot_ref.x      = temp_pre->footprint.rfoot[0];
                    window[i].right_foot_ref.y      = temp_pre->footprint.rfoot[1];
                    window[i].right_foot_ref.z      = temp_pre->footprint.rfoot[2];
                    window[i].right_foot_ref.yaw    = temp_pre->footprint.rori[0];
                    window[i].right_foot_ref.roll   = temp_pre->footprint.rori[1];
                    window[i].right_foot_ref.pitch  = temp_pre->footprint.rori[2];

                    break;
                }

                refWritten = true;
                break;  // break the for loop

            }else if(cnt_2_time(w_cnt) <= (tempTime = tempTime + target_foot[j].time.ssp_time)){
                // In the SSP time ---

                // calculate curTime
                curTime = cnt_2_time(w_cnt) - (tempTime - target_foot[j].time.ssp_time);

                switch(check_moving_leg(j-1)){
                case MOVING_RIGHT:

                    window[i].right_foot_ref.x = trajectory_5th(curTime, temp_cur->time.ssp_time,
                                                                temp_pre->footprint.rfoot[0],
                                                                temp_cur->footprint.rfoot[0]);
                    window[i].right_foot_ref.y = trajectory_5th(curTime, temp_cur->time.ssp_time,
                                                                temp_pre->footprint.rfoot[1],
                                                                temp_cur->footprint.rfoot[1]);

                    window[i].right_foot_ref.yaw = trajectory_5th(curTime, temp_cur->time.ssp_time,
                                                                temp_pre->footprint.rori[0],
                                                                temp_cur->footprint.rori[0]);
                    window[i].right_foot_ref.roll = trajectory_5th(curTime, temp_cur->time.ssp_time,
                                                                temp_pre->footprint.rori[1],
                                                                temp_cur->footprint.rori[1]);
                    window[i].right_foot_ref.pitch= trajectory_5th(curTime, temp_cur->time.ssp_time,
                                                                temp_pre->footprint.rori[2],
                                                                temp_cur->footprint.rori[2]);


                    if(curTime <= temp_cur->time.ssp_time/2.0){
                        // leg up
                        window[i].right_foot_ref.z = trajectory_5th(curTime, temp_cur->time.ssp_time/2.0,
                                                                temp_pre->footprint.rfoot[2],
                                                                temp_pre->footprint.rfoot[2]+ height);
                    }else{
                        // leg down
                        window[i].right_foot_ref.z = trajectory_5th(curTime - temp_cur->time.ssp_time/2.0, temp_cur->time.ssp_time/2.0,
                                                                temp_pre->footprint.rfoot[2]+ height,
                                                                temp_pre->footprint.rfoot[2]);
                    }

                    window[i].left_foot_ref.x = prev_foot.footprint.lfoot[0];
                    window[i].left_foot_ref.y = prev_foot.footprint.lfoot[1];
                    window[i].left_foot_ref.z = prev_foot.footprint.lfoot[2];
                    window[i].left_foot_ref.yaw = prev_foot.footprint.lori[0];
                    window[i].left_foot_ref.roll = prev_foot.footprint.lori[1];
                    window[i].left_foot_ref.pitch= prev_foot.footprint.lori[2];

                    break;
                case MOVING_LEFT:

                    window[i].left_foot_ref.x = trajectory_5th(curTime, temp_cur->time.ssp_time,
                                                                temp_pre->footprint.lfoot[0],
                                                                temp_cur->footprint.lfoot[0]);
                    window[i].left_foot_ref.y = trajectory_5th(curTime, temp_cur->time.ssp_time,
                                                                temp_pre->footprint.lfoot[1],
                                                                temp_cur->footprint.lfoot[1]);

                    window[i].left_foot_ref.yaw = trajectory_5th(curTime, temp_cur->time.ssp_time,
                                                                temp_pre->footprint.lori[0],
                                                                temp_cur->footprint.lori[0]);
                    window[i].left_foot_ref.roll = trajectory_5th(curTime, temp_cur->time.ssp_time,
                                                                temp_pre->footprint.lori[1],
                                                                temp_cur->footprint.lori[1]);
                    window[i].left_foot_ref.pitch= trajectory_5th(curTime, temp_cur->time.ssp_time,
                                                                temp_pre->footprint.lori[2],
                                                                temp_cur->footprint.lori[2]);


                    if(curTime <= temp_cur->time.ssp_time/2.0){
                        // leg up
                        window[i].left_foot_ref.z = trajectory_5th(curTime, temp_cur->time.ssp_time/2.0,
                                                                temp_pre->footprint.lfoot[2],
                                                                temp_pre->footprint.lfoot[2]+ height);
                    }else{
                        // leg down
                        window[i].left_foot_ref.z = trajectory_5th(curTime - temp_cur->time.ssp_time/2.0, temp_cur->time.ssp_time/2.0,
                                                                temp_pre->footprint.lfoot[2]+ height,
                                                                temp_pre->footprint.lfoot[2]);
                    }

                    window[i].right_foot_ref.x = prev_foot.footprint.rfoot[0];
                    window[i].right_foot_ref.y = prev_foot.footprint.rfoot[1];
                    window[i].right_foot_ref.z = prev_foot.footprint.rfoot[2];
                    window[i].right_foot_ref.yaw = prev_foot.footprint.rori[0];
                    window[i].right_foot_ref.roll = prev_foot.footprint.rori[1];
                    window[i].right_foot_ref.pitch= prev_foot.footprint.rori[2];

                    break;
                case MOVING_NOTHING:
                case MOVING_EMPTY:

                    window[i].left_foot_ref.x       = temp_pre->footprint.lfoot[0];
                    window[i].left_foot_ref.y       = temp_pre->footprint.lfoot[1];
                    window[i].left_foot_ref.z       = temp_pre->footprint.lfoot[2];
                    window[i].left_foot_ref.yaw     = temp_pre->footprint.lori[0];
                    window[i].left_foot_ref.roll    = temp_pre->footprint.lori[1];
                    window[i].left_foot_ref.pitch   = temp_pre->footprint.lori[2];

                    window[i].right_foot_ref.x      = temp_pre->footprint.rfoot[0];
                    window[i].right_foot_ref.y      = temp_pre->footprint.rfoot[1];
                    window[i].right_foot_ref.z      = temp_pre->footprint.rfoot[2];
                    window[i].right_foot_ref.yaw    = temp_pre->footprint.rori[0];
                    window[i].right_foot_ref.roll   = temp_pre->footprint.rori[1];
                    window[i].right_foot_ref.pitch  = temp_pre->footprint.rori[2];

                    break;
                }

                refWritten = true;
                break;  // break the for loop
            }
        }

        if(refWritten == false){
            if(target_foot_filled == 0){
                temp_pre = &prev_foot;
            }else{
                temp_pre = &target_foot[target_foot_filled-1];
            }

            window[i].left_foot_ref.x       = temp_pre->footprint.lfoot[0];
            window[i].left_foot_ref.y       = temp_pre->footprint.lfoot[1];
            window[i].left_foot_ref.z       = temp_pre->footprint.lfoot[2];
            window[i].left_foot_ref.yaw     = temp_pre->footprint.lori[0];
            window[i].left_foot_ref.roll    = temp_pre->footprint.lori[1];
            window[i].left_foot_ref.pitch   = temp_pre->footprint.lori[2];

            window[i].right_foot_ref.x      = temp_pre->footprint.rfoot[0];
            window[i].right_foot_ref.y      = temp_pre->footprint.rfoot[1];
            window[i].right_foot_ref.z      = temp_pre->footprint.rfoot[2];
            window[i].right_foot_ref.yaw    = temp_pre->footprint.rori[0];
            window[i].right_foot_ref.roll   = temp_pre->footprint.rori[1];
            window[i].right_foot_ref.pitch  = temp_pre->footprint.rori[2];
        }

        w_cnt++;

    }
}

void update_window(){
    static long window_cnt = 0;
    static long save_cnt = 0;

    // check if the target_foot is filled.
    // if not, check it can be filled from short_foot
    // and fill it if it is possible.
    if(target_foot_filled < TARGET_FOOT_NUM){
        for(int i=target_foot_filled; i<TARGET_FOOT_NUM; i++){
            _footprint_info new_footprint;

            if(pull_short_foot(new_footprint) == false){
                // there is no available short_foot
                break;
            }else{
                FILE_LOG(logWARNING) << "ERROR";
                // if the target_foot_filled is zero,
                // it means that this is the first step of walking.
                if(target_foot_filled == 0){
                    new_footprint.info = FOOTINFO_FIRST_STEP;
                    new_footprint.time.dsp_time += 0.4;
                }

                // update the last copied short_foot
                memcpy(&last_short_foot, &new_footprint, sizeof(_footprint_info));
                // add pulled footprint to target_foot
                memcpy(&target_foot[i], &new_footprint, sizeof(_footprint_info));
                // increase target_foot_filled
                target_foot_filled = i+1;
            }
        }
    }

    // if there is no target_foot information,
    // just return this function
    if(target_foot_filled == 0){
        return;
    }

    // update target_foot if the given time has been passed.
    // also update the prev_foot to target_foot[0]
    if(target_foot[0].time.dsp_time + target_foot[0].time.ssp_time < cnt_2_time(window_cnt)){

        //printf("=== %d === %.4f, %.4f === %.4f, %.4f\n", prev_foot.info, prev_foot.footprint.rfoot[0], prev_foot.footprint.rfoot[1], prev_foot.footprint.lfoot[0], prev_foot.footprint.lfoot[1]);

        // update prev_foot
        memcpy(&prev_foot, &target_foot[0], sizeof(_footprint_info));

        printf("%.4f, %.4f, %.4f, %.4f\n", prev_foot.footprint.rfoot[0], prev_foot.footprint.rfoot[1], prev_foot.footprint.lfoot[0], prev_foot.footprint.lfoot[1]);

        // get new footprint here
        _footprint_info new_footprint;
        if(pull_short_foot(new_footprint) == false){
            // move forward
            for(int i=1; i<target_foot_filled; i++){
                memcpy(&target_foot[i-1], &target_foot[i], sizeof(_footprint_info));
            }

            // there is no available short_foot
            // decrease target_foot_filled
            target_foot_filled--;

        }else{
            FILE_LOG(logWARNING) << "ERROR";
            // update the last copied short_foot
            memcpy(&last_short_foot, &new_footprint, sizeof(_footprint_info));

            // move forward
            for(int i=1; i<target_foot_filled; i++){
                memcpy(&target_foot[i-1], &target_foot[i], sizeof(_footprint_info));
            }

            // insert new footprint information
            // target_foot_filled is preserved
            memcpy(&target_foot[target_foot_filled-1], &new_footprint, sizeof(_footprint_info));

        }

        // update the last_moving_leg
        if(target_foot_filled >= 1)
            last_moving_leg = check_moving_leg(target_foot_filled-2);

        // start new window_cnt
        window_cnt = 0;
    }

    // check the last step
    for(int i=0; i<target_foot_filled; i++){
        if(target_foot[i].info == FOOTINFO_FIRST_STEP)
            continue;

        // target_foot is full
        if(target_foot_filled == TARGET_FOOT_NUM){
            if(target_foot[i].info == FOOTINFO_LAST_STEP){
                target_foot[i].time.dsp_time -= 0.4;
            }
            target_foot[i].info = FOOTINFO_NO;
        }else{
            if(i == target_foot_filled-1){
                if(target_foot[i].info != FOOTINFO_LAST_STEP){
                    target_foot[i].info = FOOTINFO_LAST_STEP;
                    target_foot[i].time.dsp_time += 0.4;
                    printf("!!!!!!!!!!!! LAST STEP !!!!!!!!!!!!!!!!!!!  %d\n", i);
                    printf("!!!!!!!!!!!! LAST STEP !!!!!!!!!!!!!!!!!!!  %d\n", i);
                    printf("!!!!!!!!!!!! LAST STEP !!!!!!!!!!!!!!!!!!!  %d\n", i);
                }
            }else{
                if(target_foot[i].info == FOOTINFO_LAST_STEP){
                    target_foot[i].time.dsp_time -= 0.4;
                }
                target_foot[i].info = FOOTINFO_NO;
            }
        }
    }

//    // check the empty & nothing thing!!
//    for(int i=1; i<target_foot_filled; i++){
//        int prev_moving_leg = check_moving_leg(i-2);
//        int cur_moving_leg = check_moving_leg(i-1);

//        if(target_foot[i].info == FOOTINFO_LAST_STEP || target_foot[i].info == FOOTINFO_FIRST_STEP)
//            continue;

//        if(prev_moving_leg != MOVING_NOTHING && cur_moving_leg == MOVING_NOTHING){
//            FILE_LOG(logERROR) << "LAST LAST!! " << prev_moving_leg << ", " << cur_moving_leg << ", :::" << i;
//            target_foot[i].info = FOOTINFO_LAST_STEP;
//        }else if(prev_moving_leg == MOVING_NOTHING && cur_moving_leg != MOVING_NOTHING){
//            FILE_LOG(logWARNING) << "FIRST FIRST!!" << prev_moving_leg << ", " << cur_moving_leg << ", :::" << i;
//            target_foot[i].info = FOOTINFO_FIRST_STEP;
//        }
//    }


    //printf("[%4d] ", window_cnt);

    update_window_state(window_cnt);

    update_window_zmp(window_cnt);

    update_window_foot_ref(window_cnt);


//    if(save_cnt % 300 == 0){
//        FILE *fp = fopen("data.txt","w");

//        for(int i=0;i<WIN_NUM;i++)
//        {
//            fprintf(fp,"%g, %g\t%g\t%g\t%g\t%g\t%g\t%g\t%g\n", cnt_2_time(save_cnt+i), window[i].zmp.x, window[i].zmp.y, window[i].right_foot_ref.x, window[i].right_foot_ref.y, window[i].right_foot_ref.z, window[i].left_foot_ref.x, window[i].left_foot_ref.y, window[i].left_foot_ref.z);
//        }

//        fclose(fp);

//        printf("\n");
//    }
    save_cnt++;
    // update window here
    // ..
    // ..
    // ..
    // ..
    // ..
    // ===================

    state_change(window[0].state);

    static long state_cnt[8] = {0,};

    for(int k=0;k<STATE_EMPTY;k++)
    {
        if(k!=window[0].state){
            state_cnt[k] = 0;
        }
    }
    window[0].timer.current = (double)(state_cnt[window[0].state])*0.005;

    state_cnt[current_walking_state] ++;

    window_cnt++;

}

void state_machine(){
    static long state_cnt[8] = {0,};

    for(int i=0;i<STATE_EMPTY;i++)
    {
        if(i!=current_walking_state){
            state_cnt[i] = 0;
        }
    }
    window[0].timer.current = state_cnt[current_walking_state];

    switch(current_walking_state){
    case DSP_INIT_RF:

        // insert your algorithm here
        // ..
        // ..
        // ..
        // ..
        // ..
        // ===================

        break;
    case DSP_INIT_LF:

        // insert your algorithm here
        // ..
        // ..
        // ..
        // ..
        // ..
        // ===================

        break;
    case DSP_FINAL:

        // insert your algorithm here
        // ..
        // ..
        // ..
        // ..
        // ..
        // ===================

        break;
    case DSP_RF:

        // insert your algorithm here
        // ..
        // ..
        // ..
        // ..
        // ..
        // ===================

        break;
    case DSP_LF:

        // insert your algorithm here
        // ..
        // ..
        // ..
        // ..
        // ..
        // ===================

        break;
    case SSP_RF:

        // insert your algorithm here
        // ..
        // ..
        // ..
        // ..
        // ..
        // ===================

        break;
    case SSP_LF:

        // insert your algorithm here
        // ..
        // ..
        // ..
        // ..
        // ..
        // ===================

        break;
    default:
        break;
    }

    state_cnt[current_walking_state] ++;
}









//void save()
//{
//    if(Save_Index < ROW)
//    {
//            Save_Data[0][Save_Index] = window[0].zmp.x;
//            Save_Data[1][Save_Index] = window[1].zmp.x;
//            Save_Data[2][Save_Index] = window[299].zmp.x;
//            Save_Data[3][Save_Index] = window[0].zmp.y;
//            Save_Data[4][Save_Index] = window[1].zmp.y;
//            Save_Data[5][Save_Index] = window[299].zmp.y;
//            Save_Data[6][Save_Index] = window[0].state;
//            Save_Data[7][Save_Index] = window[0].right_foot_ref.x;
//            Save_Data[8][Save_Index] = window[0].right_foot_ref.y;
//            Save_Data[9][Save_Index] = window[0].right_foot_ref.z;
//            Save_Data[10][Save_Index] = window[0].left_foot_ref.x;
//            Save_Data[11][Save_Index] = window[0].left_foot_ref.y;
//            Save_Data[12][Save_Index] = window[0].left_foot_ref.z;

//            Save_Index++;

//            if(Save_Index >= ROW) Save_Index = 0;

//    }
//}
