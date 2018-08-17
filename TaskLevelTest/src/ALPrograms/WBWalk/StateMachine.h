#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <string.h>
#include <math.h>
#include <stdio.h>

#include "Definitions.h"
#include "RBLog.h"

extern _window_element     window[WIN_NUM];
extern _footprint_info    target_foot[TARGET_FOOT_NUM];
extern _footprint_info    prev_foot;

// for short footprint for test
extern _footprint_info    short_foot[SHORT_FOOT_NUM];
extern int ring_short_head;
extern int ring_short_tail;
extern int approach_last_flag;
extern WALKING_STATE current_walking_state;

extern int FLAG_SingleLog;


#define D2R 3.141592/180.0
#define R2D 180.0/3.141592

int push_short_foot(_footprint_info fp);
int pull_short_foot(_footprint_info &fp);
void zero_window();
void zero_localfoot();
void init_targetfoot();
void targetfoot_update(_footprint_info new_footprint);
void state_change(WALKING_STATE state);
float cnt_2_time(long cnt);
double foot_difference(double fp1[3], double fp2[3]);
MOVING_LEG check_moving_leg(int start_index);
void update_window_state(long w_cnt);
double trajectory_5th(double cur_time, double tot_time, double cur_pos, double last_pos);
double trajectory_onecos(double cur_time, double tot_time, double cur_pos, double last_pos);
void update_window_zmp(long w_cnt);
void update_window_foot_ref(long w_cnt);
void update_window_foot_ref_singlelog(long w_cnt);
void update_window();
void save();
#endif // STATEMACHINE_H
