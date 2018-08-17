#include "approachbox.h"

/****************************** 4. Foot Print Generator *********************/
void FPG_TEST(int dir,int step_num,double step_length,double step_angle,double step_offset,double LPEL2PEL)
{
    // example short footprint information =============================

    zero_window();

    zero_localfoot();

    if(step_angle>15.0)
    {
        step_angle = 15.0;
    }else if(step_angle<-15.0)
    {
        step_angle = -15.0;
    }

    step_angle = step_angle*D2R;

    double rl = 0;

    int right_left = 0;

    if(dir == APPROACHBOX_RIGHT_WALKING){
        right_left = 0;
    }else if(dir == APPROACHBOX_LEFT_WALKING){
        right_left  = 1;
    }

    printf("***********step angle: %f     step offset: %f  \n",step_angle,step_offset);

    _footprint_info dummyfoot;
    // remove old foot printf;
    while(pull_short_foot(dummyfoot)){
        ;
    }

    _footprint_info tempFoot;

    tempFoot.footprint.rfoot[0] = FK_pRFoot_3x1[0];//0.0;
    tempFoot.footprint.rfoot[1] = -0.105;//FK_pRFoot_3x1[1];//-0.105;
    tempFoot.footprint.rfoot[2] = FK_pRFoot_3x1[2];//0.0;

    tempFoot.footprint.rori[0] = FK_RFoot_yaw*R2D;//FK_pRFoot_3x1[0];//0.0;
    tempFoot.footprint.rori[1] = FK_RFoot_roll*R2D;//FK_pRFoot_3x1[1];//-0.105;
    tempFoot.footprint.rori[2] = FK_RFoot_pitch*R2D;//FK_pRFoot_3x1[2];//0.0;


    tempFoot.footprint.lfoot[0] = FK_pLFoot_3x1[0];//0.0;
    tempFoot.footprint.lfoot[1] = FK_pLFoot_3x1[1];//0.105;
    tempFoot.footprint.lfoot[2] = FK_pLFoot_3x1[2];

    tempFoot.footprint.lori[0] = FK_LFoot_yaw*R2D;//FK_pLFoot_3x1[0];//0.0;
    tempFoot.footprint.lori[1] = FK_LFoot_roll*R2D;//FK_pLFoot_3x1[1];//0.105;
    tempFoot.footprint.lori[2] = FK_LFoot_pitch*R2D;//FK_pLFoot_3x1[2];

    printf(">>>>>>>>>> Foot Print Initial Value <<<<<<<<<<< \n");
    printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
    printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

    tempFoot.time.dsp_time = 0.0;
    tempFoot.time.ssp_time = 0.0;
    tempFoot.info = FOOTINFO_NO;//FOOTINFO_FIRST_STEP;

    memcpy(&prev_foot, &tempFoot, sizeof(_footprint_info));

    int test_short_foot_num = step_num;

    if(dir != APPROACHBOX_FORWARD_WALKING)
    {
        if(step_num%2 == 0)
        {
            step_num = step_num+1;
        }

        test_short_foot_num = step_num;

        if(test_short_foot_num < 3)
        {
            test_short_foot_num = 3;
        }

    }


    if(step_offset<0.25)
    {
        step_offset = 0.25;
    }

    int temp_index = 0;
    int temp_head = ring_short_head;

    printf("short foot num:%d     Rinf Short head : %d     Rinf Short tail : %d\n",test_short_foot_num,ring_short_head,ring_short_tail);

    if(dir == APPROACHBOX_FORWARD_WALKING)
    {
        FILE_LOG(logSUCCESS) << "Forward Foot print Generation  ";

        for(int i=0; i<test_short_foot_num; i++){
            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length;
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length;
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                tempFoot.time.dsp_time = 1.2;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-2){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] =tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }

                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;
            }else{
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length;
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length;
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                //FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];
                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;
            }

            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);



            push_short_foot(tempFoot);
            right_left ^= 1;
        }

    }else if(dir == APPROACHBOX_LEFT_WALKING)
    {
        FILE_LOG(logSUCCESS) << "Left Foot print Generation";
        for(int i=0; i<test_short_foot_num; i++){
            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1] + step_length;
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1] + step_length;
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1)
            {
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;
            }else
            {
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1] + step_length;
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];


                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1] + step_length;
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];
                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;
            }
            push_short_foot(tempFoot);
            right_left ^= 1;


        }
    }else if(dir == APPROACHBOX_RIGHT_WALKING)
    {
        FILE_LOG(logSUCCESS) << "Right Foot print Generation  ";
        for(int i=0; i<test_short_foot_num; i++){
            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1] - step_length;
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1] - step_length;
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;
            }else{
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1] - step_length;
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1] - step_length;
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];
                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;
            }

            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

            push_short_foot(tempFoot);
            right_left ^= 1;


        }
    }else if(dir == APPROACHBOX_CCW_WALKING)
    {


        FILE_LOG(logSUCCESS) << "CCW Foot print Generation  ";

        right_left = 1;

        for(int i=0; i<test_short_foot_num; i++){

            if(right_left == 1)
            {
                //rotate left direction ,, left foot first.
                rl = 1;

            }else
            {
                rl = -1;
            }

            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rori[0] = tempFoot.footprint.lori[0] + step_angle*R2D;
                    tempFoot.footprint.rori[1] = tempFoot.footprint.rori[1];
                    tempFoot.footprint.rori[2] = tempFoot.footprint.rori[2];

                    tempFoot.footprint.lori[0] = tempFoot.footprint.lori[0];
                    tempFoot.footprint.lori[1] = tempFoot.footprint.lori[1];
                    tempFoot.footprint.lori[2] = tempFoot.footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length*cos(tempFoot.footprint.rori[0]*D2R) - step_offset*sin(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.lfoot[1] + step_length*sin(tempFoot.footprint.rori[0]*D2R) + step_offset*cos(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else
                {
                    tempFoot.footprint.rori[0] = tempFoot.footprint.rori[0];
                    tempFoot.footprint.rori[1] = tempFoot.footprint.rori[1];
                    tempFoot.footprint.rori[2] = tempFoot.footprint.rori[2];

                    tempFoot.footprint.lori[0] = tempFoot.footprint.rori[0] + step_angle*R2D;
                    tempFoot.footprint.lori[1] = tempFoot.footprint.lori[1];
                    tempFoot.footprint.lori[2] = tempFoot.footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length*cos(tempFoot.footprint.lori[0]*D2R) - step_offset*sin(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.rfoot[1] + step_length*sin(tempFoot.footprint.lori[0]*D2R) + step_offset*cos(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }

                FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.8;



             }
            else if(i == test_short_foot_num-2){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + LPEL2PEL*sin(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] - LPEL2PEL*cos(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] + LPEL2PEL*sin(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + LPEL2PEL*cos(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];//  + LPEL2PEL*sin(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];//  - LPEL2PEL*cos(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];//  + LPEL2PEL*sin(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];//  + LPEL2PEL*cos(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;

            }else{
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0] + step_angle*R2D;
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + step_length*cos(tempFoot.footprint.rori[0]*D2R) - step_offset*sin(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] + step_length*sin(tempFoot.footprint.rori[0]*D2R) + step_offset*cos(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0] + step_angle*R2D;
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] + step_length*cos(tempFoot.footprint.lori[0]*D2R) - step_offset*sin(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + step_length*sin(tempFoot.footprint.lori[0]*D2R) + step_offset*cos(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];

                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;
            }

            printf(">>>>>>>>>> %d Foot Print <<<<<<<<<<< \n",i);

            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

            push_short_foot(tempFoot);
            right_left ^= 1;
        }
    }else if(dir == APPROACHBOX_CW_WALKING)
    {
        if(step_angle > 0)
        {
            step_angle  = -step_angle;
        }

        FILE_LOG(logSUCCESS) << "CW Foot print Generation  ";

        right_left = 0;

        for(int i=0; i<test_short_foot_num; i++){

            if(right_left == 1)
            {
                //rotate left direction ,, left foot first.
                rl = 1;
            }else
            {
                rl = -1;
            }

            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rori[0] = tempFoot.footprint.lori[0] + step_angle*R2D;
                    tempFoot.footprint.rori[1] = tempFoot.footprint.rori[1];
                    tempFoot.footprint.rori[2] = tempFoot.footprint.rori[2];

                    tempFoot.footprint.lori[0] = tempFoot.footprint.lori[0];
                    tempFoot.footprint.lori[1] = tempFoot.footprint.lori[1];
                    tempFoot.footprint.lori[2] = tempFoot.footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length*cos(tempFoot.footprint.rori[0]*D2R) - step_offset*sin(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.lfoot[1] + step_length*sin(tempFoot.footprint.rori[0]*D2R) + step_offset*cos(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = tempFoot.footprint.rori[0];
                    tempFoot.footprint.rori[1] = tempFoot.footprint.rori[1];
                    tempFoot.footprint.rori[2] = tempFoot.footprint.rori[2];

                    tempFoot.footprint.lori[0] = tempFoot.footprint.rori[0] + step_angle*R2D;
                    tempFoot.footprint.lori[1] = tempFoot.footprint.lori[1];
                    tempFoot.footprint.lori[2] = tempFoot.footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length*cos(tempFoot.footprint.lori[0]*D2R) - step_offset*sin(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.rfoot[1] + step_length*sin(tempFoot.footprint.lori[0]*D2R) + step_offset*cos(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-2){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;
                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + LPEL2PEL*sin(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] + LPEL2PEL*cos(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];

                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];


                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] - LPEL2PEL*sin(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + LPEL2PEL*cos(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1){
                temp_index = (i + temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];


                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;
            }else{
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0] + step_angle*R2D;
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + step_length*cos(tempFoot.footprint.rori[0]*D2R) - step_offset*sin(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] + step_length*sin(tempFoot.footprint.rori[0]*D2R) + step_offset*cos(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];

                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0] + step_angle*R2D;
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] + step_length*cos(tempFoot.footprint.lori[0]*D2R) - step_offset*sin(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + step_length*sin(tempFoot.footprint.lori[0]*D2R) + step_offset*cos(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];

                }
                FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];
                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;
            }

            printf(">>>>>>>>>> %d Foot Print <<<<<<<<<<< \n",i);
            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

            push_short_foot(tempFoot);
            right_left ^= 1;
        }
    }
}
void FPG_TEST2(int dir,int step_num,double step_length,double step_angle,double step_offset,double LPEL2PEL)
{
    // example short footprint information =============================



    if(step_angle>15.0)
    {
        step_angle = 15.0;
    }else if(step_angle<-15.0)
    {
        step_angle = -15.0;
    }

    step_angle = step_angle*D2R;

    double rl = 0;

    int right_left = 0;

    if(dir == APPROACHBOX_RIGHT_WALKING){
        right_left = 0;
    }else if(dir == APPROACHBOX_LEFT_WALKING){
        right_left  = 1;
    }

    printf("***********step angle: %f     step offset: %f  \n",step_angle,step_offset);

    _footprint_info dummyfoot;
    // remove old foot printf;
    while(pull_short_foot(dummyfoot)){
        ;
    }

    _footprint_info tempFoot;

    tempFoot.footprint.rfoot[0] = FK_pRFoot_3x1[0];//0.0;
    tempFoot.footprint.rfoot[1] = FK_pRFoot_3x1[1];//-0.105;
    tempFoot.footprint.rfoot[2] = FK_pRFoot_3x1[2];//0.0;

    tempFoot.footprint.rori[0] = FK_RFoot_yaw*R2D;//FK_pRFoot_3x1[0];//0.0;
    tempFoot.footprint.rori[1] = FK_RFoot_roll*R2D;//FK_pRFoot_3x1[1];//-0.105;
    tempFoot.footprint.rori[2] = FK_RFoot_pitch*R2D;//FK_pRFoot_3x1[2];//0.0;


    tempFoot.footprint.lfoot[0] = FK_pLFoot_3x1[0];//0.0;
    tempFoot.footprint.lfoot[1] = FK_pLFoot_3x1[1];//0.105;
    tempFoot.footprint.lfoot[2] = FK_pLFoot_3x1[2];

    tempFoot.footprint.lori[0] = FK_LFoot_yaw*R2D;//FK_pLFoot_3x1[0];//0.0;
    tempFoot.footprint.lori[1] = FK_LFoot_roll*R2D;//FK_pLFoot_3x1[1];//0.105;
    tempFoot.footprint.lori[2] = FK_LFoot_pitch*R2D;//FK_pLFoot_3x1[2];


    printf(">>>>>>>>>> Foot Print Initial Value <<<<<<<<<<< \n");
    printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
    printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

    tempFoot.time.dsp_time = 0.0;
    tempFoot.time.ssp_time = 0.0;
    tempFoot.info = FOOTINFO_NO;//FOOTINFO_FIRST_STEP;

    memcpy(&prev_foot, &tempFoot, sizeof(_footprint_info));


    int test_short_foot_num = step_num;

    if(dir != APPROACHBOX_FORWARD_WALKING)
    {
        if(step_num%2 == 0){
            step_num = step_num+1;
        }

        test_short_foot_num = step_num;

        if(test_short_foot_num ){
            test_short_foot_num = 3;
        }
    }


    if(step_offset<0.25)
    {
        step_offset = 0.25;
    }



    int temp_index = 0;
    int temp_head = ring_short_head;

    printf("---------- Rinf Short head : %d     Rinf Short tail : %d\n",ring_short_head,ring_short_tail);

    if(dir == APPROACHBOX_FORWARD_WALKING)
    {
        FILE_LOG(logSUCCESS) << "Forward Foot print Generation  ";

        for(int i=0; i<test_short_foot_num; i++){
            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length;
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length;
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                //FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-2){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                tempFoot.time.dsp_time = 0.2;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;

            }else{
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + step_length;
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] + step_length;
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                //FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];
                tempFoot.time.dsp_time = 0.2;
                tempFoot.time.ssp_time = 0.8;
            }

            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

            push_short_foot(tempFoot);
            right_left ^= 1;
        }

    }else if(dir == APPROACHBOX_LEFT_WALKING)
    {
        FILE_LOG(logSUCCESS) << "Left Foot print Generation";
        for(int i=0; i<test_short_foot_num; i++){
            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1] + step_length;
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1] + step_length;
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1)
            {
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;
            }else
            {
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1] + step_length;
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];


                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1] + step_length;
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];
                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;
            }


            push_short_foot(tempFoot);
            right_left ^= 1;


        }
    }else if(dir == APPROACHBOX_RIGHT_WALKING)
    {
        FILE_LOG(logSUCCESS) << "Right Foot print Generation  ";
        for(int i=0; i<test_short_foot_num; i++){
            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1] - step_length;
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1] - step_length;
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.8;


            }else if(i == test_short_foot_num-1){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;
            }else{
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1] - step_length;
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1] - step_length;
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];
                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;
            }

            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

            push_short_foot(tempFoot);
            right_left ^= 1;


        }
    }else if(dir == APPROACHBOX_CCW_WALKING)
    {


        FILE_LOG(logSUCCESS) << "CCW Foot print Generation  ";

        right_left = 1;

        for(int i=0; i<test_short_foot_num; i++){

            if(right_left == 1)
            {
                //rotate left direction ,, left foot first.
                rl = 1;

            }else
            {
                rl = -1;
            }

            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rori[0] = tempFoot.footprint.lori[0] + step_angle*R2D;
                    tempFoot.footprint.rori[1] = tempFoot.footprint.rori[1];
                    tempFoot.footprint.rori[2] = tempFoot.footprint.rori[2];

                    tempFoot.footprint.lori[0] = tempFoot.footprint.lori[0];
                    tempFoot.footprint.lori[1] = tempFoot.footprint.lori[1];
                    tempFoot.footprint.lori[2] = tempFoot.footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length*cos(tempFoot.footprint.rori[0]*D2R) - step_offset*sin(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.lfoot[1] + step_length*sin(tempFoot.footprint.rori[0]*D2R) + step_offset*cos(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else
                {
                    tempFoot.footprint.rori[0] = tempFoot.footprint.rori[0];
                    tempFoot.footprint.rori[1] = tempFoot.footprint.rori[1];
                    tempFoot.footprint.rori[2] = tempFoot.footprint.rori[2];

                    tempFoot.footprint.lori[0] = tempFoot.footprint.rori[0] + step_angle*R2D;
                    tempFoot.footprint.lori[1] = tempFoot.footprint.lori[1];
                    tempFoot.footprint.lori[2] = tempFoot.footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length*cos(tempFoot.footprint.lori[0]*D2R) - step_offset*sin(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.rfoot[1] + step_length*sin(tempFoot.footprint.lori[0]*D2R) + step_offset*cos(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }

                FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.8;

             }
            else if(i == test_short_foot_num-2){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + LPEL2PEL*sin(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] - LPEL2PEL*cos(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] + LPEL2PEL*sin(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + LPEL2PEL*cos(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];//  + LPEL2PEL*sin(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];//  - LPEL2PEL*cos(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];//  + LPEL2PEL*sin(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];//  + LPEL2PEL*cos(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;

            }else{
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0] + step_angle*R2D;
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + step_length*cos(tempFoot.footprint.rori[0]*D2R) - step_offset*sin(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] + step_length*sin(tempFoot.footprint.rori[0]*D2R) + step_offset*cos(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0] + step_angle*R2D;
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] + step_length*cos(tempFoot.footprint.lori[0]*D2R) - step_offset*sin(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + step_length*sin(tempFoot.footprint.lori[0]*D2R) + step_offset*cos(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];

                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;
            }

            printf(">>>>>>>>>> %d Foot Print <<<<<<<<<<< \n",i);

            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

            push_short_foot(tempFoot);
            right_left ^= 1;
        }
    }else if(dir == APPROACHBOX_CW_WALKING)
    {
        if(step_angle > 0)
        {
            step_angle  = -step_angle;
        }

        FILE_LOG(logSUCCESS) << "CW Foot print Generation  ";

        right_left = 0;

        for(int i=0; i<test_short_foot_num; i++){

            if(right_left == 1)
            {
                //rotate left direction ,, left foot first.
                rl = 1;
            }else
            {
                rl = -1;
            }

            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rori[0] = tempFoot.footprint.lori[0] + step_angle*R2D;
                    tempFoot.footprint.rori[1] = tempFoot.footprint.rori[1];
                    tempFoot.footprint.rori[2] = tempFoot.footprint.rori[2];

                    tempFoot.footprint.lori[0] = tempFoot.footprint.lori[0];
                    tempFoot.footprint.lori[1] = tempFoot.footprint.lori[1];
                    tempFoot.footprint.lori[2] = tempFoot.footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length*cos(tempFoot.footprint.rori[0]*D2R) - step_offset*sin(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.lfoot[1] + step_length*sin(tempFoot.footprint.rori[0]*D2R) + step_offset*cos(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = tempFoot.footprint.rori[0];
                    tempFoot.footprint.rori[1] = tempFoot.footprint.rori[1];
                    tempFoot.footprint.rori[2] = tempFoot.footprint.rori[2];

                    tempFoot.footprint.lori[0] = tempFoot.footprint.rori[0] + step_angle*R2D;
                    tempFoot.footprint.lori[1] = tempFoot.footprint.lori[1];
                    tempFoot.footprint.lori[2] = tempFoot.footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length*cos(tempFoot.footprint.lori[0]*D2R) - step_offset*sin(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.rfoot[1] + step_length*sin(tempFoot.footprint.lori[0]*D2R) + step_offset*cos(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-2){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;
                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + LPEL2PEL*sin(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] + LPEL2PEL*cos(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];

                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];


                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] - LPEL2PEL*sin(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + LPEL2PEL*cos(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1){
                temp_index = (i + temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];


                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;
            }else{
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0] + step_angle*R2D;
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + step_length*cos(tempFoot.footprint.rori[0]*D2R) - step_offset*sin(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] + step_length*sin(tempFoot.footprint.rori[0]*D2R) + step_offset*cos(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];

                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0] + step_angle*R2D;
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] + step_length*cos(tempFoot.footprint.lori[0]*D2R) - step_offset*sin(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + step_length*sin(tempFoot.footprint.lori[0]*D2R) + step_offset*cos(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];

                }
                FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];
                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;
            }

            printf(">>>>>>>>>> %d Foot Print <<<<<<<<<<< \n",i);
            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

            push_short_foot(tempFoot);
            right_left ^= 1;
        }
    }
}
void FPG_SINGLELOG(int step_num,double step_length,double LPEL2PEL)
{
    /*
     * This function generate planning ofthe foot when the robot crosses the single log.
     * It refer to the FPG_TEST function and assume only forward walking.
     */

    /* Initialize */
    zero_window();
    zero_localfoot();

    /* Decide the foot to move now. zero is right */
    double rl = 0;
    int right_left = 0;

    printf("[[[[[Walking Input]]]]] step_num: %d     step_length: %f\n",step_num,step_length);


    /* remove old foot print */
    _footprint_info dummyfoot;
    while(pull_short_foot(dummyfoot)){
        ;
    }

    /************************************* Generate New Foot ***********************************/
    /* Foot Print Initailize to current value */
    _footprint_info tempFoot;
    tempFoot.footprint.rfoot[0] = FK_pRFoot_3x1[0];
    tempFoot.footprint.rfoot[1] = -0.105;
    tempFoot.footprint.rfoot[2] = FK_pRFoot_3x1[2];

    tempFoot.footprint.rori[0] = FK_RFoot_yaw*R2D;
    tempFoot.footprint.rori[1] = FK_RFoot_roll*R2D;
    tempFoot.footprint.rori[2] = FK_RFoot_pitch*R2D;


    tempFoot.footprint.lfoot[0] = FK_pLFoot_3x1[0];
    tempFoot.footprint.lfoot[1] = 0.105;
    tempFoot.footprint.lfoot[2] = FK_pLFoot_3x1[2];

    tempFoot.footprint.lori[0] = FK_LFoot_yaw*R2D;
    tempFoot.footprint.lori[1] = FK_LFoot_roll*R2D;
    tempFoot.footprint.lori[2] = FK_LFoot_pitch*R2D;
    printf("tempFoot]] Rz = %f, Lz = %f\n",tempFoot.footprint.rfoot[2],tempFoot.footprint.lfoot[2]);

    tempFoot.time.dsp_time = 0.0;
    tempFoot.time.ssp_time = 0.0;
    tempFoot.info = FOOTINFO_NO;//FOOTINFO_FIRST_STEP;

    /* Save the current foot pos to prev_foot */
    memcpy(&prev_foot, &tempFoot, sizeof(_footprint_info));


    int test_short_foot_num = step_num;

//    if(dir != APPROACHBOX_FORWARD_WALKING)
//    {
//        if(step_num%2 == 0)
//        {
//            step_num = step_num+1;
//        }
//        test_short_foot_num = step_num;
//        if(test_short_foot_num < 3)
//        {
//            test_short_foot_num = 3;
//        }
//    }


    int temp_index = 0;
    int temp_head = ring_short_head;
    double step_width = 0.05;
    double dsp_time_init = 1.2;
    double dsp_time = 0.3;
    double ssp_time = 1.0;


    for(int i=0; i<test_short_foot_num; i++)
    {
        if(i == 0)
        {// First Foot Print (Generally Right foot first)
            if(right_left == 0){
                tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length;
                tempFoot.footprint.rfoot[1] = - step_width/2.;
                tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
            }else{
                tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length;
                tempFoot.footprint.lfoot[1] = + step_width/2.;
                tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
            }
            tempFoot.time.dsp_time = dsp_time_init;
            tempFoot.time.ssp_time = ssp_time;

        }else if(i == test_short_foot_num-3)
        {// Third Foot Print from the LAST.
            temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

            if(right_left == 0){
                tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length;
                tempFoot.footprint.rfoot[1] = - LPEL2PEL/2.;
                tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
            }else{
                tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length;
                tempFoot.footprint.lfoot[1] = + LPEL2PEL/2.;
                tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
            }

            tempFoot.time.dsp_time = dsp_time;
            tempFoot.time.ssp_time = ssp_time;

        }else if(i == test_short_foot_num-2)
        {// Second Foot Print from the LAST.
            temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

            if(right_left == 0)
            {
                tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0];
                tempFoot.footprint.rfoot[1] = - LPEL2PEL/2.;
                tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
            }else
            {
                tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0];
                tempFoot.footprint.lfoot[1] = + LPEL2PEL/2.;
                tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
            }
            tempFoot.time.dsp_time = dsp_time;
            tempFoot.time.ssp_time = ssp_time;
        }else if(i == test_short_foot_num-1)
        {// LAST Foot Print. Only DSP.
            temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

            if(right_left == 0)
            {
                tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
            }else
            {
                tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
            }
            tempFoot.time.dsp_time = dsp_time_init;
            tempFoot.time.ssp_time = 0.0;
        }else
        {
            temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

            if(right_left == 0)
            {
                tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length;
                tempFoot.footprint.rfoot[1] = - step_width/2.;
                tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
            }else
            {
                tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length;
                tempFoot.footprint.lfoot[1] = + step_width/2.;
                tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
            }

            tempFoot.time.dsp_time = 0.1;
            tempFoot.time.ssp_time = ssp_time;
        }

        printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
        printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
        printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
        printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

        push_short_foot(tempFoot);
        right_left ^= 1;
    }
}
