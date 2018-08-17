#ifndef VISIONLANDATA_H
#define VISIONLANDATA_H

typedef struct __KINECT_DATA_
{
    float dummy;
    float pos_x;
    float pos_y;
    float pos_z;

    float ori_w;
    float ori_x;
    float ori_y;
    float ori_z;
} KINECT_DATA, *pKINECT_DATA;

#endif // VISIONLANDATA_H
