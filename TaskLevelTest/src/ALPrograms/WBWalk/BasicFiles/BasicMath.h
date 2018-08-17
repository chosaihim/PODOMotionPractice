#ifndef BASICMATRIX_H
#define BASICMATRIX_H

#include "../../../share/Headers/ik_math4.h"
#include "../../../share/Headers/ik_math2.h"
#include "approachbox.h"

int convert_euler_FT(double _pre_state,double pRF_3x1[], double pLF_3x1[],double qRF_4x1[], double qLF_4x1[], double control_result_RF[],double control_result_LF[]);
int convert_euler(double pRF_3x1[], double pLF_3x1[], double euler_global_x, double euler_global_y, double euler_global_z,
                  double &euler_stance_x, double &euler_stance_y, double &euler_stance_z, double qPEL_comp_4x1[]);
double PitchRoll_Ori_Integral(double _ref, double _angle, int _zero);
void mat30by30x30by30(double a[30][30],double b[30][30],double out[30][30]);
void mat30by30x30by1(double a[30][30],double b[30],double out[30]);
void mat30by3x3by1(double a[30][3],double b[3],double out[30]);
void mat15by3minus15by3(double a[15][3],double b[15][3],double out[15][3]);
void mat15by15x15by3(double a[15][15],double b[15][3],double out[15][3]);
void mat15by15x15by15(double a[15][15],double b[15][15],double out[15][15]);
void mat15by15x15by1(double a[15][15],double b[15],double out[15]);
void mat15by3x3by1(double a[15][3],double b[3],double out[15]);
void mat3by3x3by1(double a[3][3],double b[3],double out[3]);
void mat4by4minus4by4(double a[4][4],double b[4][4],double out[4][4]);
void mat4by4plus4by4(double a[4][4],double b[4][4],double out[4][4]);
void mat4by4x4by4(double a[4][4],double b[4][4],double out[4][4]);

void Kalman(double A[4][4],double z[4],double out[3]);
void EulerToQt(double ang[3],double z[4]);
int InvMatrix(int n, double* A, double* b);




double Estimated_Orientation[3]={0.,},Old_Estimated_Orientation[3]={0.,},HPF_Estimated_Orientation[3]={0.,},Old_HPF_Estimated_Orientation[3]={0.,};
double Comp_Orientation[3]={0.,};
#define D2R 3.141592/180.0
#define R2D 180.0/3.141592


void Fifth(double t1,double tf,double p0[3],double pf[3],double ref[3])
{

    double a[6] = {0.0,};
    double theta_0 = p0[0],theta_dot_0 = p0[1],theta_ddot_0 = p0[2],theta_f = pf[0],theta_dot_f = pf[1],theta_ddot_f = pf[2];

    a[0] = theta_0;
    a[1] = theta_dot_0;
    a[2] = theta_ddot_0/2.;
    a[3] = (20.*(theta_f-theta_0) - (8.*theta_dot_f + 12.*theta_dot_0)*tf-(3.*theta_ddot_0 - theta_ddot_f)*tf*tf)/(2.*tf*tf*tf);
    a[4] = ((30.*theta_0 - 30.*theta_f) + (14.*theta_dot_f+16.*theta_dot_0)*tf+(3.*theta_ddot_0-2.*theta_ddot_f)*tf*tf)/(2.*tf*tf*tf*tf);
    a[5] = (12.*theta_f - 12.*theta_0 - (6.*theta_dot_f + 6.*theta_dot_0)*tf - (theta_ddot_0-theta_ddot_f)*tf*tf)/(2.*tf*tf*tf*tf*tf);

    ref[0] = a[0] + a[1]*t1  + a[2]*t1*t1 + a[3]*t1*t1*t1 + a[4]*t1*t1*t1*t1 + a[5]*t1*t1*t1*t1*t1;
    ref[1] = a[1] + 2.0*a[2]*t1 + 3.0*a[3]*t1*t1 + 4.0*a[4]*t1*t1*t1 + 5.0*a[5]*t1*t1*t1*t1;
    ref[2] = 2.0*a[2] + 6.0*a[3]*t1 + 12.0*a[4]*t1*t1 + 20.0*a[5]*t1*t1*t1;
}

void Poly_5th(double _time,double _ti,double _tf,double _pi,double _vi,double _ai,double _pf,double *_info_y)
{
    double coe[6], pow5, pow4, pow3, pow2;
    double _y,_y_d,_y_dd;
    _time=_time-_ti;
    _tf=_tf-_ti;

    pow5 = _tf*_tf*_tf*_tf*_tf;//pow(_tf, 5);
    pow4 = _tf*_tf*_tf*_tf;//pow(_tf, 4);
    pow3 = _tf*_tf*_tf;//pow(_tf, 3);
    pow2 = _tf*_tf;//pow(_tf, 2);

    coe[0]= (double)(-6./pow5*_pi -3./pow4*_vi -1./2./pow3*_ai +6./pow5*_pf);
    coe[1] = (double)(15./pow4*_pi +8./pow3*_vi +3./2./pow2*_ai -15./pow4*_pf);
    coe[2] = (double)(-10./pow3*_pi -6./pow2*_vi -3./2./_tf*_ai +10./pow3*_pf);
    coe[3] = (double)(1./2.*_ai);
    coe[4] = (double)(1.*_vi);
    coe[5] = (double)(_pi);

    pow5 = _time*_time*_time*_time*_time;//pow(_time, 5);
    pow4 = _time*_time*_time*_time;//pow(_time, 4);
    pow3 = _time*_time*_time;//pow(_time, 3);
    pow2 = _time*_time;//pow(_time, 2);

    _y      = coe[0]*pow5+coe[1]*pow4+coe[2]*pow3+coe[3]*pow2+coe[4]*(_time)+coe[5];
    _y_d    = 5.*coe[0]*pow4+4.*coe[1]*pow3+3.*coe[2]*pow2+2.*coe[3]*_time+coe[4];
    _y_dd   = 4.*5.*coe[0]*pow3+3.*4.*coe[1]*pow2+2.*3.*coe[2]*_time+2.*coe[3];

    _info_y[0] = _y;
    _info_y[1] = _y_d;
    _info_y[2] = _y_dd;
}


int convert_euler(double pRF_3x1[], double pLF_3x1[], double euler_global_x, double euler_global_y, double euler_global_z,
                  double &euler_stance_x, double &euler_stance_y, double &euler_stance_z, double qPEL_comp_4x1[])
{
    double prf_3x1[3] = {pRF_3x1[0], pRF_3x1[1], pRF_3x1[2]};
    double plf_3x1[3] = {pLF_3x1[0], pLF_3x1[1], pLF_3x1[2]};
    double stance_x_3x1[3], stance_y_3x1[3];
    double stance_z_3x1[3] = {0, 0, 1};
    double stance_frame_3x3[9], pel_global_3x3[9];
    double temp, temp2_4x1[4], temp3_4x1[4], temp4_4x1[4], temp5_3x3[9];
    double frame_meas_3x3[9];

    prf_3x1[2] = 0.;
    plf_3x1[2] = 0.;

    diff_vv(plf_3x1,3, prf_3x1, stance_y_3x1);
    cross(1., stance_y_3x1, stance_z_3x1, stance_x_3x1);

    temp = norm_v(stance_y_3x1,3);
    if(temp < 1e-6)
        return -1;

    stance_y_3x1[0] /= temp;
    stance_y_3x1[1] /= temp;
    stance_y_3x1[2] /= temp;

    temp = norm_v(stance_x_3x1,3);
    if(temp < 1e-6)
        return -1;

    stance_x_3x1[0] /= temp;
    stance_x_3x1[1] /= temp;
    stance_x_3x1[2] /= temp;

    stance_frame_3x3[0] = stance_x_3x1[0];
    stance_frame_3x3[3] = stance_x_3x1[1];
    stance_frame_3x3[6] = stance_x_3x1[2];

    stance_frame_3x3[1] = stance_y_3x1[0];
    stance_frame_3x3[4] = stance_y_3x1[1];
    stance_frame_3x3[7] = stance_y_3x1[2];

    stance_frame_3x3[2] = stance_z_3x1[0];
    stance_frame_3x3[5] = stance_z_3x1[1];
    stance_frame_3x3[8] = stance_z_3x1[2];

    qtRZ(euler_global_z, temp2_4x1);
    qtRX(euler_global_x, temp3_4x1);
    QTcross(temp2_4x1,temp3_4x1, temp4_4x1);
    qtRY(euler_global_y, temp2_4x1);
    QTcross(temp4_4x1,temp2_4x1, temp3_4x1);

    QT2DC(temp3_4x1, pel_global_3x3);

    mult_mm(pel_global_3x3,3,3,stance_frame_3x3,3, frame_meas_3x3);

    transpose(1., stance_frame_3x3, 3,3);
    mult_mm(stance_frame_3x3,3,3, frame_meas_3x3,3, temp5_3x3);

    euler_stance_y = atan2(-temp5_3x3[3*2+0], temp5_3x3[3*2+2]);
    euler_stance_z = atan2(temp5_3x3[3*1+0]*cos(euler_stance_y)+temp5_3x3[3*1+2]*sin(euler_stance_y), temp5_3x3[3*0+0]*cos(euler_stance_y)+temp5_3x3[3*0+2]*sin(euler_stance_y));
    euler_stance_x = atan2(temp5_3x3[3*2+1], temp5_3x3[3*2+2]*cos(euler_stance_y)-temp5_3x3[3*2+0]*sin(euler_stance_y));

    double temp_integral;
    temp_integral=PitchRoll_Ori_Integral(0.0, euler_stance_y, 1);

    qPEL_comp_4x1[0] = cos(temp_integral);
    qPEL_comp_4x1[1] = stance_y_3x1[0]*sin(temp_integral);
    qPEL_comp_4x1[2] = stance_y_3x1[1]*sin(temp_integral);
    qPEL_comp_4x1[3] = stance_y_3x1[2]*sin(temp_integral);

    return 0;
 }

double PitchRoll_Ori_Integral(double _ref, double _angle, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.11;
    const double KD =  0.8;
    static double angle_last =0.;

    y = 4.0*(_ref - _angle) + KI*sume + KD*(_angle - angle_last);

    sume += _ref - _angle;

    if(y > 100.0*D2R) y = 100.0*D2R;
    else if(y < -100.0*D2R) y = -100.0*D2R;

    if(_zero == 0)
    {
        sume = 0.;
        y=0.;
    }

    angle_last = y;

    return y;
}

// ----------------------------------- State Estimator


void Kalman(double A[4][4],double z[4],double out[3])
{
    double K[4][4]={{0.0,}},AT[4][4]={{0.0,}},HT[4][4]={{0.0,}},Pp[4][4]={{0.0,}};
    double temp_mat[4][4]={{0.0,}},temp_mat2[4][4]={{0.0,}},err[4]={0.,},Kerr[4]={0.,},H[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    double Qg = 1,Rg = 0.1,Q[4][4] = {{Qg,0,0,0},{0,Qg,0,0},{0,0,Qg,0},{0,0,0,Qg}},R[4][4] = {{Rg,0,0,0},{0,Rg,0,0},{0,0,Rg,0},{0,0,0,Rg}};
    static double xp[4] = {0.,0.,0.,0.},x[4] = {0.,0.,0.,0.},P[4][4]={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};

    for(int kal = 0;kal<4;kal++)
    {

        for(int kal2 = 0;kal2<4;kal2++)
        {
            AT[kal2][kal]= A[kal][kal2];
            HT[kal2][kal]= H[kal][kal2];
        }
    }


    // xp = Ax;
    for(int kal = 0;kal<4;kal++)
    {
        xp[kal] = A[kal][0]*x[0] + A[kal][1]*x[1] + A[kal][2]*x[2] + A[kal][3]*x[3];
    }

    // Pp = APA'+Q;

    //APA'
    mat4by4x4by4(A,P,temp_mat);
    mat4by4x4by4(temp_mat,AT,temp_mat2);

    //Pp = APA'+Q
    mat4by4plus4by4(temp_mat2,Q,Pp);

    // K = Pp*H'*inv(H*Pp*H' + R);
    //inv(H*Pp*H' + R)
    mat4by4x4by4(H,Pp,temp_mat);
    mat4by4x4by4(temp_mat,HT,temp_mat2);
    mat4by4plus4by4(temp_mat2,R,temp_mat);

    InvMatrix(4,(double*)temp_mat,(double*)temp_mat2);

    //Pp*H
    mat4by4x4by4(Pp,HT,temp_mat);

    //K = Pp*H'*inv(H*Pp*H' + R);
    mat4by4x4by4(temp_mat,temp_mat2,K);



    // z-Hxp
    for(int kal = 0;kal<4;kal++)
    {
        err[kal] = z[kal] - (H[kal][0]*xp[0] + H[kal][1]*xp[1] + H[kal][2]*xp[2] + H[kal][3]*xp[3]);
    }



    // xp_new = xp + K*(z-Hxp)
    for(int kal = 0;kal<4;kal++)
    {
        Kerr[kal] = K[kal][0]*err[0] + K[kal][1]*err[1] + K[kal][2]*err[2] + K[kal][3]*err[3];
        x[kal] = xp[kal] + Kerr[kal];

    }

    // P = Pp-K*H*Pp
    mat4by4x4by4(K,H,temp_mat);
    mat4by4x4by4(temp_mat,Pp,temp_mat2);

    mat4by4minus4by4(Pp,temp_mat2,P);


    // Phi theta psi

    out[0] = atan2(2.0*(x[2]*x[3] + x[0]*x[1]),1.0-2.0*(x[1]*x[1] + x[2]*x[2]));
    out[1] = -asin(2.0*(x[1]*x[3] - x[0]*x[2]));
    out[2] = atan2(2.0*(x[1]*x[2] + x[0]*x[3]),1.0-2.*(x[2]*x[2] + x[3]*x[3]));



}



void EulerAccel(double ax,double ay, double out[3])
{
    double g = 9.81,temp=0.;

    out[1] = temp = asin(ax/g);

    out[0] = asin(-ay/(g*cos(temp)));

    out[2] = 0.;


}

void EulerToQt(double ang[3],double z[4])
{
    double sinphi = sin(ang[0]/2.),   cosphi = cos(ang[0]/2.);
    double sintheta = sin(ang[1]/2.), costheta = cos(ang[1]/2.);
    double sinpsi = sin(ang[2]/2.),   cospsi = cos(ang[2]/2.);

    z[0] = cosphi*costheta*cospsi + sinphi*sintheta*sinpsi;
    z[1] = sinphi*costheta*cospsi - cosphi*sintheta*sinpsi;
    z[2] = cosphi*sintheta*cospsi + sinphi*costheta*sinpsi;
    z[3] = cosphi*costheta*sinpsi - sinphi*sintheta*cospsi;
}

void ACCtoWorldFrame(double euler[3],double a[3], double new_a[3])
{
    // Rx*Ry ... Roll*Pitch Euler
    double R[3][3] = {{cos(euler[1]),sin(euler[1])*sin(euler[0]),sin(euler[1])*cos(euler[0])},
                      {0.,cos(euler[0]),-sin(euler[0])},
                      {-sin(euler[1]),cos(euler[1])*sin(euler[0]),cos(euler[0])*cos(euler[1])}};


    mat3by3x3by1(R,a,new_a);

    new_a[2] = new_a[2] + 9.81;

}



void mat4by4minus4by4(double a[4][4],double b[4][4],double out[4][4])
{
    for(int i=0;i<4;i++)
    {
        for(int j = 0;j<4;j++)
        {
            out[i][j] = a[i][j] - b[i][j] ;

        }

    }

}

void mat4by4plus4by4(double a[4][4],double b[4][4],double out[4][4])
{
    for(int i=0;i<4;i++)
    {
        for(int j = 0;j<4;j++)
        {
            out[i][j] = a[i][j] + b[i][j] ;

        }

    }

}
void mat4by4x4by4(double a[4][4],double b[4][4],double out[4][4])
{

    double sum = 0.0f;

    for(int i=0;i<4;i++)
    {
        for(int k = 0;k<4;k++)
        {
            for(int j=0;j<4;j++)
            {
                sum = sum + a[i][j]*b[j][k];
            }

            out[i][k] = sum ;
            sum = 0.0f;
        }

    }

}

int InvMatrix(int n, double* A, double* b)
{
    double m;
    register int i, j, k;
    double* a = new double[n*n];

    if(a==NULL)
  return 0;
    for(i=0; i<n*n; i++)
  a[i]=A[i];
 for(i=0; i<n; i++)
 {
  for(j=0; j<n; j++)
  {
            b[i*n+j]=(i==j)?1.:0.;
        }
    }
    for(i=0; i<n; i++)
 {
  if(a[i*n+i]==0.)
  {
   if(i==n-1)
   {
    delete[] a;
    return 0;
            }
            for(k=1; i+k<n; k++)
   {
                if(a[i*n+i+k] != 0.)
     break;
            }
            if(i+k>=n)
   {
                delete[] a;
    return 0;
            }
            for(j=0; j<n; j++)
   {
                m = a[i*n+j];
                a[i*n+j] = a[(i+k)*n+j];
                a[(i+k)*n+j] = m;
                m = b[i*n+j];
                b[i*n+j] = b[(i+k)*n+j];
                b[(i+k)*n+j] = m;
            }
        }
        m = a[i*n+i];
        for(j=0; j<n; j++)
  {
            a[i*n+j]/=m;
            b[i*n+j]/=m;
        }
        for(j=0; j<n; j++)
  {
            if(i==j)
    continue;

            m = a[j*n+i];
            for(k=0; k<n; k++)
   {
                a[j*n+k] -= a[i*n+k]*m;
                b[j*n+k] -= b[i*n+k]*m;
            }
        }
    }
    delete[] a;
    return 1;
}

void mat15by3x3by1(double a[15][3],double b[3],double out[15])
{

    double sum = 0.0f;

    for(int i=0;i<15;i++)
    {
            for(int j=0;j<3;j++)
            {
                sum = sum + a[i][j]*b[j];
            }

            out[i] = sum ;
            sum = 0.0f;


    }

}

void mat6by3x3by1(double a[6][3],double b[3],double out[6])
{

    double sum = 0.0f;

    for(int i=0;i<6;i++)
    {
            for(int j=0;j<3;j++)
            {
                sum = sum + a[i][j]*b[j];
            }

            out[i] = sum ;
            sum = 0.0f;


    }

}




void mat30by3x3by1(double a[30][3],double b[3],double out[30])
{

    double sum = 0.0f;

    for(int i=0;i<30;i++)
    {
            for(int j=0;j<3;j++)
            {
                sum = sum + a[i][j]*b[j];
            }

            out[i] = sum ;
            sum = 0.0f;

    }

}

void mat15by15x15by1(double a[15][15],double b[15],double out[15])
{

    double sum = 0.0f;

    for(int i=0;i<15;i++)
    {
            for(int j=0;j<15;j++)
            {
                sum = sum + a[i][j]*b[j];
            }

            out[i] = sum ;
            sum = 0.0f;


    }

}



void mat30by30x30by1(double a[30][30],double b[30],double out[30])
{

    double sum = 0.0f;

    for(int i=0;i<30;i++)
    {
            for(int j=0;j<30;j++)
            {
                sum = sum + a[i][j]*b[j];
            }

            out[i] = sum ;
            sum = 0.0f;


    }

}
void mat6by3x3by6(double a[6][3],double b[3][6],double out[6][6])
{

    double sum = 0.0f;

    for(int i=0;i<6;i++)
    {
        for(int k = 0;k<6;k++)
        {
            for(int j=0;j<3;j++)
            {
                sum = sum + a[i][j]*b[j][k];
            }

            out[i][k] = sum ;
            sum = 0.0f;
        }

    }

}


void mat30by30x30by30(double a[30][30],double b[30][30],double out[30][30])
{

    double sum = 0.0f;

    for(int i=0;i<30;i++)
    {
        for(int k = 0;k<30;k++)
        {
            for(int j=0;j<30;j++)
            {
                sum = sum + a[i][j]*b[j][k];
            }

            out[i][k] = sum ;
            sum = 0.0f;
        }

    }

}

void mat15by15x15by15(double a[15][15],double b[15][15],double out[15][15])
{

    double sum = 0.0f;

    for(int i=0;i<15;i++)
    {
        for(int k = 0;k<15;k++)
        {
            for(int j=0;j<15;j++)
            {
                sum = sum + a[i][j]*b[j][k];
            }

            out[i][k] = sum ;
            sum = 0.0f;
        }

    }

}

void mat15by15x15by3(double a[15][15],double b[15][3],double out[15][3])
{

    double sum = 0.0f;

    for(int i=0;i<15;i++)
    {
        for(int k = 0;k<3;k++)
        {
            for(int j=0;j<15;j++)
            {
                sum = sum + a[i][j]*b[j][k];
            }

            out[i][k] = sum ;
            sum = 0.0f;
        }

    }

}

void mat15by3minus15by3(double a[15][3],double b[15][3],double out[15][3])
{

    for(int i=0;i<15;i++)
    {
        for(int k = 0;k<3;k++)
        {
            out[i][k] = a[i][k] - b[i][k];

        }

    }

}


void mat3by3x3by1(double a[3][3],double b[3],double out[3])
{

    double sum = 0.0f;

    for(int i=0;i<=2;i++)
    {
            for(int j=0;j<=2;j++)
            {
                sum = sum + a[i][j]*b[j];
            }

            out[i] = sum ;
            sum = 0.0f;


    }

}

#endif // BASICMATRIX_H
