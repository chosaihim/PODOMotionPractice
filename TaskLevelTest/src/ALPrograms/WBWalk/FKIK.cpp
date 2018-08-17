#include "FKIK.h"

FKIK::FKIK()
{
    RH_ref_frame = 2;//Global = 0, Pelv = 1, UB = 2
    LH_ref_frame = 2;

    desired.initialize();
    desired_hat.initialize();
    FK.initialize();

    for(int i=0;i<34;i++)
    {
        WBIK_Q[i] = 0;
        WBIK_Q0[i] = 0;
    }
}
