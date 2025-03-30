#ifndef CONFIG_H
#define CONFIG_H

const double DEFAULT_GAIN = 1.0;
const double TIME_STEP = 0.01;

const int N_OUTPUTS = 5;

const int N_Q = 5;

namespace GenPosIDX
{
    enum eGenPosIDX 
    {
        STF_ANKLE = 0, 
        STF_KNEE, 
        STF_HIP, 
        SWF_HIP, 
        SWF_KNEE
    }; 
}

namespace OutputIDX
{
    enum eOutputIDX 
    {
        COM_POS_X = 0,
        COM_POS_Z,
        COM_THETA,
        SWF_POS_X,
        SWF_POS_Z
    };
}

#endif // CONFIG_H
