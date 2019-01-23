//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>
#include "sp_math.h"

//--------------------------------------------------------------------------------------------
// Variable declaration
typedef struct {
    float beta;
    float q0;
    float q1;
    float q2;
    float q3;
    float invSampleFreq;
    float roll;
    float pitch;
    float yaw;
    char anglesComputed;
} Madgwick;

void Madgwick_Init(Madgwick* mad, float betaDef, float sampleFreqDef);
void Madgwick_update(Madgwick* mad, float gx, float gy, float gz, float ax, float ay, float az, 
    float mx, float my, float mz);
void Madgwick_updateIMU(Madgwick* mad, float gx, float gy, float gz, float ax, float ay, float az);
void Madgwick_computeAngles(Madgwick* mad);
    
#endif

