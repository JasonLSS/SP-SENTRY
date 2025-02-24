//=============================================================================================
// MadgwickAHRS.c
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date            Author          Notes
// 29/09/2011    SOH Madgwick    Initial release
// 02/10/2011    SOH Madgwick    Optimised for reduced CPU load
// 19/02/2012    SOH Madgwick    Magnetometer measurement is normalised
//
//=============================================================================================

//-------------------------------------------------------------------------------------------
// Header files

#include "MadgwickAHRS.h"
#include "sp_math.h"

//-------------------------------------------------------------------------------------------
// Definitions

#define betaDef         1.f            // 2 * proportional gain

//============================================================================================
// Functions

//-------------------------------------------------------------------------------------------
// AHRS algorithm update

void init(Madgwick* ahrs, float sampleFrequency) {
    ahrs->beta = betaDef;
    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;
    ahrs->invSampleFreq = 1.0f / sampleFrequency;
    ahrs->anglesComputed = 0;
}

void updateSampFreq(Madgwick* ahrs, float sampleFrequency) {
    ahrs->invSampleFreq = 1.0f / sampleFrequency;
}

void update(Madgwick* ahrs, 
    float gx, float gy, float gz, 
    float ax, float ay, float az, 
    float mx, float my, float mz) 
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        AHRS_Man.updateIMU(ahrs, gx, gy, gz, ax, ay, az);
        return;
    }

    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-ahrs->q1 * gx - ahrs->q2 * gy - ahrs->q3 * gz);
    qDot2 = 0.5f * (ahrs->q0 * gx + ahrs->q2 * gz - ahrs->q3 * gy);
    qDot3 = 0.5f * (ahrs->q0 * gy - ahrs->q1 * gz + ahrs->q3 * gx);
    qDot4 = 0.5f * (ahrs->q0 * gz + ahrs->q1 * gy - ahrs->q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * ahrs->q0 * mx;
        _2q0my = 2.0f * ahrs->q0 * my;
        _2q0mz = 2.0f * ahrs->q0 * mz;
        _2q1mx = 2.0f * ahrs->q1 * mx;
        _2q0 = 2.0f * ahrs->q0;
        _2q1 = 2.0f * ahrs->q1;
        _2q2 = 2.0f * ahrs->q2;
        _2q3 = 2.0f * ahrs->q3;
        _2q0q2 = 2.0f * ahrs->q0 * ahrs->q2;
        _2q2q3 = 2.0f * ahrs->q2 * ahrs->q3;
        q0q0 = ahrs->q0 * ahrs->q0;
        q0q1 = ahrs->q0 * ahrs->q1;
        q0q2 = ahrs->q0 * ahrs->q2;
        q0q3 = ahrs->q0 * ahrs->q3;
        q1q1 = ahrs->q1 * ahrs->q1;
        q1q2 = ahrs->q1 * ahrs->q2;
        q1q3 = ahrs->q1 * ahrs->q3;
        q2q2 = ahrs->q2 * ahrs->q2;
        q2q3 = ahrs->q2 * ahrs->q3;
        q3q3 = ahrs->q3 * ahrs->q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * ahrs->q3 + _2q0mz * ahrs->q2 + mx * q1q1 + _2q1 * my * ahrs->q2 + _2q1 * mz * ahrs->q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * ahrs->q3 + my * q0q0 - _2q0mz * ahrs->q1 + _2q1mx * ahrs->q2 - my * q1q1 + my * q2q2 + _2q2 * mz * ahrs->q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * ahrs->q2 + _2q0my * ahrs->q1 + mz * q0q0 + _2q1mx * ahrs->q3 - mz * q1q1 + _2q2 * my * ahrs->q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * ahrs->q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * ahrs->q3 + _2bz * ahrs->q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * ahrs->q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * ahrs->q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * ahrs->q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * ahrs->q2 + _2bz * ahrs->q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * ahrs->q3 - _4bz * ahrs->q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * ahrs->q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * ahrs->q2 - _2bz * ahrs->q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * ahrs->q1 + _2bz * ahrs->q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * ahrs->q0 - _4bz * ahrs->q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * ahrs->q3 + _2bz * ahrs->q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * ahrs->q0 + _2bz * ahrs->q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * ahrs->q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= ahrs->beta * s0;
        qDot2 -= ahrs->beta * s1;
        qDot3 -= ahrs->beta * s2;
        qDot4 -= ahrs->beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    ahrs->q0 += qDot1 * ahrs->invSampleFreq;
    ahrs->q1 += qDot2 * ahrs->invSampleFreq;
    ahrs->q2 += qDot3 * ahrs->invSampleFreq;
    ahrs->q3 += qDot4 * ahrs->invSampleFreq;

    // Normalise quaternion
    recipNorm = inv_sqrt(ahrs->q0 * ahrs->q0 + ahrs->q1 * ahrs->q1 + ahrs->q2 * ahrs->q2 + ahrs->q3 * ahrs->q3);
    ahrs->q0 *= recipNorm;
    ahrs->q1 *= recipNorm;
    ahrs->q2 *= recipNorm;
    ahrs->q3 *= recipNorm;
    ahrs->anglesComputed = 0;
}

//-------------------------------------------------------------------------------------------
// IMU algorithm update

void updateIMU(Madgwick* ahrs, float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;
    
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-ahrs->q1 * gx - ahrs->q2 * gy - ahrs->q3 * gz);
    qDot2 = 0.5f * (ahrs->q0 * gx + ahrs->q2 * gz - ahrs->q3 * gy);
    qDot3 = 0.5f * (ahrs->q0 * gy - ahrs->q1 * gz + ahrs->q3 * gx);
    qDot4 = 0.5f * (ahrs->q0 * gz + ahrs->q1 * gy - ahrs->q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * ahrs->q0;
        _2q1 = 2.0f * ahrs->q1;
        _2q2 = 2.0f * ahrs->q2;
        _2q3 = 2.0f * ahrs->q3;
        _4q0 = 4.0f * ahrs->q0;
        _4q1 = 4.0f * ahrs->q1;
        _4q2 = 4.0f * ahrs->q2;
        _8q1 = 8.0f * ahrs->q1;
        _8q2 = 8.0f * ahrs->q2;
        q0q0 = ahrs->q0 * ahrs->q0;
        q1q1 = ahrs->q1 * ahrs->q1;
        q2q2 = ahrs->q2 * ahrs->q2;
        q3q3 = ahrs->q3 * ahrs->q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * ahrs->q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * ahrs->q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * ahrs->q3 - _2q1 * ax + 4.0f * q2q2 * ahrs->q3 - _2q2 * ay;
        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= ahrs->beta * s0;
        qDot2 -= ahrs->beta * s1;
        qDot3 -= ahrs->beta * s2;
        qDot4 -= ahrs->beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    ahrs->q0 += qDot1 * ahrs->invSampleFreq;
    ahrs->q1 += qDot2 * ahrs->invSampleFreq;
    ahrs->q2 += qDot3 * ahrs->invSampleFreq;
    ahrs->q3 += qDot4 * ahrs->invSampleFreq;

    // Normalise quaternion
    recipNorm = inv_sqrt(ahrs->q0 * ahrs->q0 + ahrs->q1 * ahrs->q1 + ahrs->q2 * ahrs->q2 + ahrs->q3 * ahrs->q3);
    ahrs->q0 *= recipNorm;
    ahrs->q1 *= recipNorm;
    ahrs->q2 *= recipNorm;
    ahrs->q3 *= recipNorm;
    ahrs->anglesComputed = 0;
}



//-------------------------------------------------------------------------------------------
void computeAngles(Madgwick* ahrs)
{
    ahrs->roll = atan2f(ahrs->q0*ahrs->q1 + ahrs->q2*ahrs->q3, 0.5f - ahrs->q1*ahrs->q1 - ahrs->q2*ahrs->q2);
    ahrs->pitch = asinf(-2.0f * (ahrs->q1*ahrs->q3 - ahrs->q0*ahrs->q2));
    ahrs->yaw = atan2f(ahrs->q1*ahrs->q2 + ahrs->q0*ahrs->q3, 0.5f - ahrs->q2*ahrs->q2 - ahrs->q3*ahrs->q3);
    ahrs->anglesComputed = 1;
    
    ahrs->roll = ahrs->roll * 57.29578f;
    ahrs->pitch = ahrs->pitch * 57.29578f;
    ahrs->yaw = ahrs->yaw * 57.29578f;
}


struct __AHRS_Man AHRS_Man = {
    .init= init,
    .update= update,
    .updateIMU= updateIMU,
    .computeAngles= computeAngles,
    .updateSampFreq = updateSampFreq
};

