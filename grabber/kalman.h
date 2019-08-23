// 6DOF Kalman sensor fusion filter
// All code below is (slightly) adapted from https://github.com/memsindustrygroup/Open-Source-Sensor-Fusion
// and was issued under this licence.

// Copyright (c) 2014, Freescale Semiconductor, Inc.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Freescale Semiconductor, Inc. nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL FREESCALE SEMICONDUCTOR, INC. BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*  * usage:
    * include this file in your code, so #include "kalman.h"
    * From your code call kalmanInit(X) where X is the sample period (time between samples) in seconds.
    * Then, when you have sensor data, first call kalmanSetInitialOrientation(ax,ay,az) with the initial
    * accelerometer measurements (this sets up the initial orientation more quickly and there may be some instability
    * without first doing this call) and then call kalmanRun(gx,gy,gz,ax,ay,az) whenever you have samples to process
    * (gx, gy and gz are the gyro readings (in degrees/sec) and ax, ay and az are the accelerometer readings (in gravities)).
    * Results are in q0, q1, q2 and q3 (quaternion, q0 is the scalar) and roll, pitch and yaw (Euler angles in degrees).
    * There is other interesting stuff in the kalmanData structure, eg faSePl[3] has the linear accn in the sensor
    * frame. 
*/

#ifndef _KALMAN_H

#define _KALMAN_H

// globals for fusion results
float q0 = 1;
float q1 = 0;
float q2 = 0;
float q3 = 0;
float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;

// quaternion structure definition
struct quaternion {
    float q0;	// scalar component
    float q1;	// x vector component
    float q2;	// y vector component
    float q3;	// z vector component
};

// function prototypes (shoot me)
void kalmanInit(float samplePeriod);
void kalmanSetInitialOrientation(float ax, float ay, float az);
void kalmanRun(float gx, float gy, float gz, float ax, float ay, float az);
void fAnglesDegFromRotationMatrix(float R[][3], float *roll, float *pitch, float *yaw);
void fQuaternionFromRotationVectorDeg(struct quaternion *pq, const float rvecdeg[], float fscaling);
void fQuaternionFromRotationMatrix(float R[][3], struct quaternion *pq);
void fRotationMatrixFromQuaternion(float R[][3], const struct quaternion *pq);
void fRotationVectorDegFromQuaternion(struct quaternion *pq, float rvecdeg[]);
void fmatrixAeqInvA(float *A[], int8_t iColInd[], int8_t iRowInd[], int8_t iPivot[], int8_t isize);
void f3DOFTiltNED(float fR[][3], float fGp[]);
void fqAeqNormqA(struct quaternion *pqA);
void qAeqAxB(struct quaternion *pqA, const struct quaternion *pqB);
void qAeqBxC(struct quaternion *pqA, const struct quaternion *pqB, const struct quaternion *pqC);
void fmatrixAeqI(float *A[], int16_t rc);

#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "time.h"

// vector components
#define X 0
#define Y 1
#define Z 2

#define DEGTORADMULT 0.01745329251994F		// degrees to radians conversion = pi / 180
#define RADTODEGMULT 57.2957795130823F		// radians to degrees conversion = 180 / pi

// useful multiplicative conversion constants
#define ONEOVER48 0.02083333333F		// 1 / 48
#define ONEOVER3840 0.0002604166667F	// 1 / 3840

// kalman filter noise variances
#define FQVA 2E-6F				// accelerometer noise g^2 so 1.4mg RMS
#define FQVG 0.3F				// gyro noise (deg/s)^2
#define FQWB 1E-9F				// gyro offset drift (deg/s)^2: 1E-9 implies 0.09deg/s max at 50Hz
#define FQWA 1E-4F				// linear acceleration drift g^2 (increase slows convergence to g but reduces sensitivity to shake)
// initialization of Qw covariance matrix
#define FQWINITTHTH 2000E-5F		// th_e * th_e terms
#define FQWINITBB 250E-3F		// for FXAS21000: b_e * b_e terms
#define FQWINITTHB 0.0F			// th_e * b_e terms
#define FQWINITAA 10E-5F			// a_e * a_e terms (increase slows convergence to g but reduces sensitivity to shake)
// linear acceleration time constant
#define FCA 0.5F		

#define CORRUPTQUAT 0.001F	// threshold for deciding rotation quaternion is corrupt

struct {
    // orientation matrix, quaternion and rotation vector
    float fRPl[3][3];				// a posteriori  rotation matrix
    struct quaternion fqPl;		// a posteriori orientation quaternion
    float fRVecPl[3];				// rotation vector
    float fbPl[3];					// gyro offset (deg/s)
    float fThErrPl[3];				// orientation error (deg)
    float fbErrPl[3];				// gyro offset error (deg/s)
    float fzErrMi[3];				// angular error (deg) between a priori and eCompass orientations
    float fRMi[3][3];				// a priori rotation matrix
    struct quaternion fqMi;		// a priori orientation quaternion
    struct quaternion fDeltaq;		// delta a priori or a posteriori quaternion
    float faSePl[3];				// linear acceleration (g, sensor frame) after error correction
    float faErrSePl[3];				// linear acceleration error (g, sensor frame)
    float fgErrSeMi[3];				// difference (g, sensor frame) of gravity vector (accel) and gravity vector (gyro)
    float fgSeGyMi[3];				// gravity vector (g, sensor frame) measurement from gyro
    float faSeMi[3];				// linear acceleration (g, sensor frame)
    float fQvAA;					// accelerometer terms of Qv
    float fPPlus9x9[9][9];			// covariance matrix P+
    float fK9x3[9][3];				// kalman filter gain matrix K
    float fQw9x9[9][9];				// covariance matrix Qw
    float fC3x9[3][9];				// measurement matrix C
    float fcasq;					// FCA * FCA;
    float fdeltat;					// kalman filter sampling interval (s)
    float fdeltatsq;				// fdeltat * fdeltat;
    float fQwbplusQvG;				// FQWB + FQVG;
} kalmanData;

// function initalizes the 6DOF accel + gyro Kalman filter algorithm
void kalmanInit(float samplePeriod) {
    int8_t i, j;				// loop counters

    // compute and store useful product terms to save floating point calculations later
    kalmanData.fdeltat = samplePeriod;
    kalmanData.fdeltatsq = kalmanData.fdeltat * kalmanData.fdeltat;
    kalmanData.fcasq = FCA * FCA;
    kalmanData.fQwbplusQvG = FQWB + FQVG;

    // initialize the fixed entries in the measurement matrix C
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 9; j++) kalmanData.fC3x9[i][j]= 0.0F;
    }
    kalmanData.fC3x9[0][6] = kalmanData.fC3x9[1][7] = kalmanData.fC3x9[2][8] = 1.0F;

    // zero a posteriori orientation, error vector xe+ (thetae+, be+, ae+) and b+
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) kalmanData.fRPl[i][j]= 0.0F;
    }
    kalmanData.fRPl[0][0] = kalmanData.fRPl[1][1] = kalmanData.fRPl[2][2] = 1.0F;
        
    kalmanData.fqPl.q0 = 1.0F;
    kalmanData.fqPl.q1 = kalmanData.fqPl.q2 = kalmanData.fqPl.q3 = 0.0F;

    for (i = X; i <= Z; i++){
        kalmanData.fThErrPl[i] = kalmanData.fbErrPl[i] = kalmanData.faErrSePl[i] = kalmanData.fbPl[i] = 0.0F;
    }

    // initialize noise variance for Qv and Qw matrix updates
    kalmanData.fQvAA = FQVA + FQWA + DEGTORADMULT * DEGTORADMULT * kalmanData.fdeltatsq * (FQWB + FQVG);

    // initialize the 6x6 noise covariance matrix Qw of the a priori error vector xe-
    // Qw is then recursively updated as P+ = (1 - K * C) * P- = (1 - K * C) * Qw  and Qw updated from P+
    // zero the matrix Qw
    for (i = 0; i < 9; i++) {
        for (j = 0; j < 9; j++) kalmanData.fQw9x9[i][j] = 0.0F;
    }
    // loop over non-zero values
    for (i = 0; i < 3; i++) {
        // theta_e * theta_e terms
        kalmanData.fQw9x9[i][i] = FQWINITTHTH;
        // b_e * b_e terms
        kalmanData.fQw9x9[i + 3][i + 3] = FQWINITBB;
        // th_e * b_e terms
        kalmanData.fQw9x9[i][i + 3] = kalmanData.fQw9x9[i + 3][i] = FQWINITTHB;
        // a_e * a_e terms
        kalmanData.fQw9x9[i + 6][i + 6] = FQWINITAA;
    }
    return;
}

void kalmanSetInitialOrientation(float ax, float ay, float az){
    float initialAccelData[3];
    initialAccelData[0] = ax;
    initialAccelData[1] = ay;
    initialAccelData[2] = az;
    f3DOFTiltNED(kalmanData.fRPl, initialAccelData);
    fQuaternionFromRotationMatrix(kalmanData.fRPl, &(kalmanData.fqPl));
}

// 6DOF accel + gyro Kalman filter algorithm 
void kalmanRun(float gx, float gy, float gz, float ax, float ay, float az) {	
    // local arrays and scalars
    float rvec[3];							// rotation vector
    float ftmpA9x3[9][3];					// scratch array

    // assorted array pointers
    float *pfPPlus9x9kj;
    float *pfPPlus9x9ij;
    float *pfK9x3ij;
    float *pfK9x3ik;
    float *pftmpA9x3ik;
    float *pftmpA9x3ij;
    float *pftmpA9x3kj;
    float *pfQw9x9ij;
    float *pfQw9x9ik;
    float *pfQw9x9kj;
    float *pfC3x9ik;
    float *pfC3x9jk;

    int8_t i, j, k;							// loop counters

    // working arrays for 3x3 matrix inversion
    float *pfRows[3];
    int8_t iColInd[3];
    int8_t iRowInd[3];
    int8_t iPivot[3];

    // *********************************************************************************
    // calculate a priori rotation matrix
    // *********************************************************************************

    // initialize the a priori orientation quaternion to the a posteriori orientation estimate
    kalmanData.fqMi = kalmanData.fqPl;

    // integrate the gyro readings
    rvec[0] = gx * kalmanData.fdeltat; // (((float)gyroData.iYpFast[j][i] * gyroData.fDegPerSecPerCount)) * kalmanData.fFastdeltat;
    rvec[1] = gy * kalmanData.fdeltat;
    rvec[2] = gz * kalmanData.fdeltat;

    // compute the incremental quaternion fDeltaq from the rotation vector
    fQuaternionFromRotationVectorDeg(&(kalmanData.fDeltaq), rvec, 1.0F);

    // incrementally rotate the a priori orientation quaternion fqMi
    // the a posteriori orientation is re-normalized later so this update is stable
    qAeqAxB(&(kalmanData.fqMi), &(kalmanData.fDeltaq));

    // get the a priori rotation matrix from the a priori quaternion
    fRotationMatrixFromQuaternion(kalmanData.fRMi, &(kalmanData.fqMi));

    // *********************************************************************************
    // calculate a priori gyro and accelerometer estimates of the gravity vector
    // and the error between the two
    // *********************************************************************************

    // compute the a priori **gyro** estimate of the gravitational vector (g, sensor frame)
    // using an absolute rotation of the global frame gravity vector (with magnitude 1g)
    // NED gravity is along positive z axis
    kalmanData.fgSeGyMi[0] = kalmanData.fRMi[0][Z];
    // compute a priori acceleration (a-) (g, sensor frame) from a posteriori estimate (g, sensor frame)
    kalmanData.faSeMi[0] = FCA * kalmanData.faSePl[0];
    // compute the a priori gravity error vector (accelerometer minus gyro estimates) (g, sensor frame)
    // positive sign for gravity: y = g - a and g = y + a
    kalmanData.fgErrSeMi[0] = ax + kalmanData.faSeMi[0] - kalmanData.fgSeGyMi[0];

    kalmanData.fgSeGyMi[1] = kalmanData.fRMi[1][Z];
    kalmanData.faSeMi[1] = FCA * kalmanData.faSePl[1];
    kalmanData.fgErrSeMi[1] = ay + kalmanData.faSeMi[1] - kalmanData.fgSeGyMi[1];

    kalmanData.fgSeGyMi[2] = kalmanData.fRMi[2][Z];
    kalmanData.faSeMi[2] = FCA * kalmanData.faSePl[2];
    kalmanData.fgErrSeMi[2] = az + kalmanData.faSeMi[2] - kalmanData.fgSeGyMi[2];

    // *********************************************************************************
    // update variable elements of measurement matrix C
    // *********************************************************************************

    // update measurement matrix C (3x9) with -alpha(g-)x from gyro (g, sensor frame)
    kalmanData.fC3x9[0][1] = DEGTORADMULT * kalmanData.fgSeGyMi[Z];
    kalmanData.fC3x9[0][2] = -DEGTORADMULT * kalmanData.fgSeGyMi[Y];
    kalmanData.fC3x9[1][2] = DEGTORADMULT * kalmanData.fgSeGyMi[X];
    kalmanData.fC3x9[1][0] = -kalmanData.fC3x9[0][1];
    kalmanData.fC3x9[2][0] = -kalmanData.fC3x9[0][2];
    kalmanData.fC3x9[2][1] = -kalmanData.fC3x9[1][2];
    kalmanData.fC3x9[0][4] = -kalmanData.fdeltat * kalmanData.fC3x9[0][1];
    kalmanData.fC3x9[0][5] = -kalmanData.fdeltat * kalmanData.fC3x9[0][2];
    kalmanData.fC3x9[1][5] = -kalmanData.fdeltat * kalmanData.fC3x9[1][2];
    kalmanData.fC3x9[1][3]= -kalmanData.fC3x9[0][4];
    kalmanData.fC3x9[2][3]= -kalmanData.fC3x9[0][5];
    kalmanData.fC3x9[2][4]= -kalmanData.fC3x9[1][5];

    // *********************************************************************************
    // calculate the Kalman gain matrix K (9x3)
    // K = P- * C^T * inv(C * P- * C^T + Qv) = Qw * C^T * inv(C * Qw * C^T + Qv)
    // Qw is used as a proxy for P- throughout the code
    // P+ is used here as a working array to reduce RAM usage and is re-computed later
    // *********************************************************************************

    // set ftmpA9x3 = P- * C^T = Qw * C^T where Qw and C are both sparse
    // C also has a significant number of +1 and -1 entries
    // ftmpA9x3 is also sparse but not symmetric
    for (i = 0; i < 9; i++) { // loop over rows of ftmpA9x3
        // initialize pftmpA9x3ij for current i, j=0
        pftmpA9x3ij = ftmpA9x3[i];

        for (j = 0; j < 3; j++) { // loop over columns of ftmpA9x3
            // zero ftmpA9x3[i][j]
            *pftmpA9x3ij = 0.0F;

            // initialize pfC3x9jk for current j, k=0
            pfC3x9jk = kalmanData.fC3x9[j];

            // initialize pfQw9x9ik for current i, k=0
            pfQw9x9ik = kalmanData.fQw9x9[i];

            // sum matrix products over inner loop over k
            for (k = 0; k < 9; k++) {
                if ((*pfQw9x9ik != 0.0F) && (*pfC3x9jk != 0.0F)) {
                    if (*pfC3x9jk == 1.0F) {
                        *pftmpA9x3ij += *pfQw9x9ik;
                    } else if (*pfC3x9jk == -1.0F) {
                        *pftmpA9x3ij -= *pfQw9x9ik;
                    } else {
                        *pftmpA9x3ij += *pfQw9x9ik * *pfC3x9jk;
                    }
                }

                // increment pfC3x9jk and pfQw9x9ik for next iteration of k
                pfC3x9jk++;
                pfQw9x9ik++;

            } // end of loop over k

            // increment pftmpA9x3ij for next iteration of j
            pftmpA9x3ij++;

        } // end of loop over j
    } // end of loop over i

    // set symmetric P+ (3x3 scratch sub-matrix) to C * P- * C^T + Qv
    // = C * (Qw * C^T) + Qv = C * ftmpA9x3 + Qv
    // both C and ftmpA9x3 are sparse but not symmetric
    for (i = 0; i < 3; i++) { // loop over rows of P+
        // initialize pfPPlus9x9ij for current i, j=i
        pfPPlus9x9ij = kalmanData.fPPlus9x9[i] + i;

        for (j = i; j < 3; j++) { // loop over above diagonal columns of P+
            // zero P+[i][j] 
            *pfPPlus9x9ij = 0.0F;

            // initialize pfC3x9ik for current i, k=0
            pfC3x9ik = kalmanData.fC3x9[i];

            // initialize pftmpA9x3kj for current j, k=0
            pftmpA9x3kj = *ftmpA9x3 + j;

            // sum matrix products over inner loop over k
            for (k = 0; k < 9; k++) {
                if ((*pfC3x9ik != 0.0F) && (*pftmpA9x3kj != 0.0F)) {
                    if (*pfC3x9ik == 1.0F) {
                        *pfPPlus9x9ij += *pftmpA9x3kj;
                    } else if (*pfC3x9ik == -1.0F) {
                        *pfPPlus9x9ij -= *pftmpA9x3kj;
                    } else {
                        *pfPPlus9x9ij += *pfC3x9ik * *pftmpA9x3kj;
                    }
                }

                // update pfC3x9ik and pftmpA9x3kj for next iteration of k
                pfC3x9ik++;						
                pftmpA9x3kj += 3;

            } // end of loop over k

            // increment pfPPlus9x9ij for next iteration of j
            pfPPlus9x9ij++;

        } // end of loop over j
    } // end of loop over i

    // add in noise covariance terms to the diagonal
    kalmanData.fPPlus9x9[0][0] +=  kalmanData.fQvAA;
    kalmanData.fPPlus9x9[1][1] +=  kalmanData.fQvAA;
    kalmanData.fPPlus9x9[2][2] +=  kalmanData.fQvAA;

    // copy above diagonal terms of P+ (3x3 scratch sub-matrix) to below diagonal terms
    kalmanData.fPPlus9x9[1][0] = kalmanData.fPPlus9x9[0][1];
    kalmanData.fPPlus9x9[2][0] = kalmanData.fPPlus9x9[0][2];
    kalmanData.fPPlus9x9[2][1] = kalmanData.fPPlus9x9[1][2];

    // calculate inverse of P+ (3x3 scratch sub-matrix) = inv(C * P- * C^T + Qv) = inv(C * Qw * C^T + Qv)
    for (i = 0; i < 3; i++) pfRows[i] = kalmanData.fPPlus9x9[i];
    
    fmatrixAeqInvA(pfRows, iColInd, iRowInd, iPivot, 3);

    // set K = P- * C^T * inv(C * P- * C^T + Qv) = Qw * C^T * inv(C * Qw * C^T + Qv)
    // = ftmpA9x3 * P+ (3x3 sub-matrix)
    // ftmpA9x3 = Qw * C^T is sparse but P+ (3x3 sub-matrix) is not
    // K is not symmetric because C is not symmetric
    for (i = 0; i < 9; i++) { // loop over rows of K9x3
        // initialize pfK9x3ij for i, j=0
        pfK9x3ij = kalmanData.fK9x3[i];

        for (j = 0; j < 3; j++) { // loop over columns of K9x3
            // zero the matrix element fK9x3[i][j]
            *pfK9x3ij = 0.0F;

            // initialize pftmpA9x3ik for i, k=0
            pftmpA9x3ik = ftmpA9x3[i];

            // initialize pfPPlus9x9kj for j, k=0
            pfPPlus9x9kj = *kalmanData.fPPlus9x9 + j;

            // sum matrix products over inner loop over k
            for (k = 0; k < 3; k++) {
                if (*pftmpA9x3ik != 0.0F) *pfK9x3ij += *pftmpA9x3ik * *pfPPlus9x9kj;
                // increment pftmpA9x3ik and pfPPlus9x9kj for next iteration of k
                pftmpA9x3ik++;
                pfPPlus9x9kj += 9;

            } // end of loop over k
            // increment pfK9x3ij for the next iteration of j
            pfK9x3ij++;

        } // end of loop over j
    } // end of loop over i

    // *********************************************************************************
    // calculate a posteriori error estimate: xe+ = K * ze-
    // *********************************************************************************

    // update the a posteriori state vector
    for (i = X; i <= Z; i++) {
        // zero a posteriori error terms
        kalmanData.fThErrPl[i] = kalmanData.fbErrPl[i] = kalmanData.faErrSePl[i] = 0.0F;

        // accumulate the error vector terms K * ze-
        for (k = 0; k < 3; k++) {
            kalmanData.fThErrPl[i] += kalmanData.fK9x3[i][k] * kalmanData.fgErrSeMi[k];
            kalmanData.fbErrPl[i] += kalmanData.fK9x3[i + 3][k] * kalmanData.fgErrSeMi[k];
            kalmanData.faErrSePl[i] += kalmanData.fK9x3[i + 6][k] * kalmanData.fgErrSeMi[k];
        }
    }

    // *********************************************************************************
    // apply the a posteriori error corrections to the a posteriori state vector
    // *********************************************************************************

    // get the a posteriori delta quaternion
    fQuaternionFromRotationVectorDeg(&(kalmanData.fDeltaq), kalmanData.fThErrPl, -1.0F);

    // compute the a posteriori orientation quaternion fqPl = fqMi * Deltaq(-thetae+)
    // the resulting quaternion may have negative scalar component q0
    qAeqBxC(&(kalmanData.fqPl), &(kalmanData.fqMi), &(kalmanData.fDeltaq));

    // normalize the a posteriori orientation quaternion to stop error propagation 
    // the renormalization function ensures that the scalar component q0 is non-negative
    fqAeqNormqA(&(kalmanData.fqPl));

    // compute the a posteriori rotation matrix from the a posteriori quaternion
    fRotationMatrixFromQuaternion(kalmanData.fRPl, &(kalmanData.fqPl));

    // compute the rotation vector from the a posteriori quaternion
    fRotationVectorDegFromQuaternion(&(kalmanData.fqPl), kalmanData.fRVecPl);

    // update the a posteriori gyro offset vector b+ and linear acceleration vector a+ (sensor frame)
    for (i = X; i <= Z; i++) {
        // b+[k] = b-[k] - be+[k] = b+[k] - be+[k] (deg/s)
        kalmanData.fbPl[i] -= kalmanData.fbErrPl[i];
        // a+ = a- - ae+ (g, sensor frame)
        kalmanData.faSePl[i] = kalmanData.faSeMi[i] - kalmanData.faErrSePl[i];
    }

    // *********************************************************************************
    // compute the a posteriori Euler angles from the orientation matrix
    // *********************************************************************************

    // calculate the Euler angles
    fAnglesDegFromRotationMatrix(kalmanData.fRPl, &roll, &pitch, &yaw);
    
    q0=kalmanData.fqPl.q0;
    q1=kalmanData.fqPl.q1;
    q2=kalmanData.fqPl.q2;    
    q3=kalmanData.fqPl.q3;
    
//    roll = kalmanData.fPhiPl;
//    pitch = kalmanData.fThePl;
//    yaw = kalmanData.fPsiPl;

    // ***********************************************************************************
    // calculate (symmetric) a posteriori error covariance matrix P+
    // P+ = (I12 - K * C) * P- = (I12 - K * C) * Qw = Qw - K * (C * Qw)
    // both Qw and P+ are used as working arrays in this section
    // at the end of this section, P+ is valid but Qw is over-written
    // ***********************************************************************************

    // set P+ (3x9 scratch sub-matrix) to the product C (3x9) * Qw (9x9)
    // where both C and Qw are sparse and C has a significant number of +1 entries
    // the resulting matrix is sparse but not symmetric
    for (i = 0; i < 3; i++) { // loop over the rows of P+
        // initialize pfPPlus9x9ij for current i, j=0
        pfPPlus9x9ij = kalmanData.fPPlus9x9[i];

        for (j = 0; j < 9; j++) { // loop over the columns of P+
            // zero P+[i][j] 
            *pfPPlus9x9ij = 0.0F;

            // initialize pfC3x9ik for current i, k=0
            pfC3x9ik = kalmanData.fC3x9[i];

            // initialize pfQw9x9kj for current j, k=0
            pfQw9x9kj = &kalmanData.fQw9x9[0][j];

            // sum matrix products over inner loop over k
            for (k = 0; k < 9; k++) {
                if ((*pfC3x9ik != 0.0F) && (*pfQw9x9kj != 0.0F)) {
                    if (*pfC3x9ik == 1.0F) {
                        *pfPPlus9x9ij += *pfQw9x9kj;
                    } else if (*pfC3x9ik == -1.0F) {
                        *pfPPlus9x9ij -= *pfQw9x9kj;
                    } else {
                        *pfPPlus9x9ij += *pfC3x9ik * *pfQw9x9kj;
                    }
                }

                // update pfC3x9ik and pfQw9x9kj for next iteration of k
                pfC3x9ik++;
                pfQw9x9kj += 9;

            } // end of loop over k

            // increment pfPPlus9x9ij for next iteration of j
            pfPPlus9x9ij++;

        } // end of loop over j
    } // end of loop over i

    // compute P+ = (I9 - K * C) * Qw = Qw - K * (C * Qw) = Qw - K * P+ (3x9 sub-matrix)
    // storing result P+ in Qw and over-writing Qw which is OK since Qw is later computed from P+
    // where working array P+ (6x12 sub-matrix) is sparse but K is not sparse
    // only on and above diagonal terms of P+ are computed since P+ is symmetric
    for (i = 0; i < 9; i++) {
        // initialize pfQw9x9ij for i, j=i
        pfQw9x9ij = kalmanData.fQw9x9[i] + i;

        for (j = i; j < 9; j++) {
            // initialize pfK9x3ik for i, k=0
            pfK9x3ik = kalmanData.fK9x3[i];

            // initialize pfPPlus9x9kj for j, k=0
            pfPPlus9x9kj = *kalmanData.fPPlus9x9 + j;

            // compute on and above diagonal matrix entry
            for (k = 0; k < 3; k++) {
                // check for non-zero values since P+ (3x9 scratch sub-matrix) is sparse
                if (*pfPPlus9x9kj != 0.0F) *pfQw9x9ij -= *pfK9x3ik * *pfPPlus9x9kj;
                
                // increment pfK9x3ik and pfPPlus9x9kj for next iteration of k
                pfK9x3ik++;
                pfPPlus9x9kj += 9;

            } // end of loop over k

            // increment pfQw9x9ij for next iteration of j
            pfQw9x9ij++;

        } // end of loop over j
    } // end of loop over i

    // Qw now holds the on and above diagonal elements of P+ (9x9)
    // so perform a simple copy to the all elements of P+
    // after execution of this code P+ is valid but Qw remains invalid
    for (i = 0; i < 9; i++) {
        // initialize pfPPlus9x9ij and pfQw9x9ij for i, j=i
        pfPPlus9x9ij = kalmanData.fPPlus9x9[i] + i;
        pfQw9x9ij = kalmanData.fQw9x9[i] + i;

        // copy the on-diagonal elements and increment pointers to enter loop at j=i+1
        *(pfPPlus9x9ij++) = *(pfQw9x9ij++);

        // loop over above diagonal columns j copying to below-diagonal elements
        for (j = i + 1; j < 9; j++) *(pfPPlus9x9ij++)= kalmanData.fPPlus9x9[j][i] = *(pfQw9x9ij++);
    }

    // *********************************************************************************
    // re-create the noise covariance matrix Qw=fn(P+) for the next iteration
    // using the elements of P+ which are now valid
    // Qw was over-written earlier but is here recomputed (all elements)
    // *********************************************************************************

    // zero the matrix Qw (9x9)
    for (i = 0; i < 9; i++) {
        for (j = 0; j < 9; j++) kalmanData.fQw9x9[i][j] = 0.0F;
    }

    // update the covariance matrix components
    for (i = 0; i < 3; i++) {
        // Qw[th-th-] = Qw[0-2][0-2] = E[th-(th-)^T] = Q[th+th+] + deltat^2 * (Q[b+b+] + (Qwb + QvG) * I)
        kalmanData.fQw9x9[i][i] = kalmanData.fPPlus9x9[i][i] + kalmanData.fdeltatsq * (kalmanData.fPPlus9x9[i + 3][i + 3] + kalmanData.fQwbplusQvG);

        // Qw[b-b-] = Qw[3-5][3-5] = E[b-(b-)^T] = Q[b+b+] + Qwb * I
        kalmanData.fQw9x9[i + 3][i + 3] = kalmanData.fPPlus9x9[i + 3][i + 3] + FQWB;

        // Qw[th-b-] = Qw[0-2][3-5] = E[th-(b-)^T] = -deltat * (Q[b+b+] + Qwb * I) = -deltat * Qw[b-b-]
        kalmanData.fQw9x9[i][i + 3] = kalmanData.fQw9x9[i + 3][i] = -kalmanData.fdeltat * kalmanData.fQw9x9[i + 3][i + 3];

        // Qw[a-a-] = Qw[6-8][6-8] = E[a-(a-)^T] = ca^2 * Q[a+a+] + Qwa * I
        kalmanData.fQw9x9[i + 6][i + 6] = kalmanData.fcasq * kalmanData.fPPlus9x9[i + 6][i + 6] + FQWA;
    }

    return;
}

void fRotationMatrixFromQuaternion(float R[][3], const struct quaternion *pq){
    float f2q;
    float f2q0q0, f2q0q1, f2q0q2, f2q0q3;
    float f2q1q1, f2q1q2, f2q1q3;
    float f2q2q2, f2q2q3;
    float f2q3q3;

    // calculate products
    f2q = 2.0F * pq->q0;
    f2q0q0 = f2q * pq->q0;
    f2q0q1 = f2q * pq->q1;
    f2q0q2 = f2q * pq->q2;
    f2q0q3 = f2q * pq->q3;
    f2q = 2.0F * pq->q1;
    f2q1q1 = f2q * pq->q1;
    f2q1q2 = f2q * pq->q2;
    f2q1q3 = f2q * pq->q3;
    f2q = 2.0F * pq->q2;
    f2q2q2 = f2q * pq->q2;
    f2q2q3 = f2q * pq->q3;
    f2q3q3 = 2.0F * pq->q3 * pq->q3;

    // calculate the rotation matrix assuming the quaternion is normalized
    R[X][X] = f2q0q0 + f2q1q1 - 1.0F;
    R[X][Y] = f2q1q2 + f2q0q3;
    R[X][Z] = f2q1q3 - f2q0q2;
    R[Y][X] = f2q1q2 - f2q0q3;
    R[Y][Y] = f2q0q0 + f2q2q2 - 1.0F;
    R[Y][Z] = f2q2q3 + f2q0q1;
    R[Z][X] = f2q1q3 + f2q0q2;
    R[Z][Y] = f2q2q3 - f2q0q1;
    R[Z][Z] = f2q0q0 + f2q3q3 - 1.0F;

    return;
}

void fQuaternionFromRotationMatrix(float R[][3], struct quaternion *pq){
    float fq0sq;			// q0^2
    float recip4q0;			// 1/4q0
#define SMALLQ0 0.01F		// limit of quaternion scalar component requiring special algorithm

    // the quaternion is not explicitly normalized in this function on the assumption that it
    // is supplied with a normalized rotation matrix. if the rotation matrix is normalized then
    // the quaternion will also be normalized even if the case of small q0

    // get q0^2 and q0
    fq0sq = 0.25F * (1.0F + R[X][X] + R[Y][Y] + R[Z][Z]);
    pq->q0 = sqrtf(fabs(fq0sq));

    // normal case when q0 is not small meaning rotation angle not near 180 deg
    if (pq->q0 > SMALLQ0) {
        // calculate q1 to q3
        recip4q0 = 0.25F / pq->q0;
        pq->q1 = recip4q0 * (R[Y][Z] - R[Z][Y]);
        pq->q2 = recip4q0 * (R[Z][X] - R[X][Z]);
        pq->q3 = recip4q0 * (R[X][Y] - R[Y][X]);
    } else {// end of general case
        // special case of near 180 deg corresponds to nearly symmetric matrix
        // which is not numerically well conditioned for division by small q0
        // instead get absolute values of q1 to q3 from leading diagonal
        pq->q1 = sqrtf(fabs(0.5F * (1.0F + R[X][X]) - fq0sq));
        pq->q2 = sqrtf(fabs(0.5F * (1.0F + R[Y][Y]) - fq0sq));
        pq->q3 = sqrtf(fabs(0.5F * (1.0F + R[Z][Z]) - fq0sq));

        // correct the signs of q1 to q3 by examining the signs of differenced off-diagonal terms
        if ((R[Y][Z] - R[Z][Y]) < 0.0F) pq->q1 = -pq->q1;
        if ((R[Z][X] - R[X][Z]) < 0.0F) pq->q2 = -pq->q2;
        if ((R[X][Y] - R[Y][X]) < 0.0F) pq->q3 = -pq->q3;
    } // end of special case

    return;
}


// computes normalized rotation quaternion from a rotation vector (deg)
void fQuaternionFromRotationVectorDeg(struct quaternion *pq, const float rvecdeg[], float fscaling){
    float fetadeg;			// rotation angle (deg)
    float fetarad;			// rotation angle (rad)
    float fetarad2;			// eta (rad)^2
    float fetarad4;			// eta (rad)^4
    float sinhalfeta;		// sin(eta/2)
    float fvecsq;			// q1^2+q2^2+q3^2
    float ftmp;				// scratch variable

    // compute the scaled rotation angle eta (deg) which can be both positve or negative
    fetadeg = fscaling * sqrtf(rvecdeg[X] * rvecdeg[X] + rvecdeg[Y] * rvecdeg[Y] + rvecdeg[Z] * rvecdeg[Z]);
    fetarad = fetadeg * DEGTORADMULT;
    fetarad2 = fetarad * fetarad;

    // calculate the sine and cosine using small angle approximations or exact
    // angles under sqrt(0.02)=0.141 rad is 8.1 deg and 1620 deg/s (=936deg/s in 3 axes) at 200Hz and 405 deg/s at 50Hz
    if (fetarad2 <= 0.02F) {
        // use MacLaurin series up to and including third order
        sinhalfeta = fetarad * (0.5F - ONEOVER48 * fetarad2);
    } else if (fetarad2 <= 0.06F) {
        // use MacLaurin series up to and including fifth order
        // angles under sqrt(0.06)=0.245 rad is 14.0 deg and 2807 deg/s (=1623deg/s in 3 axes) at 200Hz and 703 deg/s at 50Hz
        fetarad4 = fetarad2 * fetarad2;
        sinhalfeta = fetarad * (0.5F - ONEOVER48 * fetarad2 + ONEOVER3840 * fetarad4);
    } else {
        // use exact calculation
        sinhalfeta = (float)sinf(0.5F * fetarad);
    }

    // compute the vector quaternion components q1, q2, q3
    if (fetadeg != 0.0F) {
        // general case with non-zero rotation angle
        ftmp = fscaling * sinhalfeta / fetadeg;
        pq->q1 = rvecdeg[X] * ftmp;		// q1 = nx * sin(eta/2)
        pq->q2 = rvecdeg[Y] * ftmp;		// q2 = ny * sin(eta/2)
        pq->q3 = rvecdeg[Z] * ftmp;		// q3 = nz * sin(eta/2)
    } else {
        // zero rotation angle giving zero vector component
        pq->q1 = pq->q2 = pq->q3 = 0.0F;
    }

    // compute the scalar quaternion component q0 by explicit normalization
    // taking care to avoid rounding errors giving negative operand to sqrt
    fvecsq = pq->q1 * pq->q1 + pq->q2 * pq->q2 + pq->q3 * pq->q3;
    if (fvecsq <= 1.0F) {
        // normal case
        pq->q0 = sqrtf(1.0F - fvecsq);
    } else {
        // rounding errors are present
        pq->q0 = 0.0F;
    }
    return;
}

// function compute the quaternion product qA = qA * qB
void qAeqAxB(struct quaternion *pqA, const struct quaternion *pqB) {
    struct quaternion qProd;

    // perform the quaternion product
    qProd.q0 = pqA->q0 * pqB->q0 - pqA->q1 * pqB->q1 - pqA->q2 * pqB->q2 - pqA->q3 * pqB->q3;
    qProd.q1 = pqA->q0 * pqB->q1 + pqA->q1 * pqB->q0 + pqA->q2 * pqB->q3 - pqA->q3 * pqB->q2;
    qProd.q2 = pqA->q0 * pqB->q2 - pqA->q1 * pqB->q3 + pqA->q2 * pqB->q0 + pqA->q3 * pqB->q1;
    qProd.q3 = pqA->q0 * pqB->q3 + pqA->q1 * pqB->q2 - pqA->q2 * pqB->q1 + pqA->q3 * pqB->q0;

    // copy the result back into qA
    *pqA = qProd;

    return;
}

// function compute the quaternion product qA * qB
void qAeqBxC(struct quaternion *pqA, const struct quaternion *pqB, const struct quaternion *pqC) {
    pqA->q0 = pqB->q0 * pqC->q0 - pqB->q1 * pqC->q1 - pqB->q2 * pqC->q2 - pqB->q3 * pqC->q3;
    pqA->q1 = pqB->q0 * pqC->q1 + pqB->q1 * pqC->q0 + pqB->q2 * pqC->q3 - pqB->q3 * pqC->q2;
    pqA->q2 = pqB->q0 * pqC->q2 - pqB->q1 * pqC->q3 + pqB->q2 * pqC->q0 + pqB->q3 * pqC->q1;
    pqA->q3 = pqB->q0 * pqC->q3 + pqB->q1 * pqC->q2 - pqB->q2 * pqC->q1 + pqB->q3 * pqC->q0;

    return;
}

void fmatrixAeqInvA(float *A[], int8_t iColInd[], int8_t iRowInd[], int8_t iPivot[], int8_t isize) {
    float largest;					// largest element used for pivoting
    float scaling;					// scaling factor in pivoting
    float recippiv;					// reciprocal of pivot element
    float ftmp;						// temporary variable used in swaps
    int8_t i, j, k, l, m;				// index counters
    int8_t iPivotRow, iPivotCol;		// row and column of pivot element

    // to avoid compiler warnings
    iPivotRow = iPivotCol = 0;

    // initialize the pivot array to 0
    for (j = 0; j < isize; j++) iPivot[j] = 0;
    
    // main loop i over the dimensions of the square matrix A
    for (i = 0; i < isize; i++) {
        // zero the largest element found for pivoting
        largest = 0.0F;
        // loop over candidate rows j
        for (j = 0; j < isize; j++) {
            // check if row j has been previously pivoted
            if (iPivot[j] != 1) {
                // loop over candidate columns k
                for (k = 0; k < isize; k++) {
                    // check if column k has previously been pivoted
                    if (iPivot[k] == 0) {
                        // check if the pivot element is the largest found so far
                        if (fabs(A[j][k]) >= largest) {
                            // and store this location as the current best candidate for pivoting
                            iPivotRow = j;
                            iPivotCol = k;
                            largest = (float) fabs(A[iPivotRow][iPivotCol]);
                        }
                    }
                    else if (iPivot[k] > 1) {
                        // zero determinant situation: exit with identity matrix
                        fmatrixAeqI(A, isize);
                        return;
                    }
                }
            }
        }
        // increment the entry in iPivot to denote it has been selected for pivoting
        iPivot[iPivotCol]++;

        // check the pivot rows iPivotRow and iPivotCol are not the same before swapping
        if (iPivotRow != iPivotCol) {
            // loop over columns l
            for (l = 0; l < isize; l++) {
                // and swap all elements of rows iPivotRow and iPivotCol
                ftmp = A[iPivotRow][l];
                A[iPivotRow][l] = A[iPivotCol][l];
                A[iPivotCol][l] = ftmp;
            }
        }

        // record that on the i-th iteration rows iPivotRow and iPivotCol were swapped
        iRowInd[i] = iPivotRow;
        iColInd[i] = iPivotCol;

        // check for zero on-diagonal element (singular matrix) and return with identity matrix if detected
        if (A[iPivotCol][iPivotCol] == 0.0F) {
            // zero determinant situation: exit with identity matrix
            fmatrixAeqI(A, isize);
            return;
        }

        // calculate the reciprocal of the pivot element knowing it's non-zero
        recippiv = 1.0F / A[iPivotCol][iPivotCol];
        // by definition, the diagonal element normalizes to 1
        A[iPivotCol][iPivotCol] = 1.0F;
        // multiply all of row iPivotCol by the reciprocal of the pivot element including the diagonal element
        // the diagonal element A[iPivotCol][iPivotCol] now has value equal to the reciprocal of its previous value
        for (l = 0; l < isize; l++) {
            A[iPivotCol][l] *= recippiv;
        }
        // loop over all rows m of A
        for (m = 0; m < isize; m++) {
            if (m != iPivotCol) {
                // scaling factor for this row m is in column iPivotCol
                scaling = A[m][iPivotCol];
                // zero this element
                A[m][iPivotCol] = 0.0F;
                // loop over all columns l of A and perform elimination
                for (l = 0; l < isize; l++) A[m][l] -= A[iPivotCol][l] * scaling;
            }
        }
    } // end of loop i over the matrix dimensions

    // finally, loop in inverse order to apply the missing column swaps
    for (l = isize - 1; l >= 0; l--) {
        // set i and j to the two columns to be swapped
        i = iRowInd[l];
        j = iColInd[l];

        // check that the two columns i and j to be swapped are not the same
        if (i != j) {
            // loop over all rows k to swap columns i and j of A
            for (k = 0; k < isize; k++) {
                ftmp = A[k][i];
                A[k][i] = A[k][j];
                A[k][j] = ftmp;
            }
        }
    }
    return;
}

// function sets the matrix A to the identity matrix
void fmatrixAeqI(float *A[], int16_t rc){
	// rc = rows and columns in A

	float *pAij;	// pointer to A[i][j]
	int i, j;		// loop counters

	for (i = 0; i < rc; i++) {
		// set pAij to &A[i][j=0]
		pAij = A[i];
		for (j = 0; j < rc; j++) *(pAij++) = 0.0F;
		A[i][i] = 1.0F;
	}
	return;
}

// function normalizes a rotation quaternion and ensures q0 is non-negative
void fqAeqNormqA(struct quaternion *pqA){
    float fNorm;					// quaternion Norm

    // calculate the quaternion Norm
    fNorm = sqrtf(pqA->q0 * pqA->q0 + pqA->q1 * pqA->q1 + pqA->q2 * pqA->q2 + pqA->q3 * pqA->q3);
    if (fNorm > CORRUPTQUAT) {
        // general case
        fNorm = 1.0F / fNorm;
        pqA->q0 *= fNorm;
        pqA->q1 *= fNorm;
        pqA->q2 *= fNorm;
        pqA->q3 *= fNorm;
    } else {
        // return with identity quaternion since the quaternion is corrupted
        pqA->q0 = 1.0F;
        pqA->q1 = pqA->q2 = pqA->q3 = 0.0F;
    }

    // correct a negative scalar component if the function was called with negative q0
    if (pqA->q0 < 0.0F) {
        pqA->q0 = -pqA->q0;
        pqA->q1 = -pqA->q1;
        pqA->q2 = -pqA->q2;
        pqA->q3 = -pqA->q3;
    }
    return;
}

// Aerospace NED accelerometer 3DOF tilt function computing rotation matrix fR
void f3DOFTiltNED(float fR[][3], float fGp[]){
    // the NED self-consistency twist occurs at 90 deg pitch
	
    // local variables
    uint16_t i;				// counter
    float fmodGxyz;			// modulus of the x, y, z accelerometer readings
    float fmodGyz;			// modulus of the y, z accelerometer readings
    float frecipmodGxyz;	// reciprocal of modulus
    float ftmp;				// scratch variable

    // compute the accelerometer squared magnitudes
    fmodGyz = fGp[Y] * fGp[Y] + fGp[Z] * fGp[Z];
    fmodGxyz = fmodGyz + fGp[X] * fGp[X];

    // check for freefall special case where no solution is possible
    if (fmodGxyz == 0.0F) {
        float *pAij;	// pointer to A[i][j]
        uint8_t j;		// loop counter
        for (i = 0; i < 3; i++) {
            // set pAij to &A[i][j=0]
            pAij = fR[i];
            for (j = 0; j < 3; j++) *(pAij++) = 0.0F;
            fR[i][i] = 1.0F;
        }
        return;
    }

    // check for vertical up or down gimbal lock case
    if (fmodGyz == 0.0F){
        float *pAij;	// pointer to A[i][j]
        uint8_t j;		// counter
        for (i = 0; i < 3; i++) {
            // set pAij to &A[i][j=0]
            pAij = fR[i];
            for (j = 0; j < 3; j++) *(pAij++) = 0.0F;
        }
        fR[Y][Y] = 1.0F;
        if (fGp[X] >= 0.0F) {
            fR[X][Z] = 1.0F;
            fR[Z][X] = -1.0F;
        } else {
            fR[X][Z] = -1.0F;
            fR[Z][X] = 1.0F;
        }
        return;
    }

    // compute moduli for the general case
    fmodGyz = sqrtf(fmodGyz);
    fmodGxyz = sqrtf(fmodGxyz);
    frecipmodGxyz = 1.0F / fmodGxyz;
    ftmp = fmodGxyz / fmodGyz;

    // normalize the accelerometer reading into the z column
    for (i = X; i <= Z; i++) {
        fR[i][Z] = fGp[i] * frecipmodGxyz;
    }

    // construct x column of orientation matrix
    fR[X][X] = fmodGyz * frecipmodGxyz;
    fR[Y][X] = -fR[X][Z] * fR[Y][Z] * ftmp;
    fR[Z][X] = -fR[X][Z] * fR[Z][Z] * ftmp;

    // // construct y column of orientation matrix
    fR[X][Y] = 0.0F;
    fR[Y][Y] = fR[Z][Z] * ftmp;
    fR[Z][Y] = -fR[Y][Z] * ftmp;

    return;
}




// computes rotation vector (deg) from rotation quaternion
void fRotationVectorDegFromQuaternion(struct quaternion *pq, float rvecdeg[]){
    float fetarad;			// rotation angle (rad)
    float fetadeg;			// rotation angle (deg)
    float sinhalfeta;		// sin(eta/2)
    float ftmp;				// scratch variable

    // calculate the rotation angle in the range 0 <= eta < 360 deg
    if ((pq->q0 >= 1.0F) || (pq->q0 <= -1.0F)) {
        // rotation angle is 0 deg or 2*180 deg = 360 deg = 0 deg
        fetarad = 0.0F;
        fetadeg = 0.0F;
    } else {
        // general case returning 0 < eta < 360 deg 
        fetarad = 2.0F * acosf(pq->q0); 
        fetadeg = fetarad * RADTODEGMULT;
    }

    // map the rotation angle onto the range -180 deg <= eta < 180 deg 
    if (fetadeg >= 180.0F) {
        fetadeg -= 360.0F;
        fetarad = fetadeg * DEGTORADMULT;
    }

    // calculate sin(eta/2) which will be in the range -1 to +1
    sinhalfeta = (float)sinf(0.5F * fetarad);

    // calculate the rotation vector (deg)
    if (sinhalfeta == 0.0F) {
        // the rotation angle eta is zero and the axis is irrelevant 
        rvecdeg[X] = rvecdeg[Y] = rvecdeg[Z] = 0.0F;
    } else {
        // general case with non-zero rotation angle
        ftmp = fetadeg / sinhalfeta;
        rvecdeg[X] = pq->q1 * ftmp;
        rvecdeg[Y] = pq->q2 * ftmp;
        rvecdeg[Z] = pq->q3 * ftmp;
    }
    return;
}

// extract the angles in degrees from the rotation matrix
void fAnglesDegFromRotationMatrix(float R[][3], float *pfPhiDeg, float *pfTheDeg, float *pfPsiDeg){
	// calculate the pitch angle -90.0 <= Theta <= 90.0 deg
    *pfTheDeg = asin(-R[X][Z])*RADTODEGMULT;

	// calculate the roll angle range -180.0 <= Phi < 180.0 deg
    *pfPhiDeg = atan2(R[Y][Z], R[Z][Z]) * RADTODEGMULT;

	// map +180 roll onto the functionally equivalent -180 deg roll
    if (*pfPhiDeg == 180.0F) *pfPhiDeg = -180.0F;
	
	// calculate the yaw (compass) angle 0.0 <= Psi < 360.0 deg
    if (*pfTheDeg == 90.0F) {
		// vertical upwards gimbal lock case
		*pfPsiDeg = (atan2(R[Z][Y], R[Y][Y]) + *pfPhiDeg) * RADTODEGMULT;
    } else if (*pfTheDeg == -90.0F) {
        // vertical downwards gimbal lock case
        *pfPsiDeg = (atan2(-R[Z][Y], R[Y][Y]) - *pfPhiDeg) * RADTODEGMULT;
    } else {
		// general case
        *pfPsiDeg = atan2(R[X][Y], R[X][X]) * RADTODEGMULT;
    }

    // map yaw angle Psi onto range 0.0 <= Psi < 360.0 deg
    if (*pfPsiDeg < 0.0F) *pfPsiDeg += 360.0F;

    // check for rounding errors mapping small negative angle to 360 deg
    if (*pfPsiDeg >= 360.0F) *pfPsiDeg = 0.0F;
	
	// calculate the tilt angle from vertical Chi (0 <= Chi <= 180 deg) 
    //	*pfChiDeg = acos(R[Z][Z]) * RADTODEGMULT;

    return;
}

#endif
