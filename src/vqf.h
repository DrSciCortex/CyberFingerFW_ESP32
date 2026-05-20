// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
//
// SPDX-License-Identifier: MIT

#ifndef VQF_H
#define VQF_H

#define VQF_SINGLE_PRECISION

#include <stddef.h>

#define M_SQRT2 1.41421356237309504880

#ifndef VQF_SINGLE_PRECISION
typedef double vqf_real_t;
#else
typedef float vqf_real_t;
#endif

struct VQFParams {
    vqf_real_t tauAcc = 4.337983;
    vqf_real_t tauMag = 9.0;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    bool motionBiasEstEnabled = true;
#endif
    bool restBiasEstEnabled = true;
    bool magDistRejectionEnabled = true;
    vqf_real_t biasSigmaInit = 3.219453;
    vqf_real_t biasForgettingTime = 136.579346;
    vqf_real_t biasClip = 5.0;
    vqf_real_t biasSigmaMotion = 0.348501;
    vqf_real_t biasVerticalForgettingFactor = 0.007056;
    vqf_real_t biasSigmaRest = 0.063616;
    vqf_real_t restMinT = 2.586910;
    vqf_real_t restFilterTau = 1.114532;
    vqf_real_t restThGyr = 1.399189;
    vqf_real_t restThAcc = 1.418598;
    vqf_real_t magCurrentTau = 0.05;
    vqf_real_t magRefTau = 20.0;
    vqf_real_t magNormTh = 0.1;
    vqf_real_t magDipTh = 10.0;
    vqf_real_t magNewTime = 20.0;
    vqf_real_t magNewFirstTime = 5.0;
    vqf_real_t magNewMinGyr = 20.0;
    vqf_real_t magMinUndisturbedTime = 0.5;
    vqf_real_t magMaxRejectionTime = 60.0;
    vqf_real_t magRejectionFactor = 2.0;
};

struct VQFState {
    vqf_real_t gyrQuat[4];
    vqf_real_t accQuat[4];
    vqf_real_t delta;
    bool restDetected;
    bool magDistDetected;
    vqf_real_t lastAccLp[3];
    vqf_real_t accLpState[3 * 2];
    vqf_real_t lastAccCorrAngularRate;
    vqf_real_t kMagInit;
    vqf_real_t lastMagDisAngle;
    vqf_real_t lastMagCorrAngularRate;
    vqf_real_t bias[3];
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_real_t biasP[9];
#else
    vqf_real_t biasP;
#endif
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_real_t motionBiasEstRLpState[9 * 2];
    vqf_real_t motionBiasEstBiasLpState[2 * 2];
#endif
    vqf_real_t restLastSquaredDeviations[2];
    vqf_real_t restT;
    vqf_real_t restLastGyrLp[3];
    vqf_real_t restGyrLpState[3 * 2];
    vqf_real_t restLastAccLp[3];
    vqf_real_t restAccLpState[3 * 2];
    vqf_real_t magRefNorm;
    vqf_real_t magRefDip;
    vqf_real_t magUndisturbedT;
    vqf_real_t magRejectT;
    vqf_real_t magCandidateNorm;
    vqf_real_t magCandidateDip;
    vqf_real_t magCandidateT;
    vqf_real_t magNormDip[2];
    vqf_real_t magNormDipLpState[2 * 2];
};

struct VQFCoefficients {
    vqf_real_t gyrTs;
    vqf_real_t accTs;
    vqf_real_t magTs;
    vqf_real_t accLpB[3];
    vqf_real_t accLpA[2];
    vqf_real_t kMag;
    vqf_real_t biasP0;
    vqf_real_t biasV;
    vqf_real_t biasMotionW;
    vqf_real_t biasVerticalW;
    vqf_real_t biasRestW;
    vqf_real_t restGyrLpB[3];
    vqf_real_t restGyrLpA[2];
    vqf_real_t restAccLpB[3];
    vqf_real_t restAccLpA[2];
    vqf_real_t kMagRef;
    vqf_real_t magNormDipLpB[3];
    vqf_real_t magNormDipLpA[2];
};

class VQF {
public:
    VQF(vqf_real_t gyrTs, vqf_real_t accTs = -1.0, vqf_real_t magTs = -1.0);
    VQF(const VQFParams& params, vqf_real_t gyrTs, vqf_real_t accTs = -1.0, vqf_real_t magTs = -1.0);

    void updateGyr(const vqf_real_t gyr[3]);
    void updateAcc(const vqf_real_t acc[3]);
    void updateMag(const vqf_real_t mag[3]);

    void getQuat3D(vqf_real_t out[4]) const;
    void getQuat6D(vqf_real_t out[4]) const;
    void getQuat9D(vqf_real_t out[4]) const;
    vqf_real_t getDelta() const;
    vqf_real_t getBiasEstimate(vqf_real_t out[3]) const;
    void setBiasEstimate(vqf_real_t bias[3], vqf_real_t sigma = -1.0);
    bool getRestDetected() const;
    bool getMagDistDetected() const;
    void getRelativeRestDeviations(vqf_real_t out[2]) const;
    vqf_real_t getMagRefNorm() const;
    vqf_real_t getMagRefDip() const;
    void setMagRef(vqf_real_t norm, vqf_real_t dip);

    void setTauAcc(vqf_real_t tauAcc);
    void setTauMag(vqf_real_t tauMag);
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    void setMotionBiasEstEnabled(bool enabled);
#endif
    void setRestBiasEstEnabled(bool enabled);
    void setMagDistRejectionEnabled(bool enabled);
    void setRestDetectionThresholds(vqf_real_t thGyr, vqf_real_t thAcc);

    const VQFParams& getParams() const { return params; }
    const VQFCoefficients& getCoeffs() const { return coeffs; }
    const VQFState& getState() const { return state; }
    void setState(const VQFState& state) { this->state = state; }
    void resetState();

    static void quatMultiply(const vqf_real_t q1[4], const vqf_real_t q2[4], vqf_real_t out[4]);
    static void quatConj(const vqf_real_t q[4], vqf_real_t out[4]);
    static void quatSetToIdentity(vqf_real_t out[4]);
    static void quatApplyDelta(vqf_real_t q[4], vqf_real_t delta, vqf_real_t out[4]);
    static void quatRotate(const vqf_real_t q[4], const vqf_real_t v[3], vqf_real_t out[3]);
    static vqf_real_t norm(const vqf_real_t vec[], size_t N);
    static void normalize(vqf_real_t vec[], size_t N);
    static void clip(vqf_real_t vec[], size_t N, vqf_real_t min, vqf_real_t max);
    static vqf_real_t gainFromTau(vqf_real_t tau, vqf_real_t Ts);
    static void filterCoeffs(vqf_real_t tau, vqf_real_t Ts, vqf_real_t outB[3], vqf_real_t outA[2]);
    static void filterInitialState(vqf_real_t x0, const vqf_real_t b[], const vqf_real_t a[], vqf_real_t out[2]);
    static void filterAdaptStateForCoeffChange(vqf_real_t last_y[], size_t N, const vqf_real_t b_old[3], const vqf_real_t a_old[2], const vqf_real_t b_new[3], const vqf_real_t a_new[2], vqf_real_t state[]);
    static vqf_real_t filterStep(vqf_real_t x, const vqf_real_t b[3], const vqf_real_t a[2], vqf_real_t state[2]);
    static void filterVec(const vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts, const vqf_real_t b[3], const vqf_real_t a[2], vqf_real_t state[], vqf_real_t out[]);
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    static void matrix3SetToScaledIdentity(vqf_real_t scale, vqf_real_t out[9]);
    static void matrix3Multiply(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9]);
    static void matrix3MultiplyTpsFirst(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9]);
    static void matrix3MultiplyTpsSecond(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9]);
    static bool matrix3Inv(const vqf_real_t in[9], vqf_real_t out[9]);
#endif

protected:
    void setup();
    VQFParams params;
    VQFState state;
    VQFCoefficients coeffs;
};

#endif
