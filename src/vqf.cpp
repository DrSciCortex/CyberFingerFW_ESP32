// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
//
// SPDX-License-Identifier: MIT

#include "vqf.h"
#include <math.h>
#include <algorithm>
#include <assert.h>

#define EPS 1e-7f
#define NaN (0.0f/0.0f)

inline vqf_real_t square(vqf_real_t x) { return x*x; }

VQF::VQF(vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs)
{
    coeffs.gyrTs = gyrTs;
    coeffs.accTs = accTs > 0 ? accTs : gyrTs;
    coeffs.magTs = magTs > 0 ? magTs : gyrTs;

    setup();
}

VQF::VQF(const VQFParams &params, vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs)
{
    this->params = params;

    coeffs.gyrTs = gyrTs;
    coeffs.accTs = accTs > 0 ? accTs : gyrTs;
    coeffs.magTs = magTs > 0 ? magTs : gyrTs;

    setup();
}

void VQF::updateGyr(const vqf_real_t gyr[3], vqf_real_t gyrTs)
{
    // Caller may pass the real elapsed time; fall back to the nominal rate.
    // Only the integration step below uses it - the rest-detection filter keeps
    // the nominal rate, since its coefficients are precomputed from it.
    if (gyrTs <= vqf_real_t(0.0)) {
        gyrTs = coeffs.gyrTs;
    }

    // rest detection
    if (params.restBiasEstEnabled || params.magDistRejectionEnabled) {
        filterVec(gyr, 3, params.restFilterTau, coeffs.gyrTs, coeffs.restGyrLpB, coeffs.restGyrLpA,
                  state.restGyrLpState, state.restLastGyrLp);

        state.restLastSquaredDeviations[0] = square(gyr[0] - state.restLastGyrLp[0])
                + square(gyr[1] - state.restLastGyrLp[1]) + square(gyr[2] - state.restLastGyrLp[2]);

        vqf_real_t biasClip = params.biasClip*vqf_real_t(M_PI/180.0);
        if (state.restLastSquaredDeviations[0] >= square(params.restThGyr*vqf_real_t(M_PI/180.0))
                || fabsf(state.restLastGyrLp[0]) > biasClip || fabsf(state.restLastGyrLp[1]) > biasClip
                || fabsf(state.restLastGyrLp[2]) > biasClip) {
            state.restT = 0.0;
            state.restDetected = false;
        }
    }

    // remove estimated gyro bias
    vqf_real_t gyrNoBias[3] = {gyr[0]-state.bias[0], gyr[1]-state.bias[1], gyr[2]-state.bias[2]};
    // gyroscope prediction step
    vqf_real_t gyrNorm = norm(gyrNoBias, 3);
    vqf_real_t angle = gyrNorm * gyrTs;
    if (gyrNorm > EPS) {
        vqf_real_t c = cosf(angle/2);
        vqf_real_t s = sinf(angle/2)/gyrNorm;
        vqf_real_t gyrStepQuat[4] = {c, s*gyrNoBias[0], s*gyrNoBias[1], s*gyrNoBias[2]};
        quatMultiply(state.gyrQuat, gyrStepQuat, state.gyrQuat);
        normalize(state.gyrQuat, 4);
    }
}

void VQF::updateAcc(const vqf_real_t acc[3])
{
    // ignore [0 0 0] samples
    if (acc[0] == vqf_real_t(0.0) && acc[1] == vqf_real_t(0.0) && acc[2] == vqf_real_t(0.0)) {
        return;
    }

    // rest detection
    if (params.restBiasEstEnabled) {
        filterVec(acc, 3, params.restFilterTau, coeffs.accTs, coeffs.restAccLpB, coeffs.restAccLpA,
                  state.restAccLpState, state.restLastAccLp);

        state.restLastSquaredDeviations[1] = square(acc[0] - state.restLastAccLp[0])
                + square(acc[1] - state.restLastAccLp[1]) + square(acc[2] - state.restLastAccLp[2]);

        if (state.restLastSquaredDeviations[1] >= square(params.restThAcc)) {
            state.restT = 0.0;
            state.restDetected = false;
        } else {
            state.restT += coeffs.accTs;
            if (state.restT >= params.restMinT) {
                state.restDetected = true;
            }
        }
    }

    vqf_real_t accEarth[3];

    // filter acc in inertial frame
    quatRotate(state.gyrQuat, acc, accEarth);
    filterVec(accEarth, 3, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.accLpState, state.lastAccLp);

    // transform to 6D earth frame and normalize
    quatRotate(state.accQuat, state.lastAccLp, accEarth);
    normalize(accEarth, 3);

    // inclination correction
    vqf_real_t accCorrQuat[4];
    vqf_real_t q_w = sqrtf((accEarth[2]+1)/2);
    if (q_w > 1e-6) {
        accCorrQuat[0] = q_w;
        accCorrQuat[1] = 0.5f*accEarth[1]/q_w;
        accCorrQuat[2] = -0.5f*accEarth[0]/q_w;
        accCorrQuat[3] = 0;
    } else {
        accCorrQuat[0] = 0;
        accCorrQuat[1] = 1;
        accCorrQuat[2] = 0;
        accCorrQuat[3] = 0;
    }
    quatMultiply(accCorrQuat, state.accQuat, state.accQuat);
    normalize(state.accQuat, 4);

    state.lastAccCorrAngularRate = acosf(std::min(std::max(accEarth[2], -1.0f), 1.0f))/coeffs.accTs;

    // bias estimation
    if (params.motionBiasEstEnabled || params.restBiasEstEnabled) {
        vqf_real_t biasClip = params.biasClip*vqf_real_t(M_PI/180.0);
        vqf_real_t accGyrQuat[4];
        vqf_real_t R[9];
        vqf_real_t biasLp[2];

        getQuat6D(accGyrQuat);
        R[0] = 1 - 2*square(accGyrQuat[2]) - 2*square(accGyrQuat[3]);
        R[1] = 2*(accGyrQuat[1]*accGyrQuat[2] - accGyrQuat[0]*accGyrQuat[3]);
        R[2] = 2*(accGyrQuat[1]*accGyrQuat[3] + accGyrQuat[0]*accGyrQuat[2]);
        R[3] = 2*(accGyrQuat[1]*accGyrQuat[2] + accGyrQuat[0]*accGyrQuat[3]);
        R[4] = 1 - 2*square(accGyrQuat[1]) - 2*square(accGyrQuat[3]);
        R[5] = 2*(accGyrQuat[2]*accGyrQuat[3] - accGyrQuat[0]*accGyrQuat[1]);
        R[6] = 2*(accGyrQuat[1]*accGyrQuat[3] - accGyrQuat[0]*accGyrQuat[2]);
        R[7] = 2*(accGyrQuat[2]*accGyrQuat[3] + accGyrQuat[0]*accGyrQuat[1]);
        R[8] = 1 - 2*square(accGyrQuat[1]) - 2*square(accGyrQuat[2]);

        biasLp[0] = R[0]*state.bias[0] + R[1]*state.bias[1] + R[2]*state.bias[2];
        biasLp[1] = R[3]*state.bias[0] + R[4]*state.bias[1] + R[5]*state.bias[2];

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
        filterVec(R, 9, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.motionBiasEstRLpState, R);
        filterVec(biasLp, 2, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.motionBiasEstBiasLpState, biasLp);

        vqf_real_t w[3];
        vqf_real_t e[3];
        if (state.restDetected && params.restBiasEstEnabled) {
            e[0] = state.restLastGyrLp[0] - state.bias[0];
            e[1] = state.restLastGyrLp[1] - state.bias[1];
            e[2] = state.restLastGyrLp[2] - state.bias[2];
            matrix3SetToScaledIdentity(1.0, R);
            w[0] = w[1] = w[2] = coeffs.biasRestW;
        } else if (params.motionBiasEstEnabled) {
            e[0] = -accEarth[1]/coeffs.accTs + biasLp[0] - R[0]*state.bias[0] - R[1]*state.bias[1] - R[2]*state.bias[2];
            e[1] = accEarth[0]/coeffs.accTs + biasLp[1] - R[3]*state.bias[0] - R[4]*state.bias[1] - R[5]*state.bias[2];
            e[2] = - R[6]*state.bias[0] - R[7]*state.bias[1] - R[8]*state.bias[2];
            w[0] = coeffs.biasMotionW;
            w[1] = coeffs.biasMotionW;
            w[2] = coeffs.biasVerticalW;
        } else {
            w[0] = -1;
        }

        if (w[0] >= 0) {
            state.biasP[0] += coeffs.biasV;
            state.biasP[4] += coeffs.biasV;
            state.biasP[8] += coeffs.biasV;

            clip(e, 3, -biasClip, biasClip);

            vqf_real_t K[9];
            matrix3MultiplyTpsSecond(state.biasP, R, K);
            matrix3Multiply(R, K, K);
            K[0] += w[0]; K[4] += w[1]; K[8] += w[2];
            if (matrix3Inv(K, K)) {
                matrix3MultiplyTpsFirst(R, K, K);
                matrix3Multiply(state.biasP, K, K);

                state.bias[0] += K[0]*e[0] + K[1]*e[1] + K[2]*e[2];
                state.bias[1] += K[3]*e[0] + K[4]*e[1] + K[5]*e[2];
                state.bias[2] += K[6]*e[0] + K[7]*e[1] + K[8]*e[2];

                vqf_real_t KR[9];
                matrix3Multiply(K, R, KR);
                matrix3Multiply(KR, state.biasP, KR);
                for(int i=0; i<9; i++) state.biasP[i] -= KR[i];

                clip(state.bias, 3, -biasClip, biasClip);
            }
        }
#endif
    }
}

void VQF::updateMag(const vqf_real_t mag[3])
{
}

void VQF::getQuat3D(vqf_real_t out[4]) const { std::copy(state.gyrQuat, state.gyrQuat+4, out); }
void VQF::getQuat6D(vqf_real_t out[4]) const { quatMultiply(state.accQuat, state.gyrQuat, out); }
void VQF::getQuat9D(vqf_real_t out[4]) const { getQuat6D(out); quatApplyDelta(out, state.delta, out); }

vqf_real_t VQF::getDelta() const { return state.delta; }

vqf_real_t VQF::getBiasEstimate(vqf_real_t out[3]) const
{
    if (out) std::copy(state.bias, state.bias+3, out);
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_real_t trace = state.biasP[0] + state.biasP[4] + state.biasP[8];
    return sqrtf(trace/3.0f)*vqf_real_t(M_PI/180.0f/100.0f);
#else
    return 0;
#endif
}

void VQF::setBiasEstimate(vqf_real_t bias[3], vqf_real_t sigma)
{
    std::copy(bias, bias+3, state.bias);
}

bool VQF::getRestDetected() const { return state.restDetected; }
bool VQF::getMagDistDetected() const { return state.magDistDetected; }
void VQF::getRelativeRestDeviations(vqf_real_t out[2]) const {}
vqf_real_t VQF::getMagRefNorm() const { return state.magRefNorm; }
vqf_real_t VQF::getMagRefDip() const { return state.magRefDip; }
void VQF::setMagRef(vqf_real_t norm, vqf_real_t dip) {}

void VQF::setTauAcc(vqf_real_t tauAcc) { params.tauAcc = tauAcc; setup(); }
void VQF::setTauMag(vqf_real_t tauMag) { params.tauMag = tauMag; setup(); }
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
void VQF::setMotionBiasEstEnabled(bool enabled) { params.motionBiasEstEnabled = enabled; }
#endif
void VQF::setRestBiasEstEnabled(bool enabled) { params.restBiasEstEnabled = enabled; }
void VQF::setMagDistRejectionEnabled(bool enabled) { params.magDistRejectionEnabled = enabled; }
void VQF::setRestDetectionThresholds(vqf_real_t thGyr, vqf_real_t thAcc) { params.restThGyr = thGyr; params.restThAcc = thAcc; }

void VQF::resetState()
{
    quatSetToIdentity(state.gyrQuat);
    quatSetToIdentity(state.accQuat);
    state.delta = 0.0;
    state.restDetected = false;
    state.magDistDetected = false;
    for(int i=0; i<3; i++) state.bias[i] = 0;
    for(int i=0; i<6; i++) state.accLpState[i] = NaN;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    matrix3SetToScaledIdentity(coeffs.biasP0, state.biasP);
    for(int i=0; i<18; i++) state.motionBiasEstRLpState[i] = NaN;
    for(int i=0; i<4; i++) state.motionBiasEstBiasLpState[i] = NaN;
#endif
    state.restT = 0;
    for(int i=0; i<6; i++) state.restGyrLpState[i] = NaN;
    for(int i=0; i<6; i++) state.restAccLpState[i] = NaN;
}

void VQF::setup()
{
    filterCoeffs(params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA);
    filterCoeffs(params.restFilterTau, coeffs.gyrTs, coeffs.restGyrLpB, coeffs.restGyrLpA);
    filterCoeffs(params.restFilterTau, coeffs.accTs, coeffs.restAccLpB, coeffs.restAccLpA);
    coeffs.biasP0 = square(params.biasSigmaInit*100.0);
    coeffs.biasV = square(0.1f*100.0f)*coeffs.accTs/params.biasForgettingTime;
    vqf_real_t pMotion = square(params.biasSigmaMotion*100.0);
    coeffs.biasMotionW = square(pMotion) / coeffs.biasV + pMotion;
    coeffs.biasVerticalW = coeffs.biasMotionW / std::max(params.biasVerticalForgettingFactor, 1e-10f);
    coeffs.biasRestW = square(square(params.biasSigmaRest*100.0)) / coeffs.biasV + square(params.biasSigmaRest*100.0);
    resetState();
}

void VQF::quatMultiply(const vqf_real_t q1[4], const vqf_real_t q2[4], vqf_real_t out[4])
{
    vqf_real_t w = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    vqf_real_t x = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    vqf_real_t y = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    vqf_real_t z = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

void VQF::quatConj(const vqf_real_t q[4], vqf_real_t out[4])
{
    out[0] = q[0]; out[1] = -q[1]; out[2] = -q[2]; out[3] = -q[3];
}

void VQF::quatSetToIdentity(vqf_real_t out[4])
{
    out[0] = 1; out[1] = 0; out[2] = 0; out[3] = 0;
}

void VQF::quatApplyDelta(vqf_real_t q[4], vqf_real_t delta, vqf_real_t out[4])
{
    vqf_real_t c = cosf(delta/2);
    vqf_real_t s = sinf(delta/2);
    vqf_real_t dq[4] = {c, 0, 0, s};
    quatMultiply(dq, q, out);
}

void VQF::quatRotate(const vqf_real_t q[4], const vqf_real_t v[3], vqf_real_t out[3])
{
    vqf_real_t qv[4] = {0, v[0], v[1], v[2]};
    vqf_real_t q_conj[4];
    quatConj(q, q_conj);
    vqf_real_t tmp[4];
    quatMultiply(q, qv, tmp);
    quatMultiply(tmp, q_conj, tmp);
    out[0] = tmp[1]; out[1] = tmp[2]; out[2] = tmp[3];
}

vqf_real_t VQF::norm(const vqf_real_t vec[], size_t N)
{
    vqf_real_t s = 0;
    for(size_t i=0; i<N; i++) s += vec[i]*vec[i];
    return sqrtf(s);
}

void VQF::normalize(vqf_real_t vec[], size_t N)
{
    vqf_real_t n = norm(vec, N);
    if (n > EPS) for(size_t i=0; i<N; i++) vec[i] /= n;
}

void VQF::clip(vqf_real_t vec[], size_t N, vqf_real_t min_val, vqf_real_t max_val)
{
    for(size_t i=0; i<N; i++) vec[i] = std::min(std::max(vec[i], min_val), max_val);
}

vqf_real_t VQF::gainFromTau(vqf_real_t tau, vqf_real_t Ts)
{
    if (tau <= 0) return 1.0f;
    return 1.0f - expf(-Ts/tau);
}

void VQF::filterCoeffs(vqf_real_t tau, vqf_real_t Ts, vqf_real_t outB[3], vqf_real_t outA[2])
{
    vqf_real_t fc = M_SQRT2 / (2.0f * M_PI * tau);
    vqf_real_t C = tanf(M_PI * fc * Ts);
    vqf_real_t D = C*C + sqrtf(2.0f)*C + 1.0f;
    outB[0] = C*C/D;
    outB[1] = 2.0f*outB[0];
    outB[2] = outB[0];
    outA[0] = 2.0f*(C*C-1.0f)/D;
    outA[1] = (1.0f-sqrtf(2.0f)*C+C*C)/D;
}

void VQF::filterInitialState(vqf_real_t x0, const vqf_real_t b[], const vqf_real_t a[], vqf_real_t out[2])
{
    out[0] = x0*(1-b[0]);
    out[1] = x0*(b[2]-a[1]);
}

void VQF::filterAdaptStateForCoeffChange(vqf_real_t last_y[], size_t N, const vqf_real_t b_old[3], const vqf_real_t a_old[2], const vqf_real_t b_new[3], const vqf_real_t a_new[2], vqf_real_t state[])
{
}

vqf_real_t VQF::filterStep(vqf_real_t x, const vqf_real_t b[3], const vqf_real_t a[2], vqf_real_t state[2])
{
    vqf_real_t y = b[0]*x + state[0];
    state[0] = b[1]*x - a[0]*y + state[1];
    state[1] = b[2]*x - a[1]*y;
    return y;
}

void VQF::filterVec(const vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts, const vqf_real_t b[3], const vqf_real_t a[2], vqf_real_t state[], vqf_real_t out[])
{
    if (isnan(state[0])) {
        for(size_t i=0; i<N; i++) {
            filterInitialState(x[i], b, a, &state[2*i]);
            out[i] = x[i];
        }
    } else {
        for(size_t i=0; i<N; i++) out[i] = filterStep(x[i], b, a, &state[2*i]);
    }
}

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
void VQF::matrix3SetToScaledIdentity(vqf_real_t scale, vqf_real_t out[9])
{
    for(int i=0; i<9; i++) out[i] = 0;
    out[0] = out[4] = out[8] = scale;
}

void VQF::matrix3Multiply(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9])
{
    vqf_real_t res[9];
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            res[3*i+j] = in1[3*i+0]*in2[3*0+j] + in1[3*i+1]*in2[3*1+j] + in1[3*i+2]*in2[3*2+j];
        }
    }
    std::copy(res, res+9, out);
}

void VQF::matrix3MultiplyTpsFirst(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9])
{
    vqf_real_t res[9];
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            res[3*i+j] = in1[3*0+i]*in2[3*0+j] + in1[3*1+i]*in2[3*1+j] + in1[3*2+i]*in2[3*2+j];
        }
    }
    std::copy(res, res+9, out);
}

void VQF::matrix3MultiplyTpsSecond(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9])
{
    vqf_real_t res[9];
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            res[3*i+j] = in1[3*i+0]*in2[3*0+j] + in1[3*i+1]*in2[3*j+1] + in1[3*i+2]*in2[3*j+2];
        }
    }
    std::copy(res, res+9, out);
}

bool VQF::matrix3Inv(const vqf_real_t in[9], vqf_real_t out[9])
{
    vqf_real_t det = in[0]*(in[4]*in[8]-in[5]*in[7]) - in[1]*(in[3]*in[8]-in[5]*in[6]) + in[2]*(in[3]*in[7]-in[4]*in[6]);
    if (fabsf(det) < 1e-10f) return false;
    vqf_real_t invDet = 1.0f/det;
    out[0] = (in[4]*in[8]-in[5]*in[7])*invDet;
    out[1] = (in[2]*in[7]-in[1]*in[8])*invDet;
    out[2] = (in[1]*in[5]-in[2]*in[4])*invDet;
    out[3] = (in[5]*in[6]-in[3]*in[8])*invDet;
    out[4] = (in[0]*in[8]-in[2]*in[6])*invDet;
    out[5] = (in[2]*in[3]-in[0]*in[5])*invDet;
    out[6] = (in[3]*in[7]-in[4]*in[6])*invDet;
    out[7] = (in[1]*in[6]-in[0]*in[7])*invDet;
    out[8] = (in[0]*in[4]-in[1]*in[3])*invDet;
    return true;
}
#endif
