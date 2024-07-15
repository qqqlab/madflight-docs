# AHRS
(Attitude and Heading Reference System)

The following gyro/accelerometer/magnetometer sensor fusion filters are implemented in madflight:

### Mahony

The filter has two parameters: KP (proportional filter gain) and KI (integral filter gain) which form a PI controller. Orientation calculated from integrating gyro measurements, and is adjusted with the acc/mag measurements by the PI controller. The KI parameter acts as a gyro bias estimator.

The Mahony filter provides accurate and stable estimates even under vibration and magnetic disturbances. No wonder that it is used in many flight controller firmwares.

`#define AHRS_USE AHRS_USE_MAHONY` or `#define AHRS_USE AHRS_USE_MAHONY_BF`

madflight offers the standard filter implementation AHRS_USE_MAHONY plus a Betaflight (AHRS_USE_MAHONY_BF) flavored filter version, which only uses accelerometer data in rest.

### Madgwick

This filter is controllerd by a single parameter: beta, the algorithm gain.

`#define AHRS_USE AHRS_USE_MADGWICK`

### VQF

The new kid on the block, see [here](https://vqf.readthedocs.io). This is a more complex filter, with more parameters and longer computation times, but should give more accurate estimations.

`#define AHRS_USE AHRS_USE_VQF`

Implementation in madflight is experimental. 

Runtime is 600 us on ESP32 for full gyro/acc/mag update. AHRS_USE_MAHONY runtime is 120 us, and AHRS_USE_MADGWICK 160 us.


## Overview of AHRS used in Flight Controllers

### Betaflight

Mahony Filter

 - separate mag heading, acc, cog error updates 
 - mag weight increases with mag heading error
 - acc update only if acc in range 0.9g - 1.1g
 - Kp = 0.25 when armed, 2.5 first 20 sec, 25 after crash disarm
 - optional integration term

### INAV

Mahony Filter

 - separate mag heading, acc, cog error updates 
 - centrifugal force compensation
 - gyro bias estimate
 - optional integration term

### PX4-Autopilot

EKF (Extended Kalman Filter) and/or

Mahony Filter

 - separate mag heading, acc, Vision heading, and Mocap heading error updates 
 - mag weight increases up to 10x with spinRate
 - acc update only if acc in range 0.9g - 1.1g
 - gyro bias estimated when spinRate < 0.175 (10dps)
 - no integration term

### ArduPilot

EKF (Extended Kalman Filter) or DCM (Direction Cosine Matrix)


## Betaflight Source Code

https://github.com/betaflight/betaflight/blob/master/src/main/flight/imu.c
```
/* GLOBAL VARS

q - Quaternion
rMat - Rotation matrix
*/


// g[xyz] - low pass filtered gyro reading, in rad/s
// a[xyz] - low pass filtered accelerometer reading, direction only, normalized internally
// headingErrMag - heading error (in earth frame) derived from magnetometter, rad/s around Z axis (* dcmKpGain)
// headingErrCog - heading error (in earth frame) derived from CourseOverGround, rad/s around Z axis (* dcmKpGain)
// dcmKpGain - gain applied to all error sources
void imuMahonyAHRSupdate(float dt,
                                float gx, float gy, float gz,
                                float ax, float ay, float az,
                                float headingErrMag, float headingErrCog,
                                const float dcmKpGain)
{
    // integral error terms scaled by Ki
    static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;    

    float ex = 0, ey = 0, ez = 0;

    // Add error from magnetometer and Cog, just rotate input value to body frame
    ex += rMat[Z][X] * (headingErrCog + headingErrMag);
    ey += rMat[Z][Y] * (headingErrCog + headingErrMag);
    ez += rMat[Z][Z] * (headingErrCog + headingErrMag);

    // Use measured acceleration vector, Accept accel readings only in range 0.9g - 1.1g
    float recipAccNorm = sq(ax) + sq(ay) + sq(az);
    if ((0.81f < recipAccNorm) && (recipAccNorm < 1.21f)) {
        recipAccNorm = invSqrt(recipAccNorm);
        ax *= recipAccNorm;
        ay *= recipAccNorm;
        az *= recipAccNorm;

        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex += (ay * rMat[2][2] - az * rMat[2][1]);
        ey += (az * rMat[2][0] - ax * rMat[2][2]);
        ez += (ax * rMat[2][1] - ay * rMat[2][0]);
    }

    // Compute and apply integral feedback if enabled
    if (imuRuntimeConfig.imuDcmKi > 0.0f) {
        // Calculate general spin rate (rad/s)
        const float spin_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));
        // Stop integrating if spinning beyond the certain limit
        if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
            const float dcmKiGain = imuRuntimeConfig.imuDcmKi;
            integralFBx += dcmKiGain * ex * dt;    // integral error scaled by Ki
            integralFBy += dcmKiGain * ey * dt;
            integralFBz += dcmKiGain * ez * dt;
        }
    } else {
        integralFBx = 0.0f;    // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // Apply proportional and integral feedback
    gx += dcmKpGain * ex + integralFBx;
    gy += dcmKpGain * ey + integralFBy;
    gz += dcmKpGain * ez + integralFBz;

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    quaternion buffer;
    buffer.w = q.w;
    buffer.x = q.x;
    buffer.y = q.y;
    buffer.z = q.z;

    q.w += (-buffer.x * gx - buffer.y * gy - buffer.z * gz);
    q.x += (+buffer.w * gx + buffer.y * gz - buffer.z * gy);
    q.y += (+buffer.w * gy - buffer.x * gz + buffer.z * gx);
    q.z += (+buffer.w * gz + buffer.x * gy - buffer.y * gx);

    // Normalise quaternion
    float recipNorm = invSqrt(sq(q.w) + sq(q.x) + sq(q.y) + sq(q.z));
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;

    // Pre-compute rotation matrix from quaternion, used by imuCalcMagErr()
    rMat = imuComputeRotationMatrix(q);
}

...

// Calculate heading error derived from magnetometer
// return value rotation around earth Z axis, pointing in direction of smaller error, [rad/s]
float imuCalcMagErr(void)
{
    // Use measured magnetic field vector
    fpVector3_t mag_bf = {{mag.magADC[X], mag.magADC[Y], mag.magADC[Z]}};
    float magNormSquared = vectorNormSquared(&mag_bf);

    if (magNormSquared > 0.01f) {
        // project magnetometer reading into Earth frame
        fpVector3_t mag_ef;
        matrixVectorMul(&mag_ef, (const fpMat33_t*)&rMat, &mag_bf); // BF->EF true north
        // Normalise magnetometer measurement
        vectorScale(&mag_ef, &mag_ef, 1.0f / sqrtf(magNormSquared));

        // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
        // This way magnetic field will only affect heading and wont mess roll/pitch angles
        fpVector2_t mag2d_ef = {.x = mag_ef.x, .y = mag_ef.y};
        // mag2d_ef - measured mag field vector in EF (2D ground plane projection)
        // north_ef - reference mag field vector heading due North in EF (2D ground plane projection).
        //              Adjusted for magnetic declination (in imuConfigure)

        // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
        // increase gain on large misalignment
        const float dot = vector2Dot(&mag2d_ef, &north_ef);
        const float cross = vector2Cross(&mag2d_ef, &north_ef);
        return (dot > 0) ? cross : (cross < 0 ? -1.0f : 1.0f) * vector2Norm(&mag2d_ef);
    } else {
        // invalid magnetometer data
        return 0.0f;
    }
}

...

    // *** GoC based error estimate ***
    float cogErr = 0;
#if defined(USE_GPS)
    if (!useMag
        && sensors(SENSOR_GPS)
        && STATE(GPS_FIX) && gpsSol.numSat > GPS_MIN_SAT_COUNT) {
        static bool gpsHeadingInitialized = false;  // TODO - remove
        if (gpsHeadingInitialized) {
            float groundspeedGain;  // IMU yaw gain to be applied in imuMahonyAHRSupdate from ground course,
            if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
                // GPS_Rescue adjusts groundspeedGain during a rescue in a range 0 - 4.5,
                //   depending on GPS Rescue state and groundspeed relative to speed to home.
                groundspeedGain = gpsRescueGetImuYawCogGain();
            } else {
                // 0.0 - 10.0, heuristic based on GPS speed and stick state
                groundspeedGain = imuCalcGroundspeedGain(dt);
            }
            DEBUG_SET(DEBUG_ATTITUDE, 2, lrintf(groundspeedGain * 100.0f));
            float courseOverGround = DECIDEGREES_TO_RADIANS(gpsSol.groundCourse);
            cogErr = imuCalcCourseErr(courseOverGround) * groundspeedGain;
        } else if (gpsSol.groundSpeed > GPS_COG_MIN_GROUNDSPEED) {
            // Reset the reference and reinitialize quaternion factors when GPS groundspeed > GPS_COG_MIN_GROUNDSPEED
            imuComputeQuaternionFromRPY(&qP, attitude.values.roll, attitude.values.pitch, gpsSol.groundCourse);
            gpsHeadingInitialized = true;
        }
    }
#endif
```

## INAV Source Code

https://github.com/iNavFlight/inav/blob/master/src/main/flight/imu.c

```
static void imuMahonyAHRSupdate(float dt, const fpVector3_t * gyroBF, const fpVector3_t * accBF, const fpVector3_t * magBF, bool useCOG, float courseOverGround, float accWScaler, float magWScaler)
{
    STATIC_FASTRAM fpVector3_t vGyroDriftEstimate = { 0 };

    fpQuaternion_t prevOrientation = orientation;
    fpVector3_t vRotation = *gyroBF;

    /* Calculate general spin rate (rad/s) */
    const float spin_rate_sq = vectorNormSquared(&vRotation);

    /* Step 1: Yaw correction */
    // Use measured magnetic field vector
    if (magBF || useCOG) {
        float wMag = 1.0f;
        float wCoG = 1.0f;
        if(magBF){wCoG *= imuConfig()->gps_yaw_weight / 100.0f;}

        fpVector3_t vMagErr = { .v = { 0.0f, 0.0f, 0.0f } };
        fpVector3_t vCoGErr = { .v = { 0.0f, 0.0f, 0.0f } };

        if (magBF && vectorNormSquared(magBF) > 0.01f) {
            wMag *= bellCurve((fast_fsqrtf(vectorNormSquared(magBF)) - 1024.0f) / 1024.0f, MAX_MAG_NEARNESS);
            fpVector3_t vMag;

            // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
            // This way magnetic field will only affect heading and wont mess roll/pitch angles

            // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
            // This should yield direction to magnetic North (1; 0; 0)
            quaternionRotateVectorInv(&vMag, magBF, &orientation);    // BF -> EF

            // Ignore magnetic inclination
            vMag.z = 0.0f;

            // We zeroed out vMag.z -  make sure the whole vector didn't go to zero
            if (vectorNormSquared(&vMag) > 0.01f) {
                // Normalize to unit vector
                vectorNormalize(&vMag, &vMag);

#ifdef USE_SIMULATOR
            if (ARMING_FLAG(SIMULATOR_MODE_HITL) || ARMING_FLAG(SIMULATOR_MODE_SITL)) {
                imuSetMagneticDeclination(0);
            }
#endif

                // Reference mag field vector heading is Magnetic North in EF. We compute that by rotating True North vector by declination and assuming Z-component is zero
                // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
                vectorCrossProduct(&vMagErr, &vMag, &vCorrectedMagNorth);

                // Rotate error back into body frame
                quaternionRotateVector(&vMagErr, &vMagErr, &orientation);
            }
        }
        if (useCOG) {
            fpVector3_t vForward = { .v = { 0.0f, 0.0f, 0.0f } };
            //vForward as trust vector
            if (STATE(MULTIROTOR) && (!isMixerTransitionMixing)){
                vForward.z = 1.0f;
            }else{
                vForward.x = 1.0f;
            }
            fpVector3_t vHeadingEF;

            // Use raw heading error (from GPS or whatever else)
            while (courseOverGround >  M_PIf) courseOverGround -= (2.0f * M_PIf);
            while (courseOverGround < -M_PIf) courseOverGround += (2.0f * M_PIf);

            // William Premerlani and Paul Bizard, Direction Cosine Matrix IMU - Eqn. 22-23
            // (Rxx; Ryx) - measured (estimated) heading vector (EF)
            // (-cos(COG), sin(COG)) - reference heading vector (EF)

            float airSpeed = gpsSol.groundSpeed;
            // Compute heading vector in EF from scalar CoG,x axis of accelerometer is pointing backwards.
            fpVector3_t vCoG = { .v = { -cos_approx(courseOverGround), sin_approx(courseOverGround), 0.0f } };
#if defined(USE_WIND_ESTIMATOR)
            // remove wind elements in vCoG for better heading estimation
            if (isEstimatedWindSpeedValid() && imuConfig()->gps_yaw_windcomp)
            {
                vectorScale(&vCoG, &vCoG, gpsSol.groundSpeed);
                vCoG.x += getEstimatedWindSpeed(X);
                vCoG.y -= getEstimatedWindSpeed(Y);
                airSpeed = fast_fsqrtf(vectorNormSquared(&vCoG));
                vectorNormalize(&vCoG, &vCoG);
            }
#endif
            wCoG *= scaleRangef(constrainf((airSpeed+gpsSol.groundSpeed)/2, 400, 1000), 400, 1000, 0.0f, 1.0f);
            // Rotate Forward vector from BF to EF - will yield Heading vector in Earth frame
            quaternionRotateVectorInv(&vHeadingEF, &vForward, &orientation);

            if (STATE(MULTIROTOR)){
                //when multicopter`s orientation or speed is changing rapidly. less weight on gps heading
                wCoG *= imuCalculateMcCogWeight();
                //scale accroading to multirotor`s tilt angle
                wCoG *= scaleRangef(constrainf(vHeadingEF.z, COS20DEG, COS10DEG), COS20DEG, COS10DEG, 1.0f, 0.0f);
                //for inverted flying, wCoG is lowered by imuCalculateMcCogWeight no additional processing needed
            }
            vHeadingEF.z = 0.0f;

            // We zeroed out vHeadingEF.z -  make sure the whole vector didn't go to zero
            if (vectorNormSquared(&vHeadingEF) > 0.01f) {
                // Normalize to unit vector
                vectorNormalize(&vHeadingEF, &vHeadingEF);

                // error is cross product between reference heading and estimated heading (calculated in EF)
                vectorCrossProduct(&vCoGErr, &vCoG, &vHeadingEF);

                // Rotate error back into body frame
                quaternionRotateVector(&vCoGErr, &vCoGErr, &orientation);
            }
        }
        fpVector3_t vErr = { .v = { 0.0f, 0.0f, 0.0f } };
        vectorScale(&vMagErr, &vMagErr, wMag);
        vectorScale(&vCoGErr, &vCoGErr, wCoG);
        vectorAdd(&vErr, &vMagErr, &vCoGErr);
        // Compute and apply integral feedback if enabled
        if (imuRuntimeConfig.dcm_ki_mag > 0.0f) {
            // Stop integrating if spinning beyond the certain limit
            if (spin_rate_sq < sq(DEGREES_TO_RADIANS(SPIN_RATE_LIMIT))) {
                fpVector3_t vTmp;

                // integral error scaled by Ki
                vectorScale(&vTmp, &vErr, imuRuntimeConfig.dcm_ki_mag * magWScaler * dt);
                vectorAdd(&vGyroDriftEstimate, &vGyroDriftEstimate, &vTmp);
            }
        }

        // Calculate kP gain and apply proportional feedback
        vectorScale(&vErr, &vErr, imuRuntimeConfig.dcm_kp_mag * magWScaler);
        vectorAdd(&vRotation, &vRotation, &vErr);
    }


    /* Step 2: Roll and pitch correction -  use measured acceleration vector */
    if (accBF) {
        static const fpVector3_t vGravity = { .v = { 0.0f, 0.0f, 1.0f } };
        fpVector3_t vEstGravity, vAcc, vErr;

        // Calculate estimated gravity vector in body frame
        quaternionRotateVector(&vEstGravity, &vGravity, &orientation);    // EF -> BF

        // Error is sum of cross product between estimated direction and measured direction of gravity
        vectorNormalize(&vAcc, accBF);
        vectorCrossProduct(&vErr, &vAcc, &vEstGravity);

        // Compute and apply integral feedback if enabled
        if (imuRuntimeConfig.dcm_ki_acc > 0.0f) {
            // Stop integrating if spinning beyond the certain limit
            if (spin_rate_sq < sq(DEGREES_TO_RADIANS(SPIN_RATE_LIMIT))) {
                fpVector3_t vTmp;

                // integral error scaled by Ki
                vectorScale(&vTmp, &vErr, imuRuntimeConfig.dcm_ki_acc * accWScaler * dt);
                vectorAdd(&vGyroDriftEstimate, &vGyroDriftEstimate, &vTmp);
            }
        }

        // Calculate kP gain and apply proportional feedback
        vectorScale(&vErr, &vErr, imuRuntimeConfig.dcm_kp_acc * accWScaler);
        vectorAdd(&vRotation, &vRotation, &vErr);
    }
    // Anti wind-up
    float i_limit = DEGREES_TO_RADIANS(2.0f) * (imuRuntimeConfig.dcm_kp_acc + imuRuntimeConfig.dcm_kp_mag) / 2.0f;
    vGyroDriftEstimate.x = constrainf(vGyroDriftEstimate.x, -i_limit, i_limit);
    vGyroDriftEstimate.y = constrainf(vGyroDriftEstimate.y, -i_limit, i_limit);
    vGyroDriftEstimate.z = constrainf(vGyroDriftEstimate.z, -i_limit, i_limit);

    // Apply gyro drift correction
    vectorAdd(&vRotation, &vRotation, &vGyroDriftEstimate);

    // Integrate rate of change of quaternion
    fpVector3_t vTheta;
    fpQuaternion_t deltaQ;

    vectorScale(&vTheta, &vRotation, 0.5f * dt);
    quaternionInitFromVector(&deltaQ, &vTheta);
    const float thetaMagnitudeSq = vectorNormSquared(&vTheta);

    // If calculated rotation is zero - don't update quaternion
    if (thetaMagnitudeSq >= 1e-20f) {
        // Calculate quaternion delta:
        // Theta is a axis/angle rotation. Direction of a vector is axis, magnitude is angle/2.
        // Proper quaternion from axis/angle involves computing sin/cos, but the formula becomes numerically unstable as Theta approaches zero.
        // For near-zero cases we use the first 3 terms of the Taylor series expansion for sin/cos. We check if fourth term is less than machine precision -
        // then we can safely use the "low angle" approximated version without loss of accuracy.
        if (thetaMagnitudeSq < fast_fsqrtf(24.0f * 1e-6f)) {
            quaternionScale(&deltaQ, &deltaQ, 1.0f - thetaMagnitudeSq / 6.0f);
            deltaQ.q0 = 1.0f - thetaMagnitudeSq / 2.0f;
        }
        else {
            const float thetaMagnitude = fast_fsqrtf(thetaMagnitudeSq);
            quaternionScale(&deltaQ, &deltaQ, sin_approx(thetaMagnitude) / thetaMagnitude);
            deltaQ.q0 = cos_approx(thetaMagnitude);
        }

        // Calculate final orientation and renormalize
        quaternionMultiply(&orientation, &orientation, &deltaQ);
        quaternionNormalize(&orientation, &orientation);
    }

    // Check for invalid quaternion and reset to previous known good one
    imuCheckAndResetOrientationQuaternion(&prevOrientation, accBF);

    // Pre-compute rotation matrix from quaternion
    imuComputeRotationMatrix();
}


...


static void imuCalculateEstimatedAttitude(float dT)
{
#if defined(USE_MAG)
    const bool canUseMAG = sensors(SENSOR_MAG) && compassIsHealthy();
#else
    const bool canUseMAG = false;
#endif

    float courseOverGround = 0;
    bool useMag = false;
    bool useCOG = false;
#if defined(USE_GPS)
    bool canUseCOG = isGPSHeadingValid();

    // Use compass (if available)
    if (canUseMAG) {
        useMag = true;
        gpsHeadingInitialized = true;   // GPS heading initialised from MAG, continue on GPS if compass fails
    }
    // Use GPS (if available)
    if (canUseCOG) {
        if (gpsHeadingInitialized) {
            courseOverGround = DECIDEGREES_TO_RADIANS(gpsSol.groundCourse);
            useCOG = true;
        }
        else if (!canUseMAG) {
            // Re-initialize quaternion from known Roll, Pitch and GPS heading
            imuComputeQuaternionFromRPY(attitude.values.roll, attitude.values.pitch, gpsSol.groundCourse);
            gpsHeadingInitialized = true;

            // Force reset of heading hold target
            resetHeadingHoldTarget(DECIDEGREES_TO_DEGREES(attitude.values.yaw));
        }
    } else if (!ARMING_FLAG(ARMED)) {
        gpsHeadingInitialized = false;
    }

    imuCalculateFilters(dT);
    // centrifugal force compensation
    static fpVector3_t vEstcentrifugalAccelBF_velned;
    static fpVector3_t vEstcentrifugalAccelBF_turnrate;
    float acc_ignore_slope_multipiler = 1.0f; // when using gps centrifugal_force_compensation, AccelerometerWeightRateIgnore slope will be multiplied by this value
    if (isGPSTrustworthy())
    {
        imuCalculateGPSacceleration(&vEstcentrifugalAccelBF_velned, &acc_ignore_slope_multipiler);
    }
    if (STATE(AIRPLANE))
    {
        imuCalculateTurnRateacceleration(&vEstcentrifugalAccelBF_turnrate, dT, &acc_ignore_slope_multipiler);
    }
    if (imuConfig()->inertia_comp_method == COMPMETHOD_ADAPTIVE && isGPSTrustworthy() && STATE(AIRPLANE))
    {
        //pick the best centrifugal acceleration between velned and turnrate
        fpVector3_t compansatedGravityBF_velned;
        vectorAdd(&compansatedGravityBF_velned, &imuMeasuredAccelBF, &vEstcentrifugalAccelBF_velned);
        float velned_error = fabsf(fast_fsqrtf(vectorNormSquared(&compansatedGravityBF_velned)) - GRAVITY_CMSS);

        fpVector3_t compansatedGravityBF_turnrate;
        vectorAdd(&compansatedGravityBF_turnrate, &imuMeasuredAccelBF, &vEstcentrifugalAccelBF_turnrate);
        float turnrate_error = fabsf(fast_fsqrtf(vectorNormSquared(&compansatedGravityBF_turnrate)) - GRAVITY_CMSS);

        compansatedGravityBF = velned_error > turnrate_error? compansatedGravityBF_turnrate:compansatedGravityBF_velned;
    }
    else if (((imuConfig()->inertia_comp_method == COMPMETHOD_VELNED) || (imuConfig()->inertia_comp_method == COMPMETHOD_ADAPTIVE)) && isGPSTrustworthy())
    {
        //velned centrifugal force compensation, quad will use this method
        vectorAdd(&compansatedGravityBF, &imuMeasuredAccelBF, &vEstcentrifugalAccelBF_velned);
    }
    else if (STATE(AIRPLANE))
    {
        //turnrate centrifugal force compensation
        vectorAdd(&compansatedGravityBF, &imuMeasuredAccelBF, &vEstcentrifugalAccelBF_turnrate);
    }
    else
    {
        compansatedGravityBF = imuMeasuredAccelBF;
    }
#else
    // In absence of GPS MAG is the only option
    if (canUseMAG) {
        useMag = true;
    }
    compansatedGravityBF = imuMeasuredAccelBF
#endif
    float accWeight = imuGetPGainScaleFactor() * imuCalculateAccelerometerWeightNearness(&compansatedGravityBF);
    accWeight = accWeight * imuCalculateAccelerometerWeightRateIgnore(acc_ignore_slope_multipiler);
    const bool useAcc = (accWeight > 0.001f);

    const float magWeight = imuGetPGainScaleFactor() * 1.0f;
    fpVector3_t measuredMagBF = {.v = {mag.magADC[X], mag.magADC[Y], mag.magADC[Z]}};
    imuMahonyAHRSupdate(dT, &imuMeasuredRotationBF,
                            useAcc ? &compansatedGravityBF : NULL,
                            useMag ? &measuredMagBF : NULL,
                            useCOG, courseOverGround,
                            accWeight,
                            magWeight);
    imuUpdateTailSitter();
    imuUpdateEulerAngles();
}

```

# PX4-Autopilot Source Code

https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/attitude_estimator_q/attitude_estimator_q_main.cpp


```
/* PARAMETERS
 weights: w_acc=0.2, w_mag=0.1, w_ext_hdg=0.1, w_gyro_bias=0.1
 max gyro bias: _bias_max=0.05 (3dps)
*/

bool AttitudeEstimatorQ::update(float dt)
{
    Quatf q_last = _q;

    // Angular rate of correction
    Vector3f corr;
    float spinRate = _gyro.length();

    if (_param_att_ext_hdg_m.get() > 0 && _ext_hdg_good) {
        if (_param_att_ext_hdg_m.get() == 1) {
            // Vision heading correction
            // Project heading to global frame and extract XY component
            Vector3f vision_hdg_earth = _q.rotateVector(_vision_hdg);
            float vision_hdg_err = wrap_pi(atan2f(vision_hdg_earth(1), vision_hdg_earth(0)));
            // Project correction to body frame
            corr += _q.rotateVectorInverse(Vector3f(0.0f, 0.0f, -vision_hdg_err)) * w_ext;
        }

        if (_param_att_ext_hdg_m.get() == 2) {
            // Mocap heading correction
            // Project heading to global frame and extract XY component
            Vector3f mocap_hdg_earth = _q.rotateVector(_mocap_hdg);
            float mocap_hdg_err = wrap_pi(atan2f(mocap_hdg_earth(1), mocap_hdg_earth(0)));
            // Project correction to body frame
            corr += _q.rotateVectorInverse(Vector3f(0.0f, 0.0f, -mocap_hdg_err)) * w_ext_hdg;
        }
    }

    if (_param_att_ext_hdg_m.get() == 0 || !_ext_hdg_good) {
        // Magnetometer correction
        // Project mag field vector to global frame and extract XY component
        Vector3f mag_earth = _q.rotateVector(_mag);
        float mag_err = wrap_pi(atan2f(mag_earth(1), mag_earth(0)) - _mag_decl);
        float gainMult = 1.0f;
        const float fifty_dps = 0.873f;

        if (spinRate > fifty_dps) {
            gainMult = math::min(spinRate / fifty_dps, 10.0f);
        }

        // Project magnetometer correction to body frame
        corr += _q.rotateVectorInverse(Vector3f(0.0f, 0.0f, -mag_err)) * w_mag * gainMult;
    }

    _q.normalize();


    // Accelerometer correction
    // Project 'k' unit vector of earth frame to body frame
    // Vector3f k = _q.rotateVectorInverse(Vector3f(0.0f, 0.0f, 1.0f));
    // Optimized version with dropped zeros
    Vector3f k(
        2.0f * (_q(1) * _q(3) - _q(0) * _q(2)),
        2.0f * (_q(2) * _q(3) + _q(0) * _q(1)),
        (_q(0) * _q(0) - _q(1) * _q(1) - _q(2) * _q(2) + _q(3) * _q(3))
    );

    // If we are not using acceleration compensation based on GPS velocity,
    // fuse accel data only if its norm is close to 1 g (reduces drift).
    const float accel_norm_sq = _accel.norm_squared();
    if (accel_norm_sq > 0.9 && accel_norm_sq < 1.1) {
        corr += (k % (_accel - _pos_acc).normalized()) * w_acc;
    }

    // Gyro bias estimation
    if (spinRate < 0.175f) {
        _gyro_bias += corr * (w_gyro_bias * dt);

        for (int i = 0; i < 3; i++) {
            _gyro_bias(i) = math::constrain(_gyro_bias(i), -_bias_max, _bias_max);
        }

    }

    // Calculate unbiased gyro rates
    _rates = _gyro + _gyro_bias;

    // Feed forward gyro
    corr += _rates;

    // Apply correction to state
    _q += _q.derivative1(corr) * dt;

    // Normalize quaternion
    _q.normalize();

    return true;
}
```
