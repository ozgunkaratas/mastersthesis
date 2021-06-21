import numpy as np
import quatMath

'''
code is similar to the link https://github.com/ccny-ros-pkg/imu_tools/tree/indigo/imu_complementary_filter
with minor adjustments
'''
def getMeasurement(ax, ay, az,  q0_meas, q1_meas, q2_meas, q3_meas):
    
    #Normalize acceleration vector.
    ax,ay,az = quatMath.normalizeVector(ax, ay, az)

    if (az >= 0):
    
        q0_meas = np.sqrt((az + 1) * 0.5);
        q1_meas = -ay / (2.0 * q0_meas);
        q2_meas = ax / (2.0 * q0_meas);
        q3_meas = 0;
    
    else:
    
        X = np.sqrt((1 - az) * 0.5);
        q0_meas = -ay / (2.0 * X);
        q1_meas = X;
        q2_meas = 0;
        q3_meas = ax / (2.0 * X);
    #this must return an initial quaternion config so that we can continue calculating        
    return (q0_meas,q1_meas,q2_meas,q3_meas)

    
def updateBiases(ax,ay,az,wx,wy,wz,kAccelerationThreshold,kAngularVelocityThreshold,
                 kDeltaAngularVelocityThreshold, kGravity, wx_bias_, wy_bias_, wz_bias_,
                 wx_prev_, wy_prev_, wz_prev_, bias_alpha_):
    acc_magnitude = np.sqrt(ax * ax + ay * ay + az * az)
    if (np.abs(acc_magnitude - kGravity) > kAccelerationThreshold):
        steady_state_ = False

    if (np.abs(wx - wx_prev_) > kDeltaAngularVelocityThreshold
        or np.abs(wy - wy_prev_) > kDeltaAngularVelocityThreshold
        or np.abs(wz - wz_prev_) > kDeltaAngularVelocityThreshold):
        steady_state_ = False

    if (np.abs(wx - wx_bias_) > kAngularVelocityThreshold or
        np.abs(wy - wy_bias_) > kAngularVelocityThreshold or
        np.abs(wz - wz_bias_) > kAngularVelocityThreshold):
        steady_state_ = False
    else:
        steady_state_ = True
    # #get bool comparison
    # steady_state_ = checkState(ax,ay,az,wx,wy,wz,
    #                            wx_bias_, wy_bias_, wz_bias_,
    #                            wx_prev_, wy_prev_, wz_prev_)
    
    if (steady_state_):
    
        wx_bias_ += bias_alpha_ * (wx - wx_bias_);
        wy_bias_ += bias_alpha_ * (wy - wy_bias_);
        wz_bias_ += bias_alpha_ * (wz - wz_bias_);
    

    wx_prev_ = wx;
    wy_prev_ = wy;
    wz_prev_ = wz;
    return (wx_bias_, wy_bias_, wz_bias_, wx_prev_, wy_prev_, wz_prev_)

def Prediction(wx, wy, wz, timeStep, q0_pred, q1_pred, q2_pred, q3_pred,
               q0_, q1_, q2_, q3_,
               wx_bias_, wy_bias_, wz_bias_):
    #get new biases
    wx_unb = wx - wx_bias_;
    wy_unb = wy - wy_bias_;
    wz_unb = wz - wz_bias_;
    #get new pred values
    q0_pred = q0_ + 0.5 * timeStep * (wx_unb * q1_ + wy_unb * q2_ + wz_unb * q3_);
    q1_pred = q1_ + 0.5 * timeStep * (-wx_unb * q0_ - wy_unb * q3_ + wz_unb * q2_);
    q2_pred = q2_ + 0.5 * timeStep * (wx_unb * q3_ - wy_unb * q0_ - wz_unb * q1_);
    q3_pred = q3_ + 0.5 * timeStep * (-wx_unb * q2_ + wy_unb * q1_ - wz_unb * q0_);
    
    q0_pred, q1_pred, q2_pred, q3_pred = quatMath.normalizeQuaternion(q0_pred, q1_pred, q2_pred, q3_pred)
    return(q0_pred, q1_pred, q2_pred, q3_pred)

def AccelComp(ax, ay, az, p0, p1, p2, p3, dq0, dq1, dq2, dq3):
    
    ax,ay,az = quatMath.normalizeVector(ax, ay, az)
    #init gravity vectors 
    gx = 0
    gy = 0
    gz = 0
    
    gx,gy,gz = quatMath.rotateVectorByQuaternion(ax, ay, az, p0, -p1, -p2, -p3,gx, gy, gz)
    #Delta quaternion that rotates the predicted gravity into the real gravity:
    dq0 = np.sqrt((gz + 1) * 0.5)
    dq1 = -gy / (2.0 * dq0)
    dq2 = gx / (2.0 * dq0)
    dq3 = 0.0
    
    return (dq0,dq1,dq2,dq3,gx,gy,gz)

    pass

def storeOrientation(q0, q1, q2, q3,q0_, q1_, q2_, q3_):
     q0_inv, q1_inv, q2_inv, q3_inv = quatMath.invertQuaternion(q0_, q1_, q2_, q3_, q0, q1, q2, q3)
     

