import numpy as np
'''
quaternion arithmetic functions
'''
def normalizeQuaternion(q0, q1, q2, q3):
    
    norm = np.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
    
    return(q0, q1, q2, q3)

def scaleQuaternion(gain, dq0, dq1, dq2, dq3):
    if (dq0 < 0.0):
    
       # // Slerp (Spherical linear interpolation):
        angle = np.arccos(dq0)
        A = np.sin(angle * (1.0 - gain)) / np.sin(angle)
        B = np.sin(angle * gain) / np.sin(angle)
        dq0 = A + B * dq0
        dq1 = B * dq1
        dq2 = B * dq2
        dq3 = B * dq3

    else:
        #// Lerp (Linear interpolation):
        dq0 = (1.0 - gain) + gain * dq0
        dq1 = gain * dq1;
        dq2 = gain * dq2;
        dq3 = gain * dq3;


    q0,q1,q2,q3 = normalizeQuaternion(dq0, dq1, dq2, dq3)
    return (q0,q1,q2,q3)

def quaternionMultiplication(p0, p1, p2, p3, q0, q1, q2, q3, r0, r1, r2, r3):
    
    # p= first quat
    # q = second quat
    # r = p q
    r0 = p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3
    r1 = p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2
    r2 = p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1
    r3 = p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0 
    
    return(r0, r1, r2, r3)

def invertQuaternion(q0, q1, q2, q3, q0_inv, q1_inv, q2_inv, q3_inv):
    #// Assumes quaternion is normalized.
    q0_inv = q0;
    q1_inv = -q1
    q2_inv = -q2
    q3_inv = -q3
    
    return(q0_inv, q1_inv, q2_inv, q3_inv)

def normalizeVector(x, y, z):
    
    norm = np.sqrt(x * x + y * y + z * z)
    x = x / norm
    y = y / norm
    z = z / norm
    
    return(x, y, z)

def rotateVectorByQuaternion(x, y, z, q0, q1, q2, q3, vx, vy, vz):
    
    vx = (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * x + 2 * (q1 * q2 - q0 * q3) * y + 2 * (q1 * q3 + q0 * q2) * z
    vy = 2 * (q1 * q2 + q0 * q3) * x + (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * y + 2 * (q2 * q3 - q0 * q1) * z
    vz = 2 * (q1 * q3 - q0 * q2) * x + 2 * (q2 * q3 + q0 * q1) * y + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * z 
    
    return(vx,vy,vz)
    pass