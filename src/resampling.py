import numpy as np
from scipy import signal

def hlUpsampler(accx, accy, accz,
               gyrx, gyry, gyrz,
               knownpoints,
               xsensRef):

    '''
    upsamples hl imu to xsens levels
    '''
    
    #upsample HL
    (uphlAccX, uphlAccX_time) = signal.resample(accx, len(xsensRef),knownpoints) 
    (uphlAccY, uphlAccY_time) = signal.resample(accy, len(xsensRef),knownpoints)
    (uphlAccZ, uphlAccZ_time) = signal.resample(accz, len(xsensRef),knownpoints)

    #rotation matrix aligned gyr values
    (uphlGyrX, uphlGyrX_time) = signal.resample(gyrx, len(xsensRef),knownpoints)
    (uphlGyrY, uphlGyrY_time) = signal.resample(gyry, len(xsensRef),knownpoints)
    (uphlGyrZ, uphlGyrZ_time) = signal.resample(gyrz, len(xsensRef),knownpoints)
    
    upsampledHLstack = np.array([uphlGyrX,
                                 uphlGyrY,
                                 uphlGyrZ,
                                 uphlAccX,
                                 uphlAccY,
                                 uphlAccZ,])
    return (upsampledHLstack, uphlAccX, uphlAccY, uphlAccZ,
            uphlGyrX, uphlGyrY, uphlGyrZ,
            )


def hlPartialUpsampler(x,y,z,timeline):
    '''
    upsamples hl accel to hl gyro levels
    '''
    (interp_hlAccX, uphlAccXbag_time) = signal.resample(x, len(timeline),timeline)
    (interp_hlAccY, uphlAccYbag_time) = signal.resample(y, len(timeline),timeline)
    (interp_hlAccZ, uphlAccZbag_time) = signal.resample(z, len(timeline),timeline)
    
    return(interp_hlAccX, interp_hlAccY, interp_hlAccZ)