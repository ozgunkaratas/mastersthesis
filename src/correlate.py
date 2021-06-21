import numpy as np
from scipy.signal import correlate as corr
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from scipy import signal

def coarseAlign(rosTime, utcTime,
                pvStamps_epoch, pvStamps_experiment,
                h_a_time_epoch, h_g_time_epoch,
                h_a_time_experiment_aug, h_g_time_experiment_aug,
                gyrFPS, xsensFPS, pvFPS,
                interp_hlGyrX, interp_hlGyrY, interp_hlGyrZ,
                interp_hlAccX, interp_hlAccY, interp_hlAccZ,
                accX, accY, accZ, gyrX, gyrY, gyrZ,
                queryPointsGyr):
    '''
    use experiment duration to cut the excess samples from either the end or the beginning of stream
    assign the first ROS timestamp to hl timestamp otherwise theres no way to align the streams
    '''
    
    rosFirst = rosTime[0]
    accFirst = h_a_time_epoch[0]
    gyrFirst = h_g_time_epoch[0]
    
    accDiff = float(rosFirst) - float(accFirst)
    gyrDiff = float(rosFirst) - float(gyrFirst)
    
    #experiment duration difference due to manual start up and button press delays
    #TODO: assert which one has gone for longer
    initDiff = float(h_g_time_experiment_aug[-1]) - utcTime[-1]
    initCamDiff = float(pvStamps_experiment[-1]) - utcTime[-1]
    coarseAligned_gTime = np.arange(len(h_g_time_experiment_aug),dtype = np.float64)
    

    #HL ran more 
    if initDiff > 0 :
        erasedSamples     = int(initDiff * gyrFPS)
        erasedSamplesCam  = int(initCamDiff * pvFPS)   
        coarseAligned_gTime = h_g_time_experiment_aug[erasedSamples:] - initDiff
        coarseAligned_qpG    = queryPointsGyr[erasedSamples:] - initDiff   
        
        #return unchanged stream for xsens
        erasedSamplesXsens = 0
        coarseAligned_xTime = utcTime[erasedSamplesXsens:]
        
   #xsens ran more
    if initDiff < 0:
        erasedSamplesXsens = int(-initDiff * xsensFPS)
        coarseAligned_xTime = utcTime[erasedSamplesXsens:] - initDiff

        #return unchanged stream for hololens
        erasedSamples     = 0
        erasedSamplesCam  = 0
        initCamDiff       = 0   
        coarseAligned_gTime = h_g_time_experiment_aug[erasedSamples:]
        coarseAligned_qpG    = queryPointsGyr[erasedSamples:]          

    #coarse align hl if it ran more
    coarseAligned_hlGyrX = interp_hlGyrX[erasedSamples:] 
    coarseAligned_hlGyrY = interp_hlGyrY[erasedSamples:] 
    coarseAligned_hlGyrZ = interp_hlGyrZ[erasedSamples:] 
    coarseAligned_hlAccX = interp_hlAccX[erasedSamples:]
    coarseAligned_hlAccY = interp_hlAccY[erasedSamples:]
    coarseAligned_hlAccZ = interp_hlAccZ[erasedSamples:]
         
    #coarse align xsens if it ran more
    coarseAligned_xsensGyrX = gyrX[erasedSamplesXsens:]
    coarseAligned_xsensGyrY = gyrY[erasedSamplesXsens:]
    coarseAligned_xsensGyrZ = gyrZ[erasedSamplesXsens:]
    coarseAligned_xsensAccX = accX[erasedSamplesXsens:]
    coarseAligned_xsensAccY = accY[erasedSamplesXsens:]
    coarseAligned_xsensAccZ = accZ[erasedSamplesXsens:]


    
    return(initDiff, initCamDiff, erasedSamples, erasedSamplesCam, coarseAligned_gTime, coarseAligned_xTime,
           coarseAligned_hlGyrX, coarseAligned_hlGyrY, coarseAligned_hlGyrZ,
           coarseAligned_hlAccX, coarseAligned_hlAccY, coarseAligned_hlAccZ,
           coarseAligned_xsensGyrX, coarseAligned_xsensGyrY, coarseAligned_xsensGyrZ,
           coarseAligned_xsensAccX, coarseAligned_xsensAccY, coarseAligned_xsensAccZ,
           coarseAligned_qpG,
           )

def crossCorrelate(coarseAligned_hlGyrX, coarseAligned_hlGyrY, coarseAligned_hlGyrZ,
                   coarseAligned_xsensGyrX, coarseAligned_xsensGyrY, coarseAligned_xsensGyrZ,
                   coarseAligned_xsensAccX, coarseAligned_xsensAccY, coarseAligned_xsensAccZ,
                   xsensFPS):
    '''
    classic discrete x-corr
    done on individual axes and also each delay is found in terms of samples and seconds
    '''
    #initiate cross-correlation
    shift       = -coarseAligned_hlGyrX.shape[0] + 1
    
    xcorr_x       = corr(coarseAligned_xsensGyrX, coarseAligned_hlGyrX)
    locsx        = np.argmax(xcorr_x)
    locsx        = locsx+ shift
    
    xcorr_y       = corr(coarseAligned_xsensGyrY,coarseAligned_hlGyrY)
    locsy        = np.argmax(xcorr_y)
    locsy        = locsy+ shift
    
    xcorr_z       = corr(coarseAligned_xsensGyrZ, coarseAligned_hlGyrZ)
    locsz        = np.argmax(xcorr_z)
    locsz        = locsz+ shift

    #temporally calibrate according to xorr
    #find the smallest
    allLocs = [locsx,locsy,locsz]
    smallest = min(allLocs)
    
    #           X AXIS
    if locsx > 0 : 
        zeroes = np.zeros(locsx)
        zeroes = np.array([zeroes],np.float64)
        zeroes = zeroes.T
        zeroes= zeroes.squeeze()
        
        xorrshiftx = np.append(coarseAligned_xsensGyrX,zeroes,axis=0)
        xorrshiftx = xorrshiftx[np.size(zeroes)-1:-1]
        xorrshiftx_acc = np.append(coarseAligned_xsensAccX,zeroes,axis=0)
        xorrshiftx_acc = xorrshiftx_acc[np.size(zeroes)-1:-1]
        
    elif locsx < 0:
        zeroes = np.zeros(-locsx)
        zeroes = np.array([zeroes],np.float64)
        zeroes = zeroes.T
        zeroes= zeroes.squeeze()
        locsx = -locsx
        xorrshiftx = np.append(zeroes,coarseAligned_xsensGyrX, axis=0)
        xorrshiftx = xorrshiftx[0:len(xorrshiftx)-np.size(zeroes)]
        xorrshiftx_acc = np.append(zeroes,coarseAligned_xsensAccX, axis=0)
        xorrshiftx_acc = xorrshiftx_acc[0:len(xorrshiftx_acc)-np.size(zeroes)]
        
    
    #           Y AXIS
    if locsy > 0 :        
        zeroes = np.zeros(locsy)
        zeroes = np.array([zeroes],np.float64)
        zeroes = zeroes.T
        zeroes= zeroes.squeeze()
        xorrshifty = np.append(coarseAligned_xsensGyrY,zeroes,axis=0)
        xorrshifty = xorrshifty[np.size(zeroes)-1:-1]
        xorrshifty_acc = np.append(coarseAligned_xsensAccY,zeroes,axis=0)
        xorrshifty_acc = xorrshifty_acc[np.size(zeroes)-1:-1]
    
    
    elif locsy < 0:
        zeroes = np.zeros(-locsy)
        zeroes = np.array([zeroes],np.float64)
        zeroes = zeroes.T
        zeroes= zeroes.squeeze()
        locsy = -locsy
        xorrshifty = np.append(zeroes,coarseAligned_xsensGyrY, axis=0)
        xorrshifty = xorrshifty[0:len(xorrshifty)-np.size(zeroes)]
        xorrshifty_acc = np.append(zeroes,coarseAligned_xsensAccY, axis=0)
        xorrshifty_acc = xorrshifty_acc[0:len(xorrshifty_acc)-np.size(zeroes)]
    

    #           Z AXIS
    
    if locsz > 0 :   
        zeroes = np.zeros(locsz)
        zeroes = np.array([zeroes],np.float64)
        zeroes = zeroes.T
        zeroes= zeroes.squeeze()
        xorrshiftz = np.append(coarseAligned_xsensGyrZ,zeroes,axis=0)
        xorrshiftz = xorrshiftz[np.size(zeroes)-1:-1]
        xorrshiftz_acc = np.append(coarseAligned_xsensAccZ,zeroes,axis=0)
        xorrshiftz_acc = xorrshiftz_acc[np.size(zeroes)-1:-1]
    
    
    elif locsz < 0:
        zeroes = np.zeros(-locsz)
        zeroes = np.array([zeroes],np.float64)
        zeroes = zeroes.T
        zeroes= zeroes.squeeze()
        locsz = -locsz 
        xorrshiftz = np.append(zeroes,coarseAligned_xsensGyrZ, axis=0)
        xorrshiftz = xorrshiftz[0:len(xorrshiftz)-np.size(zeroes)]
        xorrshiftz_acc = np.append(zeroes,coarseAligned_xsensAccZ,axis=0)
        xorrshiftz_acc = xorrshiftz_acc[0:len(xorrshiftz_acc)-np.size(zeroes)]
            
    
        #store preliminary delay values
    xdelay = locsx /xsensFPS
    ydelay = locsy /xsensFPS
    zdelay = locsz /xsensFPS
        #xorrshift values are xsens values that have been shifted 
        #on the timeline depending on the cross-correlation results
        
        #xcorr_ values are the cross-correlation values which will be used
        #for spline correlation as welly
    #stack everything
    corrXsensStack = np.array([
                               [xorrshiftx],
                               [xorrshifty],
                               [xorrshiftz],
                               [xorrshiftx_acc],
                               [xorrshifty_acc],
                               [xorrshiftz_acc]
        
                ])
    return(xdelay, ydelay, zdelay,
           xcorr_x, xcorr_y, xcorr_z,
           locsx, locsy, locsz, smallest, shift,
           xorrshiftx, xorrshifty, xorrshiftz,
           xorrshiftx_acc, xorrshifty_acc, xorrshiftz_acc,
           corrXsensStack)
    

 
    
def splineCorrelate(xcorr_x,xcorr_y, xcorr_z,
                    locsx, locsy, locsz, smallest,
                    xsensFPS,
                    gyrX, gyrY, gyrZ,
                    accX, accY, accZ):

    '''
    spline correlate, source code is similar to https://github.com/MobileRoboticsSkoltech/twist-n-sync but i wrote mods
    to account for dissimilarity between xsens and hl and also the roots of the spline were complex at some point so i corrected that as well
    '''
    old_smallest = smallest
    if smallest <0:
        smallest = -smallest
    #               X AXIS
    cubic = CubicSpline(np.arange(xcorr_x.shape[0]), xcorr_x, bc_type = 'natural') 
    coefficients = cubic.c[:,locsx]
    coefficients = np.squeeze(coefficients)
    order        = coefficients.shape[0] - 1
    deriv        = coefficients[-2]
    
    if deriv < 0 :
        locsx         = locsx - 1
        coefficients = cubic.c[:,locsx]
        coefficients = np.squeeze(coefficients)
    
    quad = np.roots([(order-i)*coefficients[i] for i in range(order)])
    
    if sum((order-i)*coefficients[i]*((quad[0]+quad[1])/2)**(order-i-1) for i in range(order)) < 0:
        quad = np.min(quad)
    else:
        quad = np.max(quad)
                
    delay = locsx + quad 

    if isinstance(quad,complex):
        #print("complex")
        if locsx > 0 :
            delay = np.abs(delay)                 
        elif locsx < 0 :
            delay = -np.abs(delay)
    if not isinstance(quad, complex):
         #print("not comp")
         pass

    sample_delays_x = int(np.ceil(delay))
    if sample_delays_x <0:
        delay_secs_x = sample_delays_x / xsensFPS
        zeroes = np.zeros(-sample_delays_x)
        zeroes = np.array([zeroes],np.float64)
        zeroes = zeroes.T
        zeroes= zeroes.squeeze()
        sample_delays_x = -sample_delays_x 
        rolled_x = np.append(gyrX,zeroes,axis=0)
        rolled_x = rolled_x[np.size(zeroes)-1:-1]   
        rolled_x_acc = np.append(accX,zeroes,axis=0)
        rolled_x_acc = rolled_x_acc[np.size(zeroes)-1:-1]

        
    if sample_delays_x >0 :
        delay_secs_x = sample_delays_x / xsensFPS
        zeroes = np.zeros(sample_delays_x)
        zeroes = np.array([zeroes],np.float64)
        zeroes = zeroes.T
        zeroes= zeroes.squeeze()     
        rolled_x = np.append(zeroes,gyrX, axis=0)
        rolled_x= rolled_x[0:len(rolled_x)-np.size(zeroes)]
        rolled_x_acc = np.append(zeroes,accX, axis=0)
        rolled_x_acc= rolled_x_acc[0:len(rolled_x_acc)-np.size(zeroes)]
   # print(sample_delays_x)

#################################################################################### 
    #               Y AXIS
   
    cubic = CubicSpline(np.arange(xcorr_y.shape[0]), xcorr_y, bc_type = 'natural')
    
    
    coefficients = cubic.c[:,locsy]
    coefficients = np.squeeze(coefficients)
    order        = coefficients.shape[0] - 1
    deriv        = coefficients[-2]
    
    if deriv < 0 :
        locsy         = locsy - 1
        coefficients = cubic.c[:,locsy]
        coefficients = np.squeeze(coefficients)
    
    
    quad = np.roots([(order-i)*coefficients[i] for i in range(order)])
    
    if sum((order-i)*coefficients[i]*((quad[0]+quad[1])/2)**(order-i-1) for i in range(order)) < 0:
        quad = np.min(quad)
    else:
        quad = np.max(quad)
                        
    delay = locsy + quad 
    if isinstance(quad,complex):
        #print("complex")
        if locsy > 0 :
            delay = np.abs(delay)                 
        elif locsy < 0 :
            delay = -np.abs(delay)
    if not isinstance(quad, complex):
         #print("not comp")
         pass
         
    sample_delays_y = int(np.ceil(delay))
    if sample_delays_y <0:
        delay_secs_y = sample_delays_y / xsensFPS
        zeroes = np.zeros(-sample_delays_y)
        zeroes = np.array([zeroes],np.float64)
        zeroes = zeroes.T
        zeroes= zeroes.squeeze()
        sample_delays_y = -sample_delays_y        
        rolled_y = np.append(gyrY,zeroes,axis=0)
        rolled_y = rolled_y[np.size(zeroes)-1:-1]
        rolled_y_acc = np.append(accY,zeroes,axis=0)
        rolled_y_acc = rolled_y_acc[np.size(zeroes)-1:-1]

        
    if sample_delays_y >0 :
        delay_secs_y = sample_delays_y  / xsensFPS
        zeroes = np.zeros(sample_delays_y)
        zeroes = np.array([zeroes],np.float64)
        zeroes = zeroes.T
        zeroes= zeroes.squeeze()     

        rolled_y = np.append(zeroes,gyrY, axis=0)
        rolled_y= rolled_y[0:len(rolled_y)-np.size(zeroes)]
        rolled_y_acc = np.append(zeroes,accY, axis=0)
        rolled_y_acc= rolled_y_acc[0:len(rolled_y_acc)-np.size(zeroes)]
 #   print(sample_delays_y)
####################################################################################   
    #               Z AXIS
    

    cubic = CubicSpline(np.arange(xcorr_z.shape[0]), xcorr_z, bc_type = 'natural')
    
    
    coefficients = cubic.c[:,locsz]
    coefficients = np.squeeze(coefficients)
    order        = coefficients.shape[0] - 1
    deriv        = coefficients[-2]
    
    if deriv < 0 :
        locsz         = locsz - 1
        coefficients = cubic.c[:,locsz]
        coefficients = np.squeeze(coefficients)
    
    
    quad = np.roots([(order-i)*coefficients[i] for i in range(order)])
    
    if sum((order-i)*coefficients[i]*((quad[0]+quad[1])/2)**(order-i-1) for i in range(order)) < 0:
        quad = np.min(quad)
    else:
        quad = np.max(quad)
        
    delay = locsz + quad 
    if isinstance(quad,complex):
        #print("complex")
        if locsz > 0 :
            delay = np.abs(delay)                 
        elif locsz < 0 :
            delay = -np.abs(delay)
    if not isinstance(quad, complex):
         #print("not comp")
         pass
         
    sample_delays_z = int(np.ceil(delay))
    if sample_delays_z <0:
        delay_secs_z =  sample_delays_z / xsensFPS
        zeroes = np.zeros(-sample_delays_z)
        zeroes = np.array([zeroes],np.float64)
        zeroes = zeroes.T
        zeroes= zeroes.squeeze()
        sample_delays_z = -sample_delays_z
        rolled_z = np.append(gyrZ,zeroes,axis=0)
        rolled_z = rolled_z[np.size(zeroes)-1:-1]
        rolled_z_acc = np.append(accZ,zeroes,axis=0)
        rolled_z_acc = rolled_z_acc[np.size(zeroes)-1:-1]


    if sample_delays_z >0 :
        delay_secs_z =  sample_delays_z  / xsensFPS
        zeroes = np.zeros(sample_delays_z)
        zeroes = np.array([zeroes],np.float64)
        zeroes = zeroes.T
        zeroes= zeroes.squeeze()
        

        rolled_z = np.append(zeroes,gyrZ, axis=0)
        rolled_z= rolled_z[0:len(rolled_z)-np.size(zeroes)]
        rolled_z_acc = np.append(zeroes,accZ, axis=0)
        rolled_z_acc= rolled_z_acc[0:len(rolled_z_acc)-np.size(zeroes)]


    #stack everything
    splineStackedXsens = np.array([
        rolled_x,
        rolled_y,
        rolled_z,
        rolled_x_acc,
        rolled_y_acc,
        rolled_z_acc])

    return (delay_secs_x, delay_secs_y, delay_secs_z,
            splineStackedXsens)

      
def plotUtils(coarseAligned_xTime,coarseAligned_qpG,
              upCoarseAligned_hlGyrX, upCoarseAligned_hlGyrY, upCoarseAligned_hlGyrZ,
              coarseAligned_hlGyrX, coarseAligned_hlGyrY, coarseAligned_hlGyrZ,
              xorrshiftx, xorrshifty, xorrshiftz,
              splineStackedXsens,
              coarseAligned_xsensGyrX, coarseAligned_xsensGyrY, coarseAligned_xsensGyrZ):

    '''
    plot utilities
    '''

#plot gyroscope values

    
#COARSE ALIGNED
#no delay input necessary 
    
    fig = plt.figure(figsize=(16,9))
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    plt.plot(coarseAligned_xTime,coarseAligned_xsensGyrX,'r',label="Xsens (Raw)")
    plt.plot(coarseAligned_qpG,coarseAligned_hlGyrX,'b', label="HL (Coarse Aligned)")
    plt.xticks(fontsize=30)
    plt.yticks(fontsize=30)
    plt.legend(loc="upper left",fontsize=20)
    plt.title("Angular Velocities (X-Axis)",fontsize=30)
    plt.xlabel('Time ($s$)',fontsize=30)
    plt.ylabel('Angular Velocity ($rad/s$)', fontsize=30)
    plt.savefig("x_coarse.ps")
    
    fig = plt.figure(figsize=(16,9))
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    plt.plot(coarseAligned_xTime,coarseAligned_xsensGyrY,'r',label="Xsens (Coarse Aligned)")
    plt.plot(coarseAligned_qpG,coarseAligned_hlGyrY,'b', label="HL (Coarse Aligned)")
    plt.xticks(fontsize=30)
    plt.yticks(fontsize=30)
    plt.legend(loc="upper left",fontsize=20)
    plt.title("Angular Velocities (Y-Axis)",fontsize=30)
    plt.xlabel('Time ($s$)',fontsize=30)
    plt.ylabel('Angular Velocity ($rad/s$)', fontsize=30)
    plt.savefig("Y_coarse.ps")
    
    fig = plt.figure(figsize=(16,9))
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    plt.plot(coarseAligned_xTime,coarseAligned_xsensGyrZ,'r',label="Xsens (Coarse Aligned)")
    plt.plot(coarseAligned_qpG,coarseAligned_hlGyrZ,'b', label="HL (Coarse Aligned)")
    plt.xticks(fontsize=30)
    plt.yticks(fontsize=30)
    plt.legend(loc="upper left",fontsize=20)
    plt.title("Angular Velocities (Z-Axis)",fontsize=30)
    plt.xlabel('Time ($s$)',fontsize=30)
    plt.ylabel('Angular Velocity ($rad/s$)', fontsize=30)
    plt.savefig("z_coarse.ps")
  

#####CROSS CORRELATION
#delay obtained from corr results will be used to plot 
    fig = plt.figure(figsize=(16,9))
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    plt.plot(coarseAligned_xTime,xorrshiftx,'r',label="Xsens (Cross Correlated)")
    plt.plot(coarseAligned_xTime,upCoarseAligned_hlGyrX,'b', label="HL (Cross Correlated)")
    plt.xticks(fontsize=30)
    plt.yticks(fontsize=30)
    plt.legend(loc="upper left",fontsize=20)
    plt.title("Angular Velocities (X-Axis)",fontsize=30)
    plt.xlabel('Time ($s$)',fontsize=30)
    plt.ylabel('Angular Velocity ($rad/s$)', fontsize=30)
    plt.savefig("x_corr.ps")
    
    fig = plt.figure(figsize=(16,9))
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    plt.plot(coarseAligned_xTime,xorrshifty,'r',label="Xsens (Cross Correlated)")
    plt.plot(coarseAligned_xTime,upCoarseAligned_hlGyrY,'b', label="HL (Cross Correlated)")
    plt.xticks(fontsize=30)
    plt.yticks(fontsize=30)
    plt.legend(loc="upper left",fontsize=20)
    plt.title("Angular Velocities (Y-Axis)",fontsize=30)
    plt.xlabel('Time ($s$)',fontsize=30)
    plt.ylabel('Angular Velocity ($rad/s$)', fontsize=30)
    plt.savefig("y_corr.ps")

    
    fig = plt.figure(figsize=(16,9))
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    plt.plot(coarseAligned_xTime,xorrshiftz,'r',label="Xsens (Cross Correlated)")
    plt.plot(coarseAligned_xTime,upCoarseAligned_hlGyrZ,'b', label="HL (Cross Correlated)")
    plt.xticks(fontsize=30)
    plt.yticks(fontsize=30)
    plt.legend(loc="upper left",fontsize=20)
    plt.title("Angular Velocities (Z-Axis)",fontsize=30)
    plt.xlabel('Time ($s$)',fontsize=30)
    plt.ylabel('Angular Velocity ($rad/s$)', fontsize=30)    
    plt.savefig("z_corr.ps")

######SPLINE CORRELATION       
#delay obtained from corr results will be used to plot 

    fig = plt.figure(figsize=(16,9))
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    plt.plot(coarseAligned_xTime,splineStackedXsens[0],'r', label="Xsens (Spline Correlated)")
    plt.plot(coarseAligned_xTime,upCoarseAligned_hlGyrX,'b', label="HL (Spline Correlated)")
    plt.xticks(fontsize=30)
    plt.yticks(fontsize=30)    
    plt.legend(loc="upper left",fontsize=20)
    plt.title("Angular Velocities (X-Axis)",fontsize=30)
    plt.xlabel('Time ($s$)',fontsize=30)
    plt.ylabel('Angular Velocity ($rad/s$)', fontsize=30)
    plt.savefig("x_corrspl.ps")

    
    fig = plt.figure(figsize=(16,9))
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    plt.plot(coarseAligned_xTime,splineStackedXsens[1],'r', label="Xsens (Spline Correlated)")
    plt.plot(coarseAligned_xTime,upCoarseAligned_hlGyrY,'b', label="HL (Spline Correlated)")
    plt.xticks(fontsize=30)
    plt.yticks(fontsize=30)
    plt.legend(loc="upper left",fontsize=20)
    plt.title("Angular Velocities (Y-Axis)",fontsize=30)
    plt.xlabel('Time ($s$)',fontsize=30)
    plt.ylabel('Angular Velocity ($rad/s$)', fontsize=30)
    plt.savefig("y_corrspl.ps")

    
    fig = plt.figure(figsize=(16,9))
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    plt.plot(coarseAligned_xTime,splineStackedXsens[2],'r', label="Xsens (Spline Correlated)")
    plt.plot(coarseAligned_xTime,upCoarseAligned_hlGyrZ,'b', label="HL (Spline Correlated)")
    plt.xticks(fontsize=30)
    plt.yticks(fontsize=30)
    plt.legend(loc="upper left",fontsize=20)
    plt.title("Angular Velocities (Z-Axis)",fontsize=30)
    plt.xlabel(r'Time ($s$)',fontsize=30)
    plt.ylabel('Angular Velocity ($rad/s$)', fontsize=30)    
    plt.savefig("z_corrspl.ps") 
  

