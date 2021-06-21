import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from numpy.linalg import inv

class kinematics(object):
    '''
    creates a kinematics object that can interact with accelerometer and gyroscope readings 
    to calculate the true accel values of two points attached on a rigid body
    points dont move relative to each other
    '''
    def __init__(self, hl_accelx, hl_accely, hl_accelz, hl_gyrx, hl_gyry, hl_gyrz, ):
        
        #assign accel and gyr values for hololens so that we can compare it with xsens
        self.hl_accelx = hl_accelx
        self.hl_accely = hl_accely
        self.hl_accelz = hl_accelz
        self.hl_gyrx = hl_gyrx
        self.hl_gyry = hl_gyry
        self.hl_gyrz = hl_gyrz
        
        
        
        
    def StackStreams(self,splineStackedXsens):
        '''
        stacks the values together for easier readability
        '''

        #stack accels
        hlaccelStack = np.array([
            self.hl_accelx,
            self.hl_accely,
            self.hl_accelz])
        
        #stack gyrs
        hlgyrStack = np.array([
            self.hl_gyrx,
            self.hl_gyry,
            self.hl_gyrz])
        
        
        #HL STACKS
        hlAccelStack = np.squeeze(hlaccelStack)
        hlGyroStack = np.squeeze(hlgyrStack)
        
        
        #stack for better readability
        
        xsensAccelStack =  np.array([
                                    [splineStackedXsens[3]],
                                    [splineStackedXsens[4]],
                                    [splineStackedXsens[5]]
                                    ])
        xsensAccelStack = np.squeeze(xsensAccelStack)


        xsensGyroStack =  np.array([
                                    [splineStackedXsens[0]],
                                    [splineStackedXsens[1]],
                                    [splineStackedXsens[2]]
                                    ])
        xsensGyroStack = np.squeeze(xsensGyroStack)
        

        return (hlAccelStack, hlGyroStack,
                xsensAccelStack, xsensGyroStack)


    def createOrientMatrix(self,
                       xsensnewOrienta,xsensnewOrientb,xsensnewOrientc,xsensnewOrientd,
                       hlnewOrienta,hlnewOrientb,hlnewOrientc,hlnewOrientd):
        '''
        creates rotation matrices for the following expressions
        from xsens to world world_R_xsens
        from hl to world    world_R_hl
        '''
        
        
        world_R_hl = np.zeros((len(xsensnewOrienta), 3,3), dtype=np.float64)
        world_R_xsens = np.zeros((len(xsensnewOrienta), 3,3), dtype=np.float64)
    
        XS_eulX=np.zeros(len(xsensnewOrienta), dtype=np.float64)
        XS_eulY=np.zeros(len(xsensnewOrienta), dtype=np.float64)
        XS_eulZ=np.zeros(len(xsensnewOrienta), dtype=np.float64)
        HL_eulX=np.zeros(len(xsensnewOrienta), dtype=np.float64)
        HL_eulY=np.zeros(len(xsensnewOrienta), dtype=np.float64)
        HL_eulZ=np.zeros(len(xsensnewOrienta), dtype=np.float64)
    
        for i in range(len(xsensnewOrienta)):
            
            temp_matrix= Rotation.from_quat([xsensnewOrienta[i],xsensnewOrientb[i],
                                            xsensnewOrientc[i],xsensnewOrientd[i]])
            world_R_xsens[i,:,:] = temp_matrix.as_matrix()
            r = temp_matrix.as_euler('zyx', degrees=True)
            XS_eulX[i] = r[2]
            XS_eulY[i] = r[1]
            XS_eulZ[i] = r[0]
            
        for i in range(len(xsensnewOrienta)):
            
            temp_matrix= Rotation.from_quat([hlnewOrienta[i],hlnewOrientb[i],
                                            hlnewOrientc[i],hlnewOrientd[i]])
            world_R_hl[i,:,:] = temp_matrix.as_matrix()
            r = temp_matrix.as_euler('zyx', degrees=True)
            HL_eulX[i] = r[2]
            HL_eulY[i] = r[1]
            HL_eulZ[i] = r[0]
            

        
        return(world_R_xsens,world_R_hl)
        
        
    def trueHLfromXsens(self,
                        xsensAccelStack, xsensGyroStack,
                        avg_R_hl_xsens, utcTime):
        '''
        calculate the true acceleration value of point A attached to a rotating
        reference frame of point B by using the position vector in between two 
        points.
        
        this method is used to calculate the true acceleration value of HL 
        from the acceleration and gyroscope measurements of xsens.
        
        the pre-calculated position vector (the distance between the sensors)
        is about 0.15cm in the X-axis, which points toward the center of earth.
        
        the most accurate measurement in between sensors is done with hand measurements
        therefore, the position vector is not completely accurate.
        
        reference frame is NED, north east down, x pointing towards, y pointing to east, z pointing to
        center of earth
        '''

        
        
        #      accel_A = accel_B + angular_acceleration x r + omega x ( omega x r) + 2*omega x v_rel + a_rel
        #       
        #      both frames are moving rigidly, therefore no relative acceleration or relative velocity is present
        #      the equation then reduces down to
        # 
        #       accel_A = accel_B + angular_acceleration x r + omega x (omega x r )
        

        true_hl_accel_fromXSENS=np.zeros((3,len(xsensAccelStack[0])), dtype=np.float64)        
        #use terms for easier readability
        for i in range(len(xsensAccelStack[0])):        
            #T1 = avg_R_hl_xsens # the stable orientation obtained from the complementary filter that
                                # aligns xsens with hololens frame        
            T1 = np.eye(3)
            T2 = xsensAccelStack[:,i] # true acceleration of xsens
            T3 = xsensGyroStack[:,i]  # angular rate values of xsens imu
    
            T4 = np.divide( xsensGyroStack[:,i]- xsensGyroStack[:,i-1]  , np.mean(np.diff(utcTime) ) ) 
            #numeric derivation of angular acceleration wrt xsens sampling rate, there is no other way to calculate this.
            #this can be assumed zeor on linear motion where acceleration is negligible.
            
            T5 = np.array([0.15,0,0])  #pre-calculated position vector, aka, lever-arm, aka distance between sensors
            
            #implement the equation written above, where T0 is the true hololens acceleration
            #calculated from xsens values.
            #theoretically, this must match the hololens raw readings.
            T0 =  ( np.matmul(T1,T2)) \
                + np.matmul( T1, np.cross(np.matmul(T1,T4),T5) ) \
                    + np.cross( np.matmul(T1,T3), np.cross(np.matmul(T1,T3),T5) ) 
                    
            #assign the value to hololens acceleration vector
           # print(T0)
            true_hl_accel_fromXSENS[:,i] = T0   
        
        return (true_hl_accel_fromXSENS)
    
    def trueXsensfromHL(self,
                        hlAccelStack, hlGyroStack,
                        avg_R_hl_xsens, utcTime):
        '''
        calculate the true acceleration value of point A attached to a rotating
        reference frame of point B by using the position vector in between two 
        points.
        
        this method is used to calculate the true acceleration value of HL 
        from the acceleration and gyroscope measurements of xsens.
        
        the pre-calculated position vector (the distance between the sensors)
        is about 0.15cm in the X-axis, which points toward the center of earth.
        
        the most accurate measurement in between sensors is done with hand measurements
        therefore, the position vector is not completely accurate.
        
        reference frame is NED, north east down, x pointing towards, y pointing to east, z pointing to
        center of earth
        '''

        
        
        #      accel_A = accel_B + angular_acceleration x r + omega x ( omega x r) + 2*omega x v_rel + a_rel
        #       
        #      both frames are moving rigidly, therefore no relative acceleration or relative velocity is present
        #      the equation then reduces down to
        # 
        #       accel_A = accel_B + angular_acceleration x r + omega x (omega x r )
        

        true_xsens_accel_fromHL=np.zeros((3,len(hlAccelStack[0])), dtype=np.float64)        
        #use terms for easier readability
        for i in range(len(hlAccelStack[0])):        
            T1 = inv(avg_R_hl_xsens) # the stable orientation obtained from the complementary filter that
                                # aligns xsens with hololens frame
            #T1 = np.eye(3)
        
            T2 = hlAccelStack[:,i] # true acceleration of xsens
            T3 = hlGyroStack[:,i]  # angular rate values of xsens imu
    
            T4 = np.divide( hlGyroStack[:,i]- hlGyroStack[:,i-1]  , np.mean(np.diff(utcTime) ) ) 
            #numeric derivation of angular acceleration wrt xsens sampling rate, there is no other way to calculate this.
            #this can be assumed zeor on linear motion where acceleration is negligible.
            
            T5 = np.array([0.15,0,0])  #pre-calculated position vector, aka, lever-arm, aka distance between sensors
            
            #implement the equation written above, where T0 is the true hololens acceleration
            #calculated from xsens values.
            #theoretically, this must match the xsens raw readings.
            T0 =  ( np.matmul(T1,T2)) \
                + np.matmul( T1, np.cross(np.matmul(T1,T4),T5) ) \
                    + np.cross( np.matmul(T1,T3), np.cross(np.matmul(T1,T3),T5) ) 
                    
            #assign the value to hololens acceleration vector
           # print(T0)
            true_xsens_accel_fromHL[:,i] = T0   
        
        return (true_xsens_accel_fromHL)
    
    def gravityCompensation(self,true_hl_accel_fromXSENS,
                            hlAccelStack,
                            xsensAccelStack,
                            world_R_xsens, world_R_hl):
        '''
        compensates for dynamically changing gravity vector by using the orientation of both sensors
        wrt earth gravity.
        
        
        the results are devoid of gravitational acceleration, meaning that they will output
        specific force instead of acceleration.

        due to temporal offset, the gravity compensation suffers from acceleration mismatches.
        '''
        #init grav vector on NED frame, pointing to x axis 
        gravVector = np.array([-9.81,0,0]).T

        #compensate for lever-arm calculated hl
        gravComp_true_hl_accel_fromXSENS=np.zeros((3,(len(world_R_xsens))), dtype=np.float64)        
        for i in range(len(world_R_xsens)):
            
            gravComp_true_hl_accel_fromXSENS[:,i] = true_hl_accel_fromXSENS[:,i] \
                - np.matmul(inv(world_R_xsens[i,:,:]), gravVector)

        #compensate for raw hl
        gravComp_hlAccelStack=np.zeros((3,(len(world_R_xsens))), dtype=np.float64)          
        for i in range(len(world_R_hl)):

            gravComp_hlAccelStack[:,i] =  hlAccelStack[:,i] \
                    - np.matmul(inv(world_R_hl[i,:,:]), gravVector)

        
        #compensate for xsens    
        gravComp_xsensAccelStack=np.zeros((3,(len(world_R_xsens))), dtype=np.float64)            
        for i in range(len(world_R_xsens)):

            gravComp_xsensAccelStack[:,i] = xsensAccelStack[:,i] \
                    - np.matmul(inv(world_R_xsens[i,:,:]), gravVector)
                    
        return(gravComp_true_hl_accel_fromXSENS,
               gravComp_hlAccelStack,
               gravComp_xsensAccelStack)

        
    def plotter(self,coarseAligned_xTime,
                gravComp_true_hl_accel_fromXSENS, gravComp_hlAccelStack, gravComp_xsensAccelStack,
                true_hl_accel_fromXSENS,true_xsens_accel_fromHL,
                hlAccelStack, xsensAccelStack):

        '''
        plot acceleration values
        '''
 
                            #plot regular accels
        #X axis
        fig = plt.figure(figsize=(16,9))
        plt.rc('text', usetex=True)
        plt.rc('font', family='serif')
        plt.plot(coarseAligned_xTime,hlAccelStack[0],'b', label="HL Raw Extrinsic Aligned") 
        plt.plot(coarseAligned_xTime,xsensAccelStack[0],'r', label="Xsens Raw Extrinsic Aligned")
        plt.ylim(-10,-7)    
        plt.xticks(fontsize=30)
        plt.yticks(fontsize=30)        
        plt.legend(loc="upper left",fontsize=20)
        plt.title("Acceleration (X-Axis)",fontsize=30)
        plt.xlabel('Time ($s$)',fontsize=30)
        plt.ylabel('Acceleration $(m/s^2)$', fontsize=30)
        plt.savefig("raw_acc_compareX.ps")

        #Y axis
        fig = plt.figure(figsize=(16,9))
        plt.rc('text', usetex=True)
        plt.rc('font', family='serif')
        plt.plot(coarseAligned_xTime,hlAccelStack[1],'b', label="HL Raw Extrinsic Aligned") 
        plt.plot(coarseAligned_xTime,xsensAccelStack[1],'r', label="Xsens Raw Extrinsic Aligned")
        plt.xticks(fontsize=30)
        plt.yticks(fontsize=30)
        plt.legend(loc="upper left",fontsize=20)
        plt.title("Acceleration (Y-Axis)",fontsize=30)
        plt.xlabel('Time ($s$)',fontsize=30)
        plt.ylabel('Acceleration $(m/s^2)$', fontsize=30)
        plt.savefig("raw_acc_compareY.ps")


        #Z axis
        fig = plt.figure(figsize=(16,9))
        plt.rc('text', usetex=True)
        plt.rc('font', family='serif')
        plt.plot(coarseAligned_xTime,hlAccelStack[2],'b', label="HL Raw Extrinsic Aligned") 
        plt.plot(coarseAligned_xTime,xsensAccelStack[2],'r', label="Xsens Raw Extrinsic Aligned")
        plt.xticks(fontsize=30)
        plt.yticks(fontsize=30)    
        plt.legend(loc="upper left",fontsize=20)
        plt.title("Acceleration (Z-Axis)",fontsize=30)
        plt.xlabel('Time ($s$)',fontsize=30)
        plt.ylabel('Acceleration $(m/s^2)$', fontsize=30)
        plt.savefig("raw_acc_compareZ.ps")
        
        
        #                   plot gravity compensated accels
        
        #X axis
        fig = plt.figure(figsize=(16,9))
        plt.rc('text', usetex=True)
        plt.rc('font', family='serif')
        plt.plot(coarseAligned_xTime,gravComp_hlAccelStack[0],'b', label="HL Grav Compensated") 
        plt.plot(coarseAligned_xTime,gravComp_xsensAccelStack[0],'r', label="Xsens Grav Compensated")
        plt.xticks(fontsize=18)
        plt.yticks(fontsize=18)      
        plt.legend(loc="upper left",fontsize=20)
        plt.title("Acceleration (X-Axis)",fontsize=30)
        plt.xlabel('Time ($s$)',fontsize=30)
        plt.ylabel('Acceleration $(m/s^2)$', fontsize=30)
        plt.savefig("grav_acc_compareX.ps")

        #Y axis
        fig = plt.figure(figsize=(16,9))
        plt.rc('text', usetex=True)
        plt.rc('font', family='serif')
        plt.plot(coarseAligned_xTime,gravComp_hlAccelStack[1],'b', label="HL Grav Compensated") 
        plt.plot(coarseAligned_xTime,gravComp_xsensAccelStack[1],'r', label="Xsens Grav Compensated")
        plt.legend(loc="upper left",fontsize=20)
        plt.title("Acceleration (Y-Axis)",fontsize=30)
        plt.xlabel('Time ($s$)',fontsize=30)
        plt.ylabel('Acceleration $(m/s^2)$', fontsize=30)
        plt.savefig("grav_acc_compareY.ps")


        #Z axis
        fig = plt.figure(figsize=(16,9))
        plt.rc('text', usetex=True)
        plt.rc('font', family='serif')
        plt.plot(coarseAligned_xTime,gravComp_hlAccelStack[2],'b', label="HL Grav Compensated") 
        plt.plot(coarseAligned_xTime,gravComp_xsensAccelStack[2],'r', label="Xsens Grav Compensated")
        plt.legend(loc="upper left",fontsize=20)
        plt.title("Acceleration (Z-Axis)",fontsize=30)
        plt.xlabel('Time ($s$)',fontsize=30)
        plt.ylabel('Acceleration $(m/s^2)$', fontsize=30)
        plt.savefig("grav_acc_compareZ.ps")
        
        #                           plot lever arm calculated hl accel
         #X axis
        fig = plt.figure(figsize=(16,9))
        plt.rc('text', usetex=True)
        plt.rc('font', family='serif')
        plt.plot(coarseAligned_xTime,hlAccelStack[0],'b', label="HL Raw") 
        plt.plot(coarseAligned_xTime,true_hl_accel_fromXSENS[0],'g+', label="HL from Lever-Arm")
        plt.plot(coarseAligned_xTime,xsensAccelStack[0],'r', label="Xsens Raw")
        plt.ylim(-10,-7)    
        plt.xticks(fontsize=30)
        plt.yticks(fontsize=30)  
        plt.legend(loc="upper left",fontsize=20)
        plt.title("Acceleration (X-Axis)",fontsize=30)
        plt.xlabel('Time ($s$)',fontsize=30)
        plt.ylabel('Acceleration $(m/s^2)$', fontsize=30)
        plt.savefig("true_acc_compareX.ps")

        #Y axis
        fig = plt.figure(figsize=(16,9))
        plt.rc('text', usetex=True)
        plt.rc('font', family='serif')
        plt.plot(coarseAligned_xTime,hlAccelStack[1],'b', label="HL Raw") 
        plt.plot(coarseAligned_xTime,true_hl_accel_fromXSENS[1],'g+', label="HL from Lever-Arm")
        plt.plot(coarseAligned_xTime,xsensAccelStack[1],'r', label="Xsens Raw")
        plt.xticks(fontsize=30)
        plt.yticks(fontsize=30)       
        plt.legend(loc="upper left",fontsize=20)
        plt.title("Acceleration (Y-Axis)",fontsize=30)
        plt.xlabel('Time ($s$)',fontsize=30)
        plt.ylabel('Acceleration $(m/s^2)$', fontsize=30)
        plt.savefig("true_acc_compareY.ps")


        #Z axis
        fig = plt.figure(figsize=(16,9))
        plt.rc('text', usetex=True)
        plt.rc('font', family='serif')
        plt.plot(coarseAligned_xTime,hlAccelStack[2],'b', label="HL Raw") 
        plt.plot(coarseAligned_xTime,true_hl_accel_fromXSENS[2],'g+', label="HL from Lever-Arm")
        plt.plot(coarseAligned_xTime,xsensAccelStack[2],'r', label="Xsens Raw")
        plt.xticks(fontsize=30)
        plt.yticks(fontsize=30)      
        plt.legend(loc="upper left",fontsize=20)
        plt.title("Acceleration (Z-Axis)",fontsize=30)
        plt.xlabel('Time ($s$)',fontsize=30)
        plt.ylabel('Acceleration $(m/s^2)$', fontsize=30)
        plt.savefig("true_acc_compareZ.ps")