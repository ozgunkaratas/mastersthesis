import numpy as np
import quatMath
import compFilterFuncs
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp
from scipy.spatial.transform import RotationSpline
from scipy import signal
from numpy.linalg import inv

class complementaryFilter:
    
    def __init__(self, kGravity, gamma_, kAngularVelocityThreshold, kAccelerationThreshold,
               kDeltaAngularVelocityThreshold, gain_acc_, bias_alpha_, do_bias_estimation_,
               do_adaptive_gain_,initialized_,steady_state_,timeStep,time, sensorType):
    # define CONSTANT params
        self.kGravity = kGravity
        self.gamma_ = gamma_
        self.kAngularVelocityThreshold = kAngularVelocityThreshold
        self.kAccelerationThreshold = kAccelerationThreshold
        self.kDeltaAngularVelocityThreshold = kDeltaAngularVelocityThreshold
        self.gain_acc_ =gain_acc_
        self.bias_alpha_ = bias_alpha_
        self.do_bias_estimation_ = do_bias_estimation_
        self.do_adaptive_gain_ = do_adaptive_gain_  
        self.initialized_ = initialized_
        self.steady_state_ = steady_state_
        self.timeStep = timeStep
        self.time = time
        self.sensorType = sensorType #if zero: Xsens, if 1: HL
        
        self.q0_ = 1     #these can be changed with xsens quaternions  
        self.q1_ = 0
        self.q2_ = 0
        self.q3_ = 0
        
        self.wx_prev_ = 0
        self.wy_prev_ = 0
        self.wz_prev_ = 0
        self.wx_bias_ = 0
        self.wy_bias_ = 0
        self.wz_bias_ = 0
        
        
        
        self.new_wx = np.zeros(len(time), dtype=float)
        self.new_wy = np.zeros(len(time), dtype=float)
        self.new_wz = np.zeros(len(time), dtype=float)    
        self.neworienta= np.zeros(len(time), dtype=float)
        self.neworientb= np.zeros(len(time), dtype=float)
        self.neworientc= np.zeros(len(time), dtype=float)
        self.neworientd= np.zeros(len(time), dtype=float)
        
        self.gx  = np.zeros(len(time), dtype = float)
        self.gy  = np.zeros(len(time), dtype = float)
        self.gz  = np.zeros(len(time), dtype = float)
    
    def execute(self,accX, accY, accZ, gyrX, gyrY,gyrZ,
                interp_hlAccX, interp_hlAccY, interp_hlAccZ, interp_hlGyrX, interp_hlGyrY, interp_hlGyrZ):

        if self.sensorType == 0:
            iteratorType = accX
        elif self.sensorType ==1:
            iteratorType = interp_hlAccX
            
        for i in range(len(iteratorType)):
            #x,y,z are world coords, z points DOWN, x pointing EAST, y pointing NORTH (ENU frame)
            # if self.sensorType == 0:  #xsens
            #     ax = accX[i]
            #     ay = accY[i]
            #     az = accZ[i]
                
            #     wx = gyrX[i]
            #     wy = gyrY[i]
            #     wz = gyrZ[i]
            
            # if self.sensorType ==1 : #hl
            #     ax = interp_hlAccX[i]
            #     ay = interp_hlAccY[i]
            #     az = -interp_hlAccZ[i]
                
            #     wx = interp_hlGyrX[i]
            #     wy = interp_hlGyrY[i]
            #     wz = interp_hlGyrZ[i]

            if self.sensorType == 0:  #xsens
                ax = accZ[i]
                ay = -accY[i]
                az = accX[i]
                
                wx = gyrZ[i]
                wy = -gyrY[i]
                wz = gyrX[i]
            
            if self.sensorType ==1 : #hl
                ax = interp_hlAccZ[i]
                ay = -interp_hlAccY[i]
                az = interp_hlAccX[i]
                
                wx = interp_hlGyrZ[i]
                wy = -interp_hlGyrY[i]
                wz = interp_hlGyrX[i]

            if (self.initialized_ == False):
                # start up 
                self.q0_,self.q1_,self.q2_,self.q3_ = compFilterFuncs.getMeasurement(ax, ay, az,                                                                 self.q0_, self.q1_, self.q2_, self.q3_)
                self.initialized_ = True
                
                
            self.wx_bias_, self.wy_bias_, self.wz_bias_, self.wx_prev_, self.wy_prev_, self.wz_prev_ = compFilterFuncs.updateBiases(ax,ay,az,wx,wy,wz,
                                                                    self.kAccelerationThreshold,self.kAngularVelocityThreshold,
                                                                    self.kDeltaAngularVelocityThreshold,self.kGravity,
                                                                    self.wx_bias_, self.wy_bias_, self.wz_bias_,
                                                                    self.wx_prev_, self.wy_prev_, self.wz_prev_,
                                                                    self.bias_alpha_)
            
            q0_pred = 0
            q1_pred = 0
            q2_pred = 0  
            q3_pred = 0 
            q0_pred, q1_pred, q2_pred, q3_pred = compFilterFuncs.Prediction(wx, wy, wz, self.timeStep, q0_pred, q1_pred, q2_pred, q3_pred,
                                                                     self.q0_, self.q1_, self.q2_, self.q3_,
                                                                     self.wx_bias_, self.wy_bias_, self.wz_bias_) 

            dq0_acc = 0
            dq1_acc = 0
            dq2_acc = 0
            dq3_acc = 0    
            
            dq0_acc,dq1_acc,dq2_acc,dq3_acc,self.gx[i], self.gy[i], self.gz[i] = compFilterFuncs.AccelComp(ax, ay, az,
                                                     q0_pred, q1_pred, q2_pred, q3_pred,
                                                     dq0_acc, dq1_acc, dq2_acc, dq3_acc)
        
            #adaptive gain constant
            gain = self.gain_acc_
           
           
            #FINALIZE RESULTS
            dq0_acc, dq1_acc, dq2_acc, dq3_acc=  quatMath.scaleQuaternion(gain, dq0_acc, dq1_acc, dq2_acc, dq3_acc)
            
            self.q0_, self.q1_, self.q2_, self.q3_ =   quatMath.quaternionMultiplication(q0_pred, q1_pred, q2_pred, q3_pred,
                                                                    dq0_acc, dq1_acc, dq2_acc, dq3_acc,
                                                                    self.q0_, self.q1_, self.q2_, self.q3_);
            
            self.q0_, self.q1_, self.q2_, self.q3_ =   quatMath.normalizeQuaternion(self.q0_, self.q1_, self.q2_, self.q3_)
            
            
            self.neworienta[i] = self.q0_
            self.neworientb[i] = self.q1_
            self.neworientc[i] = self.q2_
            self.neworientd[i]= self.q3_
        
            ##handle angular velocity bias
            
            self.new_wx[i] = wx - self.wx_bias_
            self.new_wy[i] = wy - self.wy_bias_
            self.new_wz[i] = wz - self.wz_bias_
            

        
        return (self.neworienta, self.neworientb, self.neworientc, self.neworientd,
                self.new_wx, self.new_wy, self.new_wz,self.gx, self.gy, self.gz)

    def solveRotationMatrix(self,neworienta, neworientb, neworientc, neworientd,time):
        stackedMatrix = np.zeros((len(time)+1,3,3))   
        for i in range(len(time)):
            r = Rotation.from_quat([neworienta[i],neworientb[i],neworientc[i],neworientd[i]])
            rotmat = r.as_matrix()
            stackedMatrix[i,:,:]= rotmat
                
        return(stackedMatrix)
    

    def solveOrientation(self,xsensstackedMatrix,hlstackedMatrix,xsensFPS):
        
        #jump the first five seconds because the person needs to 
        #start the streams and get into a stable position
        #after the first 5 seconds, an averaging of the first second
        #with euler angles can be obtained and from this a rotation matrix can be created
        jump = int(np.floor(60* xsensFPS))
        XS_eulX=np.zeros(int(xsensFPS), dtype=np.float64)
        XS_eulY=np.zeros(int(xsensFPS), dtype=np.float64)
        XS_eulZ=np.zeros(int(xsensFPS), dtype=np.float64)
        HL_eulX=np.zeros(int(xsensFPS), dtype=np.float64)
        HL_eulY=np.zeros(int(xsensFPS), dtype=np.float64)
        HL_eulZ=np.zeros(int(xsensFPS), dtype=np.float64)
        eulerX=np.zeros(int(xsensFPS), dtype=np.float64)
        eulerY=np.zeros(int(xsensFPS), dtype=np.float64)
        eulerZ=np.zeros(int(xsensFPS), dtype=np.float64)

        
        #from xsens to world, and then from world to hl
        R_hl_xsens = np.zeros((len(xsensstackedMatrix)-1,3,3))
        for i in range(len(xsensstackedMatrix)-1):
            R_hl_xsens[i,:,:] = np.matmul(inv(hlstackedMatrix[i,:,:]),xsensstackedMatrix[i,:,:])
  
        for i in range(int(xsensFPS)):

            temp = Rotation.from_matrix(R_hl_xsens[i+jump])
            r = temp.as_euler('zyx', degrees=True)
            eulerX[i] = r[2]
            eulerY[i] = r[1]
            eulerZ[i] = r[0]         

        for i in range(int(xsensFPS)):
            temp = Rotation.from_matrix(xsensstackedMatrix[i+jump])
            r = temp.as_euler('zyx', degrees=True)
            XS_eulX[i] = r[2]
            XS_eulY[i] = r[1]
            XS_eulZ[i] = r[0]
            
            
        for i in range(int(xsensFPS)):
            temp = Rotation.from_matrix(hlstackedMatrix[i+jump])            
            r = temp.as_euler('zyx', degrees=True)
            HL_eulX[i] = r[2]
            HL_eulY[i] = r[1]
            HL_eulZ[i] = r[0]

        #average the results to optimize the orientation between them
        avg_hl_eulX = np.mean(HL_eulX)
        avg_hl_eulY = np.mean(HL_eulY)
        avg_hl_eulZ = np.mean(HL_eulZ)
        avg_xs_eulX = np.mean(XS_eulX)
        avg_xs_eulY = np.mean(XS_eulY)
        avg_xs_eulZ = np.mean(XS_eulZ)
        
        avg_eulX = np.mean(eulerX)
        avg_eulY = np.mean(eulerY)
        avg_eulZ = np.mean(eulerZ)


        
        #create a new rotation matrix in between xsens and the hl
        
        #rot matrix that takes values from HL to world
        avg_R_hl = Rotation.from_euler('zyx',[avg_hl_eulZ, avg_hl_eulY, avg_hl_eulX], degrees=True)
        avg_R_hl = avg_R_hl.as_matrix()
        
        #rot matrix that takes values from XSENS to world
        avg_R_xs = Rotation.from_euler('zyx',[avg_xs_eulZ, avg_xs_eulY, avg_xs_eulX], degrees=True)
        avg_R_xs = avg_R_xs.as_matrix()
        
        #rot matrix that takes values from Xsens to HL
        
#        avg_R_hl_xsens = np.matmul(inv(avg_R_hl),avg_R_xs)


        avg_R_hl_xsens = Rotation.from_euler('zyx',[avg_eulX, avg_eulY, avg_eulZ], degrees = True)
        avg_R_hl_xsens = avg_R_hl_xsens.as_matrix()
        
        #kalibr
 #        avg_R_hl_xsens = np.array([
 #            [-0.95052692,  0.29522118, -0.09665936],
 # [-0.2675078,  -0.61972534,  0.73782118],
 # [ 0.15791818,  0.72717602,  -0.66803958]
 # ])

        return(R_hl_xsens, avg_R_hl_xsens)
    
    
    def XsensOnHL(self,coarseAligned_xsensGyrX, coarseAligned_xsensGyrY, coarseAligned_xsensGyrZ,
                  coarseAligned_xsensAccX, coarseAligned_xsensAccY, coarseAligned_xsensAccZ, avg_R_hl_xsens):
        '''
        transfers the xsens gyro values on HL for cross-correlation
        '''
        
        xsensStacked_omega = np.array([coarseAligned_xsensGyrX,
                                       coarseAligned_xsensGyrY,
                                       coarseAligned_xsensGyrZ
            ])
        xsensStacked_alpha = np.array([coarseAligned_xsensAccX,
                                       coarseAligned_xsensAccY,
                                       coarseAligned_xsensAccZ
            ])
        #copy the original stack to a var
        xsensStacked_omega_copy = xsensStacked_omega 
        xsensStacked_alpha_copy = xsensStacked_alpha

        xsens_Stacked_omega = np.matmul(avg_R_hl_xsens, xsensStacked_omega)
        coarseAligned_xsensGyrX = xsens_Stacked_omega[0]
        coarseAligned_xsensGyrY = xsens_Stacked_omega[1]
        coarseAligned_xsensGyrZ = xsens_Stacked_omega[2]
        

        xsens_Stacked_alpha = np.matmul(avg_R_hl_xsens, xsensStacked_alpha)
        coarseAligned_xsensAccX = xsens_Stacked_alpha[0]
        coarseAligned_xsensAccY = xsens_Stacked_alpha[1]
        coarseAligned_xsensAccZ = xsens_Stacked_alpha[2]
        
        
        return (coarseAligned_xsensGyrX, coarseAligned_xsensGyrY, coarseAligned_xsensGyrZ,
                coarseAligned_xsensAccX, coarseAligned_xsensAccY, coarseAligned_xsensAccZ,
                xsensStacked_omega_copy,xsensStacked_alpha_copy)
    