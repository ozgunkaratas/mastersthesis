#import modules
import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np
import re
import regex
from decimal import Decimal
import time

#import funcs and classes
import readBags
import readHL
import timestampReader
import interpolate
import resampling
import correlate
import rosbagWriter
import camBagCreator
import imuBagCreator
import xsensBagCreator
import mergeBags
from classcomplementaryFilter import complementaryFilter
import cameraMatrix
import carryRigNode
from newKinematics import kinematics

if __name__ == '__main__':
    
    
    
######################################  PARSE RAW DATA
    print("make sure that this workspace has the raw xsens bag, raw accel, gyroscope and \n")
    print("raw magnetometer formatted text files.")
    print("you will also need the other txt files produced from the premilinary shell script\n")
    print("that you ran to process HL files.")
    print("confirm that all necessary files are in the workspace. \n")
    print("\n\n\n")
    input("Press any key to continue.")
    print("Extraction is beginning...\n")
    
    #parse the xsens bag
    (accX, accY, accZ,
     gyrX, gyrY, gyrZ,
     quatW, quatX, quatY, quatZ,
     linvelX, linvelY, linvelZ,
     utcTime, rosTime) = readBags.parseXsensBag()
    
    print("the xsens rosbag is parsed, IMU variables are obtained.\n")
    print("extracting the HL data... \n")
    
    #parse the HL txt file for epoch unity.
    readHL.parse()
    print("formatted txt files are created for further processing. \n")
    print("extracting HL IMU data... \n")
    (hlAccX, hlAccY, hlAccZ,
     hlAccNorm, hlGyrX, hlGyrY,
     hlGyrZ, hlGyrNorm, h_a_time_epoch,
     h_a_time_experiment, h_g_time_epoch, h_g_time_experiment) = readHL.extract()  
    print("HL IMU data extracted, moving on to process HL Cam and IMU timestamps...\n")
    
##################################################################################################################




###################################### CREATE CAMERA TIMESTAMPS   
    #UNCOMMENT THIS IF YOU WANT TO CREATE HL IMAGE TIMESTAMPS FROM SCRATCH
    #NOT NECESSARY BECAUSE I ALREADY CREATED THESE AND THE RAW IMAGES ARE TOO BIG TO 
    #UPLOAD TO THE REPO
    #create camfile    
    #timestampReader.createCamFile()
##################################################################################################################
    

    
    
###################################### ALIGN INTERNAL STREAMS OF THE HL
    #get cam duration, cam stamps, hl duration in float values for further processing
    (pvDuration, pvStamps_epoch,
     pvStamps_experiment,
     hlDurationAcc, hlDurationGyr) =timestampReader.initialize(h_a_time_epoch,h_g_time_epoch)
    
    #erase IMU data that continues to be recorded after cam is shut down.
    #this is a bug, even though the stop button is clicked, the IMU data is written to storage
    #until the images are not finished. this workaround cuts the stream at around the same timestamp
    #that the stop button is pressed.
    (h_a_time_experiment_aug,h_g_time_experiment_aug,
     hlDurationAcc, hlDurationGyr,
     endSampleAcc, endSampleGyr,
     hlGyrX, hlGyrY, hlGyrZ,
     hlAccX, hlAccY, hlAccZ)               = timestampReader.alignTimelines(pvDuration, hlDurationAcc,
                                             hlDurationGyr, h_a_time_experiment,h_g_time_experiment,
                                             hlGyrX, hlGyrY, hlGyrZ,
                                             hlAccX, hlAccY, hlAccZ)  


##################################################################################################################




######################################    RENAME IMU AXES
    #rename hl and xsens axes so that x,y,z corresponds to x,y,z on both sensors
    #done for easy readability, only cosmetic.                                                                            
    hlAccX,hlAccY,hlAccZ,hlGyrX,hlGyrY,hlGyrZ  = carryRigNode.changeAxes(hlAccX,hlAccY,hlAccZ,hlGyrX,hlGyrY,hlGyrZ)    


##################################################################################################################




######################################  ESTIMATE DYNAMIC FPS                                                                            
    #get dynamically changing fps data by counting the experiment durationg and dividing
    #it by overall samples. floor it for integer-type requirements but keep float values as well.
    (pvFPS, accFPS, gyrFPS, xsensFPS,
     avg_pvFPS, avg_accFPS, avg_gyrFPS,
     avg_xsensFPS)                        = timestampReader.estimateFPS(pvDuration,pvStamps_experiment, utcTime,
                                            h_a_time_experiment_aug, h_g_time_experiment_aug,
                                            endSampleAcc, endSampleGyr)                                                                                                                                                                     
    print("Timestamps, sampling rates, aligned experiment durations and timelines are extracted.\n")
    
##################################################################################################################


#####################################   INTERPOLATE AND UPSAMPLE
   
    #interpolate to uniform sampling because hl timestamps and corresponding readings are 
    #erratic. this is also necessary for kalibr syntax and other linear interp is just a good
    #way to get equidistant readings.
    (queryPointsAcc,queryPointsGyr,interp_hlAccX, interp_hlAccY, interp_hlAccZ,
     interp_hlGyrX, interp_hlGyrY, interp_hlGyrZ) = interpolate.interpolate(hlAccX, hlAccY, hlAccZ,
                                                                 hlGyrX, hlGyrY, hlGyrZ,
                                                                 h_a_time_experiment_aug, h_g_time_experiment_aug,
                                                                 )
                                                                
    #upsample hl accel to keep up with gyro
    #hl accel and hl gyro dont publish data at the same rate and for further post-pro
    #tasks this is necessary.
    interp_hlAccX,interp_hlAccY,interp_hlAccZ=resampling.hlPartialUpsampler(interp_hlAccX,interp_hlAccY,interp_hlAccZ,queryPointsGyr)
    
##################################################################################################################   
    

    
######################################  COARSE ALIGNMENT 
    #coarse align to deal with manual start-up and button press delays.
    #theres no way to know when the HL started the stream because it publishes data in wintime
    #and also is not connected to the internet so it lags behind 10 days from the utc time
    #it would be a good idea to use file creation times but this just cant be done because 
    #hl file creation doesnt make sense.
    
    #cut the excess samples by comparing total experiment duration and assign hl timestamp to
    #the first ros timestamp. 
    print("coarse alignment to compensate for manual start up and button press delays. \n")
    (initDiff, initCamDiff, erasedSamples, erasedSamplesCam, coarseAligned_gTime, coarseAligned_xTime,
           coarseAligned_hlGyrX, coarseAligned_hlGyrY, coarseAligned_hlGyrZ,
           coarseAligned_hlAccX, coarseAligned_hlAccY, coarseAligned_hlAccZ,
           coarseAligned_xsensGyrX, coarseAligned_xsensGyrY, coarseAligned_xsensGyrZ,
           coarseAligned_xsensAccX, coarseAligned_xsensAccY, coarseAligned_xsensAccZ,
           coarseAligned_qpG)                     = correlate.coarseAlign(rosTime, utcTime,
                                                                    pvStamps_epoch, pvStamps_experiment,
                                                                    h_a_time_epoch, h_g_time_epoch,
                                                                    h_a_time_experiment_aug, h_g_time_experiment_aug,
                                                                    gyrFPS, xsensFPS, pvFPS,
                                                                    interp_hlGyrX, interp_hlGyrY, interp_hlGyrZ,
                                                                    interp_hlAccX, interp_hlAccY, interp_hlAccZ,
                                                                    accX, accY, accZ, gyrX, gyrY, gyrZ,
                                                                    queryPointsGyr)    
    
    print("coarse experiment duration offset is %f seconds" %initDiff)                                                                                    

##################################################################################################################



###################################### UPSAMPLE TO XSENS LEVELS  
    #upsample hl to xsens levels, because different sample count signals cant go under
    #cross-corr.
    #downsampling xsens leads to loss of data, not recommended.
    print("resampling the HL data to match xsens sampling rates.\n")
    (upsampledHLstack,upCoarseAligned_hlAccX, upCoarseAligned_hlAccY, upCoarseAligned_hlAccZ,
      upCoarseAligned_hlGyrX, upCoarseAligned_hlGyrY, upCoarseAligned_hlGyrZ) = resampling.hlUpsampler(coarseAligned_hlAccX, coarseAligned_hlAccY, coarseAligned_hlAccZ,
                                                                                                       coarseAligned_hlGyrX, coarseAligned_hlGyrY, coarseAligned_hlGyrZ,
                                                                                                       coarseAligned_qpG, coarseAligned_xsensAccX)
                                                                                                       
                                                                                                       
##################################################################################################################




######################################  COMPLEMENTARY FILTER                                                                                                    
    #complementary filter for the stable region extrinsic rotation matrix calculation.
    #since we have no way of knowing if the streams are aligned or not, the stable region will give
    #good orientation estimation so that we use this to calculate the extrinsic rotation matrix.
    print("applying complementary filter to calculate the dynamically changing orientation of sensors.\n")
    #hl filter object
    hlFilter = complementaryFilter(9.81,0.025,0.1,0.05,0.005,0.005,0.005
                                      ,True,False, False, False, 1/xsensFPS,coarseAligned_xsensAccX, 1)
    (hlnewOrienta, hlnewOrientb, hlnewOrientc, hlnewOrientd,
     hlnew_wx, hlnew_wy, hlnew_wz,
     hlGravity_x, hlGravity_y, hlGravity_z)   = hlFilter.execute(coarseAligned_xsensAccX, coarseAligned_xsensAccY, coarseAligned_xsensAccZ, coarseAligned_xsensGyrX, coarseAligned_xsensGyrY,coarseAligned_xsensGyrZ,
                                                                                            upCoarseAligned_hlAccX, upCoarseAligned_hlAccY, upCoarseAligned_hlAccZ, upCoarseAligned_hlGyrX, upCoarseAligned_hlGyrY, upCoarseAligned_hlGyrZ)
                                                                                                              
    hlstackedMatrix = hlFilter.solveRotationMatrix(hlnewOrienta, hlnewOrientb, hlnewOrientc, hlnewOrientd, upCoarseAligned_hlAccX)                                                               
   
    #xsens filter object                                                     
    xsensFilter = complementaryFilter(9.81,0.01,0.2,0.1,0.01,0.01,0.01
                                      ,True,False, False, False, 1/xsensFPS,coarseAligned_xsensAccX, 0)
    
    (xsensnewOrienta, xsensnewOrientb, xsensnewOrientc, xsensnewOrientd,
     xsensnew_wx, xsensnew_wy, xsensnew_wz,
     xsensGravity_x, xsensGravity_y, xsensGravity_z)  =xsensFilter.execute(coarseAligned_xsensAccX, coarseAligned_xsensAccY, coarseAligned_xsensAccZ, coarseAligned_xsensGyrX, coarseAligned_xsensGyrY,coarseAligned_xsensGyrZ,
                                                                                               upCoarseAligned_hlAccX, upCoarseAligned_hlAccY, upCoarseAligned_hlAccZ, upCoarseAligned_hlGyrX, upCoarseAligned_hlGyrY, upCoarseAligned_hlGyrZ)
                                                                                
    xsensstackedMatrix  = xsensFilter.solveRotationMatrix(xsensnewOrienta, xsensnewOrientb, xsensnewOrientc, xsensnewOrientd, coarseAligned_xTime)                                                               
    
    
    
    #calculate R_x_hl rotation matrix that transfers xsens values to the hl values
    R_hl_xsens,avg_R_hl_xsens = xsensFilter.solveOrientation(xsensstackedMatrix, hlstackedMatrix,xsensFPS) 
    #align accel and gyro axes
    (coarseAligned_xsensGyrX, coarseAligned_xsensGyrY,
     coarseAligned_xsensGyrZ, coarseAligned_xsensAccX,
     coarseAligned_xsensAccY, coarseAligned_xsensAccZ,
     xsensStacked_omega_copy,xsensStacked_alpha_copy) = xsensFilter.XsensOnHL(coarseAligned_xsensGyrX, coarseAligned_xsensGyrY, coarseAligned_xsensGyrZ,
                                                                            coarseAligned_xsensAccX, coarseAligned_xsensAccY, coarseAligned_xsensAccZ, avg_R_hl_xsens)
    
                                                                              
                                                                              
##################################################################################################################




######################################  CROSS CORRELATION AND SPLINE CORRELATION
    print("moving on to discrete time cross-correlation. \n")
    (xdelay, ydelay, zdelay,
           xcorr_x, xcorr_y, xcorr_z,
           locsx, locsy, locsz, smallest, shift,
           xorrshiftx, xorrshifty, xorrshiftz,
           xorrshiftx_acc, xorrshifty_acc, xorrshiftz_acc,
           corrXsensStack)                =correlate.crossCorrelate(upCoarseAligned_hlGyrX, upCoarseAligned_hlGyrY, upCoarseAligned_hlGyrZ,
                                                                                             coarseAligned_xsensGyrX, coarseAligned_xsensGyrY, coarseAligned_xsensGyrZ,
                                                                                             coarseAligned_xsensAccX, coarseAligned_xsensAccY, coarseAligned_xsensAccZ,
                                                                                             xsensFPS)
                                         
    
    print("cross-correlation complete, the following temporal calibration\n")
    print("values have been extracted. \n")
    print("X-Axis: %f seconds" %xdelay)
    print("Y-Axis: %f seconds" %ydelay)
    print("Z-Axis: %f seconds" %zdelay)
    
    print("refining the cross-correlation by calculating spline correlation.\n")
    (delay_secs_x, delay_secs_y, delay_secs_z,
            splineStackedXsens)                =correlate.splineCorrelate(xcorr_x,xcorr_y, xcorr_z,
                                                        locsx, locsy, locsz, smallest,
                                                        xsensFPS,
                                                        coarseAligned_xsensGyrX, coarseAligned_xsensGyrY, coarseAligned_xsensGyrZ,
                                                        coarseAligned_xsensAccX,coarseAligned_xsensAccY,coarseAligned_xsensAccZ)
    print("following temporal calibration values have been obtained.\n")
    print("X-Axis: %f seconds" %delay_secs_x)
    print("Y-Axis: %f seconds" %delay_secs_y)
    print("Z-Axis: %f seconds" %delay_secs_z)

    

##################################################################################################################


######################################  POST PROCESS CAMERA ROTATION MATIRX
    #parse camera data
    (stackedCameraMatrix, stackedCameraTrans,
           camRotateFromMatrix, camQuat, camEul, camRotVec,camtime) = cameraMatrix.extractCameraMatrix()
    
    #use spline correlation results to align the camera timeline
    (alignedStackedCameraMatrix, alignedStackedCameraTrans,alignedCamTime) = cameraMatrix.alignCamera(stackedCameraMatrix, stackedCameraTrans,
                                                                                       erasedSamplesCam, pvFPS,
                                                                                       delay_secs_x, initDiff, pvStamps_epoch[0], h_g_time_epoch[0],camtime)
    
    #upsample position values to match xsens sampling rate
    (up_NED_pv_X, up_NED_pv_Y, up_NED_pv_Z, upNEDTime) = cameraMatrix.upsampleTranslation(alignedStackedCameraTrans,utcTime,alignedCamTime)
    
    #save cam output to matfile
    cameraMatrix.saveMatlab(up_NED_pv_X, up_NED_pv_Y, up_NED_pv_Z, upNEDTime)
    
##################################################################################################################



######################################  RIGID BODY KINEMATICS TO VERIFY LEVER-ARM EFFECT
    
    #initiate kinematics object with upsampled and temporally calibrated HL values
    kinem = kinematics(upCoarseAligned_hlAccX, upCoarseAligned_hlAccY, upCoarseAligned_hlAccZ,
                       upCoarseAligned_hlGyrX,upCoarseAligned_hlGyrY, upCoarseAligned_hlGyrZ)
    
    #stack accel timelines so that kinem equations can be done on them
    (hlAccelStack, hlGyroStack, xsensAccelStack, xsensGyroStack) = kinem.StackStreams(splineStackedXsens)
    #get orient matrix of sensors
    world_R_xsens,world_R_hl = kinem.createOrientMatrix(xsensnewOrienta,xsensnewOrientb,xsensnewOrientc,xsensnewOrientd,
                       hlnewOrienta,hlnewOrientb,hlnewOrientc,hlnewOrientd)
    
    #calculate true hl values from xsens by rotating reference frame equations
    true_hl_accel_fromXSENS = kinem.trueHLfromXsens(xsensAccelStack, xsensGyroStack,
                                                    avg_R_hl_xsens,utcTime)
    
    true_xsens_accel_fromHL = kinem.trueXsensfromHL(hlAccelStack, hlGyroStack,
                                                    avg_R_hl_xsens,utcTime)
    #compensate for gravity
    gravComp_true_hl_accel_fromXSENS,gravComp_hlAccelStack, gravComp_xsensAccelStack = kinem.gravityCompensation(true_hl_accel_fromXSENS,
                                                                                                                  hlAccelStack,
##################################################################################################################



                                                                                                             xsensAccelStack,
                                                                                                                  world_R_xsens, world_R_hl)
######################################  PLOT GRAPHS    
    #plot accel graphs
    kinem.plotter(coarseAligned_xTime,
                    gravComp_true_hl_accel_fromXSENS, gravComp_hlAccelStack, gravComp_xsensAccelStack,
                    true_hl_accel_fromXSENS,true_xsens_accel_fromHL,
                    hlAccelStack, xsensAccelStack)
    
    print("saving plots to current working directory...\n")
    correlate.plotUtils(coarseAligned_xTime,coarseAligned_qpG,
              upCoarseAligned_hlGyrX, upCoarseAligned_hlGyrY, upCoarseAligned_hlGyrZ,
              coarseAligned_hlGyrX, coarseAligned_hlGyrY, coarseAligned_hlGyrZ,
              xorrshiftx, xorrshifty, xorrshiftz,
              splineStackedXsens,
              coarseAligned_xsensGyrX, coarseAligned_xsensGyrY, coarseAligned_xsensGyrZ)
##################################################################################################################



######################################  CREATE ROSBAGS

#uncomment if you want to produce rosbags from post-processed imu files and hl images
#this wont work because the HL images are not included in the repo. 



    # print("creating matrices for rosbags... \n")
    
    # rosbagWriter.writeMatrices(coarseAligned_xTime, rosTime, coarseAligned_qpG, initDiff,
    #           coarseAligned_hlGyrX, coarseAligned_hlGyrY, coarseAligned_hlGyrZ,
    #           coarseAligned_hlAccX, coarseAligned_hlAccY, coarseAligned_hlAccZ,
    #           coarseAligned_xsensAccX, coarseAligned_xsensAccY, coarseAligned_xsensAccZ, 
    #           coarseAligned_xsensGyrX, coarseAligned_xsensGyrY, coarseAligned_xsensGyrZ)
    

    # adjusted_camtime = rosbagWriter.camAlignment(initCamDiff, rosTime, pvStamps_experiment, erasedSamplesCam)
 
    # print("creating the HL camera Rosbag... \n")
    # print("this may take a few seconds. \n")
    
    # camBagCreator.make(erasedSamplesCam, adjusted_camtime)    
    # print("creating new IMU bags... \n")
    # imuBagCreator.make()
    # xsensBagCreator.make()
    
    # print("bags are merged under merged.bag. \n")
    # mergeBags.merge()