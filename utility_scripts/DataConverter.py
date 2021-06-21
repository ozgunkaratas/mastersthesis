# -*- coding: utf-8 -*-

import numpy as np

import scipy

import scipy.io

from matplotlib import pyplot

#from HexapodController import HexapodController

import os

class DataConverter:
    """
    Created on Tue Mar 23 10:18:00 2021    
    @author: Nicolai Weddig
    This class converts .mat files (from matlab) into hexapod controller readable
    files. Furthermore, it is also possible to convert output from hexapod
    recordings into .txt files, which are spaced in such a way 
    that it is easy to read them into matlab. 
    The data is furthermore fit to the hexapod specifications. 
    Usually, these are then given to the hexapod controller
    class object in order to move the hexapod.
    Furthermore, it provides functions to load the stored movement .txt
    data from the hexapod, and convenience functions 
    to extract/change data from it. 
    
    ...
    
    Attributes and constants
    ------------------------
    TIME_WAYPOINTS : numeric list
        Test list. Can be used to test waypoint generation functions
    POS_WAYPOINTS : numeric list
        Test list. Can be used to test waypoint generation functions
    ATT_WAYPOINTS : numeric list
        Test list. Can be used to test waypoint generation functions
    MAT_FILE_NAME : string
        This string contains the name of the matlab .mat file which should
        be loaded. Is saved so that other objects can access it. 
    Methods
    -------
    def create_rot_mat(self, roll, pitch, yaw)
        Create a nx3x3 rotation matrix array, where n is the number of 
        available angles. Roll, pitch and yaw should be n x 1
    
    def get_abs_max_value(self, list1)
        Get the absolute max value from a list 
        (the element with the largest overall value, disregarding sign)    
    
    def get_scale_factor(self, max_value, high_limit, low_limit)
        Calculate scale factor based on given limits.
        This function calculates the scale factor which is needed to 
        scale the attitudes, velocities and positions to an acceptable level.
        This is done by assuming that the high and low limit are equal in size.
        Then, a scale factor is computed by dividing the (absolute) mean limit 
        value by the max_vale 
    
    def get_lim_values(self, list1, high_limit, low_limit)
        This function cuts off list values which are larger than high_limit, or
        smaller than low_limit. All values which are higher or lower are set to
        high_limit and low_limit, respectively.
        
    def convert_matlab_waypoints_to_pi_waypoints(self,
            mat_name, scaling_flag_Vel = 0, 
            scaling_flag_Att = 0, 
            scaling_flag_Pos = 0, 
            conv_to_mm = 1)
        This function converts data from a .mat file (in waypoint format) to
        position and attitude values which are usable by the hexapod controller.

    def load_position_file(self, file_name)
        This function loads a .txt file which contains recorded position and
        attitude data from a hexapod.
        
    def isfloat(self, value)
        Check if a given value can be converted from string to float
        
    def calculate_fourier_transforms(self, matrix)
        Take a matrix of the form [n x m], where m is the dimension along time,
        and calculate the fourier transform entries of those values. 
        Returns the transformed points, the corresponding frequencies
        (if the first row of the given matrix is time), as well
        as the norm of these values, and the phase values.

    def wb_to_rpy_dot(self, roll, pitch, yaw, wbx, wby, wbz)
        Transforms angular rates, defined in the body frame, to the 
        equivalent n-frame representation (roll-dot, pitch-dot, yaw-dot)

    def rpy_dot_to_wb(self, roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot)
        The reverse transform of the above function, as defined by Titterton
        (2004) (Strapdown Inertial Navigation Technology)

    def plot_waypoints(self, time_waypoints, pos_waypoints, attitude_waypoints)
        Convenience plotting function. Plots pos waypoints and attitude waypoints
        vs. time. Is called by the convert_matlab_waypoints_to_pi_waypoints
        function, to present to the user how the waypoints were transformed

    def check_for_length(self, variable)
        This method is used to to check whether the given variable
        (usually an integer/float) needs space padding so that individual
        rows are equally spaced in the written .txt output files. 
        The value (19 spaces) is hardcoded inside the function!
        Returns an integer, which indicates how many spaces should be appended
        Note that the returned integer is never 0, since there needs to be
        some amount of spacing in order to even reload the data from the
        .txt file.
    
    def save_data(self, matrix, file_name, header = "")
        This method saves the given data to file. Matrix can be any 
        n x m matrix, while header is written to the file at the beginning.
        file_name specifies the location where the data should be written to.

    def interpolate_to_array(self, time_array, old_timepoints, data)
        This function takes two arrays, time array and old_timepoints, 
        as well as a data series (which should be given as n x m, where
        m is along the axis of the time series). 
        If the arrays are not given in the correct format, the method tries
        to reshape the arrays and matrices so that they fit. Afterwards,
        it interpolates the data points by using the time_array as a reference.
        old_timepoints are compared to the time_array entries, the closest
        matching old_timepoints are chosen to calculate the slopes, and
        finally the new time_points and data_points are generated via
        a simple rectangular interpolation scheme.

    def get_time_derivative(self, time, values)
        Calculate the time derivative of the given values. Values should
        be a list/numpy array (n x 1). Returns the derivative based on
        a simple first degree euler numerical differentiation. the new
        timepoints are the mean values of the old timepoints, since it is
        assumed that the derivatives are positioned at those points. 
        This function can be used with interpolate_to_array to calculate
        "correct" derivatives of position, velocity and acceleration 
        arrays with matching timestamps. 

    def load_IMU_file(self, file_name, indices = None)
        Loads the IMU file from disk. indices is a 2x1 list, which 
        indicates the window of loaded data. If None, all data is returned

    def load_position_files(self, file_names
        , fourier_flag = 0, conv_to_rads = 0, indices = None)
        loads multiple position files. Calls load_position_file inside
        the method. file_names should be a simple list of file names, while
        indices is a [2 x 1] list, i.e. it is not yet possible to 

    def  load_position_file(self, file_name
        , fourier_flag = 0, conv_to_rads = 0, indices = None)    
        Loads a singular position file, based on the file name. 
        conv_to rads = 1 means that the attitude is converted to degrees
        indices is a window

    def generate_file_names(self, *args, nested_return = False)
        Generate file names, based on the given *args arguments.
        Essentially, this function creates all possible file names + 
        folder structures for the given args arguments. I.e. if the
        lists l1 = ["bnd_cond_A/", "bnd_cond_B/"], l2 = ["Test1_","Test2_"], 
        l3 = ["Val_x.txt", "Val_y.txt"] is passed to this file as *args, 
        this function will return the following strings, in one list 
        (but not necessarily in the same order!)
        returns = ["bnd_cond_A/Test1_Val_x.txt"
                  ,"bnd_cond_A/Test2_Val_x.txt"
                  ,"bnd_cond_A/Test1_Val_y.txt"
                  ,"bnd_cond_A/Test2_Val_y.txt"
                  ,"bnd_cond_B/Test1_Val_x.txt"
                  ,"bnd_cond_B/Test2_Val_x.txt"
                  ,"bnd_cond_B/Test1_Val_y.txt"
                  ,"bnd_cond_B/Test2_Val_y.txt"]
        It will not check for invalid file names. This has to be done in
        an extra loop. FInally, if nested return = True, The first list
        (in this case bnd_cond_A/B) is used as a reference for how lists
        should be split up.
    """    
    MAT_NAME = "SWING.mat"
    #MAT_NAME = "tripleNickRoll89.mat"
    #MAT_NAME = "circleTrajectory.mat" 
    # Time waypoints should be set in a way that the hexapod can still move 
    # fast enough. The cycle time is calculated based on time_waypoints
    TIME_WAYPOINTS = [
                0, 
                8, 
                10, 
                15, 
                20, 
                25, 
                30,
                ] 
    # POS_WAYPOINTS describes how the position changes for specific timeframes. 
    # These waypoints are interpolated in the called MOV waypoint function
    POS_WAYPOINTS = [
                [0, 0, 0],
                [10, 0, 0],
                [0, 10, 0],
                [0, 0, 10],
                [-10, -10, 0],
                [0, 0, 0],
                ]
    # ATT_WAYPOINTS describes how the attitude changes for specific timeframes.
    # These are interpolated in a simple way.
    ATT_WAYPOINTS = [
                [0, 0, 0],
                [5, 0, 0],
                [0, -5, 0],
                [5, 5, -2],
                [0, 3, 5],
                [0, 0, 0],
                ]

    def __init__(self, mat_file_name = "Test.mat", hexController = None):
        global MAT_NAME 
        self.MAT_NAME = mat_file_name
        self.mat_file_name = mat_file_name
        # if(hexController is None):
        #     self.hexController = HexapodController()
        #     self.hexController.LOW_LIMITS = (
        #         np.array([-50, -50, -25, -15, -15, -30])*0.25)
        #     self.hexController.HIGH_LIMITS = (
        #         np.array([50, 50, 25, 15, 15, 30])*0.25)
        #     self.hexController.MAX_VEL_VALUE = 1
        #     self.hexController.MAX_ACC_VALUE = 1
        #     self.hexController.MAX_MOV_DISTANCE_VAL = 4.0 
        #     self.hexController.DEF_CYCLE_TIME = 50.0
        #     self.hexController.MIN_CYCLE_TIME = 10.0
        # else:
        #     self.hexController = hexController
        #     pass
        # pass
 
    def create_rot_mat(self, roll, pitch, yaw):
        """Calculate rotation matrix array (n x 3 x 3) based on euler angles
        
        This function assumes that all angles are in format (n x 1)
        Furthermore, n is assumed to be equal for each euler angle array
        Returns (n x 3 x 3) array
        
        Parameters
        ---------
        roll  : numpy array (n x 1), roll angles
        pitch : numpy array (n x 1), pitch angles
        yaw   : numpy array (n x 1), yaw angles
        
        Returns
        -------
        rot_mat : numpy array (n x 3 x 3), rotation matrix
        """
        size_roll = roll.shape
        # Check where the major axes of the roll axis is. 
        # Base format of following matrices on this
        major_axis = np.max(size_roll)
    #    if(len(size_roll) > 1):
    #        if(size_roll[0] > 1):
    #            major_axis = size_roll[0]
    #        else:
    #            major_axis = size_roll[1]
        roll = roll.reshape(major_axis)
        pitch = pitch.reshape(major_axis)
        yaw = yaw.reshape(major_axis)
        # Create rotation matrix, based on major axis of roll angle
        size_roll = roll.shape
        rot_mat = np.zeros((size_roll[0], 3, 3))
        # Calculate arrays of sine/cosine terms. 
        # Simplifies matrix building blocks
        sine_roll = np.sin(roll)
        sine_pitch = np.sin(pitch)
        sine_yaw = np.sin(yaw)
        cosine_roll = np.cos(roll)
        cosine_pitch = np.cos(pitch)
        cosine_yaw = np.cos(yaw)
        # Compute individual matrix elements
        c_11 = cosine_pitch*cosine_yaw
        c_12 = -cosine_roll*sine_yaw + sine_roll*sine_pitch*cosine_yaw
        c_13 = sine_roll *sine_yaw + cosine_roll*sine_pitch*cosine_yaw
        c_21 = cosine_pitch*sine_yaw
        c_22 = cosine_roll *cosine_yaw + sine_roll*sine_pitch*sine_yaw
        c_23 = -sine_roll*sine_yaw + cosine_roll*sine_pitch*sine_yaw
        c_31 = -sine_pitch
        c_32 = sine_roll*cosine_pitch
        c_33 = cosine_roll*cosine_pitch
        # Insert individual rotation elements into matrix
        rot_mat[:,0,0] = c_11
        rot_mat[:,0,1] = c_12
        rot_mat[:,0,2] = c_13
        rot_mat[:,1,0] = c_21
        rot_mat[:,1,1] = c_22
        rot_mat[:,1,2] = c_23
        rot_mat[:,2,0] = c_31
        rot_mat[:,2,1] = c_32
        rot_mat[:,2,2] = c_33
        return rot_mat
    
    # This function returns the absolute max value of a list
    def get_abs_max_value(self, list1):
        """Returns the absolute max value of the given list/array
        
        This function only works for lists of shape (n x 1)
        
        Parameters
        ---------
        list1 : list/numpy array
            List/numpy array which contains numerical values

        Returns
        -------
        max_list1 : float
            absolute max value of the list
        """
        max_list1 = np.max(list1)
        min_list1 = np.min(list1)
        max_list1 = np.max([max_list1, np.abs(min_list1)])
        return max_list1
    
    # This function returns the correct scale factor, based on the 
    # max absolute! value of the function, as well as the specified high 
    # and low limits of the hexapod. It is assumed that the average of the
    # low and high limits correspond to the maximum value the hexapod can 
    # move (in the positive and negative direction, as an absolute value)
    # Therefore, the whole trajectory is then scaled to fit this theoretical
    # limit
    def get_scale_factor(self, max_value, high_limit, low_limit):
        """Returns a scale factor, scales the max_value to limit bounds
        
        This function returns a scale factor which 
        scales the max_value (scalar) to the mean value of the absolute
        high and low limits. Notice how only absolute values of high
        and low limit are taken. Furthermore, it is assumed that the limits
        are roughly equal in size.
        
        Parameters
        ---------
        max_value : float
            The max value of a list
        high_limit : float
            High limit of the hexapod parameter
        low_limit : float
            Low limit of the hexapod parameter

        Returns
        -------
        scale_factor : float
            Scale factor, which should scale max_value so that it is 
            between high and low_limit. Only works if high/low limit are 
            roughly equal. Otherwise, both max_value and min_value need
            to be estimated (and then the list needs to be scaled via 
                             indexing)
        """
        if(max_value == 0):
            scale_factor = 1
        else:            
            mean_limit = (np.abs(low_limit)+np.abs(high_limit))/2
            scale_factor = mean_limit / max_value
        return scale_factor


    # This function cuts off overhanging values of a list, based on the 
    # high and low limits specified. If a values is larger than the high limit,
    # it is set to the high limit.
    # If a value is smaller than the low limit, it is set to the low limit
    def get_lim_values(self, list1, high_limit, low_limit):
        """This function applies high_limit and low_limit to a list
        
        This function returns a list whose values do not exceed high
        limit or low limit. No scaling takes place. Instead, all values
        which are larger than high limit are replaced with high limit.
        The same is done with values which exceed low limit.
        
        Parameters
        ---------
        list1 : list / numpy array
            List on which this operation is performed
        high_limit : float
            The largest permissible value in the list
        low_limit : float
            The smallest permissible value in the list

        Returns
        -------
        lim_list1
            List who are "cut off", i.e. whose values are not exceeding the
            enforced limits
        """
        lim_list1 = list1
        lim_list1[list1 > high_limit] = high_limit
        lim_list1[list1 < low_limit] = low_limit
        return lim_list1
    

    def wb_to_rpy_dot(self, roll, pitch, yaw, wbx, wby, wbz):
        """Convert rpy_dot to body frame angular rates
        
        This method calculates body frame angular rates on the basis of 
        numerically calculated euler angular rates. 
        It should be noted that to reach best results, the euler angles
        and derivatives of euler angles should have the same timestamps.
        From Titterton 2004, page 42
        
        Parameters
        ---------
        wbx : numpy array [n x 1] 
            roll angles
        wby: numpy array [n x 1] 
            pitch angles
        wbz : numpy array [n x 1]
            yaw angles
        roll_dot : numpy array [n x 1]
            roll angular rates
        pitch_dot : numpy array [n x 1]
            pitch angular rates
        yaw_dot : numpy array [n x 1]
            yaw angular rates
        Returns
        -------
        roll_dot : numpy array [n x 1]
            body frame angular rates, x
        pitch_dot : numpy array [n x 1]
            body frame angular rates, y
        yaw_dot : numpy array [n x 1]
            body frame angular rates, z
        """    
        roll_dot = wby * np.sin(roll) + wbz * np.cos(roll) + wbx
        pitch_dot = wby * np.cos(roll) - wbz * np.sin(roll)
        yaw_dot = np.zeros(roll_dot.shape)
        c_pitch = np.cos(pitch)
        valid_entries = not (c_pitch != 0)
        sec_pitch = 1/c_pitch[valid_entries]
        wby_y = wby[valid_entries]
        wbz_y = wbz[valid_entries]
        roll_y = roll[valid_entries]
        yaw_dot[valid_entries] = (wby_y * np.sin(roll_y) + wbz_y * np.cos(roll_y)) * sec_pitch
        return roll_dot, pitch_dot, yaw_dot
    
    # Convert euler angular rates to body frame angular rates
    # From Titterton2004, inverse formula
    def rpy_dot_to_wb_dot(self, roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot):
        """Convert rpy_dot to body frame angular rates
        
        This method calculates body frame angular rates on the basis of 
        numerically calculated euler angular rates. 
        It should be noted that to reach best results, the euler angles
        and derivatives of euler angles should have the same timestamps
        
        Parameters
        ---------
        roll : numpy array [n x 1] 
            roll angles
        pitch: numpy array [n x 1] 
            pitch angles
        yaw : numpy array [n x 1] 
            yaw angles
        roll_dot : numpy array [n x 1]
            roll angular rates
        pitch_dot : numpy array [n x 1]
            pitch angular rates
        yaw_dot : numpy array [n x 1]
            yaw angular rates
        Returns
        -------
        wbx : numpy array [n x 1]
            body frame angular rates, x
        wby : numpy array [n x 1]
            body frame angular rates, y
        wbz : numpy array [n x 1]
            body frame angular rates, z
        """    
        # 1) Get corresponding rotation matrices, C3 and C2
        # C3 = rotation around roll
        # C2 = rotation around pitch
        size_roll = np.max(roll.shape)
        C2 = np.zeros((size_roll, 3, 3))
        C3 = np.zeros((size_roll, 3, 3))
        C2[:,0,0] = np.cos(pitch)
        C2[:,0,1] = np.zeros(size_roll)
        C2[:,0,2] = -np.sin(pitch)
        C2[:,1,0] = np.zeros(size_roll)
        C2[:,1,1] = np.ones(size_roll)
        C2[:,1,2] = np.zeros(size_roll)
        C2[:,2,0] = np.sin(pitch)
        C2[:,2,1] = np.zeros(size_roll)
        C2[:,2,2] = np.cos(pitch)
        
        C3[:,0,0] = np.ones(size_roll)
        C3[:,0,1] = np.zeros(size_roll)
        C3[:,0,2] = np.zeros(size_roll)
        C3[:,1,0] = np.zeros(size_roll)
        C3[:,1,1] = np.cos(roll)
        C3[:,1,2] = np.sin(roll)
        C3[:,2,0] = np.zeros(size_roll)
        C3[:,2,1] = -np.sin(roll)
        C3[:,2,2] = np.cos(roll)
        roll_array = np.zeros((size_roll, 3))
        pitch_array = np.zeros((size_roll, 3))
        yaw_array = np.zeros((size_roll, 3))
        roll_array[:,0] = roll_dot
        pitch_array[:,1] = pitch_dot
        yaw_array[:,2] = yaw_dot
        wb = np.zeros((size_roll, 3))
        for i in range(yaw_array.shape[0]):
            wb[i,:] = (roll_array[i,:] + C3[i,:,:] @ pitch_array[i,:] 
                       + C3[i,:,:] @ C2[i,:,:] @ yaw_array[i,:])
        wbx = np.zeros(size_roll)
        wbx[:] = wb[:,0]
        wby = np.zeros(size_roll)
        wby[:] = wb[:,1]
        wbz = np.zeros(size_roll)
        wbz[:] = wb[:,2]
        return wbx, wby, wbz
    # This function takes waypoints which were generated in matlab, 
    # and converts them into hexapod readable waypoints. 
    # Essentially, matlab based waypoints are giving 
    # velocity and attitude values, while hexapod based waypoints need 
    # to be in the form of "n-frame" position values and attitude values
    #
    # Therefore, the b-frame velocity values of matlab-based waypoints need 
    # to be converted.
    # Furthermore, scaling/cutoffs are also implemented here, in order 
    # to keep the resulting trajectory inside the movement limits of the hexapod
    
    def convert_matlab_waypoints_to_pi_waypoints(self,
            mat_name, scaling_flag_Vel = 0, 
            scaling_flag_Att = 0, 
            scaling_flag_Pos = 0, 
            conv_to_mm = 1, buffered = 1):
        """Convert matlab based waypoints to hexapod readable format
        
        This function reads in a .mat file, based on the name, 
        and then performs scaling and integration operations on the
        velocity and attitude values in order to get position and 
        attitude values which are applicable to the hexapod. 
        Depending on the flag values, different operations are 
        performed on the data before returning the waypoints 
        
        Parameters
        ---------
        mat_name : string
            Name of the .mat file
        scaling_flag_Vel : int
            Flag which determines whether velocity should be scaled (0)
            or cut off (1) (based on hexapod limits)
        scaling_flag_Att : int
            Flag which determines whether attitude should be scaled (0)
            or cut off (1) (based on hexapod limits)
        scaling_flag_Pos : int
            Flag which determines whether position should be scaled (0)
            or cut off (1) (based on hexapod limits)
        conv_to_mm : int
            Flag which checks whether conversion from m to mm should
            be performed. Default (1) means conversion
        buffered : int
            Flag. If 1, interpolate data so that all points have an equal distance
            to each other. Does not change the overall number of waypoints.
            Is important if the buffered-movement function is used, since 
            that function assumes that the first given cycle time is equal to
            all following cycle times!
        Returns
        -------
        time_waypoints,
            Scaled/cut off time waypoints
        pos : n x 3 numpy array
            Scaled/cut off position waypoints
        attitude
            Scaled/cut off attitude waypoints
        """
        global MAT_NAME
        MAT_NAME = mat_name
        # mat_name can be relative if in the same directory
        # TODO: Set folder structure to ease loading of mat files
        mat = scipy.io.loadmat(mat_name)
        # structure is: 
        matlab_structure = list(mat.items())
        time_waypoints = matlab_structure[3][1].reshape(
            np.max(matlab_structure[3][1].shape))
        # Convert from m to mm
        velocity = np.array(matlab_structure[4][1])
        if(conv_to_mm):
            print("Checked")
            velocity = velocity * 1000
        # Convert angles into 2pi range 
        # (waypoint structure has angles > 2pi to prevent wrap-around) 
        # Clipping them to 2pi range makes scaling of angles easier
        # In general, however, it is not recommended to load in trajectories
        # with looping/circling motions since the hexapod cannot move 
        # instantaneously from one position to another position
        roll = np.array(matlab_structure[7][1])
        pitch = np.array(matlab_structure[6][1])
        yaw = np.array(matlab_structure[5][1])
        # If all points should be equally spaced (for buffered move function),
        # perform this step here. Otherwise, skip this step and take 
        # waypoints as they are.
        if(buffered == 1):
            # average time change per waypoint index
            dtime = ((time_waypoints[-1] - time_waypoints[0]) 
                              / len(time_waypoints))
            # Equally spaced time waypoints
            time_wp_spaced = (np.cumsum(np.ones(len(time_waypoints))) 
                              * dtime)
            # Containers for equally spaced rpy, velocity
            vel_spaced = np.zeros((len(time_waypoints), 3))
            roll_spaced = np.zeros(len(time_waypoints))
            pitch_spaced = np.zeros(len(time_waypoints))
            yaw_spaced = np.zeros(len(time_waypoints))
            # Go through equally spaced time waypoints
            for i in range(len(time_waypoints)):
                # Find time waypoints which is closest to equally spaced point
                index = np.argmin(np.abs(time_wp_spaced[i] - time_waypoints))
                # Calculate the distance between the waypoint and the current
                # equally spaced timepoint. If this distance is negative, 
                # the scaled waypoint lies in the past --> subtract from 
                # current roll value indicated by index. Otherwise, add 
                time_distance = time_wp_spaced[i] - time_waypoints[index]
                cur_roll = roll[index]
                cur_pitch = pitch[index]
                cur_yaw = yaw[index]
                cur_vel = velocity[index,:]
                # Since euler method is used to calculate numerical derivatives
                # /slopes, move index one back if it would exceed number of 
                # waypoints. Since this only affects the calculation of the
                # slope, it should be fine... 
                if(index == len(time_waypoints) - 1):
                    index -= 1
                # Get distance between timepoints. Note that if index was
                # already at the end of the list, the difference can still be
                # computed (Although the distance between index-1 and index is
                # used.)
                dt = time_waypoints[index+1] - time_waypoints[index]
                # Calculate slopes  (euler method, numerical derivative)
                # for roll, pitch, yaw and velocity
                m_roll = (roll[index+1] - roll[index]) / dt
                m_pitch = (pitch[index+1] - pitch[index]) / dt
                m_yaw = (yaw[index+1] - yaw[index]) / dt
                m_velocity = (velocity[index+1] - velocity[index]) / dt
                # These are the equally spaced values. cur_values are offset
                # by the time_distance * slope of variable at that point in time
                # If time_distance is positive, this means that the equally spaced
                # time_waypoint is larger than the currently viewed timepoint.
                # If time_distance is negative, the equally spaced waypoint is 
                # smaller than the currently viewed timepoint.
                # Therefore, the scaled values might move just a tiny bit.
                roll_spaced[i] = time_distance * m_roll + cur_roll
                pitch_spaced[i] = time_distance * m_pitch + cur_pitch
                yaw_spaced[i] = time_distance * m_yaw + cur_yaw
                vel_spaced[i] = time_distance * m_velocity + cur_vel
            fig = pyplot.figure()
            pyplot.xlabel("time (s)")
            pyplot.ylabel("Attitude (degrees)")
            pyplot.title("Roll: Old vs. equally spaced")
            hr = pyplot.plot(time_waypoints, roll, "-", color = "red")
            hrS = pyplot.plot(time_wp_spaced, roll_spaced, "--", color = "red")
            pyplot.legend([hr, hrS],["old roll", "spaced roll"])
            pyplot.show()
            pyplot.figure()
            pyplot.title("Pitch: Old vs. equally spaced")
            pyplot.xlabel("time (s)")
            pyplot.ylabel("Attitude (degrees)")
            hp = pyplot.plot(time_waypoints, pitch, "-", color = "blue")
            hpS = pyplot.plot(time_wp_spaced, pitch_spaced, "--", color = "blue")
            pyplot.legend([hp, hpS],["old pitch", "spaced pitch"])
            pyplot.show()
            pyplot.figure()
            pyplot.title("Yaw: Old vs. equally spaced")
            pyplot.xlabel("time (s)")
            pyplot.ylabel("Attitude (degrees)")
            hy = pyplot.plot(time_waypoints, yaw, "-", color = "green")
            hyS = pyplot.plot(time_wp_spaced, yaw_spaced, "--", color = "green")
            pyplot.legend([hy, hyS],["old yaw", "spaced yaw"])
            pyplot.show()
            pyplot.figure()
            pyplot.title("Velocity: Old vs. equally spaced")
            pyplot.xlabel("time (s)")
            pyplot.ylabel("Velocity (mm/s)")
            hx = pyplot.plot(time_waypoints, velocity[:,0],"-", color = "red")
            hxS = pyplot.plot(time_wp_spaced, vel_spaced[:,0],"--", color = "red")
            hy = pyplot.plot(time_waypoints, velocity[:,1],"-", color = "blue")
            hyS = pyplot.plot(time_wp_spaced, vel_spaced[:,1],"--", color = "blue")
            hz = pyplot.plot(time_waypoints, velocity[:,2],"-", color = "green")
            hzS = pyplot.plot(time_wp_spaced, vel_spaced[:,2],"--", color = "green")
            pyplot.legend([hx, hy, hz, hxS, hyS, hzS]
                         ,["Old vx", "Old vy", "Old vz"
                          ,"spaced vx", "spaced vy", "spaced vz"])
            pyplot.show()
            time_waypoints = time_wp_spaced
            roll = roll_spaced
            pitch = pitch_spaced
            yaw = yaw_spaced
            velocity = vel_spaced
        print("Max deviation!")
        print(np.max(np.abs(np.diff(np.diff(time_wp_spaced)))))
        yaw = np.mod(yaw, 2*np.pi)
        yaw[yaw > np.pi] = yaw[yaw > np.pi] - 2*np.pi
        pitch = np.mod(pitch, 2*np.pi)
        pitch[pitch > np.pi] = pitch[pitch > np.pi] - 2*np.pi
        roll = np.mod(roll, 2*np.pi)
        roll[roll > np.pi] = roll[roll > np.pi] - 2*np.pi
        # scaling flag = 0 --> Scale based on max value. 
        # Scaling flag = 1 --> Cut off at max val
        # Regardless of that, cut off position if position is above max
        if(scaling_flag_Att == 0):
            # First, get max absolute orientation values. 
            # Since a scaling factor is calculated, 
            # absolute values are needed here
            max_roll = self.get_abs_max_value(roll)
            max_pitch = self.get_abs_max_value(pitch)
            max_yaw = self.get_abs_max_value(yaw)
            print("Maximum roll, pitch, yaw angles: ["
                  +str(max_roll)+", "
                  +str(max_pitch)+", "
                  +str(max_yaw)+"]")
            # If any of the orientation values is 0, scaling factor is equal to 1
            # Otherwise, a division by zero would occur
            if(max_roll == 0):
                roll_scale_factor = 1
            else:            
                roll_scale_factor = self.get_scale_factor(
                      max_roll
                    , self.hexController.LOW_LIMITS[3]
                    , self.hexController.HIGH_LIMITS[3]) / 180 * np.pi
            if(max_pitch == 0):
                pitch_scale_factor = 1
            else:
                pitch_scale_factor = self.get_scale_factor(
                     max_pitch
                   , self.hexController.LOW_LIMITS[4]
                   , self.hexController.HIGH_LIMITS[4]) / 180 * np.pi
            if(max_yaw == 0):
                yaw_scale_factor = 1
            else:
                yaw_scale_factor = self.get_scale_factor(
                      max_yaw
                    , self.hexController.LOW_LIMITS[5]
                    , self.hexController.HIGH_LIMITS[5]) / 180 * np.pi
            # Scale attitude values, afterwards perform velocity transformations
            # Maybe it is better to transform velocities
            # based on original attitude values and scale attitude values 
            # afterwards to still allow more dynamic movement
            print("Euler angle scale factors: ["
                  +str(roll_scale_factor)+", "
                  +str(pitch_scale_factor)+", "
                  +str(yaw_scale_factor)+"]")
            new_roll = roll * roll_scale_factor
            new_pitch = pitch * pitch_scale_factor
            new_yaw = yaw * yaw_scale_factor
        else:
            # If only clipping is required (Flag == 1), 
            # clip based on soft limits of hexapod given in the global arrays
            new_roll = roll
            new_pitch = pitch
            new_yaw = yaw
            new_roll = self.get_lim_values(roll
                , self.hexController.LOW_LIMITS[3] /180*np.pi
                , self.hexController.HIGH_LIMITS[3]/180*np.pi)
            new_pitch = self.get_lim_values(pitch
                , self.hexController.LOW_LIMITS[4] /180*np.pi
                , self.hexController.HIGH_LIMITS[4]/180*np.pi)
            new_yaw = self.get_lim_values(yaw
                , self.hexController.LOW_LIMITS[5] /180*np.pi
                , self.hexController.HIGH_LIMITS[5]/180*np.pi)
        # These are the new attitude values
        roll = new_roll
        pitch = new_pitch
        yaw = new_yaw
        # Create rotation matrix based on them
        rot_mat = self.create_rot_mat(roll, pitch, yaw)
        # Transform them to degrees (hexapod works with degree values)
        # while numpy works with rad values
        roll = roll / np.pi * 180
        pitch = pitch / np.pi * 180
        yaw = yaw / np.pi * 180
        # init new array collection (3D velocity over time in n-frame)
        rotated_velocity = np.zeros(velocity.shape)
        # Iterate over array structure, and compute new rotated velocity elements
        for i in range(rot_mat.shape[1]):
            rotated_velocity[:,i] = np.sum(rot_mat[:,i,:] * velocity,1)    
        # Scaling flag = 0 --> Scale based on max value. 
        # Scaling flag = 1 --> Cut off at max val
        # Regardless of that, cut off position if position is above max
        if(scaling_flag_Vel == 0):
            # Get absolute velocity value, to compute scaling factor. 
            # The hexapod is bound by its absolute velocity of 2,5 mm/s, 
            # based on the instructions manual
            absolute_velocity = np.sqrt(rotated_velocity[:,0]**2
                                        + rotated_velocity[:,1]**2 
                                        + rotated_velocity[:,2]**2)
            max_velocity = self.get_abs_max_value(absolute_velocity)
            #Get scaling factor and compute new velocity
            velocity_scaling_factor = self.get_scale_factor(
                   max_velocity
                ,  self.hexController.MAX_VEL_VALUE
                , -self.hexController.MAX_VEL_VALUE)
            scaled_velocityVector = rotated_velocity * velocity_scaling_factor
        else:
            # If no scaling should be performed, try to cut off velocity 
            # values which are too high. 
            absolute_velocity = np.sqrt(rotated_velocity[:,0]**2 
                                  + rotated_velocity[:,1]**2 
                                  + rotated_velocity[:,2]**2)
            scaled_velocity = self.get_lim_values(
                absolute_velocity
                ,   self.hexController.MAX_VEL_VALUE
                , - self.hexController.MAX_VEL_VALUE)
            # This cut off is done by limiting the absolute velocity, 
            # and then multiplying 
            # the new decreased absolute velocity by the orientation vector, 
            # element wise
            velocity_unit_vector = np.divide(rotated_velocity, 
                                   absolute_velocity.
                                   reshape(absolute_velocity.shape[0],1))
            scaled_velocityVector = (velocity_unit_vector
                                     * scaled_velocity.
                                     reshape(scaled_velocity.shape[0],1))
        # Calculate position based on these velocity measurements. Then scale/cut 
        # position so that the hexapod can perform the manouvers
        pos = np.zeros((len(time_waypoints),3))
        # For each waypoint, integrate velocity values to position values.
        for i in range(len(time_waypoints)):
            if(i == 0):
                dt = time_waypoints[i] - 0
                pos[i,:] = rotated_velocity[i,:] * dt
            else:
                dt = time_waypoints[i] - time_waypoints[i-1]
                pos[i,:] = pos[i-1,:] + rotated_velocity[i,:]*dt
        # Depending on the given scaling flag, perform different actions
        if(scaling_flag_Pos == 0):
            # Get absolute maxima, and scale by said absolute maxima
            max_px = self.get_abs_max_value(pos[:,0])
            max_py = self.get_abs_max_value(pos[:,1])
            max_pz = self.get_abs_max_value(pos[:,2])
            scaling_px = self.get_scale_factor(
                max_px
                , self.hexController.HIGH_LIMITS[0]
                , self.hexController.LOW_LIMITS[0])
            scaling_py = self.get_scale_factor(
                max_py
                , self.hexController.HIGH_LIMITS[1]
                , self.hexController.LOW_LIMITS[1])
            scaling_pz = self.get_scale_factor(
                max_pz
                , self.hexController.HIGH_LIMITS[2]
                , self.hexController.LOW_LIMITS[2])
            new_px = pos[:,0] * scaling_px
            new_py = pos[:,1] * scaling_py
            new_pz = pos[:,2] * scaling_pz
        else:
            # If no scaling should be done, 
            # cut off high position values instead, according to limit
            new_px = pos[:,0]
            new_py = pos[:,1]
            new_pz = pos[:,2]
            new_px = self.get_lim_values(
                new_px
                , self.hexController.HIGH_LIMITS[0]
                , self.hexController.LOW_LIMITS[0])
            new_py = self.get_lim_values(
                new_py
                , self.hexController.HIGH_LIMITS[1]
                , self.hexController.LOW_LIMITS[1])
            new_pz = self.get_lim_values(
                new_pz
                , self.hexController.HIGH_LIMITS[2]
                , self.hexController.LOW_LIMITS[2])
        # Calculate new scaled velocity
        diff_time = np.diff(time_waypoints)
        diff_px = np.diff(new_px)
        new_velX = diff_px / diff_time
        diff_time = np.diff(time_waypoints)
        diff_py = np.diff(new_py)
        new_vely = diff_py / diff_time
        diff_time = np.diff(time_waypoints)
        diff_pz = np.diff(new_pz)
        new_velz = diff_pz / diff_time
        # 
        # Plot resulting kinematic values (position + attitude are correct)
        pyplot.figure()
        pyplot.plot(time_waypoints, pos[:,0])
        pyplot.title("Original Position - X (mm)")
        pyplot.figure()
        pyplot.plot(time_waypoints, pos[:,1])
        pyplot.title("Original Position - Y (mm)")
    
        pyplot.figure()
        pyplot.plot(time_waypoints, pos[:,2])
        pyplot.title("Original Position - Z (mm)")
    
        pyplot.figure()
        pyplot.plot(time_waypoints, rotated_velocity[:,0])
        pyplot.title("Scaled Velocity - X (mm/s)")
    
        pyplot.figure()
        pyplot.plot(time_waypoints, rotated_velocity[:,1])
        pyplot.title("Scaled Velocity - Y (mm/s)")
    
        pyplot.figure()
        pyplot.plot(time_waypoints, rotated_velocity[:,2])
        pyplot.title("Scaled Velocity - Z (mm/s)")

        pyplot.figure()
        hR = pyplot.plot(time_waypoints, roll)
        hP = pyplot.plot(time_waypoints, pitch)
        hY = pyplot.plot(time_waypoints, yaw)
        pyplot.title("Roll-Pitch-Yaw (degrees)")
        pyplot.legend([hR, hP, hY], 
        ["Roll", "Pitch", "Yaw"])        
        pyplot.show()
        pos[:,0] = new_px
        pos[:,1] = new_py
        pos[:,2] = new_pz
        attitude = np.zeros((len(roll),3))
        attitude[:,0] = roll.reshape(len(roll))
        attitude[:,1] = pitch.reshape(len(roll))
        attitude[:,2] = yaw.reshape(len(roll))
        print(time_waypoints.shape)
        print(pos.shape)
        print(attitude.shape)
        self.plot_waypoints(time_waypoints, pos, attitude)
        print(self.MAT_NAME)
        self.hexController.set_mat_name(self.MAT_NAME)
        return time_waypoints, pos, attitude

    def plot_waypoints(self, time_waypoints, pos_waypoints, attitude_waypoints):
        """Plot the given waypoints. time_waypoints = x
        
        This function plots the given waypoints with matplotlib
        Convenience function
        
        Parameters
        ---------
        time_waypoints : numpy array (n x 1)
            Time waypoints
        pos_waypoints : numpy array (n x 3)
            Position waypoints
        attitude_waypoints : numpy array (n x 3)
            Attitude waypoints

        Returns
        -------
        Output via matplotlib. Function returns None.
        """
        pyplot.figure()
        pyplot.plot(time_waypoints, attitude_waypoints[:,0])
        pyplot.title("Roll (degrees)")
    
        pyplot.figure()
        pyplot.plot(time_waypoints, attitude_waypoints[:,1])
        pyplot.title("Pitch (degrees)")
    
        pyplot.figure()
        pyplot.plot(time_waypoints, attitude_waypoints[:,2])
        pyplot.title("Yaw (degrees)")
    
    
        pyplot.figure()
        pyplot.plot(time_waypoints, pos_waypoints[:,0])
        pyplot.title("Pos x (mm)")
        pyplot.figure()
        pyplot.plot(time_waypoints, pos_waypoints[:,1])
        pyplot.title("Pos y (mm)")
        pyplot.figure()
        pyplot.plot(time_waypoints, pos_waypoints[:,2])
        pyplot.title("Pos z (mm)")    
        
    # Checks if string is float
    def isfloat(self, value):
      try:
        float(value)
        return True
      except ValueError:
        return False
    
    
    # This function calculates the fourier transform of the passed in 2D
    # matrix. Matrix is assumed to have dimensions [n x m], where n
    # is the time dimension.
    def calculate_fourier_transforms(self, matrix, reshape_factor):
        """Takes matrix values, and performs fourier transform on them
        
        This function uses scipy FFT (discrete) to get the image
        representations of the given time series. Matrix should be in format
        [n x m], where m is time.
        
        Parameters
        ---------
        matrix : numpy array [n x m]
            Data to perform FFT on
            
        reshape_factor : float
            Reshape the given matrix by this value.
            Is used for averaging!
        Returns
        -------
        x_hz : numpy array [m x 1]
            Corresponding frequencies 
            (provided that first matrix row is time)
        y_k : numpy array [n x m]
            Resulting DFT results, row-wise
        y_k_abs : numpy array [n x m]
            Resulting magnitude data
        y_k_phase : numpy array [n x m]
            Resulting phase data
        """
        if(len(matrix.shape) > 1):
            if(matrix.shape[1] < matrix.shape[0]):
                matrix = matrix.reshape((matrix.shape[0], matrix.shape[1]))
        if(not reshape_factor is None):
            reshape_factor2 = int(np.floor(matrix.shape[1] / reshape_factor))
            # Split matrix into equal packets of size reshape factor
            matrix_reshaped = matrix[:,0:reshape_factor2 * reshape_factor].reshape(matrix.shape[0], reshape_factor2, reshape_factor)
            matrix = matrix_reshaped
        if(len(matrix.shape) > 1):
            y_k = np.zeros(matrix.shape,dtype=np.complex_)
            if(len(matrix.shape) == 3):
                y_k = np.zeros((matrix.shape[0], matrix.shape[2]),dtype=np.complex_)
            for i in range(matrix.shape[0]):
                #Format matrix with window (hanning)
                if(len(matrix.shape) == 3):
                    matrix_parts = np.zeros((matrix.shape[1], matrix.shape[2]))
                    for j in range(matrix.shape[1]):
                        matrix_i = np.hanning(matrix[i,j,:].shape[0]) * matrix[i,j,:].reshape(matrix.shape[2])
                        y_k[i,:] = y_k[i,:] + scipy.fft.fft(matrix_i).reshape((matrix.shape[2]))
                    y_k[i,:] = y_k[i,:] / matrix.shape[1]
                else:
                    matrix_i = np.hanning(matrix[i,:].shape[0]) * matrix[i,:].reshape(matrix.shape[1])
                    y_k[i,:] = scipy.fft.fft(matrix_i).reshape((matrix.shape[1]))
        else:
            y_k = np.zeros(matrix.shape,dtype=np.complex_)
            y_k = scipy.fft.fft(matrix.reshape(matrix.shape[0])
                                            ).reshape((matrix.shape[0]))
        if(len(matrix.shape) == 3):
            freq = 1/(matrix[0,0,1] - matrix[0,0,0])
        elif(len(matrix.shape) == 2):
            freq = 1/(matrix[0,1] - matrix[0,0])
        elif(len(matrix.shape) == 1):
            freq = 1/(matrix[1] - matrix[0])
        x_hz = np.fft.fftfreq(len(y_k[0,:])) * freq
        y_k_abs = np.abs(y_k)
        y_k_phase = np.angle(y_k)
        return x_hz, y_k, y_k_abs, y_k_phase
    



    # Check if character length is below 19. 
    # If that is the case, 
    # return a number which indicates how many spaces 
    # need to added to the string
    # to have a length of 19 characters. 
    # Is used to format output files
    def check_for_length(self, variable):
        """Check how many spaces are needed for equal var length
        
        This method checks if variable is long enough.
        If not, it returns the number of spaces which need
        to be added in order to print out pretty data.txt

        Parameters
        ---------
        variable : int or float
            Numeric value. Is converted to string        
        Returns
        -------
        number_of_missing_spaces : int
            This variable returns the necessary number of
            whitespaces to conform to the wanted character length
        """
        number_of_missing_spaces = np.max([19 - len(str(variable)) + 3, 3])
        return number_of_missing_spaces

    
    def save_data(self, matrix, file_name, header = ""):
        """Saves the matrix (n x m) with the specified name.
        
        This method saves the given matrix to file.
        Furthermore, a header can be given to this method, but it isn't
        required to give one. The header should be given as a list.
        This way, this file can calculate the appropriate spacing
        between header elements!
        
        Parameters
        ---------
        matrix : numpy array (n x m)
            Matrix which contains the data to be saved
        file_name : string
            File name. The data is written to this file
        header : string/list optional
            Optional header. Appears as first element in string
            
        Returns
        -------
        timestamps : numpy array
            Time stamps corresponding to the derivatives. 
            
        values_dt : numpy array
            Numerical derivatives. Standard is simple euler scheme
        """
        formatted_header = ""
        np.set_printoptions(suppress=True,
                            formatter={'float_kind':'{:f}'.format})
        if(isinstance(header, list)):
            for i in range(len(header)):
                header_el = header[i]
                missing_spaces = self.check_for_length(header[i])
                formatted_header = formatted_header + header[i] + " "*missing_spaces 
        else:
            formatted_header = header
        
        f = open(file_name, "w")
        f.write(formatted_header + os.linesep)
        missing_spaces = np.zeros(matrix.shape[0])
        for i in range(matrix.shape[1]):   
            write_string = ""
            for j in range(matrix.shape[0]):
                missing_space = self.check_for_length(matrix[j,i])
                missing_spaces[j] = missing_space
                write_string = write_string + "{:.12f}".format(matrix[j,i])+" "*missing_space
            f.write(write_string + os.linesep)
        f.close()

    
    def interpolate_to_array(self, time_array, old_timepoints, data):
        """Interpolates (euler) old_timepoints and data to new time_array
        
        This method redistributed the points so that they fit the new
        time_array. time_array and old_timepoints can be of different 
        length. It goes through the time_array points, one by one,
        gets the closest old_timepoint to the current time_array element,
        computes the slope, and then interpolates the data so that
        it matches the new time_array entry. Best results can be
        expected if old_timepoints and time_array have roughly the same
        span, and if old_timepoints contains more timepoints than time_array,
        since this enhances the resolution of the interpolation.
        
        Parameters
        ---------
        time_array : numpy array  [n x 1]
            The new timepoint array. Output corresponds to these timesteps
        old_timepoints : numpy array [m x 1]
            The old timepoint array. Is used to interpolate data correctly
        data : numpy array [p x m]
            Data can be 2D array. If [m x p] is given, this method
            transposes data first. Data is interpolated per p-dimension
        
        Returns
        -------
        time_array : numpy array  [n x 1]
            Convenience. Returns the same array as output, to provide
            timestamps directly
        interpol_data : numpy array [p x n]
            Interpolated data. Returns in the form [p x n] for easy indexing.
        """
        # If data is in wrong format, transpose first
        if(data.shape[0] == old_timepoints.shape[0]):
            data = data.T
        if(len(data.shape) == 1):
            interpol_data = np.zeros(len(time_array))
        else:
            interpol_data = np.zeros((data.shape[0], len(time_array)))
        for i in range(len(time_array)):
            # Get index which corresponds to time_array
            index = np.argmin(np.abs(time_array[i] - old_timepoints))
            time_diff = (time_array[i] - old_timepoints[index]).reshape((1, 1))
            if(len(data.shape) == 1):
                data_i = data[index]                
            else:
                data_i = data[:,index]
            # Get corresponding data entries
            # Get slope by computing index based numerical derivative
            # If index is already at last value, reduce index by 1, then 
            # perform the same operation
            # This function performs these operations under the assumption that
            # the slower data series was not generated by averaging the surrounding
            # values. Instead, data points were collected precisely 
            # at the defined time location. 
            if(index == (len(old_timepoints) - 1)):
                index = index -1
            old_dtime = old_timepoints[index + 1] - old_timepoints[index]
            if(len(data.shape) == 1):
                slope_data = (data[index + 1] - data[index]) / old_dtime                
                interpol_data[i] = data_i + time_diff * slope_data
            else:
                slope_data = (data[:,index + 1] - data[:,index]) / old_dtime
                interpol_data[:,i] = data_i + time_diff * slope_data    
        return time_array, interpol_data
    
    
    def  get_time_derivative(self, time, values):
        """Calculate time derivatives of the value numpy array
        
        This method computes the time derivative of a given numpy array
        Numpy array should be n x 1, and both arrays should be of the 
        same shape.
        
        Parameters
        ---------
        time : numpy array (n x 1)
            Time stamps of the data
        values : numpy array (n x 1)
            Values of the data            
        Returns
        -------
        timestamps : numpy array
            Time stamps corresponding to the derivatives. 
            
        values_dt : numpy array
            Numerical derivatives. Standard is simple euler scheme
        """
        
        dtime = np.diff(time)
        #if(time.shape[0] != values.shape[1]):
        #    values = values.reshape(values.shape[0], time.shape[0])
        dvalues = np.diff(values, 1)
        timestamps = np.zeros(dtime.shape)
        time1 = time[0:-1]
        time2 = time[1:time.shape[0]+1]
        # New values are located at start time + half the time step width
        # This should work, even if the timesteps are uneven!
        timestamps = time1 + (time2 - time1) / 2
        values_dt = dvalues / dtime.reshape(1,dtime.shape[0])
        return timestamps, values_dt
    
    
    # load imu data from file. Similar to the load_position_file method
    def load_IMU_file(self, file_name, indices = None):
        """Load recorded lord imu data, based on the file name        
        
        This function takes a file name as an argument, 
        and loads a .txt file which contains recorded MEMS-data.
        
        Parameters
        ---------
        file_name : string
            Name of the file
        indices : 2 element list
            List int indices. Specifies the range which should be 
            loaded. Useful after data has been accessed once
        Returns
        -------
        values : numpy array
            Recorded imu data values. 
            0,1 are timestamps (Python internal time, MEMS time)
            2,3,4 are acceleration values (b-frame)
            5,6,7 are angular rate values (b-frame)
        """
        values = []
        try:
            f = open(file_name, "r")
            for line in f:
                line_val = line.split()
                if(not self.isfloat(line_val[0])):
                    continue
                #print(line_val)
                time_python = float(line_val[0])
                time_imu = float(line_val[1])
                acc_x = float(line_val[2])
                acc_y = float(line_val[3])
                acc_z = float(line_val[4])
                gyr_x = float(line_val[5])
                gyr_y = float(line_val[6])
                gyr_z = float(line_val[7])
                values.append(np.array([time_python
                                       ,time_imu
                                       ,acc_x
                                       ,acc_y
                                       ,acc_z
                                       ,gyr_x
                                       ,gyr_y
                                       ,gyr_z]))
            if(indices is None):
                start_index = 0
                end_index = len(values)
            else:
                start_index = indices[0]
                end_index = indices[1]
            values = np.array(values).T
            indices = values[1, :].argsort()
            values = values[:, indices][:,start_index:end_index]
        finally:
            f.close()
        return values
    
    
    def load_position_files(self, file_names
        , fourier_flag = 0, conv_to_rads = 0, indices = None):
        """Load position files, based on the file names. Calls load_position_file
        
        This function takes multiple! file names as an argument, 
        and loads the .txt files which contain recorded hexapod
        position and attitude values.
        Furthermore, it calculates velocity and acceleration values
        based on position values and timestamps.
        Returns list of numpy arrays!
        
        Parameters
        ---------
        file_names : list, string
            Names of the files
            
        fourier_flag : int
            If 1, also calculate fourier-transform of said data
        conv_to_rads : int
            Flag which indicates whether loaded data should be converted to
            radians. Only applies to attitude and angular rate data.
            Should be 1 if comparison with IMU data is wanted
        indices : 2 element list
            Index region which should be returned.
            Useful to do after data has been analyzed already
        Returns
        -------
        values : numpy array
            Numpy array which contains timestamps, 
            position values, attitude values,
            velocity values, angular rates, and acceleration
            values. Velocity, angular rate and acceleration
            are computed numerically from position and attitude
            values
        x_hz : numpy array
            Corresponding fourier transform frequencies
        y_k : numpy array
            Fourier transform representation of the data.
            Only returned if fourier_flag = 1
        y_k_abs : numpy array
            Absolute values of fourier transform
            Only returned if fourier_flag = 1
        y_k_phase : numpy array
            Phase values of fourier transform
            Only returned if fourier_flag = 1
        name_list : list, string
            Corresponding names to the loaded data! Useful to order data
        """
        if(fourier_flag):
            value_list = []
            x_hz_list = []
            y_k_list = []
            y_k_abs_list = []
            y_k_phase_list = []
            name_list = []
            for name in file_names:
                if(not os.path.exists(name)):
                    print("This file doesn't exist. Skipping... ")
                    print(name)
                    continue
                values, x_hz, y_k, y_k_abs, y_k_phase = self.load_position_file(name, fourier_flag, conv_to_rads, indices = indices)
                value_list.append(values)
                name_list.append(name)
                x_hz_list.append(x_hz)
                y_k_list.append(y_k)
                y_k_abs_list.append(y_k_abs)
                y_k_phase_list.append(y_k_phase)
            return value_list, x_hz_list, y_k_abs_list, y_k_phase_list, name_list
        else:
            value_list = []
            name_list = []
            for name in file_names:
                if(not os.path.exists(name)):
                    print("This file doesn't exist. Skipping... ")
                    print(name)
                    continue
                values = self.load_position_file(name, fourier_flag, conv_to_rads)            
                value_list.append(values)
                name_list.append(name)
            return value_list, name_list


    def  load_position_file(self, file_name, fourier_flag = 0, conv_to_rads = 0, indices = None):
        """Load position files, based on the file name
        
        This function takes a file name as an argument, 
        and loads a .txt file which contains recorded hexapod
        position and attitude values.
        Furthermore calculates velocity and acceleration values
        based on position values and timestamps.
        
        Parameters
        ---------
        file_name : string
            Name of the file
            
        fourier_flag : int
            If 1, also calculate fourier-transform of said data
        conv_to_rads : int
            Flag which indicates whether loaded data should be converted to
            radians. Only applies to attitude and angular rate data.
            Should be 1 if comparison with IMU data is wanted
            
        Returns
        -------
        values : numpy array
            Numpy array which contains timestamps, 
            position values, attitude values,
            velocity values, angular rates, and acceleration
            values. Velocity, angular rate and acceleration
            are computed numerically from position and attitude
            values
        x_hz : numpy array
            Corresponding fourier transform frequencies
        y_k : numpy array
            Fourier transform representation of the data.
            Only returned if fourier_flag = 1
        y_k_abs : numpy array
            Absolute values of fourier transform
            Only returned if fourier_flag = 1
        y_k_phase : numpy array
            Phase values of fourier transform
            Only returned if fourier_flag = 1
        """
        values = []
        try:
            f = open(file_name, "r")
            index = 0
            last_time = 0
            last_px = 0
            last_py = 0
            last_pz = 0
            last_vx = 0
            last_vy = 0
            last_vz = 0
            last_roll = 0
            last_pitch = 0
            last_yaw = 0
            for line in f:
                if(index == 0):
                    index += 1
                    continue
                line_val = line.split()
                if(not self.isfloat(line_val[0])):
                    continue
                #print(line_val)
                wp = float(line_val[0])
                time = float(line_val[1])
                px = float(line_val[2])
                py = float(line_val[3])
                pz = float(line_val[4])
                roll = float(line_val[5])
                pitch = float(line_val[6])
                yaw = float(line_val[7])
                if(conv_to_rads == 1):
                    roll = roll/180 * np.pi
                    pitch = pitch/180 * np.pi
                    yaw = yaw/180 * np.pi
                if(last_time == 0 or (time-last_time) == 0):                
                    values.append(np.array([wp, time
                                            , px, py, pz
                                            , roll, pitch, yaw
                                            , 0, 0, 0
                                            , 0, 0, 0
                                            , 0, 0, 0]))
                    
                else:                
                    #print("here")
                    vx = (px - last_px)/(time - last_time)
                    vy = (py - last_py)/(time - last_time)
                    vz = (pz - last_pz)/(time - last_time)
                    ax = (vx - last_vx)/(time - last_time)
                    ay = (vy - last_vy)/(time - last_time)
                    az = (vz - last_vz)/(time - last_time)
                    r_dot = (roll - last_roll)/(time - last_time)
                    p_dot = (pitch - last_pitch)/(time - last_time)
                    y_dot = (yaw - last_yaw)/(time - last_time)
                    values.append(np.array([wp, time
                                            , px, py, pz
                                            , roll, pitch, yaw
                                            , vx, vy, vz
                                            , r_dot, p_dot, y_dot
                                            , ax, ay, az]))                
                    last_vx = vx
                    last_vy = vy
                    last_vz = vz
                last_time = time
                last_px = px
                last_py = py
                last_pz = pz
                last_roll = roll
                last_pitch = pitch
                last_yaw = yaw
                index += 1
            if(indices is None):
                start_index = 0
                end_index = len(values)
            else:
                start_index = indices[0]
                end_index = indices[1]
            values = np.array(values).T[:,start_index:end_index]
            # Get the final movement index. Assume for that the following:
            # The position is constant after that index
            # This means that change in position is minimal
            # Furthermore, change of change is also minimal
            # This obviously doesn't work if the platform stands still for 
            # a while. But it does the job if a vibration is applied to the
            # system, to cut off non relevant parts for the fourier transform
            # Is probably too fickle. Use manual trimming instead.
#            values_rel = values / np.max(np.abs(values),1).reshape(values.shape[0],1)
#            d_new_time, d_values = self.get_time_derivative(values[0,:], values)
#            d_values_rel = d_values / np.max(np.abs(d_values), 1).reshape((d_values.shape[0],1))
#            d_time_matched, d_values_rel_matched = self.interpolate_to_array(values[0,:], d_new_time, d_values_rel)
#            dd_new_time, dd_values = self.get_time_derivative(d_time_matched, d_values_rel_matched)
#            dd_values_rel = dd_values / np.max(np.abs(dd_values), 1).reshape(dd_values.shape[0],1)
#            dd_time_matched, dd_values_rel_matched = self.interpolate_to_array(values[0,:], dd_new_time, d_values_rel)
#            end_indices = np.argmin(np.abs(dd_values_rel_matched)
#                                    + np.abs(d_values_rel_matched)
#                                    + np.abs(values_rel)
#                                    - np.abs(values_rel[:,-1]).reshape((values.shape[0],1)), 1)
            
#            print(end_indices)
#            end_index = np.max(end_indices)
#            print(end_index)
        finally:
            f.close()
        if(fourier_flag == 1):
#            val_for_fourier = values[:,0:end_index]
            y_k, x_hz, y_k_abs, y_k_phase = self.calculate_fourier_transforms(values.T).T
            return values, x_hz, np.array(y_k), y_k_abs, y_k_phase
        else:
            return values


# Example. This is how *args should be passed
# pass the following arguments in ascending order!
#fold_0 = "Vibration_with_IMU/"
#fold_1 = ["X/","X2/","Z/"]
#name_0 = "Result_Vibration"
#name_1 = ["X", "Z"]
#name_2 = ["1", "5", "10", "15", "20", "25", "30"]
#name_3 = "hz"
#name_4 = "_buffered_0_"
#name_5 = ["IMU_Dat", "REC_COMM", "REC_MEAS"]
#name_6 = ".txt"
    def generate_file_names(self, *args, nested_return = False):
        """Convenience function to generate file names based on args

            To get the most out of the function, specify *args
            arguments as list. The function then loops through
            each *args list, and generates file names based on that.
            This is a recursive function. During each iteration,
            the innermost *args is kept in the outer loop. 
            By doing this, the file names are built up from the outside.
            i.e. generate_fileName(["var1"],["_X"],["_Y", "-Z"])
            should produce file names which are as follows:
            file_name_list = ["var1_X_Y", "var1_X-Z"]

        Parameters
        ---------
        *args : multiple lists or a mixture of strings and lists
            Multiple lists. For each *args, a new call to 
            generate_file_names is performed, while reducing the 
            number of lists in *args by one (i.e. all but the
            final element/list are included.).
        nested_return : flag
            Flag. If False, returns file names in a single list. 
            If true, creates nested list where file names are grouped by
            the top level list (i.e. if top level has 2 elements, the function
            returns a list(top_level0_list, top_level1_list))
            Is set to false in recursive calls, since they don't need
            to split the list up
        Returns
        -------
        file_name_lists : list or nested list of lists
            Depending on the flag set in parameters, 
            a list containing all file names, or a nested list of lists
            is given. 

        """
        if(len(args) == 1):
            #print(args)
            #print("my_files")
            if(isinstance(args[-1], str)):
                my_return_val = args
            else:
                my_return_val = args[-1]
            return my_return_val
        else:
            if(isinstance(args[-1], str)):
                my_arg_list = [args[-1]]
            else:
                my_arg_list = args[-1]
            args = args[0:-1]
            built_string_list = self.generate_file_names(*args)
            my_built_string_list = []
            for my_arg_str in my_arg_list:
                if(nested_return):
                    sub_list = []
                    for built_string in built_string_list:
                        #print("1")
                        #print(my_arg_list)
                        #print(built_string_list)
                        #print(built_string)
                        #print(my_arg_str)
                        sub_list.append(built_string + my_arg_str)
                    my_built_string_list.append(sub_list)
                else:
                    for built_string in built_string_list:
                        #print("2")
                        #print(my_arg_list)
                        #print(built_string_list)
                        #print(built_string)
                        #print(my_arg_str)
                        my_built_string_list.append(built_string + my_arg_str)
            return my_built_string_list