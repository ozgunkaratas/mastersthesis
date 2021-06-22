# mastersthesis
This repo houses my masters thesis regarding spatiotemporal calibration between the HoloLens2 and the Xsens MTi-G IMU. Docs and full source code are presented here.


Download the StreamRecorderApp as a complete folder to your directory and deploy it to the HoloLens2 directly. If the hololens throws a reference to null pointer exception, ignore and deploy again, it will fix the problem. The StreamRecorderApp stores the imu readings in individual text files. In order to merge them according to their creation times so that all readings appear in one text file i wrote a small command line entry that does this thing. 

After you do your experiment with the HL, use the regular utility scripts that help to download the contents from the Hololens (see HLs repository for further info). Use the process_all.py script to process the regular StreamRecorderApp results. In order to process the IMU readings, run the shell script in your directory or copy the .tar IMU files to a different directory and run the shell here. this will give the merged accel.txt, gyro.txt and mag.txt files inside individual IMU folders. these raw files are now ready to be supplied to the framework.


Running the framework is straightforward, copy the contents of src to a directory, and find a set of experiment raw data that you want to analyze and copy the raw data to your directory. execute main.py. a prompt will show up asking you if all files are correctly placed in the directory, follow the instructions.

Capturing data from the Xsens requires low-level drivers which can be found here: https://github.com/ethz-asl/ethzasl_xsens_driver. Further instructions are presented to guide you to install and configure the device on your ubuntu computer. works on 18.04, probably wont work on 20.04.


Instructions for installing kalibr on your computer can be found under: https://github.com/ethz-asl/kalibr/wiki Kalibr can be installed as binary as well but i have no experience with that, just build it on your ROS distro like a normal software package. Kalibr only uses rosrun for exection, apart from that, its has no connections with ROS, ROS nodes, ROS topics etc. I recommend you build my forked repo because it will also print out the trajectories and ego motion of the camera and the xsens.

You cant create bags nor merge them, although this functionality is built into the framework, because raw data is not present in the repo due to its massive size. 
