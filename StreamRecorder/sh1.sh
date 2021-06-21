pwd=$(pwd)
echo giving read/write abilities to folders.
echo "should this fail, run this : sudo chmod -R 777 <folder name>"
echo on your path.
echo make sure you have all the utility scripts from the HoloLens StreamRecorderConverter
echo make sure you have PV.tar and the *pv.txt text file that comes with it
chmod -R 777 "IMU ACCEL.tar"
chmod -R 777 "IMU GYRO.tar"
chmod -R 777 "PV.tar"
echo Starting batch processing...
python3 tar.py
rm "IMU ACCEL".tar
rm "IMU GYRO".tar
echo "unzipped IMU files to their folders, they will be merged now."
cd "IMU ACCEL"
cat $(find .  -maxdepth 1 ! -name "\.*" -type f -printf "%T+ %p\n" | sort | sed 's/^.* //') >> accel.txt
echo merged accelerometer files.
cd ..
cd "IMU GYRO"
cat $(find .  -maxdepth 1 ! -name "\.*" -type f -printf "%T+ %p\n" | sort | sed 's/^.* //') >> gyro.txt
echo merged gyroscope files.
cd ..
echo "please manually copy accel.txt (in IMU ACCEL) and gyro.txt (in IMU GYRO) to your main path"
echo "please run the second shell script in your path now."
