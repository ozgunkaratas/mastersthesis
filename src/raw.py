# -*- coding: utf-8 -*-
"""
Created on Wed Jun 16 17:30:58 2021

@author: User
"""
fig = plt.figure(figsize=(16,9))
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
plt.plot(utcTime+1.5,gyrX,'r', label="Misaligned Xsens")
plt.plot(h_g_time_experiment_aug,hlGyrX,'b', label="Misaligned HL")
plt.xticks(fontsize=30)
plt.yticks(fontsize=30)
plt.legend(loc="upper left",fontsize=20)
plt.title("Angular Velocities (X-Axis)",fontsize=30)
plt.xlabel('Time (s)',fontsize=30)
plt.ylabel('Angular Velocity (rad/s)', fontsize=30)
plt.savefig("angx_unc.ps")


fig = plt.figure(figsize=(16,9))
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
plt.plot(utcTime+1.5,gyrY,'r', label="Misaligned Xsens")
plt.plot(h_g_time_experiment_aug,hlGyrY, 'b',label="Misaligned HL")
plt.xticks(fontsize=30)
plt.yticks(fontsize=30)
plt.legend(loc="upper left",fontsize=20)
plt.title("Angular Velocities (Y-Axis)",fontsize=30)
plt.xlabel('Time ($s$)',fontsize=30)
plt.ylabel('Angular Velocity ($rad/s$)', fontsize=30)
plt.savefig("angy_unc.ps")



fig = plt.figure(figsize=(16,9))
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
plt.plot(utcTime+1.5,gyrZ, 'r',label="Misaligned Xsens")
plt.plot(h_g_time_experiment_aug,hlGyrZ, 'b',label="Misaligned HL")
plt.xticks(fontsize=30)
plt.yticks(fontsize=30)
plt.legend(loc="upper left",fontsize=20)
plt.title("Angular Velocities (Z-Axis)",fontsize=30)
plt.xlabel('Time ($s$)',fontsize=30)
plt.ylabel('Angular Velocity ($rad/s$)', fontsize=30)
plt.savefig("angz_unc.ps")



##acccel
fig = plt.figure(figsize=(16,9))
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
plt.plot(utcTime+2,accX,'r', label="Misaligned Xsens")
plt.plot(h_g_time_experiment_aug,interp_hlAccX,'b', label="Misaligned HL")
plt.xticks(fontsize=30)
plt.yticks(fontsize=30)
plt.legend(loc="upper left",fontsize=20)
plt.title("Acceleration (X-Axis)",fontsize=30)
plt.xlabel('Time ($s$)',fontsize=30)
plt.ylabel('Acceleration $(m/s^2)$', fontsize=30)
plt.savefig("accx_unc.ps")


fig = plt.figure(figsize=(16,9))
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
plt.plot(utcTime+2,accY,'r', label="Misaligned Xsens")
plt.plot(h_g_time_experiment_aug,interp_hlAccY, 'b',label="Misaligned HL")
plt.xticks(fontsize=30)
plt.yticks(fontsize=30)
plt.legend(loc="upper left",fontsize=20)
plt.title("Acceleration(Y-Axis)",fontsize=30)
plt.xlabel('Time ($s$)',fontsize=30)
plt.ylabel('Acceleration $(m/s^2)$', fontsize=30)
plt.savefig("accy_unc.ps")



fig = plt.figure(figsize=(16,9))
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
plt.plot(utcTime+2,accZ, 'r',label="Misaligned Xsens")
plt.plot(h_g_time_experiment_aug,interp_hlAccZ, 'b',label="Misaligned HL")
plt.xticks(fontsize=30)
plt.yticks(fontsize=30)
plt.legend(loc="upper left",fontsize=20)
plt.title("Acceleration (Z-Axis)",fontsize=30)
plt.xlabel('Time ($s$)',fontsize=30)
plt.ylabel('Acceleration $(m/s^2)$', fontsize=30)
plt.savefig("accz_unc.ps")







