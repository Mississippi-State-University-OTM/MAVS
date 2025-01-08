'''
Non-Commercial License - Mississippi State University Autonomous Vehicle Software (MAVS)

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

*NOTICE*
Thank you for your interest in MAVS, we hope it serves you well!  We have a few small requests to ask of you that will help us continue to create innovative platforms:
-To share MAVS with a friend, please ask them to visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to register and download MAVS for free.
-Feel free to create derivative works for non-commercial purposes, i.e. academics, U.S. government, and industry research.  If commercial uses are intended, please contact us for a commercial license at otm@msstate.edu or jonathan.rudd@msstate.edu.
-Finally, please use the ACKNOWLEDGEMENT above in your derivative works.  This helps us both!

Please visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to view the full license, or email us if you have any questions.

Copyright 2018 (C) Mississippi State University
'''
import sys
import matplotlib.pyplot as plt
import math
# Set the path to the mavs python api, mavs_interface.py
# You will have to change this on your system.
sys.path.append(r'C:\Users\cgoodin\Desktop\vm_shared\shared_repos\mavs\src\mavs_python')
# Load the mavs python modules
import mavs_interface as mavs

imu1 = mavs.MavsMems('accelerometer')
imu2 = mavs.MavsMems('accelerometer')
imu2.SetNoiseDensity(0.025)
imu2.SetBiasInstability(0.01)
imu2.SetTemperatureBias(0.0)
imu2.SetConstantBias(0.0)

temperature = 25.0
x1_accel = []
x2_accel = []
x_true = []
time = []
dt = 0.01
elapsed_t = 0.0
while elapsed_t<5.0:
    accel_in = [2.0*math.sin(elapsed_t), 0.0, 0.0]
    accel_out1 = imu1.Update(accel_in,temperature,1.0/dt)
    accel_out2 = imu2.Update(accel_in,temperature,1.0/dt)
    x1_accel.append(accel_out1[0])
    x2_accel.append(accel_out2[0])
    x_true.append(accel_in[0])
    time.append(elapsed_t)
    elapsed_t = elapsed_t + dt
    temperature = temperature + dt

plt.plot(time, x1_accel, 'b-', label='IMU 1')
plt.plot(time, x2_accel, 'g-', label='IMU 2')
plt.plot(time,x_true,'r-', label='True Acceleration')
plt.xlabel('Time (s)')
plt.ylabel('Measured Acceleration (m/s^2)')
plt.legend()
plt.savefig('imu_compare.png')
plt.show()
