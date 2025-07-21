import math
import sys
import time
sys.path.append(r'C:/Users/cgoodin/Desktop/goodin_docs/repos/mavs/src/mavs_python')
#sys.path.append(r'/cavs/projects/MAVS/software/mavs/src/mavs_python')
import mavs_interface as mavs
import mavs_python_paths
mavs_data_path = mavs_python_paths.mavs_data_path

class PidController:
    def __init__(self, kp, ki, kd, setpoint=0, output_limits=(0, 1)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
        self.output_min, self.output_max = output_limits

    def Update(self, measured_value):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0.0:
            dt = 1e-16

        error = self.setpoint - measured_value
        derivative = (error - self.last_error) / dt

        # Provisional integral update
        new_integral = self.integral + error * dt

        # Compute raw output
        output = (self.kp * error + self.ki * new_integral + self.kd * derivative)

        # Clamp output
        clamped_output = max(self.output_min, min(self.output_max, output))

        # Anti-windup: only update integral if output is not saturated
        if clamped_output == output:
            self.integral = new_integral

        self.last_error = error
        self.last_time = current_time

        return clamped_output

# create the different terrain types to test
terrains = []
ditch_nogo = mavs.MavsTerrainCreator()
ditch_nogo.AddTrapezoidalFeature(6.0, 12.0, 3.0, 20.0)
terrains.append(ditch_nogo)

ditch = mavs.MavsTerrainCreator()
ditch.AddTrapezoidalFeature(6.0, 12.0, 2.0, 20.0)
ditch.AddTrapezoidalFeature(6.0, 12.0, -2.0, 40.0)
terrains.append(ditch)

slope = mavs.MavsTerrainCreator()
slope.AddSlopeFeature(0.25)
slope.AddRoughFeature(0.065)
terrains.append(slope)

parabolic = mavs.MavsTerrainCreator()
parabolic.AddParabolicFeature(0.005)
parabolic.AddHoleFeature(20.0, 0.0, 2.0, 10.0, 1.0)
terrains.append(parabolic)

desired_speed = 5.0
frame_num = 0
nterrains = len(terrains)
for i in range(nterrains):
    # create the MAVS scene
    scene_ptr = terrains[i].CreateMavsScenePointer(-50.0, -25.0, 200.0, 25.0, 0.5 )

    # Create a MAVS environment and add the scene to it
    env = mavs.MavsEnvironment()
    env.SetScene(scene_ptr)
    env.SetTime(19) # 0-23
    #env.SetTurbidity(8.0) # 2-10

    veh = mavs.MavsRp3d()
    veh_file = 'mrzr4_tires_low_gear.json'
    veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
    veh.SetInitialPosition(0.0, 0.0, 0.0) # in global ENU
    veh.SetInitialHeading(0.0) # in radians

    cam = mavs.MavsCamera()
    #cam.Initialize(960,640,0.00525,0.0035,0.0035)
    cam.Initialize(640,428,0.00525,0.0035,0.0035)
    #cam.SetOffset([-12.0,0.0,2.0],[1.0,0.0,0.0,0.0])
    cam.SetOffset([0.0,6.0,0.5],[0.7071,0.0,0.0,-0.7071])
    cam.SetGammaAndGain(0.85,1.0)
    cam.RenderShadows(True)

    dt = 1.0/100.0 
    nstep = int(0.04/dt)
    elapsed_time = 0.0
    n = 0 
    pid = PidController(kp=0.5, ki=0.1, kd=0.05, setpoint=5.0)
    while (elapsed_time<15.0):
        speed = veh.GetSpeed()
        throttle = pid.Update(speed)
        veh.Update(env, throttle, 0.0, 0.0, dt)

        if n%nstep==0:
            pos = veh.GetPosition()
            yaw = veh.GetHeading()
            ori = [math.cos(0.5*yaw), 0.0, 0.0, math.sin(0.5*yaw)]
            cam.SetPose(pos,ori)
            cam.Update(env,dt)
            cam.Display()
            #cam.SaveCameraImage(str(frame_num).zfill(5)+"_image.bmp")
            frame_num += 1
        
        n = n+1
        elapsed_time = elapsed_time + dt