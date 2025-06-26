import math
import sys
sys.path.append(r'C:/Users/cgoodin/Desktop/goodin_docs/repos/mavs/src/mavs_python')
import mavs_interface as mavs
import mavs_python_paths
mavs_data_path = mavs_python_paths.mavs_data_path

# create the different terrain types to test
terrains = []
ditch = mavs.MavsTerrainCreator()
ditch.CreateTrapezoidalTerrain(6.0, 12.0, 0.5, 20.0)
terrains.append(ditch)

ditch = mavs.MavsTerrainCreator()
ditch.CreateTrapezoidalTerrain(6.0, 12.0, 1.0, 20.0)
terrains.append(ditch)

ditch = mavs.MavsTerrainCreator()
ditch.CreateTrapezoidalTerrain(6.0, 12.0, 1.5, 20.0)
terrains.append(ditch)

ditch = mavs.MavsTerrainCreator()
ditch.CreateTrapezoidalTerrain(6.0, 12.0, 2.0, 20.0)
terrains.append(ditch)

hill = mavs.MavsTerrainCreator()
hill.CreateTrapezoidalTerrain(6.0, 12.0, -2.0, 20.0)
terrains.append(hill)

slope = mavs.MavsTerrainCreator()
slope.CreateSlopedTerrain(0.25)
terrains.append(slope)

rough = mavs.MavsTerrainCreator()
rough.CreateRoughTerrain(0.065)
terrains.append(rough)

parabolic = mavs.MavsTerrainCreator()
parabolic.CreateParabolicTerrain(0.005)
terrains.append(parabolic)

nterrains = len(terrains)
for i in range(nterrains):
    # create the MAVS scene
    scene_ptr = terrains[i].CreateMavsScenePointer(-50.0, -25.0, 200.0, 25.0, 0.5 )

    # Create a MAVS environment and add the scene to it
    env = mavs.MavsEnvironment()
    env.SetScene(scene_ptr)
    env.SetTime(19) # 0-23
    env.SetTurbidity(8.0) # 2-10

    veh = mavs.MavsRp3d()
    veh_file = 'mrzr4_tires_low_gear.json'
    veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
    veh.SetInitialPosition(0.0, 0.0, 0.0) # in global ENU
    veh.SetInitialHeading(0.0) # in radians

    cam = mavs.MavsCamera()
    cam.Initialize(480,320,0.00525,0.0035,0.0035)
    cam.SetOffset([-12.0,0.0,2.0],[1.0,0.0,0.0,0.0])
    cam.SetGammaAndGain(0.85,1.0)
    cam.RenderShadows(True)

    dt = 1.0/100.0 
    elapsed_time = 0.0
    n = 0 
    while (elapsed_time<15.0):

        veh.Update(env, 0.3, 0.0, 0.0, dt)

        if n%4==0:
            pos = veh.GetPosition()
            yaw = veh.GetHeading()
            ori = [math.cos(0.5*yaw), 0.0, 0.0, math.sin(0.5*yaw)]
            cam.SetPose(pos,ori)
            cam.Update(env,dt)
            cam.Display()
        
        n = n+1
        elapsed_time = elapsed_time + dt