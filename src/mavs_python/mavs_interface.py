#Non-Commercial License - Mississippi State University Autonomous Vehicle Software (MAVS)
#
#ACKNOWLEDGEMENT:
#Mississippi State University, Center for Advanced Vehicular Systems (CAVS)
#
#*NOTICE*
#Thank you for your interest in MAVS, we hope it serves you well!  We have a few small requests to ask of you that will help us continue to create innovative platforms:
#-To share MAVS with a friend, please ask them to visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to register and download MAVS for free.
#-Feel free to create derivative works for non-commercial purposes, i.e. academics, U.S. government, and industry research.  If commercial uses are intended, please contact us for a commercial license at otm@msstate.edu or jonathan.rudd@msstate.edu.
#-Finally, please use the ACKNOWLEDGEMENT above in your derivative works.  This helps us both!
#
#Please visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to view the full license, or email us if you have any questions.
#
#Copyright 2018 (C) Mississippi State University

## @mainpage Python interface to the MSU Autonomous Vehicle Simulator (MAVS).
#
# API documentation for the MAVS python package.
#
# For access to the source code, see https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator.
# 
# For installation and user guide, see the wiki at https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator/wikis/home.

## @package mavs_interface
# This module provides classes and functions for interfacing with the MAVS library.
#
# Include it in your code like this:
# import mavs_interfaces as mavs
#
# MAVS is natively in C++, with C interfaces written to make features accessible from python.

import ctypes
import math
import sys
import json
import time
import mavs_lib_loader

mavs_lib = mavs_lib_loader.LoadMavsLib()
mavs_data_path = mavs_lib_loader.GetMavsDataPath()

#---- Definitions for C interface functions to MAVS --------#
mavs_lib.NewPointList2D.restype = ctypes.c_void_p;
mavs_lib.DeletePointList2D.restype = ctypes.c_void_p 
mavs_lib.DeletePointList2D.argtypes = [ctypes.c_void_p]
mavs_lib.AddPointToList2D.restypes = ctypes.c_void_p
mavs_lib.AddPointToList2D.argtypes = [ctypes.c_void_p,ctypes.c_float,ctypes.c_float]
mavs_lib.DeletePointList4D.restype = ctypes.c_void_p 
mavs_lib.DeletePointList4D.argtypes = [ctypes.c_void_p]
mavs_lib.AddPointToList4D.restypes = ctypes.c_void_p
mavs_lib.AddPointToList4D.argtypes = [ctypes.c_void_p,ctypes.c_float,ctypes.c_float, ctypes.c_float, ctypes.c_float]
#------ Embree ray tracer ------#
mavs_lib.NewEmbreeScene.restype = ctypes.c_void_p
mavs_lib.LoadEmbreeScene.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
mavs_lib.LoadEmbreeScene.restype = ctypes.c_void_p
mavs_lib.LoadEmbreeSceneWithRandomSeed.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
mavs_lib.LoadEmbreeSceneWithRandomSeed.restype = ctypes.c_void_p
mavs_lib.WriteEmbreeSceneStats.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
mavs_lib.WriteEmbreeSceneStats.restype = ctypes.c_void_p
mavs_lib.DeleteEmbreeScene.restype = ctypes.c_void_p
mavs_lib.DeleteEmbreeScene.argtypes = [ctypes.c_void_p]
mavs_lib.TurnOnMavsSceneLabeling.restype = ctypes.c_void_p
mavs_lib.TurnOnMavsSceneLabeling.argtypes = [ctypes.c_void_p]
mavs_lib.TurnOffMavsSceneLabeling.restype = ctypes.c_void_p
mavs_lib.TurnOffMavsSceneLabeling.argtypes = [ctypes.c_void_p]
mavs_lib.GetSurfaceHeight.restype = ctypes.c_float
mavs_lib.GetSurfaceHeight.argtypes = [ctypes.c_void_p,ctypes.c_float,ctypes.c_float]
#------ Animations -------#
mavs_lib.NewMavsAnimation.restype = ctypes.c_void_p
mavs_lib.DeleteMavsAnimation.restype = ctypes.c_void_p
mavs_lib.DeleteMavsAnimation.argtypes = [ctypes.c_void_p]
mavs_lib.LoadMavsAnimation.restype = ctypes.c_void_p
mavs_lib.LoadMavsAnimation.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p]
mavs_lib.LoadAnimationPathFile.restype = ctypes.c_void_p
mavs_lib.LoadAnimationPathFile.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
mavs_lib.AddAnimationToScene.restype = ctypes.c_int
mavs_lib.AddAnimationToScene.argtypes = [ctypes.c_void_p,ctypes.c_void_p]
mavs_lib.SetAnimationPositionInScene.restype = ctypes.c_void_p
mavs_lib.SetAnimationPositionInScene.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.SetMavsAnimationScale.restype = ctypes.c_void_p
mavs_lib.SetMavsAnimationScale.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.MoveMavsAnimationToWaypoint.restypes = ctypes.c_void_p
mavs_lib.MoveMavsAnimationToWaypoint.argtypes = [ctypes.c_void_p,ctypes.c_float,ctypes.c_float,ctypes.c_float]
mavs_lib.SetMavsAnimationBehavior.restypes = ctypes.c_void_p
mavs_lib.SetMavsAnimationBehavior.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
mavs_lib.SetMavsAnimationSpeed.restypes = ctypes.c_void_p
mavs_lib.SetMavsAnimationSpeed.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.SetMavsAnimationPosition.restypes = ctypes.c_void_p
mavs_lib.SetMavsAnimationPosition.argtypes = [ctypes.c_void_p,ctypes.c_float,ctypes.c_float]
mavs_lib.SetMavsAnimationHeading.restypes = ctypes.c_void_p
mavs_lib.SetMavsAnimationHeading.argtypes = [ctypes.c_void_p, ctypes.c_float]
mavs_lib.SetMavsAnimationRotations.restypes = ctypes.c_void_p
mavs_lib.SetMavsAnimationRotations.argtypes = [ctypes.c_void_p, ctypes.c_bool, ctypes.c_bool]
#------ Environment ------#
mavs_lib.NewMavsEnvironment.restype = ctypes.c_void_p
mavs_lib.DeleteMavsEnvironment.restype = ctypes.c_void_p
mavs_lib.DeleteMavsEnvironment.argtypes = [ctypes.c_void_p]
mavs_lib.AdvanceEnvironmentTime.restype = ctypes.c_void_p
mavs_lib.AdvanceEnvironmentTime.argtypes = [ctypes.c_void_p, ctypes.c_float]
mavs_lib.GetSceneDensity.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetSceneDensity.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float,
                                     ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.GetAnimationPosition.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetAnimationPosition.argtypes = [ctypes.c_void_p, ctypes.c_int]
mavs_lib.SetEnvironmentScene.argtypes = [ctypes.c_void_p,ctypes.c_void_p]
mavs_lib.SetEnvironmentScene.restype = ctypes.c_void_p
mavs_lib.FreeEnvironmentScene.argtypes = [ctypes.c_void_p]
mavs_lib.FreeEnvironmentScene.restype = ctypes.c_void_p
mavs_lib.AddActorToEnvironment.restype = ctypes.c_int
mavs_lib.AddActorToEnvironment.argtypes = [ctypes.c_void_p,ctypes.c_char_p, ctypes.c_bool]
mavs_lib.SetActorPosition.restype = ctypes.c_void_p
mavs_lib.SetActorPosition.argtypes = [ctypes.c_void_p,ctypes.c_int, ctypes.c_float*3,ctypes.c_float*4]
mavs_lib.UpdateParticleSystems.restype = ctypes.c_void_p
mavs_lib.UpdateParticleSystems.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.AddDustToActor.restype = ctypes.c_void_p
mavs_lib.AddDustToActor.argtypes = [ctypes.c_void_p,ctypes.c_int]
mavs_lib.AddDustToActorColor.restype = ctypes.c_void_p
mavs_lib.AddDustToActorColor.argtypes = [ctypes.c_void_p,ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.AddDustToEnvironment.restype = ctypes.c_void_p
mavs_lib.AddDustToEnvironment.argtypes = [ctypes.c_void_p,ctypes.c_float, ctypes.c_float,
                                          ctypes.c_float, ctypes.c_float, ctypes.c_float,
                                          ctypes.c_float, ctypes.c_float, ctypes.c_float, 
                                          ctypes.c_float]
mavs_lib.SetRainRate.restype = ctypes.c_void_p
mavs_lib.SetRainRate.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.SetSnowRate.restype = ctypes.c_void_p
mavs_lib.SetSnowRate.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.SetSnowAccumulation.restype = ctypes.c_void_p
mavs_lib.SetSnowAccumulation.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.SetTurbidity.restype = ctypes.c_void_p
mavs_lib.SetTurbidity.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.SetAlbedo.restype = ctypes.c_void_p
mavs_lib.SetAlbedo.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.SetWind.restype = ctypes.c_void_p
mavs_lib.SetWind.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
mavs_lib.SetFog.restype = ctypes.c_void_p
mavs_lib.SetFog.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.SetTerrainProperties.restype = ctypes.c_void_p
mavs_lib.SetTerrainProperties.argtypes = [ctypes.c_void_p,ctypes.c_char_p,ctypes.c_float]
mavs_lib.SetTime.restype = ctypes.c_void_p
mavs_lib.SetTime.argtypes = [ctypes.c_void_p,ctypes.c_int]
mavs_lib.TurnSkyOnOff.restype = ctypes.c_void_p
mavs_lib.TurnSkyOnOff.argtypes = [ctypes.c_void_p,ctypes.c_bool]
mavs_lib.SetSkyColor.restype = ctypes.c_void_p
mavs_lib.SetSkyColor.argtypes = [ctypes.c_void_p,ctypes.c_float,ctypes.c_float,ctypes.c_float]
mavs_lib.SetSunColor.restype = ctypes.c_void_p
mavs_lib.SetSunColor.argtypes = [ctypes.c_void_p,ctypes.c_float,ctypes.c_float,ctypes.c_float]
mavs_lib.SetSunLocation.restype = ctypes.c_void_p
mavs_lib.SetSunLocation.argtypes = [ctypes.c_void_p,ctypes.c_float,ctypes.c_float]
mavs_lib.SetSunSolidAngle.restype = ctypes.c_void_p
mavs_lib.SetSunSolidAngle.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.SetTimeSeconds.restype = ctypes.c_void_p
mavs_lib.SetTimeSeconds.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]
mavs_lib.SetDate.restype = ctypes.c_void_p
mavs_lib.SetDate.argtypes = [ctypes.c_void_p,ctypes.c_int,ctypes.c_int,ctypes.c_int]
mavs_lib.SetCloudCover.restype = ctypes.c_void_p
mavs_lib.SetCloudCover.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.AddPointLight.restype = ctypes.c_int
mavs_lib.AddPointLight.argtypes = [ctypes.c_void_p, 
                                   ctypes.c_float, ctypes.c_float, ctypes.c_float, 
                                   ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.AddSpotLight.restype = ctypes.c_int
mavs_lib.AddSpotLight.argtypes = [ctypes.c_void_p, 
                                   ctypes.c_float, ctypes.c_float, ctypes.c_float, 
                                   ctypes.c_float, ctypes.c_float, ctypes.c_float,
                                   ctypes.c_float, ctypes.c_float, ctypes.c_float,
                                   ctypes.c_float]
mavs_lib.MoveLight.argtypes = [ctypes.c_void_p, ctypes.c_int,
                                   ctypes.c_float, ctypes.c_float, ctypes.c_float, 
                                   ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.GetNumberOfObjectsInEnvironment.argtypes=[ctypes.c_void_p]
mavs_lib.GetNumberOfObjectsInEnvironment.restype = ctypes.c_int
mavs_lib.GetObjectBoundingBox.argtypes = [ctypes.c_void_p, ctypes.c_int]
mavs_lib.GetObjectBoundingBox.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetObjectName.argtypes = [ctypes.c_void_p, ctypes.c_int]
mavs_lib.GetObjectName.restype = ctypes.c_char_p
mavs_lib.SetLocalOrigin.argtypes = [ctypes.c_void_p,ctypes.c_double, ctypes.c_double, ctypes.c_double]
mavs_lib.SetLocalOrigin.restype = ctypes.c_void_p
#------ Mavs Plotting utility ----------#
mavs_lib.NewMavsPlotter.restype = ctypes.c_void_p 
mavs_lib.DeleteMavsPlotter.restype = ctypes.c_void_p
mavs_lib.DeleteMavsPlotter.argtypes = [ctypes.c_void_p]
mavs_lib.PlotColorMatrix.restype = ctypes.c_void_p
mavs_lib.PlotColorMatrix.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
mavs_lib.PlotGrayMatrix.restype = ctypes.c_void_p
mavs_lib.PlotGrayMatrix.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
mavs_lib.PlotTrajectory.restype = ctypes.c_void_p
mavs_lib.PlotTrajectory.argtypes =  [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]
mavs_lib.AddPlotToTrajectory.restype = ctypes.c_void_p
mavs_lib.AddPlotToTrajectory.argtypes =  [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]
#------ Vehicle functions -----#
mavs_lib.NewMavsRp3dVehicle.restype = ctypes.c_void_p
mavs_lib.LoadMavsRp3dVehicle.restype = ctypes.c_void_p
mavs_lib.LoadMavsRp3dVehicle.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
mavs_lib.SetMavsRp3dVehicleReloadVis.restype = ctypes.c_void_p
mavs_lib.SetMavsRp3dVehicleReloadVis.argtypes = [ctypes.c_void_p, ctypes.c_bool]
mavs_lib.GetRp3dVehicleTireDeflection.restype = ctypes.c_float
mavs_lib.GetRp3dTireNormalForce.argtypes = [ctypes.c_void_p, ctypes.c_int]
mavs_lib.GetRp3dTireNormalForce.restype = ctypes.c_float
mavs_lib.GetRp3dTireSlip.argtypes = [ctypes.c_void_p, ctypes.c_int]
mavs_lib.GetRp3dTireSlip.restype = ctypes.c_float
mavs_lib.GetRp3dTireSteeringAngle.argtypes = [ctypes.c_void_p, ctypes.c_int]
mavs_lib.GetRp3dTireSteeringAngle.restype = ctypes.c_float
mavs_lib.GetRp3dTireForces.argtypes = [ctypes.c_void_p, ctypes.c_int]
mavs_lib.GetRp3dTireForces.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.SetRp3dExternalForce.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.GetRp3dLookTo.argtypes = [ctypes.c_void_p]
mavs_lib.GetRp3dLookTo.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetRp3dTireAngularVelocity.argtypes = [ctypes.c_void_p, ctypes.c_int]
mavs_lib.GetRp3dTireAngularVelocity.restype = ctypes.c_float
mavs_lib.GetMavsVehicleTirePositionAndOrientation.argtypes = [ctypes.c_void_p, ctypes.c_int]
mavs_lib.GetMavsVehicleTirePositionAndOrientation.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetRp3dVehicleTireDeflection.argtypes = [ctypes.c_void_p, ctypes.c_int]
mavs_lib.SetRp3dTerrain.restype = ctypes.c_void_p
mavs_lib.SetRp3dTerrain.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_float, ctypes.c_char_p, ctypes.c_float, ctypes.c_float]
mavs_lib.SetRp3dGravity.restype = ctypes.c_void_p
mavs_lib.SetRp3dGravity.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.GetRp3dLatAccel.restype = ctypes.c_float
mavs_lib.GetRp3dLatAccel.argtypes = [ctypes.c_void_p]
mavs_lib.GetRp3dLonAccel.restype = ctypes.c_float
mavs_lib.GetRp3dLonAccel.argtypes = [ctypes.c_void_p]
mavs_lib.SetRp3dUseDrag.argtypes = [ctypes.c_void_p, ctypes.c_bool]
mavs_lib.SetRp3dUseDrag.restype = ctypes.c_void_p
mavs_lib.NewChronoVehicle.restype = ctypes.c_void_p
mavs_lib.LoadChronoVehicle.restype = ctypes.c_void_p
mavs_lib.LoadChronoVehicle.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
mavs_lib.GetChronoTireNormalForce.argtypes = [ctypes.c_void_p, ctypes.c_int]
mavs_lib.GetChronoTireNormalForce.restype = ctypes.c_float
mavs_lib.UpdateMavsVehicle.restype = ctypes.c_void_p
mavs_lib.UpdateMavsVehicle.argtypes = [ctypes.c_void_p,ctypes.c_void_p,ctypes.c_float,ctypes.c_float,ctypes.c_float]
mavs_lib.DeleteMavsVehicle.restype = ctypes.c_void_p
mavs_lib.DeleteMavsVehicle.argtypes = [ctypes.c_void_p]
mavs_lib.GetMavsVehicleFullState.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetMavsVehicleFullState.argtypes = [ctypes.c_void_p]
mavs_lib.GetMavsVehiclePosition.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetMavsVehiclePosition.argtypes = [ctypes.c_void_p]
mavs_lib.GetMavsVehicleVelocity.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetMavsVehicleVelocity.argtypes = [ctypes.c_void_p]
mavs_lib.GetMavsVehicleOrientation.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetMavsVehicleOrientation.argtypes = [ctypes.c_void_p]
mavs_lib.GetMavsVehicleHeading.restype = ctypes.c_float
mavs_lib.GetMavsVehicleHeading.argtypes = [ctypes.c_void_p]
mavs_lib.GetMavsVehicleSpeed.restype = ctypes.c_float
mavs_lib.GetMavsVehicleSpeed.argtypes = [ctypes.c_void_p]
mavs_lib.SetMavsVehiclePosition.restype = ctypes.c_void_p 
mavs_lib.SetMavsVehiclePosition.argtypes = [ctypes.c_void_p,ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.SetMavsVehicleHeading.restype = ctypes.c_void_p 
mavs_lib.SetMavsVehicleHeading.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.AddHeadlightsToVehicle.restype = ctypes.POINTER(ctypes.c_int)
mavs_lib.AddHeadlightsToVehicle.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
mavs_lib.MoveHeadlights.restype = ctypes.c_void_p
mavs_lib.MoveHeadlights.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_int, ctypes.c_int]
#------ Vehicle Controller Functions -----#
mavs_lib.NewVehicleController.restype = ctypes.c_void_p
mavs_lib.DeleteVehicleController.restype = ctypes.c_void_p
mavs_lib.DeleteVehicleController.argtypes = [ctypes.c_void_p]
mavs_lib.SetControllerDesiredSpeed.restype = ctypes.c_void_p 
mavs_lib.SetControllerDesiredSpeed.argtypes = [ctypes.c_void_p, ctypes.c_float]
mavs_lib.SetControllerSpeedParams.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.SetControllerSpeedParams.restype = ctypes.c_void_p
mavs_lib.GetControllerDrivingCommand.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetControllerDrivingCommand.argtypes = [ctypes.c_void_p, ctypes.c_float]
mavs_lib.SetControllerDesiredPath.restype = ctypes.c_void_p 
mavs_lib.SetControllerDesiredPath.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.c_int]
mavs_lib.SetControllerVehicleState.restype = ctypes.c_void_p 
mavs_lib.SetControllerVehicleState.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.SetControllerWheelbase.restype = ctypes.c_void_p
mavs_lib.SetControllerWheelbase.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.SetControllerMaxSteeringAngle.restype = ctypes.c_void_p
mavs_lib.SetControllerMaxSteeringAngle.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.SetControllerMinLookAhead.restype = ctypes.c_void_p
mavs_lib.SetControllerMinLookAhead.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.SetControllerMaxLookAhead.restype = ctypes.c_void_p
mavs_lib.SetControllerMaxLookAhead.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.SetControllerSteeringScale.restype = ctypes.c_void_p
mavs_lib.SetControllerSteeringScale.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.SetLooping.restype = ctypes.c_void_p
mavs_lib.SetLooping.argtypes = [ctypes.c_void_p]
#------ RgbCamera -----#
mavs_lib.SetMavsPathTracerCameraNormalization.restype = ctypes.c_void_p 
mavs_lib.SetMavsPathTracerCameraNormalization.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
mavs_lib.SetMavsPathTracerFixPixels.restype = ctypes.c_void_p 
mavs_lib.SetMavsPathTracerFixPixels.argtypes = [ctypes.c_void_p, ctypes.c_bool]
mavs_lib.NewMavsRgbCamera.restype = ctypes.c_void_p
mavs_lib.SaveMavsCameraImage.restype = ctypes.c_void_p
mavs_lib.SaveMavsCameraImage.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
mavs_lib.GetCameraBuffer.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetCameraBuffer.argtypes = [ctypes.c_void_p]
mavs_lib.GetCameraBufferSize.restype = ctypes.c_int
mavs_lib.GetCameraBufferSize.argtypes = [ctypes.c_void_p]
mavs_lib.GetCameraBufferWidth.restype = ctypes.c_int
mavs_lib.GetCameraBufferWidth.argtypes = [ctypes.c_void_p]
mavs_lib.GetCameraBufferHeight.restype = ctypes.c_int
mavs_lib.GetCameraBufferHeight.argtypes = [ctypes.c_void_p]
mavs_lib.GetCameraBufferDepth.restype = ctypes.c_int
mavs_lib.GetCameraBufferDepth.argtypes = [ctypes.c_void_p]
mavs_lib.NewMavsRgbCameraDimensions.restype = ctypes.c_void_p
mavs_lib.NewMavsRgbCameraDimensions.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float] 
mavs_lib.NewMavsPathTraceCamera.restype = ctypes.c_void_p
mavs_lib.NewMavsPathTraceCamera.argtypes = [ctypes.c_char_p, ctypes.c_int,ctypes.c_int,ctypes.c_float]
mavs_lib.NewMavsPathTraceCameraExplicit.restype = ctypes.c_void_p
mavs_lib.NewMavsPathTraceCameraExplicit.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_float, ctypes.c_float, 
                                                    ctypes.c_float, ctypes.c_float, ctypes.c_int,ctypes.c_int,ctypes.c_float]
#mavs_lib.NewMavsPathTraceCameraLowRes.restype = ctypes.c_void_p
#mavs_lib.NewMavsPathTraceCameraLowRes.argtypes = [ctypes.c_int,ctypes.c_int,ctypes.c_float]
#mavs_lib.NewMavsPathTraceCameraHighRes.restype = ctypes.c_void_p
#mavs_lib.NewMavsPathTraceCameraHighRes.argtypes = [ctypes.c_int,ctypes.c_int,ctypes.c_float]
#mavs_lib.NewMavsPathTraceCameraHalfHighRes.restype = ctypes.c_void_p
#mavs_lib.NewMavsPathTraceCameraHalfHighRes.argtypes = [ctypes.c_int,ctypes.c_int,ctypes.c_float]
mavs_lib.NewMavsCameraModel.restype = ctypes.c_void_p
mavs_lib.NewMavsCameraModel.argtypes = [ctypes.c_char_p]
mavs_lib.SetMavsCameraShadows.restype = ctypes.c_void_p
mavs_lib.SetMavsCameraShadows.argtypes = [ctypes.c_void_p,ctypes.c_bool]
mavs_lib.SetMavsCameraBlur.restype = ctypes.c_void_p
mavs_lib.SetMavsCameraBlur.argtypes = [ctypes.c_void_p,ctypes.c_bool]
mavs_lib.SetMavsCameraAntiAliasingFactor.restype = ctypes.c_void_p
mavs_lib.SetMavsCameraAntiAliasingFactor.argtypes = [ctypes.c_void_p,ctypes.c_int]
mavs_lib.SetMavsCameraEnvironmentProperties.restype = ctypes.c_void_p
mavs_lib.SetMavsCameraEnvironmentProperties.argtypes = [ctypes.c_void_p,ctypes.c_void_p]
mavs_lib.SetMavsCameraLensDrops.restype = ctypes.c_void_p
mavs_lib.SetMavsCameraLensDrops.argtypes = [ctypes.c_void_p,ctypes.c_bool]
mavs_lib.FreeCamera.restype = ctypes.c_void_p
mavs_lib.FreeCamera.argtypes = [ctypes.c_void_p]
mavs_lib.SetMavsCameraElectronics.restype = ctypes.c_void_p
mavs_lib.SetMavsCameraElectronics.argtypes = [ctypes.c_void_p,ctypes.c_float,ctypes.c_float]
mavs_lib.SetMavsCameraTempAndSaturation.restype = ctypes.c_void_p
mavs_lib.SetMavsCameraTempAndSaturation.argtypes = [ctypes.c_void_p,ctypes.c_float,ctypes.c_float]
mavs_lib.GetCameraGamma.restype = ctypes.c_float
mavs_lib.GetCameraGamma.argtypes = [ctypes.c_void_p]
mavs_lib.GetCameraGain.restype = ctypes.c_float
mavs_lib.GetCameraGain.argtypes = [ctypes.c_void_p]
mavs_lib.ConvertToRccb.restype = ctypes.c_void_p
mavs_lib.ConvertToRccb.argtypes = [ctypes.c_void_p]
mavs_lib.GetDrivingCommandFromCamera.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetDrivingCommandFromCamera.argtypes = [ctypes.c_void_p]
#------ Red Edge functions -----------------------------------------------#
mavs_lib.NewMavsRedEdge.restype = ctypes.c_void_p
mavs_lib.SaveRedEdge.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
mavs_lib.SaveRedEdge.restype = ctypes.c_void_p
mavs_lib.SaveRedEdgeBands.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
mavs_lib.SaveRedEdgeBands.restype = ctypes.c_void_p
mavs_lib.SaveRedEdgeFalseColor.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_char_p]
mavs_lib.SaveRedEdgeFalseColor.restype = ctypes.c_void_p
mavs_lib.DisplayRedEdge.restype = ctypes.c_void_p
mavs_lib.DisplayRedEdge.argtypes = [ctypes.c_void_p]
#----- LWIR Camera functions --------------------------------------------#
mavs_lib.NewMavsLwirCamera.restype = ctypes.c_void_p
mavs_lib.NewMavsLwirCamera.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.LoadLwirThermalData.restype = ctypes.c_void_p
mavs_lib.LoadLwirThermalData.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
#---------MEMS functions ----------------------------------------------#
mavs_lib.NewMavsMems.restype = ctypes.c_void_p
mavs_lib.NewMavsMems.argtypes = [ctypes.c_char_p]
mavs_lib.SetMemsMeasurmentRange.argtypes = [ctypes.c_void_p, ctypes.c_float]
mavs_lib.SetMemsResolution.argtypes = [ctypes.c_void_p, ctypes.c_float]
mavs_lib.SetMemsConstantBias.argtypes = [ctypes.c_void_p,ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.SetMemsNoiseDensity.argtypes = [ctypes.c_void_p,ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.SetMemsBiasInstability.argtypes = [ctypes.c_void_p,ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.SetMemsAxisMisalignment.argtypes = [ctypes.c_void_p,ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.SetMemsRandomWalk.argtypes = [ctypes.c_void_p,ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.SetMemsTemperatureBias.argtypes = [ctypes.c_void_p,ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.SetMemsTemperatureScaleFactor.argtypes = [ctypes.c_void_p,ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.SetMemsAccelerationBias.argtypes = [ctypes.c_void_p,ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.MemsUpdate.argtypes = [ctypes.c_void_p,ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.MemsUpdate.restype = ctypes.POINTER(ctypes.c_float)
#------ Rtk -------#
mavs_lib.NewMavsRtk.restype = ctypes.c_void_p
mavs_lib.SetRtkError.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.SetRtkDroputRate.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.SetRtkWarmupTime.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.GetRtkPosition.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetRtkPosition.argtypes = [ctypes.c_void_p]
mavs_lib.GetRtkOrientation.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetRtkOrientation.argtypes = [ctypes.c_void_p]
#------ Lidar -----#
mavs_lib.GetChamferDistance.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),ctypes.c_int, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]
mavs_lib.GetChamferDistance.restype = ctypes.c_float
mavs_lib.NewMavsLidar.argtypes = [ctypes.c_char_p]
mavs_lib.NewMavsLidar.restype = ctypes.c_void_p
mavs_lib.MavsLidarSetScanPattern.restype = ctypes.c_void_p
mavs_lib.MavsLidarSetScanPattern.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float,
                                             ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.WriteMavsLidarToColorizedCloud.restype = ctypes.c_void_p
mavs_lib.WriteMavsLidarToColorizedCloud.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
mavs_lib.WriteMavsLidarToLabeledCloud.restype = ctypes.c_void_p
mavs_lib.WriteMavsLidarToLabeledCloud.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
mavs_lib.WriteMavsLidarToPcd.restype = ctypes.c_void_p
mavs_lib.WriteMavsLidarToPcd.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
mavs_lib.WriteMavsLidarToLabeledPcd.restype = ctypes.c_void_p
mavs_lib.WriteMavsLidarToLabeledPcd.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
mavs_lib.WriteMavsLidarToLabeledPcdWithNormals.restype = ctypes.c_void_p
mavs_lib.WriteMavsLidarToLabeledPcdWithNormals.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
mavs_lib.SaveMavsLidarImage.restype = ctypes.c_void_p
mavs_lib.SaveMavsLidarImage.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
mavs_lib.SaveProjectedMavsLidarImage.restype = ctypes.c_void_p
mavs_lib.SaveProjectedMavsLidarImage.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
mavs_lib.SetPointCloudColorType.restype = ctypes.c_void_p
mavs_lib.SetPointCloudColorType.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
mavs_lib.AnalyzeCloud.restype = ctypes.c_void_p
mavs_lib.AnalyzeCloud.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_bool]
mavs_lib.GetMavsLidarRegisteredPoints.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetMavsLidarRegisteredPoints.argtypes = [ctypes.c_void_p]
mavs_lib.GetMavsLidarNumberPoints.restype = ctypes.c_int
mavs_lib.GetMavsLidarNumberPoints.argtypes = [ctypes.c_void_p]
mavs_lib.GetMavsLidarUnRegisteredPointsXYZI.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetMavsLidarUnRegisteredPointsXYZI.argtypes = [ctypes.c_void_p]
mavs_lib.GetMavsLidarUnRegisteredPointsXYZIL.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetMavsLidarUnRegisteredPointsXYZIL.argtypes = [ctypes.c_void_p]
mavs_lib.DisplayMavsLidarPerspective.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
mavs_lib.DisplayMavsLidarPerspective.restype = ctypes.c_void_p
mavs_lib.SetMavsSensorVelocity.restype = ctypes.c_void_p
mavs_lib.SetMavsSensorVelocity.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.AddPointsToImage.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
#------ Radar -------------------#
mavs_lib.NewMavsRadar.restype = ctypes.c_void_p
mavs_lib.SetRadarMaxRange.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.SetRadarMaxRange.restype = ctypes.c_void_p
mavs_lib.SetRadarFieldOfView.argtypes = [ctypes.c_void_p,ctypes.c_float, ctypes.c_float]
mavs_lib.SetRadarFieldOfView.restype = ctypes.c_void_p
mavs_lib.SetRadarSampleResolution.argtypes = [ctypes.c_void_p,ctypes.c_float]
mavs_lib.SetRadarSampleResolution.restype = ctypes.c_void_p
mavs_lib.SaveMavsRadarImage.restype = ctypes.c_void_p
mavs_lib.SaveMavsRadarImage.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
mavs_lib.GetRadarReturnLocations.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetRadarReturnLocations.argtypes = [ctypes.c_void_p]
mavs_lib.GetRadarTargets.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetRadarTargets.argtypes = [ctypes.c_void_p]
mavs_lib.GetRadarNumTargets.restype = ctypes.c_int
mavs_lib.GetRadarNumTargets.argtypes = [ctypes.c_void_p]
#------ Sensor functions -----#
mavs_lib.DeleteMavsSensor.restype = ctypes.c_void_p
mavs_lib.DeleteMavsSensor.argtypes = [ctypes.c_void_p]
mavs_lib.UpdateMavsSensor.restype = ctypes.c_void_p
mavs_lib.UpdateMavsSensor.argtypes = [ctypes.c_void_p,ctypes.c_void_p,ctypes.c_float]
mavs_lib.SaveMavsSensorRaw.restype = ctypes.c_void_p
mavs_lib.SaveMavsSensorRaw.argtypes = [ctypes.c_void_p]
mavs_lib.SetMavsSensorPose.restype = ctypes.c_void_p
mavs_lib.SetMavsSensorPose.argtypes = [ctypes.c_void_p,ctypes.c_float*3,ctypes.c_float*4]
mavs_lib.SetMavsSensorRelativePose.restype = ctypes.c_void_p
mavs_lib.SetMavsSensorRelativePose.argtypes = [ctypes.c_void_p,ctypes.c_float*3,ctypes.c_float*4]
mavs_lib.DisplayMavsSensor.restype = ctypes.c_void_p
mavs_lib.DisplayMavsSensor.argtypes = [ctypes.c_void_p]
mavs_lib.SaveMavsSensorAnnotation.restype = ctypes.c_void_p
mavs_lib.SaveMavsSensorAnnotation.argtypes = [ctypes.c_void_p,ctypes.c_void_p,ctypes.c_char_p]
mavs_lib.SaveMavsCameraAnnotationFull.restype = ctypes.c_void_p
mavs_lib.SaveMavsCameraAnnotationFull.argtypes = [ctypes.c_void_p,ctypes.c_void_p,ctypes.c_char_p]
mavs_lib.AnnotateMavsSensorFrame.restype = ctypes.c_void_p
mavs_lib.AnnotateMavsSensorFrame.argtypes = [ctypes.c_void_p,ctypes.c_void_p]
mavs_lib.GetSensorPose.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetSensorPose.argtypes = [ctypes.c_void_p]
#------ Waypoint functions -----#
mavs_lib.LoadAnvelReplayFile.restype = ctypes.c_void_p
mavs_lib.LoadAnvelReplayFile.argtypes = [ctypes.c_char_p]
mavs_lib.LoadWaypointsFromJson.restype = ctypes.c_void_p
mavs_lib.LoadWaypointsFromJson.argtypes = [ctypes.c_char_p]
mavs_lib.GetNumWaypoints.restype = ctypes.c_int
mavs_lib.GetNumWaypoints.argtypes = [ctypes.c_void_p]
mavs_lib.SaveWaypointsAsJson.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
mavs_lib.SaveWaypointsAsJson.restype = ctypes.c_void_p
mavs_lib.GetWaypoint.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetWaypoint.argtypes = [ctypes.c_void_p,ctypes.c_int]
mavs_lib.DeleteMavsWaypoints.restype = ctypes.c_void_p
mavs_lib.DeleteMavsWaypoints.argtypes = [ctypes.c_void_p]
mavs_lib.PutWaypointsOnGround.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.PutWaypointsOnGround.argtypes = [ctypes.c_void_p,ctypes.c_void_p]
#------ Random Scene creator -----#
mavs_lib.CreateSceneFromRandom.restype = ctypes.c_void_p
mavs_lib.CreateSceneFromRandom.argtypes = [ctypes.c_float, ctypes.c_float,
                                           ctypes.c_float, ctypes.c_float,
                                           ctypes.c_float, ctypes.c_float,
                                           ctypes.c_float, ctypes.c_float,
                                           ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p,
                                           ctypes.c_float,  
                                           ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p]
mavs_lib.CreateGapScene.restype = ctypes.c_void_p
mavs_lib.CreateGapScene.argtypes = [ctypes.c_float, ctypes.c_float,
                                    ctypes.c_float, ctypes.c_float,
                                    ctypes.c_char_p, ctypes.c_float,  
                                    ctypes.c_char_p, ctypes.c_char_p,
                                    ctypes.c_float, ctypes.c_float, ctypes.c_float ]
#----- Add rain to existing image -----#
mavs_lib.AddRainToExistingImage.restype = ctypes.c_void_p
mavs_lib.AddRainToExistingImage.argtypes = [ctypes.c_char_p, ctypes.c_float, ctypes.c_bool]
mavs_lib.AddRainToExistingImageRho.restype = ctypes.c_void_p
mavs_lib.AddRainToExistingImageRho.argtypes = [ctypes.c_char_p, ctypes.c_float, ctypes.c_float, ctypes.c_bool]
#---- Mavs OrthoViewer -----------------#
mavs_lib.CreateOrthoViewer.restype = ctypes.c_void_p
mavs_lib.DeleteOrthoViewer.restype = ctypes.c_void_p
mavs_lib.DeleteOrthoViewer.argtypes = [ctypes.c_void_p]
mavs_lib.UpdateOrthoViewerWaypoints.restype = ctypes.c_void_p
mavs_lib.UpdateOrthoViewerWaypoints.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p] 
mavs_lib.UpdateOrthoViewer.restype = ctypes.c_void_p
mavs_lib.UpdateOrthoViewer.argtypes = [ctypes.c_void_p, ctypes.c_void_p] 
mavs_lib.DisplayOrtho.restype = ctypes.c_void_p
mavs_lib.DisplayOrtho.argtypes = [ctypes.c_void_p]
mavs_lib.SaveOrtho.restype = ctypes.c_void_p
mavs_lib.SaveOrtho.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
mavs_lib.GetOrthoBuffer.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetOrthoBuffer.argtypes = [ctypes.c_void_p]
mavs_lib.GetOrthoBufferSize.restype = ctypes.c_int
mavs_lib.GetOrthoBufferSize.argtypes = [ctypes.c_void_p]
#----Rp3d vehicle viewer -------------------------------------------------
mavs_lib.CreateRp3dViewer.restype = ctypes.c_void_p
mavs_lib.DeleteRp3dViewer.argtypes = [ctypes.c_void_p] 
mavs_lib.Rp3dViewerLoadVehicle.argtypes = [ctypes.c_void_p,ctypes.c_char_p]
mavs_lib.Rp3dViewerDisplay.argtypes = [ctypes.c_void_p,ctypes.c_bool] 
mavs_lib.Rp3dViewerUpdate.argtypes = [ctypes.c_void_p,ctypes.c_bool] 
mavs_lib.Rp3dViewerGetSideImage.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.Rp3dViewerGetSideImage.argtypes = [ctypes.c_void_p]
mavs_lib.Rp3dViewerGetFrontImage.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.Rp3dViewerGetFrontImage.argtypes = [ctypes.c_void_p]
mavs_lib.Rp3dViewerSaveSideImage.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
mavs_lib.Rp3dViewerSaveFrontImage.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
mavs_lib.Rp3dViewerGetSideImageSize.argtypes = [ctypes.c_void_p]
mavs_lib.Rp3dViewerGetSideImageSize.restype = ctypes.c_int
mavs_lib.Rp3dViewerGetFrontImageSize.argtypes = [ctypes.c_void_p]
mavs_lib.Rp3dViewerGetFrontImageSize.restype = ctypes.c_int
#---- MAVS material viewer ----------------------------------#
mavs_lib.CreateMaterialViewer.restype = ctypes.c_void_p
mavs_lib.DeleteMaterialViewer.argtypes = [ctypes.c_void_p ]
mavs_lib.DeleteMaterialViewer.restype = ctypes.c_void_p
mavs_lib.UpdateMaterialViewer.argtypes = [ctypes.c_void_p ]
mavs_lib.UpdateMaterialViewer.restype = ctypes.c_void_p
mavs_lib.ResetMaterialViewer.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.ResetMaterialViewer.restype = ctypes.c_void_p
mavs_lib.LoadMaterialViewerMesh.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
mavs_lib.LoadMaterialViewerMesh.restype = ctypes.c_void_p
mavs_lib.GetMaterialViewerNumMats.restype = ctypes.c_int
mavs_lib.GetMaterialViewerNumMats.argtypes = [ctypes.c_void_p]
mavs_lib.GetMaterialViewerMatName.restype = ctypes.c_char_p
mavs_lib.GetMaterialViewerMatName.argtypes = [ctypes.c_void_p, ctypes.c_int]
mavs_lib.GetMaterialViewerSpectrumName.restype = ctypes.c_char_p
mavs_lib.GetMaterialViewerSpectrumName.argtypes = [ctypes.c_void_p, ctypes.c_int]
mavs_lib.GetMaterialViewerMaterial.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetMaterialViewerMaterial.argtypes = [ctypes.c_void_p, ctypes.c_int]
#--------------- DEM ----------------------------------------------------------------------#
mavs_lib.LoadDem.argtypes = [ctypes.c_char_p, ctypes.c_bool, ctypes.c_bool ]
mavs_lib.LoadDem.restype = ctypes.c_void_p
mavs_lib.DownsampleDem.argtypes = [ctypes.c_void_p, ctypes.c_int]
mavs_lib.DisplayDem.argtypes = [ctypes.c_void_p]
mavs_lib.ExportDemToObj.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
mavs_lib.ExportDemToEsriAscii.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
#--------------- MAVS Map Viewer -----------------------------------------------------------#
mavs_lib.NewMavsMapViewer.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.NewMavsMapViewer.restype = ctypes.c_void_p
mavs_lib.DeleteMavsMapViewer.argtypes = [ctypes.c_void_p]
mavs_lib.UpdateMap.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
mavs_lib.AddWaypointsToMap.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.c_int]
mavs_lib.AddCircleToMap.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.AddLineToMap.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
mavs_lib.MapIsOpen.argtypes = [ctypes.c_void_p]
mavs_lib.MapIsOpen.restype = ctypes.c_bool
#-------------- MAVS Oak-D Sensor -----------------------------------------------------------------------#
mavs_lib.NewMavsOakDCamera.restype = ctypes.c_void_p
mavs_lib.GetOakDDepthBuffer.argtypes = [ctypes.c_void_p]
mavs_lib.GetOakDDepthBuffer.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetOakDDepthBufferSize.argtypes = [ctypes.c_void_p]
mavs_lib.GetOakDDepthBufferSize.restype = ctypes.c_int
mavs_lib.GetOakDImageBuffer.argtypes = [ctypes.c_void_p]
mavs_lib.GetOakDImageBuffer.restype = ctypes.POINTER(ctypes.c_float)
mavs_lib.GetOakDImageBufferSize.restype = ctypes.c_int
mavs_lib.GetOakDImageBufferSize.argtypes = [ctypes.c_void_p]
mavs_lib.GetOakDMaxRangeCm.restype = ctypes.c_float
mavs_lib.GetOakDMaxRangeCm.argtypes = [ctypes.c_void_p]
mavs_lib.SetOakDMaxRangeCm.argtypes = [ctypes.c_void_p, ctypes.c_float]
mavs_lib.GetOakDCamera.argtypes = [ctypes.c_void_p]
mavs_lib.GetOakDCamera.restype = ctypes.c_void_p
mavs_lib.SetOakDCameraDisplayType.argtypes = [ctypes.c_void_p, ctypes.c_char_p] 
mavs_lib.CameraDisplayOpen.argtypes = [ctypes.c_void_p]
mavs_lib.CameraDisplayOpen.restype = ctypes.c_bool

def PyStringToChar(py_string):
    """Convert a python string to a C char array.
    
    """
    b_string = py_string.encode('utf-8')
    return ctypes.c_char_p(b_string)

def AddRainToImage(fname,rate,add_drops=False,rho=1.0):
    """Add rain streaks to an existing image.

    Parameters:
    fname (string): Full path to the image file
    rate (float): The rain rate to add, in mm/h
    """
    mavs_lib.AddRainToExistingImageRho(PyStringToChar(fname),ctypes.c_float(rate),ctypes.c_float(rho),ctypes.c_bool(add_drops))

class MavsDem(object):
    """MavsDem class.

    Load an ESRI ASCII DEM file (.asc extension) and manipulate it
    """
    def __init__(self):
        """Constructor for MavsDem

        """
        self.dem = None
    def LoadEsriAscii(self, fname, interp_no_data=False, recenter=False):
        """Load a DEM file

        Must be in ESRI ASCII format:
        https://en.wikipedia.org/wiki/Esri_grid

        Parameters:
        fname (string): The file name to load, including the path.
        interp_no_data (bool): If true, missing data will be filled in using interpoloat
        recenter (bool): If true, the file will be recentered to 0,0
        """
        self.dem = mavs_lib.LoadDem(PyStringToChar(fname), ctypes.c_bool(interp_no_data), ctypes.c_bool(recenter))
    def Downsample(self, downsample_factor):
        """Downsample the DEM by a factor of resample_factor

        Parameters:
        resample_factor (int): Factor to downsample by
        """
        if (self.dem):
            mavs_lib.DownsampleDem(self.dem, ctypes.c_int(downsample_factor))
    def Display(self):
        """Display the current DEM"""
        if (self.dem):
            mavs_lib.DisplayDem(self.dem)
    def SaveAsObj(self, obj_fname):
        """Save the current DEM as an obj file

        Parameters:
        obj_fname (string): The output file name
        """
        if (self.dem):
            mavs_lib.ExportDemToObj(self.dem, PyStringToChar(obj_fname))
    def SaveAsAscii(self, ascii_fname):
        """Save the current DEM as an ESRI ASCII file

        Parameters:
        ascii_fname (string): The output file name
        """
        if (self.dem):
            mavs_lib.ExportDemToEsriAscii(self.dem, PyStringToChar(ascii_fname))

class MavsRp3dViewer(object):
    """MavsRp3dViewer class.

    Displays a front and side view of a vehicle.
    Optionally shows the associated physics objects.
    """
    def __init__(self):
        """Constructor for MavsRp3dViewer

        """
        self.viewer = mavs_lib.CreateRp3dViewer()
    def __del__(self):
        """Destructor for MavsRp3dViewer class"""
        mavs_lib.DeleteRp3dViewer(self.viewer)
    def LoadVehicle(self, vehfile):
        """Load a vehicle file to view.

        Must be a MAVS .json file specifying a vehicle model.

        Parameters:
        fname (string): The file name to load, including the path.
        """
        mavs_lib.Rp3dViewerLoadVehicle(self.viewer,PyStringToChar(vehfile))
    def Display(self, show_debug):
        """Show the front and side views of the vehicle.
        
        Optionally, draw the debug info on the screen.

        Parameters:
        show_debug (bool): True to draw physics debug info
        """
        mavs_lib.Rp3dViewerDisplay(self.viewer,ctypes.c_bool(show_debug))
    def Update(self, show_debug):
        """Show the front and side views of the vehicle without displaying
        
        Optionally, draw the debug info on the screen.

        Parameters:
        show_debug (bool): True to draw physics debug info
        """
        mavs_lib.Rp3dViewerUpdate(self.viewer,ctypes.c_bool(show_debug))
    def GetSideImageBuffer(self):
        """Return a python list with the rgb values of the side-view camera.

        The buffer will read out rows in the sequentially, listing each pixel's r-g-b
        as three floats.

        Returns:
        buffer (list of floats): The side-view camera buffer.
        """
        pointbuff = mavs_lib.Rp3dViewerGetSideImage(self.viewer)
        buffsize = mavs_lib.Rp3dViewerGetSideImageSize(self.viewer)
        buffer = pointbuff[:buffsize]
        return buffer
    def GetFrontImageBuffer(self):
        """Return a python list with the rgb values of the front-view camera.

        The buffer will read out rows in the sequentially, listing each pixel's r-g-b
        as three floats.

        Returns:
        buffer (list of floats): The front-view camera buffer.
        """
        pointbuff = mavs_lib.Rp3dViewerGetFrontImage(self.viewer)
        buffsize = mavs_lib.Rp3dViewerGetFrontImageSize(self.viewer)
        buffer = pointbuff[:buffsize]
        return buffer
    def SaveSideImage(self,fname):
        """Save the side-view to an image file

        Parameters:
        fname (string): The file name to save, including the path.
        """
        mavs_lib.Rp3dViewerSaveSideImage(self.viewer,PyStringToChar(fname))
    def SaveFrontImage(self,fname):
        """Save the front-view to an image file

        Parameters:
        fname (string): The file name to save, including the path.
        """
        mavs_lib.Rp3dViewerSaveFrontImage(self.viewer,PyStringToChar(fname))

class MavsMapViewer(object):
    """Class for viewing a top-down map with waypoints
    
    """
    def __init__(self,llx, lly, urx, ury, res):
        """Constructor for MavsOrthoViewer class.
        
        Attributes:
        map (void): Pointer to a MAVS map viewer.
        
        Parameters:
        llx (float): X-coordinate of the lower-left corner of the map, in local ENU meters
        lly (float): Y-coordinate of the lower-left corner of the map, in local ENU meters
        urx (float): X-coordinate of the upper-right corner of the map, in local ENU meters
        ury (float): Y-coordinate of the upper-right corner of the map, in local ENU meters
        res (float): resolution of the map, in meters
        """
        self.map = mavs_lib.NewMavsMapViewer(ctypes.c_float(llx), ctypes.c_float(lly), ctypes.c_float(urx), ctypes.c_float(ury),ctypes.c_float(res))
    def __del__(self):
        """Destructor for MavsMapeViewer class."""
        mavs_lib.DeleteMavsMapViewer(self.map)
        self.viewer = None
    def Display(self, env):
        """Display the current map
        
        Parameters:
        env (pointer): Pointer to a MAVS environment
        """
        mavs_lib.UpdateMap(self.map, env.obj)
    def IsOpen(self):
        is_open = bool(mavs_lib.MapIsOpen(self.map))
        return is_open
    def AddWaypoints(self, waypoints):
        """Add waypoints to the map display
        
        Parameters:
        waypoints (float): N X 2 list of waypoints
        """
        nwp = len(waypoints)
        x = []
        y = []
        for i in range(nwp):
            x.append(waypoints[i][0])
            y.append(waypoints[i][1])
        x_pointer = (ctypes.c_float * len(x))(*x)
        y_pointer = (ctypes.c_float * len(y))(*y)
        mavs_lib.AddWaypointsToMap(self.map, x_pointer, y_pointer, ctypes.c_int(nwp)) 
    def AddCircle(self, center, radius):
        """Add a circle to the map display
        
        Parameters:
        center (float): Array with the x and y coordinates of the center in local ENU meters
        radius (float): Radius of the cirlce in menters
        """
        mavs_lib.AddCircleToMap(self.map, ctypes.c_float(center[0]), ctypes.c_float(center[1]), ctypes.c_float(radius))
    def AddLine(self, p0, p1):
        """Add a line to the map display
        
        Parameters:
        p0 (float): Array with the x and y coordinates of the first endpoint
        p1 (float): Array with the x and y coordinates of the second endpoint
        """
        mavs_lib.AddLineToMap(self.map, ctypes.c_float(p0[0]), ctypes.c_float(p0[1]), ctypes.c_float(p1[0]), ctypes.c_float(p1[1]))

class MavsOrthoViewer(object):
    """Camera class for a parallel ray camera.

    The orth viewer is a top-down, parallel ray renderer with no shadows.

    Attributes:
    viewer (void): Pointer to a MAVS OrthoViewer.
    """
    def __init__(self):
        """Constructor for MavsOrthoViewer class.
        
        Attributes:
        viewer (void): Pointer to a MAVS camera.
        """
        ## viewer (void): Pointer to a MAVS OrthoViewer.
        self.viewer = mavs_lib.CreateOrthoViewer()
    def __del__(self):
        """Destructor for MavsOrthoViewer class."""
        mavs_lib.DeleteOrthoViewer(self.viewer)
        self.viewer = None
    def Update(self,env):
        """Update the OrthoViewer

        Parameters:
        env (MavsEnvironment): The Mavs environment to render
        """
        mavs_lib.UpdateOrthoViewer(self.viewer,env.obj)
    def UpdateWaypoints(self,env, wp):
        """Update the OrthoViewer with a set of waypoints to plot

        Parameters:
        env (MavsEnvironment): The Mavs environment to render
        wp (MavsWaypoints): The MAVS waypoint object to render
        """
        mavs_lib.UpdateOrthoViewerWaypoints(self.viewer,env.obj, wp.mavs_waypoints)
    def Display(self):
        """Display the current OrthoViewer

        """
        mavs_lib.DisplayOrtho(self.viewer)
    def SaveImage(self,fname):
        """Save the current Ortho View to an image file.

        Parameters:
        fname (string): The file name to save to, including the path.
        """
        mavs_lib.SaveOrtho(self.viewer,PyStringToChar(fname))
    def GetImageBuffer(self):
        """Return a python list with the rgb values of the ortho-camera frame.

        The buffer will read out rows in the sequentially, listing each pixel's r-g-b
        as three floats.

        Returns:
        buffer (list of floats): The ortho camera buffer.
        """
        pointbuff = mavs_lib.GetOrthoBuffer(self.viewer)
        buffsize = mavs_lib.GetOrthoBufferSize(self.viewer)
        buffer = pointbuff[:buffsize]
        return buffer

class MavsMaterial(object):
    """Class that defines python materials.

    Attributes:
    name (string): The material name.
    ka ([float, float, float]): The rgb ambient reflectnace.
    kd ([float, float, float]): The rgb diffuse reflectance.
    ks ([float, float, float]): The rgb specular reflectance.
    ke ([float, float, float]): The rgb emission.
    tr ([float, float, float]): Transmission coefficient.
    ns (float): Specular exponent.
    ni (float): Index of refraction.
    dissolve (float): Not used.
    illum (int): Reflectance model.
    map_kd (string): Diffuse texture map.
    map_ka (string): Ambient texture map.
    map_ks (string): Specular texture map.
    map_ns (string): Phong exponent texture map.
    map_bump (string): Normal map.
    map_d (string): Transparency map.
    disp (string): Height texture map.
    refl (string): Spectral reflectance file.
    """
    def __init__(self):
        """MavsMaterial constructor."""
        ## name (string): The material name.
        self.name = 'default'
        ## ka ([float, float, float]): The rgb ambient reflectnace.
        self.ka = [0.0, 0.0, 0.0]
        ## kd ([float, float, float]): The rgb diffuse reflectance.
        self.kd = [1.0, 1.0, 1.0]
        ## ks ([float, float, float]): The rgb specular reflectance.
        self.ks = [0.0, 0.0, 0.0]
        ## tr ([float, float, float]): Transmission coefficient.
        self.tr = [0.0, 0.0, 0.0]
        ## ke ([float, float, float]): The rgb emission.
        self.ke = [0.0, 0.0, 0.0]
        ## ns (float): Specular exponent.
        self.ns = 2.0
        ## ni (float): Index of refraction.
        self.ni = 0.0
        ## dissolve (float): Not used.
        self.dissolve = 0.0
        ## illum (int): Reflectance model.
        self.illum = 2.0
        ## map_kd (string): Diffuse texture map.
        self.map_kd = ''
        ## map_ka (string): Ambient texture map.
        self.map_ka = ''
        ## map_ks (string): Specular texture map.
        self.map_ks = ''
        ## map_ns (string): Phong exponent texture map.
        self.map_ns = ''
        ## map_bump (string): Normal map.
        self.map_bump = ''
        ## map_d (string): Transparency map.
        self.map_d = ''
        ## disp (string): Height texture map.
        self.disp = ''
        ## refl (string): Spectral reflectance file.
        self.refl = ''

class MavsMaterialViewer(object):
    """Viewer for MAVS materials.

    Attributes:
    viewer (void): Pointer to MAVS material viewer.
    mat_name_list (list of strings): Names of available materials.
    avail_materials (list of MavsMaterials): List of loaded materials.
    num_mats (int): Number of loaded materials.
    """
    def __init__(self):
        """Constructor for the MavsMaterialViewer."""
        ## viewer (void): Pointer to MAVS material viewer.
        self.viewer = mavs_lib.CreateMaterialViewer()
        ## mat_name_list (list of strings): Names of available materials.
        self.mat_name_list = []
        ## avail_materials (list of MavsMaterials): List of loaded materials.
        self.avail_materials = []
        ## num_mats (int): Number of loaded materials.
        self.num_mats = 0
    def __del__(self):
        """Destructor for the MavsMaterialViewer."""
        mavs_lib.DeleteMaterialViewer(self.viewer)
        self.viewer = None
    def Update(self):
        """Update the viewer."""
        mavs_lib.UpdateMaterialViewer(self.viewer)
    def SetMaterial(self,matnum):
        """Set the current matieral.

        Parameters:
        matnum (int): The number of the material to view.
        """
        mavs_lib.ResetMaterialViewer(self.viewer, ctypes.c_float(self.avail_materials[matnum].kd[0]), ctypes.c_float(self.avail_materials[matnum].kd[1]), ctypes.c_float(self.avail_materials[matnum].kd[2]))
    def LoadMaterialsFromObj(self,meshfile):
        """Load all the materials from a given obj file.

        Parameters:
        meshfile (string): FUll path to the .obj file to load.
        """
        mavs_lib.LoadMaterialViewerMesh(self.viewer, PyStringToChar(meshfile))
        self.num_mats = mavs_lib.GetMaterialViewerNumMats(self.viewer)
        for i in range(self.num_mats):
            mname = mavs_lib.GetMaterialViewerMatName(self.viewer,ctypes.c_int(i))
            self.mat_name_list.append(mname.decode("utf-8"))
            mat = MavsMaterial()
            this_mat = mavs_lib.GetMaterialViewerMaterial(self.viewer, ctypes.c_int(i))
            mat.kd[0] = this_mat[0]
            mat.kd[1] = this_mat[1]
            mat.kd[2] = this_mat[2]
            mat.ks[0] = this_mat[3]
            mat.ks[1] = this_mat[4]
            mat.ks[2] = this_mat[5]
            mat.ns = this_mat[6]
            mat.name = mname
            sname =  mavs_lib.GetMaterialViewerSpectrumName(self.viewer,ctypes.c_int(i))
            mat.refl = sname.decode("utf-8")
            self.avail_materials.append(mat)
        return self.mat_name_list

class MavsAnimation(object):
    """MavsAnimation class.

    A MavsAnimation has a sequence of keyframes and behavior associated with it,
    as well as scaling parameters for the keyframes.

    Attributes:
    object (void): Pointer to a MAVS animation.
    """
    def __init__(self):
        """Constructor for a MavsAnimation."""
        ## object (void): Pointer to a MAVS animation.
        self.object = mavs_lib.NewMavsAnimation()
    def __del__(self):
        """Destructor for a MavsAnimation."""
        if (self.object):
            mavs_lib.DeleteMavsAnimation(self.object)
    def Load(self,path,file):
        """Load an animation from an input file.

        The animation input file lists the mesh scaling and keyframes.
        Example input files can be found in mavs/data/actors.

        Parameters:
        path (string): The path to the animation file.
        file (string): The animation file name.
        """
        mavs_lib.LoadMavsAnimation(self.object, PyStringToChar(path), PyStringToChar(file))
    def SetScale(self,scale):
        """Set the scale of the animation.

        Scale the animation equally in the x-y-z dimensions.

        Parameters:
        scale (float): Factor to scale the animation by.
        """
        mavs_lib.SetMavsAnimationScale(self.object,ctypes.c_float(scale))
    def MoveToWaypoint(self,dt,x,y):
        """Move the animation to a specified waypoint.

        Moves the animation to an x-y point and automatically places it on the ground.
        The dt parameter is the length of time it takes to move the animation, which
        will be used to calculate its velocity.

        Parameters:
        dt (float): The duration in seconds to move the animation.
        x (float): The global x ENU coordinate to move to.
        y (float): The global y ENU coordinate to move to.
        """
        mavs_lib.MoveMavsAnimationToWaypoint(self.object,ctypes.c_float(dt),ctypes.c_float(x),ctypes.c_float(y))
    #def SetFrameRate(self,fr):
    #    pass
    def LoadPathFile(self,path_file):
        """Load a path file for the animation to follow.

        Path files are in the .vprp format. Examples can be found in mavs/data/waypoints.

        Parameters:
        path_file (string): Full path to the file to load.
        """
        mavs_lib.LoadAnimationPathFile(self.object,PyStringToChar(path_file))
    def SetSpeed(self,speed):
        """Set the speed of the animation in m/s.

        The animation will move following a prescribed behavior. 
        Call this to set the speed of the linear motion in m/s.

        Parameters:
        speed (float): Desired speed.
        """
        mavs_lib.SetMavsAnimationSpeed(self.object,ctypes.c_float(speed))
    def SetBehavior(self,behavior):
        """Set the motion behavior of the animation.

        Options are 'wander', 'straight', or 'circle'.

        Parameters:
        behavior (string): The desired behavior.
        """
        mavs_lib.SetMavsAnimationBehavior(self.object,PyStringToChar(behavior))
    def SetPosition(self, x, y):
        """Set the position without updating the velocity.

        Move the animation to a specified position.

        Parameters:
        x (float): The global x ENU coordinate to move to.
        y (float): The global y ENU coordinate to move to.
        """
        mavs_lib.SetMavsAnimationPosition(self.object,ctypes.c_float(x),ctypes.c_float(y))
    def SetHeading(self, heading):
        """Set the heading of the animation in radians relative to global ENU.

        East is 0, North is pi/2, West is pi, south is -pi/2 or 3pi/2.

        Parameters:
        heading (float): Desired heading.
        """
        mavs_lib.SetMavsAnimationHeading(self.object,ctypes.c_float(heading))
    def SetRotations(self, y_to_x, y_to_z):
        """Set the rotations to be applied to the mesh.

        Use these if the mesh was created in a coordinate system that doesn't match MAVS.
        For example, if the mesh was created in a "y-up" coordinate system, set y_to_z to True.

        Parameters:
        y_to_x (bool): Rotate y to x.
        y_to_z (bool): Rotate y to z.
        """
        mavs_lib.SetMavsAnimationRotations(self.object, ctypes.c_bool(y_to_x),ctypes.c_bool(y_to_z))

class MavsPlot(object):
    """Class for plotting matrices and lines using CImg through the MAVS interface.
    
    Allows for easier creation of animations / dynamically updated plots 
    that default python functions.

    Attributes:
    plot (void): Pointer to a MAVS plot.
    """
    def __init__(self):
        """Constructor for MavsPlot."""
        ## plot (void): Pointer to a MAVS plot.
        self.plot = mavs_lib.NewMavsPlotter()
    def __del__(self):
        """Destructor for MavsPlot."""
        if (self.plot):
            mavs_lib.DeleteMavsPlotter(self.plot)
    def PlotColorMatrix(self,data):
        """Plot a color matrix to the screen.

        Parameters: 
        data (list): An width x height x 3 array to plot.
        """
        width = len(data)
        if (width>0):
            height = len(data[0])
        else:
            return
        if (height>0):
            depth = len(data[0][0])
        else: 
            return
        if not (depth==3):
            return
        flattened = []
        for i in range(width):
            for j in range(height):
                for k in range(depth):
                    flattened.append(data[i][j][k])
        data_pointer = (ctypes.c_float * len(flattened))(*flattened)
        mavs_lib.PlotColorMatrix(self.plot, ctypes.c_int(width), ctypes.c_int(height), data_pointer)
    def PlotGrayMatrix(self,data):
        """ Plot a grayscale matrix

        Parameters: 
        data (list): An width x height x 3 array to plot.
        """
        width = len(data)
        if (width>0):
            height = len(data[0])
        else:
            return
        if (height<=0):
            return

        flattened = []
        for i in range(width):
            for j in range(height):
                flattened.append(data[i][j])
        data_pointer = (ctypes.c_float * len(flattened))(*flattened)
        mavs_lib.PlotGrayMatrix(self.plot, ctypes.c_int(width), ctypes.c_int(height), data_pointer)
    def PlotFlatGrayscale(self,width,height,data):
        """Plot a flattened grayscale matrix.

        Parameters: 
        width (int): width of the array
        height (int): height of the array
        data (list): a flattened width x height array to plot
        """
        data_pointer = (ctypes.c_float * len(data))(*data)
        mavs_lib.PlotGrayMatrix(self.plot, ctypes.c_int(width), ctypes.c_int(height), data_pointer)
    def PlotTrajectory(self,x,y):
        """Plot a trajectory as a sequence of x-y points.

        Parameters:
        x (float): List of x coordinates.
        y (float): List of y coordinates.
        """
        numpoints = len(x)
        x_pointer = (ctypes.c_float * len(x))(*x)
        y_pointer = (ctypes.c_float * len(y))(*y)
        mavs_lib.PlotTrajectory(self.plot, ctypes.c_int(numpoints), x_pointer, y_pointer)
    def AddToTrajectory(self,x,y):
        """Add more points to an existing trajectory plot.

        Parameters:
        x (float): List of x coordinates to add.
        y (float): List of y coordinates to add.
        """
        numpoints = len(x)
        x_pointer = (ctypes.c_float * len(x))(*x)
        y_pointer = (ctypes.c_float * len(y))(*y)
        mavs_lib.AddPlotToTrajectory(self.plot, ctypes.c_int(numpoints), x_pointer, y_pointer)

class MavsDrivingCommand(object):
    """Class the specifies the attributes of a MAVS driving command.

    Attributes:
    throttle (float): Throttle value, from 0-1
    steering (float): Steering value, [-1:1]
    braking (float): Braking value, from 0-1
    """
    def __init__(self):
        """Constructor for a MavsDrivingCommand."""
        ## throttle (float): Throttle value, from 0-1
        self.throttle = 0.0
        ## steering (float): Steering value, [-1:1]
        self.steering = 0.0
        ## braking (float): Braking value, from 0-1
        self.braking = 0.0

class MavsVehicleController(object):
    """Class for MavsVehicleController.

    The vehicle controller will automatically create driving commands
    based on the vehicles current state and a set of waypoints.

    Attributes:
    object (void): Pointer to the MAVS vehicle controller.
    steering_coeff (float): The gain in the steering PID controller.
    wheelbase (float): Wheelbase of the vehicle in meters.
    max_steering_angle (float): Max steering angle of the vehicle in radians.
    desired_speed (float): Target speed for the throttle controller in m/s.
    """
    def __init__(self):
        """Constructor for the MavsVehicleController."""
        ## object (void): Pointer to the MAVS vehicle controller.
        self.object = mavs_lib.NewVehicleController()   
        ## steering_coeff (float): The gain in the steering PID controller.
        self.steering_coeff = 0.6
        ## wheelbase (float): Wheelbase of the vehicle in meters.
        self.wheelbase = 1.25
        ## max_steering_angle (float): Max steering angle of the vehicle in radians.
        self.max_steering_angle = 0.75
        ## desired_speed (float): Target speed for the throttle controller in m/s.
        self.desired_speed = 5.0
    def __del__(self):
        """Destructor for the MavsVehiclecontroller."""
        if (self.object):
            mavs_lib.DeleteVehicleController(self.object)
    def TurnOnLooping(self):
        """Set looping to true.

        Calling this will make the vehicle loop through the waypoints indefinitely,
        automatically returning to the first waypoint when it reaches the last.
        """
        mavs_lib.SetLooping(self.object)
    def SetDesiredSpeed(self,speed):
        """Set the target speed for the vehicle in m/s.

        Parameters:
        speed (float): The desired speed in m/s.
        """
        self.desired_speed = speed
        mavs_lib.SetControllerDesiredSpeed(self.object,ctypes.c_float(speed))
    def SetWheelbase(self,wb):
        """Set the wheelbase of the vehicle in meters.

        Parameters:
        wb (float): The vehicle wheelbase in meters.
        """
        self.wheelbase = wb
        mavs_lib.SetControllerWheelbase(self.object,ctypes.c_float(wb))
    def SetMaxSteerAngle(self,max_sa):
        """Set the max steering angle for the vehicle in radians.

        Parameters:
        max_sa (float): The max steering angle in radians.
        """
        self.max_steering_angle = max_sa
        mavs_lib.SetControllerMaxSteeringAngle(self.object,ctypes.c_float(max_sa))
    def SetMinLookAhead(self,min_la):
        """Set the minimum look-ahead distance in meters.

        The look-ahead distance is how far ahead the vehicle looks to plan the path.

        Parameters:
        min_la (float): The minimum look-ahead distance in meters.
        """
        mavs_lib.SetControllerMinLookAhead(self.object,ctypes.c_float(min_la))
    def SetMaxLookAhead(self,max_la):
        """Set the maximum look-ahead distance in meters.

        The look-ahead distance is how far ahead the vehicle looks to plan the path.

        Parameters:
        max_la (float): The maximum look-ahead distance in meters.
        """
        mavs_lib.SetControllerMaxLookAhead(self.object,ctypes.c_float(max_la))
    def SetSteeringScale(self,steering_k):
        """Set steering scale value.
        
        The controller uses the pure-pursuit algorithm.
        This parameters essentially functions as the coefficient
        of the 'proportional' term in a steerig PID controller.

        Parameters:
        steering_k (float): The steering scale factor.
        """
        self.steering_coeff = steering_k
        mavs_lib.SetControllerSteeringScale(self.object,ctypes.c_float(steering_k))
    def SetSpeedControllerParams(self,kp,ki,kd):
        """Set the coefficients of the PID speed controller.

        Parameters:
        kp (float): Proportional coefficient, default is 0.3
        ki (float): Integral coefficient, default is 0.0
        kd (float): Derivative coefficient, default is 0.05
        """
        mavs_lib.SetControllerSpeedParams(self.object, ctypes.c_float(kp), ctypes.c_float(ki), ctypes.c_float(kd))
    def SetCurrentState(self,px,py,speed,heading):
        """Set the current vehicle state.

        The steering controller uses the current vehicle position, 
        orientation, and speed to update the driving commands.

        Parameters:
        px (float): The current vehicle position in the x-coordinate, global ENU.
        py (float): The current vehicle position in the y-coordinate, global ENU.
        speed (float): The current vehicle speed in m/s.
        heading (float): The current heading in radians relative to East / X.
        """
        mavs_lib.SetControllerVehicleState(self.object,ctypes.c_float(px),ctypes.c_float(py),ctypes.c_float(speed),ctypes.c_float(heading))
    def GetDrivingCommand(self, dt):
        """Get a driving command based on the current vehicle state and waypoints.

        This will calculate an updated driving command.
        You need to have set the desired path and the current state 
        for the driving command to refresh.

        Parameters:
        dt (float): The time step for the update, in seconds.
        """
        dc = MavsDrivingCommand()
        dc_p = mavs_lib.GetControllerDrivingCommand(self.object, ctypes.c_float(dt))
        dc.throttle = dc_p[0]
        dc.steering = dc_p[1]
        dc.braking = dc_p[2]
        return dc
    def SetDesiredPath(self,path):
        """Set the path for the vehicle to follow.

        The path is in global ENU coordinates.
        It should be a sequence of at least three waypoints.
        Paths work better if the waypoints are not spaced out too far.
        Optimal spacing is about 1 meter.

        Parameters:
        path (float): An nx2 list of x-y points.
        """
        x = []
        y = []
        numpoints = 0
        for p in path:
            x.append(p[0])
            y.append(p[1])
            numpoints = numpoints + 1
        x_pointer = (ctypes.c_float * len(x))(*x)
        y_pointer = (ctypes.c_float * len(y))(*y)
        mavs_lib.SetControllerDesiredPath(self.object,x_pointer,y_pointer,ctypes.c_int(numpoints))

class MavsSensor(object):
    """Base class for all types of MAVS sensors.

    Attributes:
    sensor (void): Pointer to a MAVS sensor.
    name (string): Name of the sensor.
    type (string): Type of the sensor. Can be 'lidar', 'camera', 'gps', 'compass', 'fisheye', 'radar',  or 'imu'.
    offset ([float, float, float]): x-y-z offset of the sensor from the vehicle CG.
    rel_or ([float, float, float, float]): Quaternion in q-x-y-z format specifying the relative orientation of the sensor w.r.t the vehicle.
    position ([float, float, float]): Current position of the vehicle the sensor is mounted to in global ENU.
    orientation ([float, float, float, float]): Current orientation of the vehicle the sensor is mounted to in global ENU.
    update_rate (float): The update rate of the sensor in Hz.
    elapsed_since_last (float): The amount of elapsed time since the last sensor update.
    is_active (bool): Set to True if the sensor is turned on, False if it is not.
    save_labeled (bool): Set to True if user wants to save labeled sensor data.
    save_raw (bool): Set to True if the user wants to save raw sensor data.
    display (bool): Set to True to display the output of the sensor to the screen during simulation.
    """
    def __init__(self):
        """MavsSensor constructor."""
        ## sensor (void): Pointer to a MAVS sensor.
        self.sensor = None 
        ## name (string): Name of the sensor.
        self.name = 'sensor'
        ## model (string): Name of the sensor model (ie 'HDL-64E')
        self.model = ''
        ## type (string): Type of the sensor. Can be 'lidar', 'camera', 'gps', 'compass', 'fisheye', 'radar',  or 'imu'.
        self.type = ''
        ## offset ([float, float, float]): x-y-z offset of the sensor from the vehicle CG.
        self.offset = [0.0, 0.0, 0.0]
        ## rel_or ([float, float, float, float]): Quaternion in q-x-y-z format specifying the relative orientation of the sensor w.r.t the vehicle.
        self.rel_or = [1.0, 0.0, 0.0, 0.0]
        ## position ([float, float, float]): Current position of the vehicle the sensor is mounted to in global ENU.
        self.position = [0.0, 0.0, 1.0]
        ## orientation ([float, float, float, float]): Current orientation of the vehicle the sensor is mounted to in global ENU.
        self.orientation = [1.0, 0.0, 0.0, 0.0]
        ## update_rate (float): The update rate of the sensor in Hz.
        self.update_rate = 10.0 
        ## elapsed_since_last (float): The amount of elapsed time since the last sensor update.
        self.elapsed_since_last = 0.0
        ## is_active (bool): Set to True if the sensor is turned on, False if it is not.
        self.is_active = False
        ## save_labeled (bool): Set to True if user wants to save labeled sensor data.
        self.save_labeled = False
        ## display (bool): Set to True to display the output of the sensor to the screen during simulation.
        self.display = False
        ## save_raw (bool): Set to True if the user wants to save raw sensor data.
        self.save_raw = False
    def __del__(self):
        """MavsSensor destructor."""
        if (self.sensor):
            mavs_lib.DeleteMavsSensor(self.sensor)
    def GetPose(self):
        """Get the current sensor pose.

        Return the current position and orientation.

        Returns:
        p ([float, float, float]): The current sensor position, with the offset included.
        q ([float, float, float, float): The current sensor orientation, with the offset included.
        """
        posebuff = mavs_lib.GetSensorPose(self.sensor)  
        p =[]
        q = []
        p.append(posebuff[0])
        p.append(posebuff[1])
        p.append(posebuff[2])
        q.append(posebuff[3])
        q.append(posebuff[4])
        q.append(posebuff[5])
        q.append(posebuff[6])
        return p,q
    def GetDict(self):
        """Return a dictionary with sensor properties.

        Used for writing the sensor properties to a json file.
        """
        dict = {}    
        dict['Name']=self.name
        dict['Type']=self.type
        dict['Model']=self.model
        dict['Offset']=self.offset
        dict['Orientation']=self.orientation
        dict['Repitition Rate (Hz)']=self.update_rate
        return dict
    def Update(self,env,dt):
        """Update the sensor.

        This method calls the internal update function of each sensor.
        SetPose should be called beforehand to move the sensor to the proper spot.

        Parameters:
        env (MavsEnvironment): The environment object.
        dt (float): The time step in seconds.
        """
        mavs_lib.UpdateMavsSensor(self.sensor,env.obj,ctypes.c_float(dt))
    def SaveRaw(self):
        """Save raw sensor data (point cloud, image, etc.

        Output will be given a generic file name. 
        To save output with specific file names, 
        use the save functions that are unique to each sensor.
        """
        mavs_lib.SaveMavsSensorRaw(self.sensor)
    def AnnotateFrame(self,env):
        """Calculate semantic labeled data for the sensor.

        Use MAVS automated labeling to generate labeled sensor data.

        Parameters:
        env (MavsEnvironment): The environment object.
        """
        mavs_lib.AnnotateMavsSensorFrame(self.sensor,env.obj)
    def SetOffset(self,pos,quat):
        """Set the offset of the sensor relative to the vehicle CG.

        Parameters:
        pos ([float,float,float]): x-y-z offset relative to the vehicle CG.
        quat ([float,float,float,float]): w-x-y-z quaternion orientation relative to the vehicle.
        """
        offset = pos
        rel_or = quat
        position = ctypes.c_float*3;
        ori = ctypes.c_float*4;
        p = position(pos[0],pos[1],pos[2])
        q = ori(quat[0],quat[1],quat[2],quat[3])
        mavs_lib.SetMavsSensorRelativePose(self.sensor,p,q)
    def SetVelocity(self, vx, vy, vz):
        """Set the linear velocity of the sensor.

        When this is set, the motion of the sensor will be calculated,
        for point cloud / image blurring

        Parameters:
        vx (float) The velocity in the global x direction.
        vy (float) The velocity in the global y direction.
        vz (float) The velocity in the global z direction.
        """
        mavs_lib.SetMavsSensorVelocity(self.sensor,ctypes.c_float(vx),ctypes.c_float(vy), ctypes.c_float(vz))
    def SetPose(self,pos,quat):
        """Set the position of the vehicle carrying the sensor.

        This does not include the offsets, 
        it is the position of the vehicle that the sensor is attached to.
        Offsets will be included automatically if they have been set using the
        'SetOffset' method.

        Parameters:
        pos ([float,float,float]): x-y-z position of the sensor.
        quat ([float,float,float,float]): w-x-y-z quaternion orientation of the sensor.
        """
        self.position = pos
        self.orientation = quat
        position = ctypes.c_float*3;
        ori = ctypes.c_float*4;
        p = position(pos[0],pos[1],pos[2])
        q = ori(quat[0],quat[1],quat[2],quat[3])
        mavs_lib.SetMavsSensorPose(self.sensor,p,q)
    def Display(self):
        """Display the output of the sensor in an X-window."""
        mavs_lib.DisplayMavsSensor(self.sensor)
    def SaveAnnotation(self,env,fname):
        """Save an annotated frame.

        The 'AnnotateFrame' method should be called first.

        Parameters:
        env (MavsEnvironment): MavsEnvironment object.
        fname (string): The file name of the annotation output.
        """
        mavs_lib.SaveMavsSensorAnnotation(self.sensor,env.obj,PyStringToChar(fname))
    def load_block(self,data):
        """Load a sensor block from a json file.

        Parameters:
        data (json): A json dictionary block.
        """
        self.name = data["Name"]
        self.offset = data["Offset"]
        self.orientation = data["Orientation"]
        self.SetOffset(self.offset,self.orientation)
        self.update_rate = data["Repitition Rate (Hz)"]
        self.type = data["Type"]
        #if 'Draw Annotations' in data:
        #    self.annotate = data["Draw Annotations"]
        if 'Model' in data:
            self.model = data["Model"]

class MavsMems:
    """Mems sensor class.

    MEMS sensors include accelerometers, gyroscopes, and magnetometers.

    See mavs_imu.pdf for additional documentation of parameters.

    Attributes:
    type (string): Can be 'gyro', 'accelerometer', or 'magnetometer'.
    sensor (void): Pointer to MAVS mems sensor.
    """
    def __init__(self,type):
        """Constructor for a MavsMems sensor."""
        ## type (string): Can be 'gyro', 'accelerometer', or 'magnetometer'.
        self.type = type
        ## sensor (void): Pointer to MAVS mems sensor.
        self.sensor = mavs_lib.NewMavsMems(PyStringToChar(type))
    def SetMeasurementRange(self,range):
        """Set the measurment range of the sensor.

        Parameters:
        range (float): The maximum measurement value.
        """
        mavs_lib.SetMemsMeasurmentRange(self.sensor,ctypes.c_float(range))
    def SetMeasurementResolution(self,resolution):
        """Set the measurement range for the sensor.

        Parameters:
        resolution (float): The measurement resolution.
        """
        mavs_lib.SetMemsResolution(self.sensor,ctypes.c_float(resolution))
    def SetConstantBias(self,bias):
        """Set the constant bias error for the sensor.

        Parameters:
        bias (float): The constant bias value.
        """
        if isinstance(bias,list):
            mavs_lib.SetMemsConstantBias(self.sensor, ctypes.c_float(bias[0]), ctypes.c_float(bias[1]), ctypes.c_float(bias[2]))
        else:
            mavs_lib.SetMemsConstantBias(self.sensor, ctypes.c_float(bias), ctypes.c_float(bias), ctypes.c_float(bias))
    def SetNoiseDensity(self,nd):
        """Set the noise density error for the sensor.

        Parameters:
        nd (float): The noise density value.
        """
        if isinstance(nd,list):
            mavs_lib.SetMemsNoiseDensity(self.sensor, ctypes.c_float(nd[0]), ctypes.c_float(nd[1]), ctypes.c_float(nd[2]))
        else:
            mavs_lib.SetMemsNoiseDensity(self.sensor, ctypes.c_float(nd), ctypes.c_float(nd), ctypes.c_float(nd))
    def SetBiasInstability(self,bi):
        """Set the bias instability error for the sensor.

        Parameters:
        bi (float): The bias instability value.
        """
        if isinstance(bi,list):
            mavs_lib.SetMemsConstantBias(self.sensor, ctypes.c_float(bi[0]), ctypes.c_float(bi[1]), ctypes.c_float(bi[2]))
        else:
            mavs_lib.SetMemsConstantBias(self.sensor, ctypes.c_float(bi), ctypes.c_float(bi), ctypes.c_float(bi))
    def SetAxisMisalignment(self,ma):
        """Set the misalignment error for the sensor.

        Parameters:
        ma (float): The misalignment error value.
        """
        mavs_lib.SetMemsAxisMisalignment(self.sensor, ctypes.c_float(ma[0]), ctypes.c_float(ma[1]), ctypes.c_float(ma[2]))
    def SetRandomWalk(self,rw):
        """Set the random walk error for the sensor.

        Parameters:
        rw (float): The random walk error value.
        """
        if isinstance(rw,list):
            mavs_lib.SetMemsRandomWalk(self.sensor, ctypes.c_float(rw[0]), ctypes.c_float(rw[1]), ctypes.c_float(rw[2]))
        else:
            mavs_lib.SetMemsRandomWalk(self.sensor, ctypes.c_float(rw), ctypes.c_float(rw), ctypes.c_float(rw))
    def SetTemperatureBias(self,tb):
        """Set the temperature bias error for the sensor.

        Parameters:
        tb (float): The temperature bias value.
        """
        if isinstance(tb,list):
            mavs_lib.SetMemsTemperatureBias(self.sensor, ctypes.c_float(tb[0]), ctypes.c_float(tb[1]), ctypes.c_float(tb[2]))
        else:
            mavs_lib.SetMemsTemperatureBias(self.sensor, ctypes.c_float(tb), ctypes.c_float(tb), ctypes.c_float(tb))
    def SetTemperatureScaleFactor(self,tsf):
        """Set the temperature scale error for the sensor.

        Parameters:
        tsf (float): The temperature scale error value.
        """
        if isinstance(tsf,list):
            mavs_lib.SetTemperatureScaleFactor(self.sensor, ctypes.c_float(tsf[0]), ctypes.c_float(tsf[1]), ctypes.c_float(tsf[2]))
        else:
            mavs_lib.SetTemperatureScaleFactor(self.sensor, ctypes.c_float(tsf), ctypes.c_float(tsf), ctypes.c_float(tsf))
    def SetAccelerationBias(self,ab):
        """Set the acceleration bias error for the sensor.

        Parameters:
        ab (float): The acceleration bias value.
        """
        if isinstance(ab,list):
            mavs_lib.SetMemsAccelerationBias(self.sensor, ctypes.c_float(ab[0]), ctypes.c_float(ab[1]), ctypes.c_float(ab[2]))
        else:
            mavs_lib.SetMemsAccelerationBias(self.sensor, ctypes.c_float(ab), ctypes.c_float(ab), ctypes.c_float(ab))
    def Update(self,accel_in,temperature, sample_rate):
        """Update the sensor and return the 3-axis measurement.

        Parameters:
        accel_in ([float, float, float]): True acceleration / signal for the sensor.
        temperature (float): Ambient temperature in degrees Celsius.
        sample_rate (float): Sample rate in Hz.
        """
        accel_out = mavs_lib.MemsUpdate(self.sensor, ctypes.c_float(accel_in[0]), ctypes.c_float(accel_in[1]), ctypes.c_float(accel_in[2]), 
                                        ctypes.c_float(temperature), ctypes.c_float(sample_rate))
        accel = []
        for i in range(3):
            accel.append(accel_out[i])
        return accel

class MavsCamera(MavsSensor):
    """MavsCamera class.

    Base class for several types of cameras.

    Attributes:
    type (string): Must be set to 'camera'.
    sensor (void): Pointer to a MAVS sensor.
    gamma (float): Camera compression factor.
    gain (float): Camera amplification factor.
    aa_fac (int): Anti-aliasing factor - pixel oversampling rate. Default is 1.
    render_shadows (bool): Set to True to render shadows.
    raindrop_lens (bool): If raining, set to true to add raindrops to the camera lens.
    """
    def __init__(self):
        """Constructor for a MavsCamera."""
        MavsSensor.__init__(self)
        ## type (string): Must be set to 'camera'.
        self.type = 'camera'
        ## sensor (void): Pointer to a MAVS sensor.
        self.sensor = None 
        ## gamma (float): Camera compression factor.
        self.gamma = 1.0
        ## gain (float): Camera amplification factor.
        self.gain = 1.0
        ## aa_fac (int): Anti-aliasing factor - pixel oversampling rate. Default is 1.
        self.aa_fac = 1
        ## render_shadows (bool): Set to True to render shadows.
        self.render_shadows = False
        ## raindrop_lens (bool): If raining, set to true to add raindrops to the camera lens.
        self.raindrop_lens = False
        self.use_blur = False
    def Model(self,model):
        """ Initialize a camera to a certain model.

        Available models are :
        'XCD-V60', 'Flea', 'HD1080', 'MachineVision', 
        'MachineVisionPathTraced', 'HDPathTraced',
        'HalfHDPathTraced', 'Sf3325', 'Sf3325PathTraced',
        'UavCamera', 'UavCameraPathTraced', 'UavCameraPathTracedLow',
        'Phantom4Camera', 'Phantom4CameraPathTraced', and 'Phantom4CameraPathTracedLow'

        Parameters:
        model (string): The camera model string.
        """
        self.__init__()
        self.sensor = mavs_lib.NewMavsCameraModel(PyStringToChar(model))
        self.gamma = mavs_lib.GetCameraGamma(self.sensor)
        self.gain = mavs_lib.GetCameraGain(self.sensor)
    def DisplayOpen(self):
        """Returns true if the display is open"""
        return mavs_lib.CameraDisplayOpen(self.sensor)
    def AddLidarPointsToImage(self, mavs_lidar_sensor):
        """Project the current lidar point cloud to the camera image"""
        mavs_lib.AddPointsToImage(mavs_lidar_sensor.sensor, self.sensor)
    def SaveCameraImage(self,fname):
        """Save the current camera frame to a file.

        Must specify the extension. Currently .bmp and .png are supported.

        Parameters:
        fname (string): The file save name, including path and extension
        """
        mavs_lib.SaveMavsCameraImage(self.sensor,PyStringToChar(fname))
    def SaveBoxAnnotation(self,env, fname):
        """Save the bounding box labels for the current frame

        Parameters:
        env (MavsEnvironment): The mavs environment class
        fname (string): The file save name, including path and extension
        """
        mavs_lib.SaveMavsCameraAnnotationFull(self.sensor, env.obj, PyStringToChar(fname))
    def Initialize(self,nx,ny,faw,fah,fl):
        """Initialize the camera system to a particular geometry.

        Parameters:
        nx (int): The number of pixels in the horizontal direction.
        ny (int): The number of pixels in the vertical direction.
        faw (float): The width of the focal plane in meters.
        fah (float): The height of the focal plane in meters.
        fl (float): The focal length in meters.
        """
        self.sensor = mavs_lib.NewMavsRgbCameraDimensions(ctypes.c_int(nx),ctypes.c_int(ny),
                                                          ctypes.c_float(faw),ctypes.c_float(fah),ctypes.c_float(fl))
    def FreePose(self):
        """Make the camera moveable through the display window.

        When this is called, the camera position and orientation can
        be moved with the W-A-S-D keys, arrow keys, and Page Up / Page Down keys.
        """
        mavs_lib.FreeCamera(self.sensor)
    def GetBuffer(self):
        """Return a python list with the rgb values of the camera frame.

        The buffer will read out rows in the sequentially, listing each pixel's r-g-b
        as three floats.

        Returns:
        buffer (list of floats): The camera buffer.
        """
        pointbuff = mavs_lib.GetCameraBuffer(self.sensor)
        buffsize = mavs_lib.GetCameraBufferSize(self.sensor)
        buffer = pointbuff[:buffsize]
        return buffer
    def GetNumpyArray(self):
        """Return a numpy array that can be converted directly to an image.
        
        Faster version recommended by Kasi Viswanth.

        Example usage: 
        from PIL import Image
        img_data = cam.GetNumpyArray()
        img = Image.fromarray(img_data, 'RGB')
        img.show()

        Returns:
        img_data (numpy array of floats): The camera buffer.
        """
        try:
            import numpy as np
        except ImportError:
            print("WARNING, TRIED TO GET MAVS IMAGE AS NUMPY ARRAY, BUT NUMPY IS NOT INSTALLED \n")
            return None
        pointbuff = mavs_lib.GetCameraBuffer(self.sensor)
        buffsize = mavs_lib.GetCameraBufferSize(self.sensor)
        buffer = pointbuff[:buffsize]
        imagedim = self.GetDimensions()
        buffer = np.asarray(buffer,dtype = 'float32')
        shape = buffer.shape
        buffer = buffer.ravel()
        buffer[np.isnan(buffer)] = 0
        buffer = buffer.reshape(shape)
        img_data = np.zeros((imagedim[1], imagedim[0], imagedim[2]), dtype=np.uint8)
        raw = np.reshape(buffer.astype('uint8'),(imagedim[2],imagedim[1], imagedim[0]))
        img_data[:,:,0] = raw[0]
        img_data[:,:,1] = raw[1]
        img_data[:,:,2] = raw[2]
        return img_data 
    #def GetNumpyArray(self):
    #    """Return a numpy array that can be converted directly to an image.
    #    
    #    Example usage: 
    #    from PIL import Image
    #    img_data = cam.GetNumpyArray()
    #    img = Image.fromarray(img_data, 'RGB')
    #    img.show()
    #    Returns:
    #    img_data (numpy array of floats): The camera buffer.
    #    """
    #    pointbuff = mavs_lib.GetCameraBuffer(self.sensor)
    #    buffsize = mavs_lib.GetCameraBufferSize(self.sensor)
    #    buffer = pointbuff[:buffsize]
    #    imagedim = self.GetDimensions() 
    #    img_data = np.zeros((imagedim[1], imagedim[0], imagedim[2]), dtype=np.uint8)
    #    n = 0
    #    for k in range(imagedim[2]):
    #        for j in range(imagedim[1]):
    #            for i in range(imagedim[0]):
    #                img_data[j][i][k] = np.uint8(buffer[n])
    #                n = n+1
    #    return img_data

    def RenderShadows(self,shadows):
        """Turn shadow rendering on/off.

        Rendering is faster with shadows turned off.

        Parameters:
        shadows (bool): Turn shadows on (True) or off (False).
        """
        self.render_shadows = shadows
        mavs_lib.SetMavsCameraShadows(self.sensor,ctypes.c_bool(shadows))
    def UseBlur(self,blur):
        """Turn camera blur on/off.

        Rendering is faster with blur turned off.

        Parameters:
        blur (bool): Turn blur on (True) or off (False).
        """
        self.use_blur = blur
        mavs_lib.SetMavsCameraBlur(self.sensor,ctypes.c_bool(blur))
    def SetAntiAliasingFactor(self,numsamples):
        """Set the camera anti-aliasing factor.
        
        Each pixel will be oversampled by a factor of numsamples
        Default is 1, increasing numsamples also increases the rendering time

        Parameters:
        numsamples (int): The number of samples at each pixel
        """
        self.aa_fac = numsamples
        mavs_lib.SetMavsCameraAntiAliasingFactor(self.sensor,ctypes.c_int(numsamples))
    def SetEnvironmentProperties(self,env):
        """Set the environmental properties for the camera.
        
        Give the camera a pointer to the current environment to set
        properties like the sun color and position.

        Parameters:
        env (void): Pointer to a MavsEnvironment object.
        """
        mavs_lib.SetMavsCameraEnvironmentProperties(self.sensor,env)
    def SetDropsOnLens(self,onlens):
        """If raining, turn raindrops on the camera lens on/off.

        Parameters:
        onlens (bool): Set to true for raindrops on lens, false for none.
        """
        self.raindrop_lens = onlens
        mavs_lib.SetMavsCameraLensDrops(self.sensor,ctypes.c_bool(onlens))
    def SetGammaAndGain(self,gamma,gain):
        """Set the camera compression and gain.

        Pixels are modified by is given by I = gain*I_0^gamma.

        Parameters:
        gamma (float): Compression value.
        gain (float): Gain value.
        """
        self.gamma = gamma
        self.gain = gain
        mavs_lib.SetMavsCameraElectronics(self.sensor,ctypes.c_float(gamma),ctypes.c_float(gain))

    def SetSaturationAndTemp(self,saturation,temp):
        """Set the image temperature and saturation

        Parameters:
        temp (float): Color temperature
        saturation (float): Color saturation
        """
        mavs_lib.SetMavsCameraTempAndSaturation(self.sensor,ctypes.c_float(temp),ctypes.c_float(saturation))
    def GetDrivingCommand(self):
        """Get driving command as user input through the camera display window.

        When the camera window is highlighted, user can issue driving commands
        with the W-A-S-D keys.

        Returns:
        dc (MavsDrivingCommand): Returns a MavsDrivingCommand.
        """
        dc = MavsDrivingCommand()
        dc_p = mavs_lib.GetDrivingCommandFromCamera(self.sensor)
        dc.throttle = dc_p[0]
        dc.steering = dc_p[1]
        dc.braking = dc_p[2]
        return dc
    def GetDimensions(self):
        """Get the dimensions of the current camera frame.

        Returns:
        width (int): Width of the image in pixels.
        height (int): Height of the image in pixels.
        depth (int): Depth of the image, usually 3.
        """
        width = mavs_lib.GetCameraBufferWidth(self.sensor)
        height = mavs_lib.GetCameraBufferHeight(self.sensor)
        depth = mavs_lib.GetCameraBufferDepth(self.sensor)
        return [width,height,depth]
    def ConvertToRccb(self):
        """Convert an image generated with an RGB colormask to RCCB.

        The green channel is modified by the equation.
        green = 0.3*red + 0.59*green + 0.11*blue
        """
        mavs_lib.ConvertToRccb(self.sensor)
        

class MavsOakDCamera(MavsSensor):
    def __init__(self):
        self.sensor = mavs_lib.NewMavsOakDCamera()
        self.range_data = None
        self.max_range_m = 12.0
        self.width = 0
        self.height = 0
        self.depth = 0
    def SetDisplayType(self, display_type):
        """Set the camera display type

        Parameters:
        display_type (string): Can be "rgb", "range", or "both"
        """
        mavs_lib.SetOakDCameraDisplayType(self.sensor, PyStringToChar(display_type))
    def GetDimensions(self):
        """Get the dimensions of the current camera frame.

        Returns:
        width (int): Width of the image in pixels.
        height (int): Height of the image in pixels.
        depth (int): Depth of the image, usually 3.
        """
        sens = mavs_lib.GetOakDCamera(self.sensor)
        self.width = mavs_lib.GetCameraBufferWidth(sens)
        self.height = mavs_lib.GetCameraBufferHeight(sens)
        self.depth = mavs_lib.GetCameraBufferDepth(sens)
        return [self.width,self.height,self.depth]
    def DisplayOpen(self):
        sens = mavs_lib.GetOakDCamera(self.sensor)
        is_open = mavs_lib.CameraDisplayOpen(sens)
        return is_open
    def GetImage(self):
        """Return a numpy array that can be converted directly to an image.
        
        Faster version recommended by Kasi Viswanth.

        Example usage: 
        from PIL import Image
        img_data = cam.GetNumpyArray()
        img = Image.fromarray(img_data, 'RGB')
        img.show()

        Returns:
        img_data (numpy array of floats): The camera buffer.
        """
        try:
            import numpy as np
        except ImportError:
            print("WARNING, TRIED TO GET MAVS IMAGE AS NUMPY ARRAY, BUT NUMPY IS NOT INSTALLED \n")
            return None
        pointbuff = mavs_lib.GetOakDImageBuffer(self.sensor)
        buffsize = mavs_lib.GetOakDImageBufferSize(self.sensor)
        buffer = pointbuff[:buffsize]
        imagedim = self.GetDimensions()
        buffer = np.asarray(buffer,dtype = 'float32')
        shape = buffer.shape
        buffer = buffer.ravel()
        buffer[np.isnan(buffer)] = 0
        buffer = buffer.reshape(shape)
        img_data = np.zeros((imagedim[1], imagedim[0], imagedim[2]), dtype=np.uint8)
        raw = np.reshape(buffer.astype('uint8'),(imagedim[2],imagedim[1], imagedim[0]))
        img_data[:,:,0] = raw[0]
        img_data[:,:,1] = raw[1]
        img_data[:,:,2] = raw[2]
        return img_data 
    def GetDepthImage(self):
        """Return a numpy array that can be converted directly to an image.
        
        Faster version recommended by Kasi Viswanth.

        Example usage: 
        from PIL import Image
        img_data = cam.GetNumpyArray()
        img = Image.fromarray(img_data, 'RGB')
        img.show()

        Returns:
        img_data (numpy array of floats): The camera buffer.
        """
        try:
            import numpy as np
        except ImportError:
            print("WARNING, TRIED TO GET MAVS IMAGE AS NUMPY ARRAY, BUT NUMPY IS NOT INSTALLED \n")
            return None
        pointbuff = mavs_lib.GetOakDDepthBuffer(self.sensor)
        buffsize = mavs_lib.GetOakDDepthBufferSize(self.sensor)
        buffer = pointbuff[:buffsize]
        imagedim = self.GetDimensions()
        buffer = np.asarray(buffer,dtype = 'float32')
        shape = buffer.shape
        buffer = buffer.ravel()
        buffer[np.isnan(buffer)] = 0
        buffer = buffer.reshape(shape)
        img_data = np.zeros((imagedim[1], imagedim[0], imagedim[2]), dtype=np.uint8)
        raw = np.reshape(buffer.astype('uint8'),(imagedim[2],imagedim[1], imagedim[0]))
        img_data[:,:,0] = raw[0]
        img_data[:,:,1] = raw[1]
        img_data[:,:,2] = raw[2]
        self.range_data = img_data
        return self.range_data
    def GetMaxRangeCm(self):
        return mavs_lib.GetOakDMaxRangeCm(self.sensor)
    def SetMaxRangeCm(self, max_range_cm):
        mavs_lib.SetOakDMaxRangeCm(self.sensor, ctypes.c_float(max_range_cm))
    def GetRangeAtPixelMeters(self, u, v):
        u = int(u)
        v = int(v)
        if (u>=self.width or u<0 or v>=self.height or v<0):
            return 0.0
        if self.width<=0:
            return 0.0
        mr_cm = self.GetMaxRangeCm() # max_range in meters
        range_cm = (mr_cm / 255.0) * self.range_data[v][u][0]
        return range_cm/100.0
        

class MavsLwirCamera(MavsCamera):
    def __init__(self, nx, ny, dx, dy, flen):
        """Constructor for a LWIR camera

        """
        MavsCamera.__init__(self)
        self.sensor = mavs_lib.NewMavsLwirCamera(ctypes.c_int(nx), ctypes.c_int(ny), ctypes.c_float(dx), ctypes.c_float(dy), ctypes.c_float(flen))
    def LoadThermalData(self, fname):
        mavs_lib.LoadLwirThermalData(self.sensor, PyStringToChar(fname))

class MavsRedEdge(MavsCamera):
    def __init__(self):
        """Constructor for a MavsRedEdge.
        
        The red-edge is a 5-band multispectral Camera
        """
        MavsCamera.__init__(self)
        self.sensor = mavs_lib.NewMavsRedEdge()
    def Display(self):
        """Display the output of the sensor in an X-window.
        
        Displays only the RGB bands.
        Overrides the MavsSensor base class Display method
        """
        mavs_lib.DisplayRedEdge(self.sensor)
    def SaveCameraImage(self,fname):
        """Save the current camera frame to a file.

        Must specify the extension. Currently .bmp and .png are supported.

        Overrides the MavsCamera base class SaveCameraImage method.

        Parameters:
        fname (string): The file save name, including path and extension
        """
        mavs_lib.SaveRedEdge(self.sensor,PyStringToChar(fname))
    def SaveFalseColor(self,band1, band2, band3, fname):
        """Save the current red edge to a false color file

        Must specify the extension. Currently .bmp and .png are supported.

        Specify the bands to go in the R-G-B channels

        Band numbers must be 1-5

        Parameters:
        band1 (int): Band to occupy the R channel.
        band2 (int): Band to occupy the G channel.
        band3 (int): Band to occupy the B channel.
        fname (string): The file save name, including path and extension.
        """
        mavs_lib.SaveRedEdgeFalseColor(self.sensor, ctypes.c_int(band1), ctypes.c_int(band2), ctypes.c_int(band3), PyStringToChar(fname))
    def SaveBands(self,fname):
        """Save the current frame to individual grayscale bands

        Must specify the extension. Currently .bmp and .png are supported.

        Parameters:
        fname (string): The file save name, including path and extension
        """
        mavs_lib.SaveRedEdgeBands(self.sensor,PyStringToChar(fname))

class MavsPathTraceCamera(MavsCamera):
    """Path tracer camera that inherits from the MavsCamera class.

    The path tracer camera uses physics-based path tracing to render 
    an image. It is much slower than the default camera but makes 
    nicer images.

    Attributes:
    sensor (void): Pointer to a MAVS sensor.
    """
    def __init__(self,type,numrays,raydepth,rr_cutoff,nx=256,ny=256,h_s=0.0035,v_s=0.0035,flen=0.0035,gamma=0.75):
        """Constructor for a MavsPathTracerCamera.

        Available types are 'high', 'half', 'uav', 'uavlow',
        'phantom4', 'phantom4low', 'sf3325', 'sf3325low', or 'custom'.

        If type is set to 'custom', the camera will be initalized using 
        the camera parameters supplied by the user

        Parameters:
        numrays (int): The number of rays per pixel.
        raydepth (int): The maximum number of reflections for a ray.
        rr_cutoff (float): The intensity cutoff factor for a ray [0:1]
        nx (int): Number of horizontal pixels
        ny (int): Number of vertical pixels
        h_s (float): Horizontal dimension of image plane (meters)
        v_s (float): Vertical dimension of image plane (meters)
        flen (float): Focal length of camera
        gamma (float): Compression factor of camera
        """
        if type=='custom':
            self.sensor = mavs_lib.NewMavsPathTraceCameraExplicit(ctypes.c_int(nx), ctypes.c_int(ny), 
                                                                  ctypes.c_float(h_s), ctypes.c_float(v_s),
                                                                  ctypes.c_float(flen), ctypes.c_float(gamma),
                                                                  ctypes.c_int(numrays), ctypes.c_int(raydepth),
                                                                  ctypes.c_float(rr_cutoff))
                                                                  
        else:
            self.sensor = mavs_lib.NewMavsPathTraceCamera(PyStringToChar(type), ctypes.c_int(numrays), ctypes.c_int(raydepth),ctypes.c_float(rr_cutoff))
    def SetNormalizationType(self,type):
        """Set the type of image normalization.

        Options are 'max' or 'average'. 
        If 'max' is set, the image will be normalized so that the 
        maximum pixel intensity = 255. 
        if 'average' is selected, the image will be normalized so that
        the average pixel intensity is 128

        Default is 'average'.

        Parameters:
        type (string): The normalization type.

        """
        mavs_lib.SetMavsPathTracerCameraNormalization(self.sensor,PyStringToChar(type))
    def SetFixPixels(self,fix):
        """Fix bad pixels in the path traced image.

        Because path-tracing involves random sampling, 
        there are occasionally outlier pixels if the sampling factor is too low.
        Calling this will instruct the ray-tracer to attempt to identify these
        outlier pixels afer the rendering pass and set them to match the local average.

        Note that for very low sampling rates where the image is very grainy,
        this process may produce undesirable results.

        Parameters:
        fix (bool): Set to True to fix bad pixels, False to ignore.
        """
        mavs_lib.SetMavsPathTracerFixPixels(self.sensor,ctypes.c_bool(fix))

def ChamferDistance(pc1, pc2):
    """Get the chamfer distance between two point clouds
        
    Parameters:
    pc1 (float): N X 3 list of points
    pc2 (float): M X 3 list of points
    """
    npc1 = len(pc1)
    pc1x = []
    pc1y = []
    pc1z = []
    for i in range(npc1):
        pc1x.append(pc1[i][0])
        pc1y.append(pc1[i][1])
        pc1z.append(pc1[i][2])
    pc1x_pointer = (ctypes.c_float * len(pc1x))(*pc1x)
    pc1y_pointer = (ctypes.c_float * len(pc1y))(*pc1y)
    pc1z_pointer = (ctypes.c_float * len(pc1z))(*pc1z)
    npc2 = len(pc2)
    pc2x = []
    pc2y = []
    pc2z = []
    for i in range(npc2):
        pc2x.append(pc2[i][0])
        pc2y.append(pc2[i][1])
        pc2z.append(pc2[i][2])
    pc2x_pointer = (ctypes.c_float * len(pc2x))(*pc2x)
    pc2y_pointer = (ctypes.c_float * len(pc2y))(*pc2y)
    pc2z_pointer = (ctypes.c_float * len(pc2z))(*pc2z)
    chamfer_distance = mavs_lib.GetChamferDistance(ctypes.c_int(npc1), pc1x_pointer, pc1y_pointer, pc1z_pointer, ctypes.c_int(npc2), pc2x_pointer, pc2y_pointer, pc2z_pointer) 
    return chamfer_distance
        
class MavsLidar(MavsSensor):
    """MavsLidar class.

    The API allows you to select from a variety of lidar models.

    Attributes:
    type (string): Must be 'lidar'.
    sensor (void): Pointer to a MAVS sensor.
    """
    def __init__(self,type):
        """Construct a MavsLidar.
        
        Available models are 'HDL-32E', 'HDL-64E', 'M8','OS1', 'OS1-16', 'OS2',
        'LMS-291', 'VLP-16', 'RS32', 'AnvelApiLidar', 'OS0', 'BPearl', and 'FourPi'

        Parameters:
        type (string): The model of the lidar.
        """
        MavsSensor.__init__(self)
        ## type (string): Must be 'lidar'.
        self.type = 'lidar'
        ## sensor (void): Pointer to a MAVS sensor.
        self.sensor = mavs_lib.NewMavsLidar(PyStringToChar(type))
    def __del__(self):
        """Destructor for MavsLidar."""
        if (self.sensor):
            mavs_lib.DeleteMavsSensor(self.sensor)
    def SetScanPattern(self,
                       horiz_fov_low, horiz_fov_high, horiz_res,
                       vert_fov_low, vert_fov_high,vert_res):
        """Set the scan patter of the lidar.

        This will override the default for the model when the lidar was created.

        The angular inputs are in degrees, not radians.

        Parameters:
        horiz_fov_low (float): The low value (degrees) of the horizontal field-of-view.
        horiz_fov_high (float): The high value (degrees) of the horizontal field-of-view.
        horiz_res (float): The resolution (degrees) of the horizontal field-of-view.
        vert_fov_low (float): The low value (degrees) of the vertical field-of-view.
        vert_fov_high (float): The high value (degrees) of the vertical field-of-view.
        vert_res (float): The resolution (degrees) of the vertical field-of-view.
        """
        if (self.sensor):
            mavs_lib.DeleteMavsSensor(self.sensor)
        self.sensor = mavs_lib.MavsLidarSetScanPattern(ctypes.c_float(horiz_fov_low), ctypes.c_float(horiz_fov_high),
                                         ctypes.c_float(horiz_res),ctypes.c_float(vert_fov_low),
                                         ctypes.c_float(vert_fov_high),ctypes.c_float(vert_res))
    def SaveLidarImage(self,fname):
        """Save the current lidar point cloud to a top-down image.

        Parameters:
        fname (string): The output file name, including path and extension.
        """
        mavs_lib.SaveMavsLidarImage(self.sensor,PyStringToChar(fname))
    def SaveProjectedLidarImage(self,fname):
        """Save the current lidar point cloud to a projected image.

        Saves a "first-person" view of the lidar point cloud.

        Parameters:
        fname (string): The output file name, including path and extension
        """
        mavs_lib.SaveProjectedMavsLidarImage(self.sensor,PyStringToChar(fname))
    def DisplayPerspective(self,width=768,height=256):
        """Display a perspective view of the point cloud to the screen.

        Parameters:
        width (int): The width of the display.
        height (int): The height of the display.
        """
        mavs_lib.DisplayMavsLidarPerspective(self.sensor,ctypes.c_int(width),ctypes.c_int(height))
    def SaveColorizedPointCloud(self,fname):
        """Save the current lidar point cloud to a column file.

        Saves x,y,z,intensity,r,g,b to a space delimited text file.

        Parameters:
        fname (string): The output file name, including path and extension.
        """
        mavs_lib.WriteMavsLidarToColorizedCloud(self.sensor,PyStringToChar(fname))
    def SavePcd(self,fname):
        """Save the current lidar point cloud to a Point Cloud Library pcd file.

        Saves a column file with x,y,z,intensity

        Parameters:
        fname (string): The output file name, including path and extension.
        """
        mavs_lib.WriteMavsLidarToPcd(self.sensor,PyStringToChar(fname))
    def SaveLabeledPcd(self,fname):
        """Save the current lidar point cloud to a Point Cloud Library pcd file.

        Saves a column file with x,y,z,intensity,label
        where labelnum is an int defined in the labels.json file.

        Parameters:
        fname (string): The output file name, including path and extension.
        """
        mavs_lib.WriteMavsLidarToLabeledPcd(self.sensor,PyStringToChar(fname))
    def SaveLabeledPcdWithNormals(self,fname):
        """Save the current lidar point cloud to a Point Cloud Library pcd file.

        Saves a column file with x,y,z,intensity,normal_x, normal_y, normal_z,label
        where labelnum is an int defined in the labels.json file.

        Parameters:
        fname (string): The output file name, including path and extension.
        """
        mavs_lib.WriteMavsLidarToLabeledPcdWithNormals(self.sensor,PyStringToChar(fname))

    def SaveLabeledPointCloud(self,fname):
        """Save the current lidar point cloud to a column text file.

        Saves a column file with x,y,z,intensity,labelnum
        where labelnum is an int defined in the labels.json file.

        Parameters:
        fname (string): The output file name, including path and extension.
        """
        mavs_lib.WriteMavsLidarToLabeledCloud(self.sensor,PyStringToChar(fname))
    def GetPoints(self):
        """ Get a list of the x-y-z points in the point cloud.

        Returns a Nx3 list of points where N is the number of returns.
        Points are 'registered' to world coordinates.

        Returns:
        points (list of floats): The x-y-z point cloud.
        """
        numpoints = mavs_lib.GetMavsLidarNumberPoints(self.sensor)
        tp = 3*numpoints
        mavs_lib.GetMavsLidarRegisteredPoints.restype = ctypes.POINTER(ctypes.c_float*tp)
        inter_flat_points = mavs_lib.GetMavsLidarRegisteredPoints(self.sensor)
        flat_points = inter_flat_points.contents
        points = [] 
        n = 0
        for p in range(numpoints):
            point = [flat_points[n],flat_points[n+1],flat_points[n+2]]
            if (not (point[0]==0.0 and point[1]==0.0 and point[2]==0.0)):
                points.append(point)
            n = n + 3
        return points

    def GetUnRegisteredPointsXYZIL(self):
        """ Get a list of the x-y-z-intensity-label points in the point cloud.

        Returns a Nx5 list of points where N is the number of returns.
        Points are 'unregistered', or in the sensor frame.

        Returns:
        points (list of floats): The x-y-z-intensity-label point cloud.
        """
        flat_points = mavs_lib.GetMavsLidarUnRegisteredPointsXYZIL(self.sensor)
        numpoints = mavs_lib.GetMavsLidarNumberPoints(self.sensor)
        points = [] 
        n = 0
        for p in range(numpoints):
            point = [flat_points[n],flat_points[n+1],flat_points[n+2],flat_points[n+3],flat_points[n+4]]
            if (not (point[0]==0.0 and point[1]==0.0 and point[2]==0.0)):
                points.append(point)
            n = n + 5
        return points
    def SetDisplayColorType(self,type):
        """ Set the colorization of the lidar display.
        
        Options are 'height', 'color', 'range', 'intensity', 'label', or 'white'.

        Parameters:
        type (string): The display type.
        """
        mavs_lib.SetPointCloudColorType(self.sensor,PyStringToChar(type))
    def AnalyzeCloud(self,fname,frame_num,display):
        """ Automatically annotates the point cloud and saves it to a file.

        Parameters:
        fname (string): The output file name, without extension.
        frame_num (int): The number of the current frame.
        display (bool): True to display the result to the screen, False otherwise.
        """
        mavs_lib.AnalyzeCloud(self.sensor,PyStringToChar(fname),ctypes.c_int(frame_num),ctypes.c_bool(display))

class MavsRtk(MavsSensor):
    """MavsRtk is an empirical model of a real-time-kinematics positioning sensor.

    Inherits from MavsSensor base class.

    Attributes:
    type (string): Must be 'rtk'.
    sensor (void): Pointer to a MAVS sensor.
    """
    def __init__(self):
        """Constructor for a MavsRtk."""
        MavsSensor.__init__(self)
        ## type (string): Must be 'rtk'.
        self.type = 'rtk'
        ## sensor (void): Pointer to a MAVS sensor.
        self.sensor = mavs_lib.NewMavsRtk()
    def SetError(self, error):
        """ Set the error in the MAVS Rtk sensor.

        Parmeters:
        error (float): The error factor in meters.
        """
        mavs_lib.SetRtkError(self.sensor,ctypes.c_float(error))
    def SetDropoutRate(self,dropout_rate):
        """Set the dropout rate in GPS dropouts/hour.

        Parameters:
        dropout_rate (float): The number of GPS dropouts/hour.
        """
        mavs_lib.SetRtkDroputRate(self.sensor,ctypes.c_float(dropout_rate))
    def SetWarmupTime(self,warmup_time):
        """Set the warmup time of the sensor in seconds.

        The sensor error exponentially decreases in the minimum error value.
        This parameter controls the rate of that decrease.

        Parameters:
        warmup_time (float): The sensor warmup time in seconds.
        """
        mavs_lib.SetRtkWarmupTime(self.sensor,ctypes.c_float(warmup_time))
    def GetPosition(self):
        """Get the position measured by the RTK sensor.

        Returns:
        position ([float, float, float]): The measured x-y-z position in local ENU.
        """
        position = [0.0, 0.0, 0.0]
        pos = mavs_lib.GetRtkPosition(self.sensor)
        position[0] = pos[0]
        position[1] = pos[1]
        position[2] = pos[2]
        return position
    def GetOrientation(self):
        """Get the orientation measured by the RTK sensor.

        Returns:
        orientation ([float, float, float, float]): The measured w-x-y-z orientation in local ENU.
        """
        orientation = [1.0, 0.0, 0.0, 0.0]
        ori = mavs_lib.GetRtkOrientation(self.sensor)
        orientation[0] = ori[0]
        orientation[1] = ori[1]
        orientation[2] = ori[2]
        orientation[3] = ori[3]
        return orientation

class MavsRadarTarget(object):
    def __init__(self):
        self.id = 0
        self.status = 0
        self.range = 0.0
        self.range_rate = 0.0
        self.range_accleration = 0.0
        self.angle = 0.0
        self.width=0.0
        self.lateral_rate = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
    def __str__(self):
        retstr = "ID = "+str(self.id)+"\n"
        retstr = retstr + "Status = "+str(self.status)+"\n"
        retstr = retstr + "Range = "+str(self.range)+"\n"
        retstr = retstr + "Range Rate = "+str(self.range_rate)+"\n"
        retstr = retstr + "Range Accel = "+str(self.range_accleration)+"\n"
        retstr = retstr + "Angle = "+str(self.angle)+"\n"
        retstr = retstr + "Width = "+str(self.width)+"\n"
        retstr = retstr + "Lateral Rate = "+str(self.lateral_rate)+"\n"
        retstr = retstr + "Position.x = "+str(self.position_x)+"\n"
        retstr = retstr + "Position.y = "+str(self.position_y)+"\n"
        return retstr

class MavsRadar(MavsSensor):
    """MavsRadar model.

    Uses ray-tracing and target cross-sections based solely on size.
    Inherits from MavsSensor.

    Attributes:
    type (string): Must be set to 'radar'.
    sensor (void): Pointer to a MAVS sensor.
    """
    def __init__(self):
        """MavsRadar constructor."""
        MavsSensor.__init__(self)
        ## type (string): Must be set to 'radar'.
        self.type = 'radar'
        ## sensor (void): Pointer to a MAVS sensor.
        self.sensor = mavs_lib.NewMavsRadar()
    def SetMaxRange(self,mr):
        """Set the maximum range of the radar, in meters.

        Parameters:
        mr (float): The maximum range in meters.
        """
        mavs_lib.SetRadarMaxRange(self.sensor,ctypes.c_float(mr))
    def SetSampleResolution(self,samp_res_degrees):
        """Set the angular resolution of the radar model calculation

        Parameters:
        samp_res_degrees (float): The angular resolution in degrees
        """
        mavs_lib.SetRadarSampleResolution(self.sensor,ctypes.c_float(samp_res_degrees))
    def SetFieldOfView(self,fov, samp_res_degrees):
        """Set the horizontal field of view of the radar, in degrees.

        Parameters:
        fov (float): The horizontal field of view in degrees.
        samp_res_degrees (float): The angular resolution in degrees
        """
        mavs_lib.SetRadarFieldOfView(self.sensor,ctypes.c_float(fov), ctypes.c_float(samp_res_degrees))
    def SaveImage(self,fname):
        """Save the current radar scan to a top-down image.

        Parameters:
        fname (string): The output file name, including path and extension.
        """
        mavs_lib.SaveMavsRadarImage(self.sensor,PyStringToChar(fname))
    def GetTargets(self):
        """Get a list containing the x-y position of all the returned targets.

        The target locations are in the sensor frame.
        Returns a list of Nx2 points where N is the number of targets.

        Returns:
        xy_targ (list of floats): x-y positions of each target.
        """
        num_targ = mavs_lib.GetRadarNumTargets(self.sensor)
        target_data = mavs_lib.GetRadarTargets(self.sensor)
        radar_targets = []
        for i in range(0,num_targ):
            n=i*10
            target = MavsRadarTarget()
            target.id = int(target_data[n])
            target.status = int(target_data[n+1])
            target.range = float(target_data[n+2])
            target.range_rate = float(target_data[n+3])
            target.range_accleration = float(target_data[n+4])
            target.angle = float(target_data[n+5])
            target.width = float(target_data[n+6])
            target.lateral_rate = float(target_data[n+7])
            target.position_x = float(target_data[n+8])
            target.position_y = float(target_data[n+9])
            radar_targets.append(target)
        return radar_targets

class MavsScene(object):
    """MavsScene class.

    A Mavs Scene is a geometrical description of the environment and the associated raytracer.
    While the C++ version of the API can support any type of raytracer, the Python version only
    supports the Embree ray-tracer that is the default raytracing kernel in MAVS.

    Attributes:
    scene (void): Pointer to an Embree Raytracer scene.
    """
    def __init__(self):
        """Constructor for a MavsScene."""
        ## scene (void): Pointer to an Embree Raytracer scene.
        self.scene = None
    def __del__(self):
        """Destructor for a MavsScene."""
        if (self.scene):
            mavs_lib.DeleteEmbreeScene(self.scene)
        self.scene = None
    def DeleteCurrentScene(self):
        """Free the pointer to the current scene."""
        if (self.scene):
            mavs_lib.DeleteEmbreeScene(self.scene)
        self.scene = None
    def WriteStats(self, directory='./'):
        """Write the stats of the scene to scene_stats.txt."""
        mavs_lib.WriteEmbreeSceneStats(self.scene, PyStringToChar(directory))
    def DeleteScene(self):
        """Free the pointer to the current scene.

        Duplicate of 'DeleteCurrentScene'.
        """
        mavs_lib.DeleteEmbreeScene(self.scene)
    def TurnOnLabeling(self):
        """Turn on scene labeling. 
        
        This must be called in order to use a scene to generate annotated data.
        Labeling should only be turned on if you plan to use the labeled data 
        because it slows the simulation down.
        """
        mavs_lib.TurnOnMavsSceneLabeling(self.scene)
    def TurnOffLabeling(self):
        """Turn off scene labeling.

        Call this if you don't plan to use labeled data.
        It will speed the simulation upp slightly.
        """
        mavs_lib.TurnOffMavsSceneLabeling(self.scene)
    def AddAnimation(self, animation):
        """Add an animation to a scene.

        Parameters:
        animation (MavsAnimation): An animation object. It should already have been loaded.

        Returns:
        anim_num (int): A unique ID for the animation.
        """
        anim_num = mavs_lib.AddAnimationToScene(self.scene,animation.object)
        return anim_num
    def GetSurfaceHeight(self,x,y):
        """Get the height of the surface at a given lateral position.

        Parameters:
        x (float): The x-coordinate in global ENU to get the height.
        y (float): The y-coordinate in global ENU to get the height.

        Returns:
        h (float): The height at (x,y) in global ENU meters.
        """
        h = mavs_lib.GetSurfaceHeight(self.scene,ctypes.c_float(x),ctypes.c_float(y))
        return h

class MavsEmbreeScene(MavsScene):
    """MavsEmbreeScene class.

    Has methods to load embree scenes.
    Inherits from the MavsScene class.

    Attributes:
    scene (void): Poiner to a MAVS Embree scene.
    """
    def __init__(self):
        """Constructor for a MavsEmbreeScene."""
        ## scene (void): Poiner to a MAVS Embree scene.
        self.scene = mavs_lib.NewEmbreeScene()
    def WriteStats(self,output_directory):
        mavs_lib.WriteEmbreeSceneStats(self.scene,PyStringToChar(output_directory))
    def Load(self,fname):
        """Load a json scene file.

        Example input files can be found in mavs/data/scenes

        Parameters:
        fname (string): The scene file name, relative to the MAVS data path.
        """
        mavs_lib.LoadEmbreeScene(self.scene,PyStringToChar(fname))
    def LoadRandom(self,fname):
        """Load a json file with random seed for veg placement

        Parameters:
        fname (string): The scene file name, relative to the MAVS data path
        """
        mavs_lib.LoadEmbreeSceneWithRandomSeed(self.scene,PyStringToChar(fname))

class MavsRandomScene(MavsScene):
    """MavsRandomScene class.
    
    A random scene is created with specific terrain and vegetation properties.
    Inherits from the MavsScene class.

    Attributes:
    scene (void): Pointer to a MAVS scene.
    terrain_width (float): Width (x-dimension) of the terrain in meters.
    terrain_length (float): Length (y-dimension) of the terrain in meters.
    lo_mag (float): Magnitude in meters of the low-frequency terrain roughness.
    hi_mag (float): Magnitude in meters of the high-frequency terrain roughness.
    mesh_resolution (float): Resolution of the mesh in meters.
    trail_width (float): Width of the automatically generated trail in meters.
    track_width (float): Width of the tire tracks on the trail in meters.
    wheelbase (float): Distance between tire tracks on the trail in meters.
    pothole_depth (float): Depth of potholes in the trail in meters.
    pothole_diameter (float): Diameter of the potholes in the trail in meters.
    num_potholes (int): Total number of potholes in the scene.
    path_type (string): Type of automatically generated trail. Options are 'Loop', 'Ridges', or 'Valleys'.
    eco_file (string): The ecosystem file to generate vegetation distribution. Examples in mavs/data/ecosystem_files.
    output_directory (string): Directory to save the generated scene. cwd is the default.
    basename (string): Naming to use for all the generated output files.
    plant_density (float): Density of vegetation from [0:1]
    pothole_locations (list of floats): Nx2 list where N is the number of potholes and x-y position is specified.
    """
    def __init__(self):
        """MavsRandomScene constructor."""
        ## scene (void): Pointer to a MAVS scene.
        self.scene = None
        ## terrain_width (float): Width (x-dimension) of the terrain in meters.
        self.terrain_width = 50.0
        ## terrain_length (float): Length (y-dimension) of the terrain in meters.
        self.terrain_length = 50.0
        ## lo_mag (float): Magnitude in meters of the low-frequency terrain roughness.
        self.lo_mag = 0.0
        ## hi_mag (float): Magnitude in meters of the high-frequency terrain roughness.
        self.hi_mag = 0.0
        ## mesh_resolution (float): Resolution of the mesh in meters.
        self.mesh_resolution = 1.0
        ## trail_width (float): Width of the automatically generated trail in meters.
        self.trail_width = 2.0
        ## track_width (float): Width of the tire tracks on the trail in meters.
        self.track_width = 0.3
        ## wheelbase (float): Distance between tire tracks on the trail in meters.
        self.wheelbase = 1.8
        ## pothole_depth (float): Depth of potholes in the trail in meters.
        self.pothole_depth = 0.0
        ## pothole_diameter (float): Diameter of the potholes in the trail in meters.
        self.pothole_diameter = 0.0
        ## num_potholes (int): Total number of potholes in the scene.
        self.num_potholes = 0
        ## path_type (string): Type of automatically generated trail. Options are 'Loop', 'Ridges', or 'Valleys'.
        self.path_type = 'Loop'
        ## eco_file (string): The ecosystem file to generate vegetation distribution. Examples in mavs/data/ecosystem_files.
        self.eco_file = 'american_southeast_forest.json'
        ## output_directory (string): Directory to save the generated scene. cwd is the default.
        self.output_directory = './'
        ## basename (string): Naming to use for all the generated output files.
        self.basename = 'forest'
        ## plant_density (float): Density of vegetation from [0:1]
        self.plant_density = 0.1
        ## pothole_locations (list of floats): Nx2 list where N is the number of potholes and x-y position is specified.
        self.pothole_locations = mavs_lib.NewPointList2D()
        ## type of surface roughness - can be "variable", "gaussian", or "perlin"
        self.surface_roughness_type = "perlin"
    def __del__(self):
        """MavsRandomScene destructor."""
        mavs_lib.DeleteEmbreeScene(self.scene)
        mavs_lib.DeletePointList4D(self.pothole_locations);
    def AddPotholeAt(self,x,y,depth,diameter):
        """Add pothole of a given size and location.

        Parameters:
        x (float): x-coordinate of the pothole in global ENU.
        y (float): y-coordinate of the pothole in global ENU.
        depth (float): Depth of the pothole in meters.
        diameter (float): Diameter of the pothole in meters.
        """
        #mavs_lib.AddPointToList4D(self.pothole_locations,ctypes.c_float(diameter),ctypes.c_float(x),ctypes.c_float(y),ctypes.c_float(depth))
        mavs_lib.AddPointToList4D(self.pothole_locations, ctypes.c_float(x),ctypes.c_float(y),ctypes.c_float(depth), ctypes.c_float(diameter))
    def CreateGapScene(self, gap_width, gap_height, gap_angle_radians):
        """Generate a gap crossing scene with given parameters.

        """
        self.scene = mavs_lib.CreateGapScene(ctypes.c_float(self.terrain_width),
                                                    ctypes.c_float(self.terrain_length),
                                                    ctypes.c_float(self.hi_mag),
                                                    ctypes.c_float(self.mesh_resolution),
                                                    PyStringToChar(self.basename),
                                                    ctypes.c_float(self.plant_density),
                                                    PyStringToChar(self.eco_file),
                                                    PyStringToChar(self.output_directory),
                                                    ctypes.c_float(gap_width),
                                                    ctypes.c_float(gap_height),
                                                    ctypes.c_float(gap_angle_radians)
                                                    )
    def CreateScene(self):
        """Generate the scene from the given parameters.

        Once all the parameters have been set, call this to generate the scene.
        Will perform ecosystem simulation and save files to the output directory.
        """
        self.scene = mavs_lib.CreateSceneFromRandom(ctypes.c_float(self.terrain_width),
                                                    ctypes.c_float(self.terrain_length),
                                                    ctypes.c_float(self.lo_mag),
                                                    ctypes.c_float(self.hi_mag),
                                                    ctypes.c_float(self.mesh_resolution),
                                                    ctypes.c_float(self.trail_width),
                                                    ctypes.c_float(self.wheelbase),
                                                    ctypes.c_float(self.track_width),
                                                    PyStringToChar(self.path_type),
                                                    PyStringToChar(self.surface_roughness_type),
                                                    PyStringToChar(self.basename),
                                                    ctypes.c_float(self.plant_density),
                                                    self.pothole_locations,
                                                    PyStringToChar(self.eco_file),
                                                    PyStringToChar(self.output_directory))

class MavsEnvironment(object):
    """MavsEnvironment class.

    A Mavs environment is a description of properties like the
    atmosphere, weather, geo-location, and time of day.
    
    The environment must also contain a pointer to a MavsScene, a description of the geometry.

    Attributes:
    obj (void): Pointer to a MAVS Environment.
    actor_ids (list of ints): A list of ID numbers for all the actors that have been added.
    rain_rate (float): Rain rate in mm/h, [0-25].
    turbidity (float): Turbidity (haze) factor, [2-10].
    hour (int): Time of day from 0-23.
    fog (float): Fog cover from 0-100.
    year (int): The year in XXXX format.
    snow_rate (float): The snow rate in mm/h [0-25].
    cloud_cover (float): The cloud cover fraction (0-1.0).
    wind (float): 2D vector specifying the lateral windspeed and direction in m/s.
    albedo (float): Global albedo of the local terrain, (0.0-1.0).
    month (int): Month of the year, 1-12
    day (int): Day of the month, 1-31
    minute (int): Minute of the hour, 0-59
    second (int): Seconds of the minute, 0-59
    """
    def __init__(self):
        """Constructor for a MavsEnvironment."""
        ## obj (void): Pointer to a MAVS Environment.
        self.obj = mavs_lib.NewMavsEnvironment()
        ## actor_ids (list of ints): A list of ID numbers for all the actors that have been added.
        self.actor_ids = []
        ## rain_rate (float): Rain rate in mm/h, [0-25].
        self.rain_rate = 0.0
        ## turbidity (float): Turbidity (haze) factor, [2-10].
        self.turbidity = 3.0
        ## hour (int): Time of day from 0-23.
        self.hour = 12
        ## fog (float): Fog cover from 0-100.
        self.fog = 0.0
        ## year (int): The year in XXXX format.
        self.year = 2004
        ## snow_rate (float): The snow rate in mm/h [0-25].
        self.snow_rate = 0.0
        ## cloud_cover (float): The cloud cover fraction (0-1.0).
        self.cloud_cover = 0.3
        ## wind (float): 2D vector specifying the lateral windspeed and direction in m/s.
        self.wind = [0.0, 0.0]
        ## albedo (float): Global albedo of the local terrain, (0.0-1.0).
        self.albedo = 0.1
        ## Month of the year, 1-12
        self.month = 6
        ## Day of the month, 1-31
        self.day = 5
        ## Minute of the hour, 0-59
        self.minute = 0
        ## Seconds of the minute, 0-59
        self.second = 0
        # The scene to use
        self.scene = MavsEmbreeScene()
        #Lat of local origin
        self.local_origin_lat = 32.3033
        #Lon of local origin
        self.local_origin_lon = 90.8742
        #Alt of local origin 
        self.local_origin_alt = 73.152
    def __del__(self):
        """Destructor for a MavsEnvironment."""
        if (self.obj):
            mavs_lib.DeleteMavsEnvironment(self.obj)
        self.obj = None
    def DeleteEnvironment(self):
        """Free the pointer to the environment object."""
        if (self.obj):
            mavs_lib.DeleteMavsEnvironment(self.obj)
        self.obj = None
    def SetTerrainProperties(self,type, strength):
        """Set the soil type and strength of the terrain.

        Parameters:
        type (string): Can be 'dry', 'wet', 'snow', 'clay', or 'sand'.
        strength (float): Soil strngth in Cone-Index (Pascals).
        """
        mavs_lib.SetTerrainProperties(self.obj,PyStringToChar(type),ctypes.c_float(strength))
    def SetScene(self,scene):
        """Set the MAVS scene. If a pointer is supplied as an arg.
        then the referenced object must stay in scope for the life of the scene.
        If a MavsScene object is supplied, a copy is made that stays with the
        MavsEnvironment.

        Parameters:
        scene (void): Pointer to the MAVS scene.
        """
        if isinstance(scene,MavsScene):
            self.scene = scene
            mavs_lib.SetEnvironmentScene(self.obj, self.scene.scene)
        else:
            print("WARNING, setting scene with pointer may cause seg fault if scene goes out of scope!")
            sys.stdout.flush()
            mavs_lib.SetEnvironmentScene(self.obj,scene)
    def GetVegDensityOnAGrid(self,ll,ur,res):
        """Get the vegetation density on a 3d grid

        Parameters:
        ll ([float, float, float]): lower left corner of the grid in ENU
        ur ([float, float, float]): upper right corner of the grid in ENU
        res (float): resolution of the grid in meters

        Returns:
        grid : A 3d Array of impermeability values with range from 0 (totally permeable) to 1 (totally impermable)
        """
        out_grid = mavs_lib.GetSceneDensity(self.obj, ctypes.c_float(ll[0]), ctypes.c_float(ll[1]), ctypes.c_float(ll[2]),
                                            ctypes.c_float(ur[0]), ctypes.c_float(ur[1]), ctypes.c_float(ur[2]), ctypes.c_float(res))
        nx = int(math.ceil((ur[0]-ll[0])/res))
        ny = int(math.ceil((ur[1]-ll[1])/res))
        nz = int(math.ceil((ur[2]-ll[2])/res))
        grid = []
        if (nx<=0 or ny<=0 or nz<=0):
            return grid
        else:
            grid = [[ [0.0 for k in range(nz)] for j in range(ny)] for k in range(nx)] 
            n = 0
            for i in range(nx):
                for j in range (ny):
                    for k in range (nz):
                        grid[i][j][k] = out_grid[n]
                        n = n+1
            return grid
    def LoadScene(self,scene_file):
        """Load a MAVS Scene

        Parameters:
        scene_file (string): Full path the the MAVS scene file.
        """
        self.scene.Load(scene_file)
        self.SetScene(self.scene)
    def FreeScene(self):
        """Free the pointer to the MAVS scene."""
        mavs_lib.FreeEnvironmentScene(self.obj)
    def AdvanceTime(self,dt):
        """Advance environment time by dt seconds.

        This will move the snowflakes, actors, and 
        anything else dynamic in the environment.

        Paramters:
        dt (float): The length of time to advance in seconds.
        """
        mavs_lib.AdvanceEnvironmentTime(self.obj,ctypes.c_float(dt))
    def GetAnimationPosition(self,anim_num):
        """Return the position of an animation in global ENU.

        Parameters:
        anim_num (int): The animation ID number.

        Returns:
        position ([float, float, float]): The x-y-z position of the animation in global ENU.
        """
        p = mavs_lib.GetAnimationPosition(self.obj,ctypes.c_int(anim_num))
        position = [p[0],p[1],p[2]]
        return position
    def SetAnimationPosition(self,anim_id,x,y,heading):
        """Set the position of the animation in global ENU.

        The animation will be automatically locked to the ground.

        Parameters:
        anim_id (int): The animation ID number.
        x (float): The x-positon in global ENU.
        y (float): The y-positon in global ENU.
        heading (float): Heading relative to East/X in radians.
        """
        mavs_lib.SetAnimationPositionInScene(self.obj, ctypes.c_int(anim_id),ctypes.c_float(x), ctypes.c_float(y), ctypes.c_float(heading))
    def AddPointLight(self,color,position):
        """Add a point-light to the scene

        A point light has a 1/r^2 falloff in all directions.

        Parameters:
        color ([float, float, float]): The RGB color of the light in the range [0:255]
        position ([float, float, float]): The x-y-z position of the light in global ENU.
        """
        mavs_lib.AddPointLight(self.obj,ctypes.c_float(color[0]),ctypes.c_float(color[1]),ctypes.c_float(color[2]),
                               ctypes.c_float(position[0]),ctypes.c_float(position[1]),ctypes.c_float(position[2]))
    def AddSpotLight(self,color,position,direction,angle):
        """Add a spotlight to the scene

        A point light has a 1/r falloff in the direction of the light.

        Parameters:
        color ([float, float, float]): The RGB color of the light in the range [0:255]
        position ([float, float, float]): The x-y-z position of the light in global ENU.
        direction ([float, float, float]): The normalized x-y-z direction vector of the light in global ENU.
        angle (float): The opening angle of the spotlight in radians.
        """
        light_id = mavs_lib.AddSpotLight(self.obj,ctypes.c_float(color[0]),ctypes.c_float(color[1]),ctypes.c_float(color[2]),
                               ctypes.c_float(position[0]),ctypes.c_float(position[1]),ctypes.c_float(position[2]),
                               ctypes.c_float(direction[0]),ctypes.c_float(direction[1]),ctypes.c_float(direction[2]),
                               ctypes.c_float(angle))
        return light_id
    def MoveLight(self,light_id,position,direction):
        """Move a light to a certain position.

        light_id (int): ID num of the light to move
        position ([float,float,float]): New position of the light
        direction ([float, float, float]): New orientation of the light, as a "look to" vector
        """
        mavs_lib.MoveLight(self.obj,ctypes.c_int(light_id), ctypes.c_float(position[0]),ctypes.c_float(position[1]),ctypes.c_float(position[2]),ctypes.c_float(direction[0]),ctypes.c_float(direction[1]),ctypes.c_float(direction[2]))
    def AddActor(self,actorfile,auto_update=True):
        """Add an actor to the environment.

        An actor is a moving object like a car or pedestrian.
        Example actor files can be found in mavs/data/actors.

        Parameters:
        actorfile (string): Name of the json file with the actor inputs.
        auto_update (bool): Set to True to have MAVS automatically move the actor using the prescribed behavior.

        Returns:
        actor_id (int): An ID number for the actor to be used in future modifications.
        """
        actor_id = mavs_lib.AddActorToEnvironment(self.obj,PyStringToChar(actorfile),ctypes.c_bool(auto_update))
        self.actor_ids.append(actor_id)
        return actor_id
    def AddDustToActor(self,actor_num):
        '''Add dust behind and actor.

        An actor lica a car can generate dust as it moves.
        Call this to add dust to a specific actor.

        Parameters:
        actor_num (int): The ID of the actor to which dust will be added.
        '''
        mavs_lib.AddDustToActor(self.obj,ctypes.c_int(actor_num))
    def AddDustToActorColor(self,actor_num, col):
        '''Add dust behind and actor and set the color.

        An actor lica a car can generate dust as it moves.
        Call this to add dust to a specific actor.

        Parameters:
        actor_num (int): The ID of the actor to which dust will be added.
        col (float): 3-array of the r-g-b reflectance (0-1)
        '''
        mavs_lib.AddDustToActorColor(self.obj,ctypes.c_int(actor_num), ctypes.c_float(col[0]),ctypes.c_float(col[1]),ctypes.c_float(col[2]))
    def AddDustToLocation(self,position,velocity,dust_size,dust_rate, vel_rand_fac):
        """Add dust at a given spot and rate.

        Dust can be added to the scene at a given position,
        independent of any actor.

        Parameters:
        position ([float, float, float]): x-y-z position of the dust in global ENU.
        velocity ([float, float, float]): x-y-z velocity of the dust in global ENU.
        dust_size (float): Dust ball radius in meters.
        dust_rate (float): Rate that dust is added to the scene in particles/second.
        vel_rand_fac (float): The velocity randomization in m/s.
        """
        mavs_lib.AddDustToEnvironment(self.obj,ctypes.c_float(position[0]),ctypes.c_float(position[2]),ctypes.c_float(position[2]),
                                      ctypes.c_float(velocity[0]),ctypes.c_float(velocity[1]),ctypes.c_float(velocity[2]),
                                      ctypes.c_float(dust_rate), ctypes.c_float(dust_size), ctypes.c_float(vel_rand_fac))
    def SetActorPosition(self,actor_id, pos, quat):
        """Set the position of the actor in world coordinates.

        Parameters:
        actor_id (int): The ID number of the actor to be moved.
        pos ([float,float,float]): New x-y-z position of the actor in global ENU.
        quat ([float,float,float,float]): New w-x-y-z orientation of the actor in global ENU.
        """
        position = ctypes.c_float*3;
        ori = ctypes.c_float*4;
        p = position(pos[0],pos[1],pos[2])
        q = ori(quat[0],quat[1],quat[2],quat[3])
        mavs_lib.SetActorPosition(self.obj,ctypes.c_int(actor_id),p,q)
    def UpdateParticleSystems(self, dt):
        """Call this to update particle systems in the environment.

        Particle systems include smoke and dust.
        This will be called automatically if 'AdvanceTime(dt)'
        method is invoked.

        Parameters:
        dt (float): The time step of the update in seconds.
        """
        mavs_lib.UpdateParticleSystems(self.obj,ctypes.c_float(dt))
    def SetRainRate(self, r):
        """Set the rain rate in the environment in mm/h.

        Typical rain rates are 5-10 mm/h (light rain)
        to 25 mm/h (heavy rain).

        Parameters:
        r (float): Rain rate in mm/h.
        """
        mavs_lib.SetRainRate(self.obj,ctypes.c_float(r))
        self.rain_rate = r
    def SetTurbidity(self, turbid):
        """Set the turbidity of the atmosphere.

        Turbidity is a measure of haziness. It should range from
        2 (very clear) to 10 (very hazy)

        Parameters:
        turbid (float): The turbidity index.
        """
        mavs_lib.SetTurbidity(self.obj,ctypes.c_float(turbid))
        self.turbidity = turbid
    def SetAlbedo(self,albedo):
        """Set the local albedo.

        Albedo is average surface reflectance.

        Parameters:
        albedo (float): Albedo from 0-1.
        """
        mavs_lib.SetAlbedo(self.obj,ctypes.c_float(albedo), ctypes.c_float(albedo), ctypes.c_float(albedo))
    def SetFog(self,fog):
        """Set the fogginess.

        Parameters:
        fog (float): Fog from 0-100. 100 is very foggy.
        """
        mavs_lib.SetFog(self.obj,ctypes.c_float(fog))
        self.fog = fog
    def SetTime(self,hour):
        """Set the hour in military (0-23) time.

        Parameters:
        hour (int): The hour from 0 (midnight) to 23 (11 PM).
        """
        mavs_lib.SetTime(self.obj,ctypes.c_int(hour))
        self.hour = hour
    def SetSkyOnOff(self, sky_on):
        """Turn the sky model on/off

        Parameters:
        sky_on (bool): True to turn the sky on. True by default.
        """
        mavs_lib.TurnSkyOnOff(self.obj, ctypes.c_bool(sky_on))
    def SetSkyColor(self, r,g,b):
        """Set the RGB color of the sky

        Parameters:
        r (float): Red channel, 0.0-255.0
        g (float): Green channel, 0.0-255.0
        b (float): Blue channel, 0.0-255.0
        """
        mavs_lib.SetSkyColor(self.obj, ctypes.c_float(r), ctypes.c_float(g), ctypes.c_float(b))
    def SetSunColor(self, r,g,b):
        """Set the RGB color of the sun

        Parameters:
        r (float): Red channel, 0.0-255.0
        g (float): Green channel, 0.0-255.0
        b (float): Blue channel, 0.0-255.0
        """
        mavs_lib.SetSunColor(self.obj, ctypes.c_float(r), ctypes.c_float(g), ctypes.c_float(b))
    def SetSunPosition(self, azimuth_degrees, zenith_degrees):
        """Set the relative position of the sun in the sky

        Parameters:
        azimuth_degrees (float): Azimuth angle in degrees east of north
        zenith_degrees (float): Zenith angle in degrees off vertical
        """
        mavs_lib.SetSunLocation(self.obj, ctypes.c_float(azimuth_degrees), ctypes.c_float(zenith_degrees))
    def SetSunSolidAngle(self, solid_angle_degrees):
        """Set the solid angle of the sun in the sky

        Parameters:
        solid_angle_degrees (float): Solid angle of the sun in the sky
        """
        mavs_lib.SetSunSolidAngle(self.obj, ctypes.c_float(solid_angle_degrees))
    def SetTimeSeconds(self,hour, minute, second):
        """Set the hour in military (0-23) time.

        Including minutes and seconds

        Parameters:
        hour (int): The hour from 0 (midnight) to 23 (11 PM).
        minute (int): The minute from 0 to 59.
        second (int): The second from 0 to 59.
        """
        mavs_lib.SetTimeSeconds(self.obj, ctypes.c_int(hour), ctypes.c_int(minute), ctypes.c_int(second))
        self.hour = hour
        self.minute = minute
        self.second = second
    def SetDate(self,year,month,day):
        """Set the date of the simulation.

        This will influence the location of the sun and stars.

        Parameters:
        year (int): The year in XXXX format.
        month (int): The month in 1-12 format.
        day (int): Day of the year in 1-365 format.
        """
        mavs_lib.SetDate(self.obj,ctypes.c_int(year),ctypes.c_int(month),ctypes.c_int(day))
        self.year = year
        self.month = month
        self.day = day
    def SetCloudCover(self,cover):
        """Set the cloud cover fraction.
        
        Set the fraction of the sky that is covered by clouds, from [0,1].

        Parameters:
        cover (float): The cloud cover fraction, 0.0-1.0.
        """
        mavs_lib.SetCloudCover(self.obj,ctypes.c_float(cover))
        self.cloud_cover = cover
    def SetSnow(self,snow_rate):
        """Set the snow rate.

        Parameters:
        snow_rate (float): Snow rate in mm/h, 0.0-25.0.
        """
        self.snow_rate = snow_rate
        mavs_lib.SetSnowRate(self.obj,ctypes.c_float(snow_rate))
    def SetSnowAccumulation(self, snow_accum):
        """Set the snow accumalation factor.

        Parameters:
        snow_accum (float): Snow accumulation factor in mm/hour.
        """
        mavs_lib.SetSnowAccumulation(self.obj,ctypes.c_float(snow_accum))
    def SetWind(self,wind):
        """Set the wind speed and direction.

        Parameters:
        wind ([float, float]): The lateral wind speed and direction in m/s.
        """
        self.wind = wind
        mavs_lib.SetWind(self.obj, ctypes.c_float(wind[0]),ctypes.c_float(wind[1]))
    def SetLocalOrigin(self, lat, lon, alt): 
        """Set lat lon and alt of the local origin.

        Parameters:
        lat (float): latitude of the local origin 
        lon (float): longitude of the local origin
        alt (float): altitude of the local origin
        """
        mavs_lib.SetLocalOrigin(self.obj, ctypes.c_double(lat), ctypes.c_double(lon), ctypes.c_double(alt))
        self.lat = lat 
        self.long = lon
        self.alt = alt
    def load_block(self,data):
        """Load environment parameters.

        Loads environment parameters from a json dictionary.

        Parameters:
        data (dictionary): Data block to load.
        """
        self.year = data["Year"]
        self.hour = data["Hour"]
        self.minute = data["Minute"]
        self.second = data["Second"]
        self.month = data["Month"]
        self.day = data["Day"]
        self.SetTime(self.hour)
        self.SetDate(self.year,self.month,self.day)
        if 'Fog' in data:
            self.SetFog(data["Fog"])
        if 'Snow Rate' in data:
            self.snow_rate = data["Snow Rate"]
            self.SetSnow(self.snow_rate)
        if 'Turbidity' in data:
            self.turbidity = data["Turbidity"]
            self.SetTurbidity(self.turbidity)
        if 'Local Albedo' in data:
            self.albedo = data["Local Albedo"]
            self.SetAlbedo(self.albedo)
        if 'Cloud Cover' in data:
            self.cloud_cover = data['Cloud Cover']
            self.SetCloudCover(0.01*self.cloud_cover)
        if 'Rain Rate' in data:
            self.rain_rate = data["Rain Rate"]
            self.SetRainRate(self.rain_rate)
        if 'Wind' in data:
            self.wind = data["Wind"]
            self.SetWind(self.wind)
        if 'Snow Rate' in data:
            self.snow_rate = data["Snow Rate"]
            self.SetSnow(self.snow_rate)
    def GetNumberOfObjects(self):
        """Return the total number of unique objects (meshes) in the scene.

        Returns:
        num_obj (int): The number of objects in the scene.
        """
        num_obj = mavs_lib.GetNumberOfObjectsInEnvironment(self.obj)
        return num_obj
    def GetObjectBoundingBox(self,object_id):
        """Get the bound box of a particular object.

        Parameters:
        object_id (int): The ID number of the object in question.

        Returns:
        bounding_box ([float, float, float],[float, float, float]): A 2X3 list containing the lower left and upper right corners.
        """
        bb = mavs_lib.GetObjectBoundingBox(self.obj, ctypes.c_int(object_id))
        llc = [bb[0], bb[1], bb[2]]
        urc = [bb[3], bb[4], bb[5]]
        bounding_box = [llc,urc]
        return bounding_box
    def GetObjectName(self,object_id):
        """Get the name of a particular object.

        Parameters:
        object_id (int): The ID number of the object in question.

        Returns:
        ret_name (string): The name of the object.
        """
        obj_name = mavs_lib.GetObjectName(self.obj,ctypes.c_int(object_id))
        ret_name = obj_name.decode("utf-8","ignore")
        return ret_name

class MavsVehicle(object):
    """MavsVehicle class.

    Base class for Mavs Vehicles.
    There are two different inherited classes - Rp3d and Chrono.

    Attributes:
    vehicle (void): Pointer to a MAVS vehicle.
    position ([float, float, float]): The position of the vehicle in global ENU.
    orientation ([float, float, float, float]) The w-x-y-z orientation of the vehicle in global ENU.
    headlight_offset (float): How far forward the headlights are from the CG, in meters.
    headlight_width (float): How far apart he headlights are, in meters.
    headlight_ids ([int, int]): ID numbers for the headlights.
    """
    def __init__(self):
        """Constructor for a MavsVehicle."""
        ## vehicle (void): Pointer to a MAVS vehicle.
        self.vehicle = None
        ## position ([float, float, float]): The position of the vehicle in global ENU.
        self.position = [0.0,0.0,0.0]
        ## orientation ([float, float, float, float]) The w-x-y-z orientation of the vehicle in global ENU.
        self.orientation = [1.0,0.0,0.0,0.0]
        ## headlight_offset (float): How far forward the headlights are from the CG, in meters.
        self.headlight_offset = 1.5
        ## headlight_width (float): How far apart he headlights are, in meters.
        self.headlight_width = 1.5
        ## headlight_ids ([int, int]): ID numbers for the headlights.
        self.headlight_ids = []
    def Update(self,env,throttle,steering, brake, dt):
        """Update the vehicle model.

        Apply throttle and steering and move the vehicle.

        Parameters:
        env (MavsEnvironment): The MAVS environment.
        throttle (float): Throttle from 0 to 1.
        steering (float): Steering from -1 to 1.
        dt (float): The time step in seconds.
        """
        mavs_lib.UpdateMavsVehicle(self.vehicle, env.obj, ctypes.c_float(throttle), ctypes.c_float(steering), ctypes.c_float(brake), ctypes.c_float(dt))
        if len(self.headlight_ids)!=0:
            mavs_lib.MoveHeadlights(env.obj, self.vehicle, ctypes.c_float(self.headlight_offset), ctypes.c_float(self.headlight_width), ctypes.c_int(self.headlight_ids[0]), ctypes.c_int(self.headlight_ids[1]))
        self.position = self.GetPosition()
        self.orientation = self.GetOrientation()
    def AddHeadlights(self,env):
        """Add headlights to the vehicle.
       
        The various headlight parameters should be set before calling this.

        Parameters:
        env (MavsEnvironment): The mavs environment containing the vehicle.
        """
        if (len(self.headlight_ids)==0):
            ids = mavs_lib.AddHeadlightsToVehicle(env.obj, self.vehicle, ctypes.c_float(self.headlight_offset), ctypes.c_float(self.headlight_width))
            self.headlight_ids.append(ids[0])
            self.headlight_ids.append(ids[1])
    def SetInitialPosition(self,x,y,z):
        """Set the initial position of the vehicle in global ENU.

        Parameters:
        x (float): Initial x-coordinate in global ENU.
        y (float): Initial y-coordinate in global ENU.
        z (float): Initial z-coordinate in global ENU.
        """
        mavs_lib.SetMavsVehiclePosition(self.vehicle,ctypes.c_float(x),ctypes.c_float(y),ctypes.c_float(z))
        self.position = self.GetPosition()
    def SetInitialHeading(self,theta):
        """Set the initial heading of the vehicle.

        Relative to the East/X direction.

        Parameters:
        theta (float): Initial heading in radians.
        """
        mavs_lib.SetMavsVehicleHeading(self.vehicle,ctypes.c_float(theta))
        self.orientation = self.GetOrientation()
    def GetYawRate(self):
        """Get the current full state of the vehicle.

        Returns:
        yaw_rate (float): yaw rate of the vehicle in rad/s
        """
        statevec = mavs_lib.GetMavsVehicleFullState(self.vehicle)
        return statevec[15]
    def GetFullState(self):
        """Get the current full state of the vehicle.

        Returns:
        position ([float, float, float]): x-y-z position of the vehicle.
        orientation ([float, float, float, float]): w-x-y-z orientation of the vehicle.
        linear_velocity ([float, float, float]): x-y-z (world coordinates) velocity of the vehicle in m/s.
        angular_velocity ([float, float, float]): x-y-z angular velocity of the vehicle in rad/s.
        linear_acceleration ([float, float, float]): x-y-z (world coordinates) velocity of the vehicle in m/s (world coordinates).
        angular_acceleration ([float, float, float]): x-y-z angular acceleration of the vehicle in rad/s (world coordinates).
        """
        statevec = mavs_lib.GetMavsVehicleFullState(self.vehicle)
        position = [statevec[0], statevec[1], statevec[2]]
        orientation = [statevec[3], statevec[4], statevec[5], statevec[6]]
        linear_velocity = [statevec[7], statevec[8], statevec[9]]
        angular_velocity = [statevec[10], statevec[11], statevec[12]]
        linear_acceleration = [statevec[13], statevec[14], statevec[15]]
        angular_acceleration = [statevec[16], statevec[17], statevec[18]]
        return position,orientation,linear_velocity,angular_velocity,linear_acceleration,angular_acceleration
    def GetPosition(self):
        """Get the current position of the vehicle in global ENU.

        Returns:
        position ([float, float, float]): x-y-z position of the vehicle.
        """
        p = mavs_lib.GetMavsVehiclePosition(self.vehicle)
        position = [p[0],p[1],p[2]]
        return position
    def GetVelocity(self):
        """Get the current velocity of the vehicle in global ENU.

        Returns:
        velocity ([float, float, float]): x-y-z velocity of the vehicle in m/s.
        """
        v = mavs_lib.GetMavsVehicleVelocity(self.vehicle)
        velocity = [v[0],v[1],v[2]]
        return velocity
    def GetOrientation(self):
        """Get the current orientation of the vehicle in global ENU.

        Returns:
        orientation ([float, float, float, float]): w-x-y-z orientation of the vehicle.
        """
        q = mavs_lib.GetMavsVehicleOrientation(self.vehicle)
        orientation = [q[0],q[1],q[2],q[3]]
        return orientation
    def GetSpeed(self):
        """Get the current speed of the vehicle in m/s.

        Returns:
        speed (float): The speed of the vehicle in m/s.
        """
        speed = mavs_lib.GetMavsVehicleSpeed(self.vehicle)
        return speed
    def GetHeading(self):
        """Get the current heading of the vehicle.

        Relative to the East/X direction.

        returns:
        heading (float): Current heading in radians.
        """
        heading = mavs_lib.GetMavsVehicleHeading(self.vehicle)
        return heading
    def GetTirePositionAndOrientation(self, tire_num):
        """ Tire position and orientation in local world coordinates
        
        Parameters:
        tire_num (int): The id of the tire

        Returns:
        p,q (float): Tire position and orientation, as a quaternion
        """
        data = mavs_lib.GetMavsVehicleTirePositionAndOrientation(self.vehicle, ctypes.c_int(tire_num))
        p = [data[0],data[1],data[2]]
        q = [data[3], data[4], data[5], data[6]]
        return p,q
    def UnloadVehicle(self):
        """Free the pointer associated with the vehicle."""
        if (self.vehicle):
            mavs_lib.DeleteMavsVehicle(self.vehicle)
        self.vehicle=None

class MavsRp3d(MavsVehicle):
    """MavsRp3d class.
    
    Inherits from the MavsVehicle base class.

    Attributes:
    vehicle (void): Pointer to a MavsRp3dVehicle.
    """
    def __init__(self):
        """MavsRp3dVehicle constructor."""
        MavsVehicle.__init__(self)
        ## vehicle (void): Pointer to a MavsRp3dVehicle.
        self.vehicle = mavs_lib.NewMavsRp3dVehicle()
    def __del__(self):
        """MavsRp3dVehicle destructor."""
        if (self.vehicle):
            mavs_lib.DeleteMavsVehicle(self.vehicle)
    def Load(self, fname, reload_vis=True):
        """Load an RP3D vehicle file.

        Examples are in mavs/data/vehicles/rp3d_vehicles.

        Parameters:
        fname (string): Full path to the vehicle input file to load.
        """
        if not reload_vis:
            mavs_lib.SetMavsRp3dVehicleReloadVis(self.vehicle, ctypes.c_bool(reload_vis))
        mavs_lib.LoadMavsRp3dVehicle(self.vehicle,PyStringToChar(fname));
    def SetGravity(self, gx, gy, gz):
        """Set the gravity constant in m/s^2 in local ENU.

        Default is [0.0, 0.0, -9.806].

        Parameters:
        gx (float): Gravity constant in the x-direction (m/s^2).
        gy (float): Gravity constant in the y-direction (m/s^2).
        gz (float): Gravity constant in the z-direction (m/s^2).
        """
        mavs_lib.SetRp3dGravity(self.vehicle, ctypes.c_float(gx), ctypes.c_float(gy), ctypes.c_float(gz))

    def SetExternalForceOnCg(self, fx, fy, fz):
        """Set an external force on the CG, in Newtons

        Parameters:
        fx (float): x-component of the force
        fy (float): y-component of the force
        fz (float): z-component of the force
        """
        mavs_lib.SetRp3dExternalForce(self.vehicle, ctypes.c_float(fx), ctypes.c_float(fy), ctypes.c_float(fz))
    def GetLookTo(self):
        """Get the vehicle look to vector in world coordinates"""
        lt = mavs_lib.GetRp3dLookTo(self.vehicle)
        look_to = [lt[0],lt[1],lt[2]]
        return look_to
    def GetTireDeflection(self,i):
        """ Tire deflection as a fraction of section height.

        Returns:
        d (float): Tire deflection as a fraction of section height (0-1).
        """
        d = mavs_lib.GetRp3dVehicleTireDeflection(self.vehicle,ctypes.c_int(i))
        return d
    def GetTireNormalForce(self, tire_id):
        """ Tire normal force in Newtons

        Returns:
        nf (float): Tire normal force in newtons.
        """
        nf = mavs_lib.GetRp3dTireNormalForce(self.vehicle,ctypes.c_int(tire_id))
        return nf

    def GetTireForces(self, tire_id):
        """ Get the x-y-z force acting on the tire, in world coordinates

        Returns:
        forces ([float,float,float]): Tire forces in newtos
        """
        f = mavs_lib.GetRp3dTireForces(self.vehicle,ctypes.c_int(tire_id))
        forces = [f[0],f[1],f[2]]
        return forces
    def GetTireAngularVelocity(self, tire_id):
        """ Tire angular velocity in rad/s

        Parameters:
        tire_id (int): The id number of the tire

        Returns:
        ang_vel (float): Tire angular velocity in rad/s
        """
        ang_vel = mavs_lib.GetRp3dTireAngularVelocity(self.vehicle,ctypes.c_int(tire_id))
        return ang_vel
    def GetTireSlip(self, tire_id):
        """ Tire longintudinal slip

        Parameters:
        tire_id (int): The id number of the tire

        Returns:
        slip (float): Tire longitudinal slip
        """
        slip = mavs_lib.GetRp3dTireSlip(self.vehicle,ctypes.c_int(tire_id))
        return slip
    def GetTireSteerAngle(self, tire_id):
        """ Tire steer angle in radians

        Parameters:
        tire_id (int): The id number of the tire

        Returns:
        steer_angle (float): Tire steer angle in radians slip
        """
        steer_angle = mavs_lib.GetRp3dTireSteeringAngle(self.vehicle,ctypes.c_int(tire_id))
        return steer_angle
    def GetLateralAcceleration(self):
        """ Lateral acceleration in m/s^2

        Returns:
        lat_acc (float): Lateral acceleration in m/s^2
        """
        lat_acc = mavs_lib.GetRp3dLatAccel(self.vehicle)
        return lat_acc
    def GetLongitudinalAcceleration(self):
        """ Longitudinal acceleration in m/s^2

        Returns:
        lon_acc (float): Longitudinal acceleration in m/s^2
        """
        lon_acc = mavs_lib.GetRp3dLonAccel(self.vehicle)
        return lon_acc
    def SetUseDrag(self,use_drag):
        """ Turn drag forces on/off

        Returns:
        use_drag (bool): Set to true to apply drag forces, false to turn them off
        """
        mavs_lib.SetRp3dUseDrag(self.vehicle,ctypes.c_bool(use_drag))
    def SetTerrainProperties(self,terrain_type='flat',terrain_param1=0.0,terrain_param2=0.0,soil_type='paved',soil_strength=100.0):
        """Set the properties of the terrain.
 
        Calling this will dis-associate the loaded mesh file and use an analytical surface instead.
        Set the soil type and strength. Available soil types are
	    'snow', 'ice', 'wet', 'sand', 'clay', 'paved'
	    The soil strength param is in PSI and is only used when the type is 'clay' or 'sand'
	    AND
	    Set the terrain height function. The available terrain types are
	    'flat', 'slope', 'sine', and 'rough'. The second argument is a list of
	    parameters for the height model.

	    flat: terrain_param1 = terrain height, terrain_param2 = not used

	    sloped: terrain_param1 = fractional slope (1 = 45 degrees), terrain_param2 = not used

	    sine: terrain_param1 = wavelength in meters, terrain_param2 = magnitude of oscillation

	    rough: terrain_param1 = wavelength of roughness in meters, terrain_param2 = magnitude of roughness, in meters

        Parameters:
        terrain_type (string): The type of terrain.
        terrain_param1 (float): Parameter 1, see above.
        terrain_param2 (float): Parameter 2, see above.
        soil_type (string): Soil type, see above.
        soil_strngth (float): RCI of soil, see above.
        """
        mavs_lib.SetRp3dTerrain(self.vehicle,PyStringToChar(soil_type),ctypes.c_float(6894.76*soil_strength),PyStringToChar(terrain_type),
                                ctypes.c_float(terrain_param1),ctypes.c_float(terrain_param2))

class ChronoVehicle(MavsVehicle):
    """ChronoVehicle class.
    
    Inherits from MavsVehicle base class.

    Attributes:
    vehicle (void): Pointer to a Chrono vehicle.
    """
    def __init__(self):
        """ChronoVehicle constructor."""
        MavsVehicle.__init__(self)
        ## vehicle (void): Pointer to a Chrono vehicle.
        self.vehicle = mavs_lib.NewChronoVehicle()
    def __del__(self):
        """Chrono vehicle destructor."""
        if (self.vehicle):
            mavs_lib.DeleteMavsVehicle(self.vehicle)
        self.vehicle=None
    def Load(self,fname):
        """Load an Chrono vehicle file.

        Examples are in mavs/data/vehicles/chrono_inputs.

        Parameters:
        fname (string): Full path to the vehicle input file to load.
        """
        mavs_lib.LoadChronoVehicle(self.vehicle, PyStringToChar(fname))
    def GetTireNormalForce(self, tire_id):
        """Get tire normal force in Newtons

        """
        force = mavs_lib.GetChronoTireNormalForce(self.vehicle, ctypes.c_int(tire_id))
        return force

def GetQuatFromPoints(p,q):
    """Create a quaternion orientation from two points.

    The points are in 2D, creates a rotation about Z axis

    Parameters:
    p ([float,float]): First point
    q ([float,float]): Second point

    Returns:
    quat ([float, float, float, float]): w-x-y-z quaternion orientation.
    """
    x = q[0]-p[0]
    y = q[1]-p[1]
    m = math.sqrt(x*x + y*y)
    x = x/m
    y = y/m
    theta = 0.5*math.atan2(y,x)
    quat = [math.cos(theta),0.0, 0.0, math.sin(theta)]
    return quat

class MavsWaypoints(object):
    """MavsWaypoints class.
    
    Waypoints are a list of x-y locations.

    Attributes:
    mavs_waypoints (void): Pointer to MAVS waypoints object.
    num_waypoints (int): Total number of waypoints.
    waypoints (list of floats): Nx2 list of waypoints.
    """
    def __init__(self):
        """MavsWaypoints constructor."""
        ## mavs_waypoints (void): Pointer to MAVS waypoints object.
        self.mavs_waypoints = None
        ## num_waypoints (int): Total number of waypoints.
        self.num_waypoints = 0
        ## waypoints (list of floats): Nx2 list of waypoints.
        self.waypoints = []
    def __del__(self):
        """MavsWaypoints destructor."""
        self.UnloadWaypoints()
    def UnloadWaypoints(self):
        """Free pointer to waypoints object and unload waypoints from memory."""
        if (self.waypoints):
            mavs_lib.DeleteMavsWaypoints(self.mavs_waypoints)
        self.mavs_waypoints = None
        self.num_waypoints = 0
        self.waypoints = []
    def LoadJson(self,fname):
        """Load a list of waypoints from a json file.

        Parameters:
        fname (string): The name of the .json file to load
        """
        self.mavs_waypoints = mavs_lib.LoadWaypointsFromJson(PyStringToChar(fname))
        self.num_waypoints = mavs_lib.GetNumWaypoints(self.mavs_waypoints)
        for i in range(self.num_waypoints):
            p = mavs_lib.GetWaypoint(self.mavs_waypoints,ctypes.c_int(i))
            q = [p[0],p[1],0.0]
            self.waypoints.append(q)
    def Load(self,fname):
        """Load a list of waypoints from an ANVEL .vprp file.

        From a .vprp file in ANVEL (in text format), load and generate waypoints.
        Example inputs are in the mavs/data/waypoints directory.

        Parameters:
        fname (string): The name of the .vprp file to load
        """
        self.mavs_waypoints = mavs_lib.LoadAnvelReplayFile(PyStringToChar(fname))
        self.num_waypoints = mavs_lib.GetNumWaypoints(self.mavs_waypoints)
        for i in range(self.num_waypoints):
            p = mavs_lib.GetWaypoint(self.mavs_waypoints,ctypes.c_int(i))
            q = [p[0],p[1],0.0]
            self.waypoints.append(q)
    def FillIn(self,spacing):
        """Fill in gaps between waypoints.

        The waypoint follower works best if the waypoints are about 1 meter apart.
        Calling this will fill in gaps between waypoints to the specified distance.

        Parameters:
        spacing (float): Minimum allowed spacing between waypoints.
        """
        new_waypoints = []
        for i in range(len(self.waypoints)-1):
            new_waypoints.append([self.waypoints[i][0],self.waypoints[i][1],0.0])
            v = [self.waypoints[i+1][0]-self.waypoints[i][0], self.waypoints[i+1][1]-self.waypoints[i][1]]
            d = math.sqrt(v[0]*v[0]+v[1]*v[1])
            if d>0:
                v[0] = v[0]/d
                v[1] = v[1]/d
                x = self.waypoints[i][0]
                y = self.waypoints[i][1]
                while d>spacing:
                    x = x + spacing*v[0]
                    y = y + spacing*v[1]
                    vn = [self.waypoints[i+1][0]-x, self.waypoints[i+1][1]-y]
                    new_waypoints.append([x,y,0.0])
                    d = math.sqrt(vn[0]*vn[0]+vn[1]*vn[1])
        self.waypoints = new_waypoints
        self.num_waypoints = len(self.waypoints)
    def GetNumWaypoints(self):
        """Return the number of waypoints.

        Returns:
        self.num_waypoints (int): The total number of waypoints.
        """
        return self.num_waypoints
    def PutWaypointsOnGround(self,scene):
        """Put waypoints on the ground.
        
        Moves the "z" of the waypoints to the ground
        Waypoints from a .vprp file may appear to be floating.
        This does not affect the MavsVehicleController.

        Parameters:
        scene (MavsScene): The scene containing the geometry.
        """
        #heights = mavs_lib.PutWaypointsOnGround(scene.obj,self.mavs_waypoints)
        for i in range(self.num_waypoints):
            h = mavs_lib.GetSurfaceHeight(scene.scene,ctypes.c_float(self.waypoints[i][0]),ctypes.c_float(self.waypoints[i][1]))
            self.waypoints[i][2] = h+0.01
    def GetWaypoint(self,i):
        """Return a specific waypoint.

        Parameters:
        i (int): The waypoint number.

        Returns:
        waypoint ([float, float, float]): The x-y-z position of the waypoint in global ENU.
        """
        waypoint = [0.0, 0.0, 0.0]
        if (i>=0 and i<self.num_waypoints):
            waypoint =  self.waypoints[i]
        return waypoint
    def GetWaypoints2D(self):
        """Get a 2D list of waypoints, removing the Z coordinate.

        Returns:
        wp (list of floats): Nx2 list of waypoints in global ENU.
        """
        wp = []
        for w in self.waypoints:
            wp.append([w[0],w[1]])
        return wp
    def SaveAsJson(self, json_name):
        """Save the current waypoints a json file

        Parameters:
        json_name (string): The name of the output json file
        """
        mavs_lib.SaveWaypointsAsJson(self.mavs_waypoints, PyStringToChar(json_name))
    def GetOrientation(self,i):
        """Get the path direction at a waypoint.

        For a given waypoint, returns the direction to the next waypoint as a quaternion.

        Parameters:
        i (int): The waypoint number in question.

        Returns:
        q ([float, float, float, float]): w-x-y-z quaternion specifying the direction of the next waypoint.
        """
        q = [1.0,0.0,0.0,0.0]
        if (i>=0 and i<(self.num_waypoints-1)):
            q =  GetQuatFromPoints(self.waypoints[i],self.waypoints[i+1])
        elif (i == self.num_waypoints-1):
            q =  GetQuatFromPoints(self.waypoints[self.num_waypoints-2],self.waypoints[self.num_waypoints-1])
        return q

def remove_prefix(text, prefix):
    """Remove the prefix from a string.

    Parameters:
    text (string): The original string.
    prefix (string): The string to remove.

    Returns:
    rm_str (string): String with the prefix removed.
    """
    rm_str = text[text.startswith(prefix) and len(prefix):]
    return rm_str

class MavsSimulation(object):
    """MavsSimulation class.

    Combines vehicle, scene, environment and sensors
    into a simulation object that can be automatically 
    updated.

    Attributes:
    env (MavsEnvironment): Environment object.
    scene (MavsEmbreeScene): Scene object.
    vehicle (MavsRp3d()): Vehicle object.
    waypoints (MavsWaypoints): Waypoints object.
    controller (MavsVehicleController): Controller object.
    elapsed_time (float): Elapsed simulation time in seconds.
    wait_time (float): Bootup time of the simulation in seconds. 
    veh_actor_num (int): ID for the vehicle actor.
    origin ([float, float, float]): Scene origin in local ENU.
    time_zone (int): Time zone offset, CST=6.
    sensors ([MavsSensor]): List of sensors on the vehicle.
    scenefile (string): Name of the json scene file.
    vehicle_file (string): Name of the json vehicle file.
    posefile (string): Name of the waypoints pose file.
    posetype (string): Type of pose file, can be 'anvel' or 'json'.
    dusty (bool): Is the environment dusty?
    env_block (dict): JSON dictionary describing environment.
    free_driving (bool): If True, user drives vehicle with keyboard.
    start_heading (float): Initial vehicle heading in radians.
    start_heading_loaded (bool): Initial heading supplied in input file?
    env_time (float): Wall time taken to simulate environment (seconds).
    veh_time (float): Wall time taken to simulate the vehicle (seconds).
    sensor_times (list of floats): Wall time taken to simulate each sensor (seconds).
    save_location (string): Full path to the default save location for sensor data.
    time_to_update_actor (bool): Actor is only updated when this is true.
    start_pos ([float, float, float]): Initial position of the vehicle in global ENU.
    """
    def __init__(self):
        """Construct a simulation."""
        ## env (MavsEnvironment): Environment object.
        self.env = MavsEnvironment()
        ## scene (MavsEmbreeScene): Scene object.
        self.scene = MavsEmbreeScene()
        ## vehicle (MavsRp3d()): Vehicle object.
        self.vehicle = MavsRp3d()
        ## waypoints (MavsWaypoints): Waypoints object.
        self.waypoints = MavsWaypoints()
        ## controller (MavsVehicleController): Controller object.
        self.controller = MavsVehicleController()
        ## elapsed_time (float): Elapsed simulation time in seconds.
        self.elapsed_time = 0.0
        ## wait_time (float): Bootup time of the simulation in seconds.
        self.wait_time = 1.0
        ## veh_actor_num (int): ID for the vehicle actor.
        self.veh_actor_num = 0
        ## origin ([float, float, float]): Scene origin in local ENU.
        self.origin = [0.0, 0.0, 0.0]
        ## time_zone (int): Time zone offset, CST=6.
        self.time_zone = 6
        ## sensors ([MavsSensor]): List of sensors on the vehicle.
        self.sensors = []
        ## scenefile (string): Name of the json scene file.
        self.scenefile = ''
        ## vehicle_file (string): Name of the json vehicle file.
        self.vehicle_file = ''
        ## posefile (string): Name of the waypoints pose file.
        self.posefile = ''
        ## posetype (string): Type of pose file, can be 'anvel' or 'json'.
        self.posetype = 'anvel'
        ## dusty (bool): Is the environment dusty?
        self.dusty = False
        ## env_block (dict): JSON dictionary describing environment.
        self.env_block = None
        ## free_driving (bool): If True, user drives vehicle with keyboard.
        self.free_driving = False
        ## start_heading (float): Initial vehicle heading in radians.
        self.start_heading = 0.0
        ## start_heading_loaded (bool): Initial heading supplied in input file?
        self.start_heading_loaded = False
        ## env_time (float): Wall time taken to simulate environment (seconds).
        self.env_time = 0.0
        ## veh_time (float): Wall time taken to simulate the vehicle (seconds).
        self.veh_time = 0.0
        ## sensor_times (list of floats): Wall time taken to simulate each sensor (seconds).
        self.sensor_times = []
        ## save_location (string): Full path to the default save location for sensor data.
        self.save_location = './'
        ## time_to_update_actor (bool): Actor is only updated when this is true.
        self.time_to_update_actor = False
        ## start_pos ([float, float, float]): Initial position of the vehicle in global ENU.
        self.start_pos = [0.0, 0.0, 0.0]
    def __del__(self):
        self.env = MavsEnvironment()
        self.scene = MavsEmbreeScene()
        self.vehicle = MavsRp3d()
        self.waypoints = MavsWaypoints()
        self.controller = MavsVehicleController()
        self.elapsed_time = 0.0
        self.wait_time = 1.0
        self.veh_actor_num = 0
        self.origin = [0.0, 0.0, 0.0]
        self.time_zone = 6
        self.sensors = []
        self.scenefile = ''
        self.vehicle_file = ''
        self.posefile = ''
        self.posetype = 'anvel'
        self.dusty = False
        self.env_block = None
        self.start_heading = 0.0
        self.start_heading_loaded = False
        self.env_time = 0.0
        self.veh_time = 0.0
        self.sensor_times = []
        self.save_location = './'
        self.time_to_update_actor = False
        self.start_pos = [0.0, 0.0, 0.0]
    def LoadScene(self):
        """Load the scene specified by scenefile.

        scenefile should already be specified.
        """
        self.scene.Load(self.scenefile)
        self.env.SetScene(self.scene)
        #---- Set up the vehicle actor --------------
        startheight = mavs_lib.GetSurfaceHeight(self.scene.scene,ctypes.c_float(self.start_pos[0]),ctypes.c_float(self.start_pos[1])) + 1.0
        self.vehicle.SetInitialPosition(self.start_pos[0],self.start_pos[1],startheight)
        if self.start_heading_loaded:
                self.vehicle.SetInitialHeading(self.start_heading)
        else:
            nextpos = self.waypoints.GetWaypoint(1)
            heading = math.atan2(nextpos[1]-self.start_pos[1],nextpos[0]-self.start_pos[0])
            self.vehicle.SetInitialHeading(heading)
        if (self.dusty):
            self.env.AddDustToActor(0)
    def UnloadScene(self):
        """Free the pointer to the Embree scene and clear all memory."""
        self.scene.DeleteCurrentScene()
        self.env.DeleteEnvironment()
        self.env = MavsEnvironment()
        self.scene = MavsEmbreeScene()
    def LoadNewScene(self,scenefile):
        """Load a new scene, specified by scenefile.

        This will unload the existing scene and load a new scene.

        Parameters:
        scenefile (string): Full path to the scene file to load.
        """
        self.UnloadScene()
        self.scenefile = scenefile
        self.LoadScene()
        self.env.load_block(self.env_block)
        self.vehicle.UnloadVehicle()
        self.vehicle = MavsRp3d()
        startheight = mavs_lib.GetSurfaceHeight(self.scene.scene,ctypes.c_float(self.start_pos[0]),ctypes.c_float(self.start_pos[1])) + 3.0
        self.vehicle.SetInitialPosition(self.start_pos[0],self.start_pos[1],startheight)
        if self.start_heading_loaded:
                self.vehicle.SetInitialHeading(self.start_heading)
        else:
            nextpos = self.waypoints.GetWaypoint(1)
            heading = math.atan2(nextpos[1]-self.start_pos[1],nextpos[0]-self.start_pos[0])
            self.vehicle.SetInitialHeading(heading)
        self.vehicle.Load(self.vehicle_file)
        #self.Update(0.0001)
        self.vehicle.Update(self.env,0.0, 0.0, 0.0, 0.0000000001)
        self.elapsed_time = 0.0
    def LoadNewVehicle(self,veh_dyn_file):
        """Load a new vehicle, specified by veh_dyn_file.

        This will unload the existing vehicle and load a new vehicle.

        Parameters:
        veh_dyn_file (string): Full path to the rp3d vehicle file to load.
        """
        self.vehicle.UnloadVehicle()
        self.vehicle = MavsRp3d()
        self.vehicle_file = veh_dyn_file
        startheight = mavs_lib.GetSurfaceHeight(self.scene.scene,ctypes.c_float(self.start_pos[0]),ctypes.c_float(self.start_pos[1])) + 3.0
        self.vehicle.SetInitialPosition(self.start_pos[0],self.start_pos[1],startheight)
        if self.start_heading_loaded:
                self.vehicle.SetInitialHeading(self.start_heading)
        else:
            nextpos = self.waypoints.GetWaypoint(1)
            heading = math.atan2(nextpos[1]-self.start_pos[1],nextpos[0]-self.start_pos[0])
            self.vehicle.SetInitialHeading(heading)
        self.vehicle.Load(self.vehicle_file)
        self.LoadNewScene(self.scenefile)
        #self.Update(0.0001)
        self.vehicle.Update(self.env,0.0, 0.0, 0.0, 0.0000000001)
        self.elapsed_time = 0.0
    def LoadNewWaypoints(self,wp_file):
        """Load a new waypoint file, specified by wp_file.

        This will unload the existing waypoints and load a new set of waypoints.

        Parameters:
        wp_file (string): Full path to the waypoint file to load.
        """
        self.waypoints.UnloadWaypoints()
        self.posefile = wp_file
        self.waypoints.Load(wp_file)
        self.LoadNewVehicle(self.vehicle_file)
        self.controller.SetDesiredPath(self.waypoints.GetWaypoints2D())
    def Load(self,fname):
        """Load a simulation input file.

        Example input files are in mavs/data/sims/sensor_sims

        Parameters:
        fname (string): Full path the MAVS json input file.
        """
        with open(fname,'r') as fin:
            txtdata = fin.read()
        data = json.loads(txtdata)
        if "Scene" in data:
            if "Input File" in data["Scene"]:
                in_scene_file = data["Scene"]["Input File"]
                self.scenefile = mavs_data_path+'/'+in_scene_file
            if "Origin" in data["Scene"]:
                self.origin = data["Scene"]["Origin"]
            if "Time Zone" in data["Scene"]:
                self.time_zone = data["Scene"]["Time Zone"]
        if "Environment" in data:
            self.env_block = data["Environment"]
            self.env.load_block(data["Environment"])
        if "Poses" in data:
            if "Pose File" in data["Poses"]:
                self.posefile = mavs_data_path+'/'+data["Poses"]["Pose File"]
            if "Pose Type" in data["Poses"]:
                self.posetype = data["Poses"]["Pose Type"]
        else:
            print("WARNING: NO POSE FILE GIVEN, SIMULATION MAY CRASH!!!")
        #---- Load the waypoints  -----
        self.waypoints.Load(self.posefile)
        # load the starting pose
        if "Starting Pose" in data:
            if "Position" in data["Starting Pose"]:
                self.start_pos[0] = data["Starting Pose"]["Position"][0]
                self.start_pos[1] = data["Starting Pose"]["Position"][1]
                self.start_pos[2] = data["Starting Pose"]["Position"][2]
            if "Heading" in data["Starting Pose"]:
                self.start_heading = data["Starting Pose"]["Heading"]
                self.start_heading_loaded = True
        else:
            self.start_pos = self.waypoints.GetWaypoint(0)

        if "Controller" in data:
            if "Steering Coefficient" in data["Controller"]:
                self.controller.steering_coeff = data["Controller"]["Steering Coefficient"]
            if "Max Steering Angle" in data["Controller"]:
                self.controller.max_steering_angle = data["Controller"]["Max Steering Angle"]
            if "Max Speed" in data["Controller"]:
                self.controller.desired_speed = data["Controller"]["Max Speed"]
            if "Wheelbase" in data["Controller"]:
                self.controller.wheelbase = data["Controller"]["Wheelbase"]
        if "Vehicle" in data:
            if "Input File" in data["Vehicle"]:
                self.vehicle_file = mavs_data_path+'/'+data["Vehicle"]["Input File"]
            if "Dusty" in data["Vehicle"]:
                self.dusty = data["Vehicle"]["Dusty"]
        del self.sensors[:]
        for item in data["Sensors"]:
            type = item["Type"]
            if (type=='camera'):
                new_sens = MavsCamera()
                new_sens.Model(item["Model"])
                if 'Gamma' in item:
                    new_sens.gamma = item['Gamma']
                if 'Gain' in item:
                    new_sens.gain = item['Gain']
                if 'Anti-Aliasing' in item:
                    new_sens.aa_fac = item['Anti-Aliasing']
                if 'Render Shadows' in item:
                    new_sens.render_shadows = item['Render Shadows']
                if 'Raindrops on Lens' in item:
                    new_sens.raindrop_lens = item['Raindrops on Lens']
            elif (type=='lidar'):
                new_sens = MavsLidar(item["Model"])
            elif (type=='rtk'):
                new_sens = MavsRtk()
                if 'Error' in item:
                    new_sens.SetError(item['Error'])
                if 'Dropout Rate' in item:
                    new_sens.SetDropoutRate(item['Dropout Rate'])
                if 'Warmup Time' in item:
                    new_sens.SetWarmupTime(item['Warmup Time'])
            new_sens.load_block(item)
            self.sensors.append(new_sens)
            self.sensor_times.append(0.0)
            
            #---- Set up the controller --------
            self.controller.SetDesiredPath(self.waypoints.GetWaypoints2D())
            self.controller.SetDesiredSpeed(self.controller.desired_speed) 
            self.controller.SetSteeringScale(self.controller.steering_coeff)
            self.controller.SetWheelbase(self.controller.wheelbase)
            self.controller.SetMaxSteerAngle(self.controller.max_steering_angle)
            #---- Load the scene -------------------------------
            self.LoadScene()
            #---- Load the vehicle  and place it-------
            self.vehicle.Load(self.vehicle_file)
            startheight = mavs_lib.GetSurfaceHeight(self.scene.scene,ctypes.c_float(self.start_pos[0]),ctypes.c_float(self.start_pos[1])) + 1.0
            self.vehicle.SetInitialPosition(self.start_pos[0],self.start_pos[1],startheight)
            if self.start_heading_loaded:
                self.vehicle.SetInitialHeading(self.start_heading)
            else:
                nextpos = self.waypoints.GetWaypoint(1)
                heading = math.atan2(nextpos[1]-self.start_pos[1],nextpos[0]-self.start_pos[0])
                self.vehicle.SetInitialHeading(heading)
            self.vehicle.Update(self.env,0.0, 0.0, 0.0, 0.0000000001)        
    #---------  Done loading simulation --------------------------------------------------------------------------------#

    def TurnOnSensor(self,sensor_id,display=False,save_raw=False,labeling=False):
        """Turn on a particular sensor.

        Parameters:
        sensor_id (int): The ID number of the sensor to turn on.
        display (bool): True to display the sensor output to screen.
        save_raw (bool): True to save raw sensor data to disk.
        labeling (bool): True to save labeled data to disk.
        """
        self.sensors[sensor_id].is_active = True
        self.sensors[sensor_id].save_raw = save_raw
        if display:
            self.TurnOnSensorDisplay(sensor_id)
        if labeling:
            self.TurnOnSensorLabeling(sensor_id)
    def TurnOffSensor(self,sensor_id):
        """Turn off a sensor.

        Parameters:
        sensor_id (int): The ID number of the sensor to turn off.
        """
        self.sensors[sensor_id].is_active = False
    def TurnOnSensorLabeling(self, sensor_id):
        """Turn on labeling for a sensor.

        Parameters:
        sensor_id (int): The ID number of the sensor to turn on labeling.
        """
        self.sensors[sensor_id].save_labeled = True
        self.scene.TurnOnLabeling()
    def TurnOffSensorLabeling(self,sensor_id):
        """Turn off labeling for a sensor.

        Parameters:
        sensor_id (int): The ID number of the sensor to turn off labeling.
        """
        self.sensors[sensor_id].save_labeled = False
    def TurnOnSensorDisplay(self,sensor_id):
        """Turn on display for a sensor.

        Parameters:
        sensor_id (int): The ID number of the sensor to turn on display.
        """
        self.sensors[sensor_id].display = True
    def TurnOffSensorDisplay(self,sensor_id):
        """Turn off display for a sensor.

        Parameters:
        sensor_id (int): The ID number of the sensor to turn off display.
        """
        self.sensors[sensor_id].display = False
    def Update(self, dt,throttle=0.0,steering=0.0,braking=1.0,update_actor=False):
        """Update the simulation.

        Parameters:
        dt (float): The time step in seconds.
        throttle (float): Throttle from 0.0-1.0
        steering (float): Steering from -1.0 to 1.0.
        braking (float): Braking from 0.0 to 1.0
        update_actor (bool): If True, update the actor position.
        """
        t0 = time.time()
        if (update_actor):
            self.time_to_update_actor = True
        #if self.time_to_update_actor:
        #    self.env.SetActorPosition(self.veh_actor_num,self.vehicle.GetPosition(),self.vehicle.GetOrientation())
        self.env.AdvanceTime(dt)
        t1 = time.time()
        self.env_time = (t1-t0)/dt
        if (self.elapsed_time>self.wait_time):
            current_pos = self.vehicle.GetPosition()
            self.controller.SetCurrentState(current_pos[0],current_pos[1],self.vehicle.GetSpeed(),self.vehicle.GetHeading())
            current_driving_command = MavsDrivingCommand()
            current_driving_command.throttle = throttle
            current_driving_command.steering = steering
            current_driving_command.braking = braking
            if (not self.free_driving):
                current_driving_command = self.controller.GetDrivingCommand(dt)
            self.vehicle.Update(self.env,current_driving_command.throttle, current_driving_command.steering,current_driving_command.braking, dt)
        else:
            self.vehicle.Update(self.env, 0.0, 0.0, 0.0, dt)
        t2 = time.time()
        self.veh_time = (t2-t1)/dt
        snum=0
        self.time_to_update_actor = False
        for s in self.sensors:
            s.elapsed_since_last = s.elapsed_since_last + dt
            if s.elapsed_since_last > (1.0/s.update_rate):
                self.time_to_update_actor = True
                if (s.is_active):
                    st0 = time.time()
                    if (s.type=='camera'):
                        s.SetEnvironmentProperties(self.env.obj)
                    if (s.type=='lidar'):
                        vehvel = self.vehicle.GetVelocity()
                        s.SetVelocity(vehvel[0],vehvel[1],vehvel[2] )
                    s.SetPose(self.vehicle.GetPosition(), self.vehicle.GetOrientation())
                    s.Update(self.env,dt)
                    base_string = self.save_location+'/%05d'%(100*self.elapsed_time)+'_'+s.name
                    if s.display:
                        if (s.type=='lidar'):
                            s.DisplayPerspective()
                        else:
                            s.Display()
                    if (s.save_raw):
                        if (s.type=='lidar'):
                            s.SaveColorizedPointCloud((base_string+'.pts'))
                        elif(s.type=='camera'):
                            s.SaveCameraImage((base_string+'.bmp'))
                    if s.save_labeled:
                        s.SaveAnnotation(self.env,base_string+'_labeled')
                    s.elapsed_since_last = 0.0
                    st1 = time.time()
                    self.sensor_times[snum] = (st1-st0)*s.update_rate
            snum = snum+1
        self.elapsed_time = self.elapsed_time + dt
#--------- Done with update ----------------------------------------------#
    def GetSensorDict(self):
        """Return a dictionary entry for the sensor block.

        This is for saving the current simulation config to a file.

        Returns:
        outlist (dictionary): Dictionary with all sensor config information.
        """
        outlist = []
        for s in self.sensors:
            dict = s.GetDict()
            if s.type=='camera':
                dict['Gamma']=s.gamma
                dict['Gain']=s.gain
                dict['Anti-Aliasing']=s.aa_fac
                dict['Render Shadows']=s.render_shadows
                dict['Raindrops on Lens']=s.raindrop_lens
            outlist.append(dict)
        return outlist
    def WriteToJson(self,fname):
        """Write the current simulation configuration to a json file.

        Parameters:
        fname (string): The output JSON file name.
        """
        f = open(fname,'w')
        f.write(json.dumps({
            'Scene':{
                'Input File':remove_prefix(self.scenefile,(mavs_data_path+'/')),
                'Origin':self.origin,
                'Time Zone': self.time_zone
            },
            'Poses':{
                'Pose File':remove_prefix(self.posefile,(mavs_data_path+'/')),
                'Pose Type':self.posetype
            },
            'Controller':{
                'Steering Coefficient':self.controller.steering_coeff,
                'Max Steering Angle':self.controller.max_steering_angle,
                'Max Speed':self.controller.desired_speed,
                'Wheelbase':self.controller.wheelbase
            },
            'Vehicle':{
                'Input File':remove_prefix(self.vehicle_file,(mavs_data_path+'/')),
                #'Animation File':remove_prefix(self.vehicle_actor_file,(mavs_data_path+'/'))
            },
            'Environment':{
                'Month': self.env.month,
                'Day': self.env.day,
                'Year': self.env.year,
                'Hour': self.env.hour,
                'Minute': self.env.minute,
                'Second': self.env.second,
                'Turbidity': self.env.turbidity,
                'Rain Rate': self.env.rain_rate,
                'Fog': self.env.fog,
                'Cloud Cover': self.env.cloud_cover,
                'Local Albedo': self.env.albedo,
                'Snow Rate': self.env.snow_rate
            },
            'Sensors': self.GetSensorDict()
            },
            sort_keys=False,indent=2*' ') )
        f.close()
