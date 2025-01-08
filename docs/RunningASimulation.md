# Running A Simulation
There are two primary executables built for MAVS - *mavs_sim* and *mavs_sensor_sim*. Closed loop autonomy simulations should be run with *mavs_sim*, while sensor analysis without integrated autonomy should be run with *mavs_sim*. 

This page discusses the details of launching jobs from the command line. However, there is a [GUI available](./Gui/RunningMavsGUI.md)

## On Ubunutu 16.04

## On Windows

## On MSU HPC2 Resources
See [Building Mavs](./MavsBuildInstructions.md) for info on how to configure the software environment for running MAVS. MSU HPC2 resources can be accessed using PuTTY
1. Launch PuTTY
2. HostName is *yourusername*@*computername*.hpc.msstate.edu where *yourusername* is the username assigned to you by HPC2 and *computername* might be viking1, shadow-login, or another MSU HPC2 resource. 
3. Make sure you enable x11 forwarding and X-Win32 is running on your local machine if you plan on running interactive simulations.

### Shadow
Shadow is a high-performance computer (HPC) that uses the PBS batch scheduling system for parallel applications. It is possible to run simulations interactively or in batch mode using PBS. 

Note that the instructions for running simulations on Talon are similar.

#### Running a simulation interactively
From the terminal, type 
```
$qsub -N mavs -l nodes=1:ppn=20 -q q200p48h -l walltime=1:00:00 -r n -V -I -X
```

The *-I* option indicates that this will be an interactive PBS session, while the *-X* option enables X11 forwarding. After executing this command, you may have to wait for some time for the interactive PBS job to start, depending on the current usage level of Shadow.

#### Submitting a simulation to the batch queue

### Viking1

#### Running the GUI on Viking1

#### Running the forester example on Viking1
1. $cd /scratch
2. If you don't have a directroy on scratch: $mkdir myusername
3. $cd /myusername
4. mpirun -np 6 /home/cgoodin/cavs_shared/bin/examples/forester_example /home/cgoodin/cavs_shared/data/sims/forester_sim.json
