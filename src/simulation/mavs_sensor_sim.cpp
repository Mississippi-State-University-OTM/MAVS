/*
MIT License

Copyright (c) 2024 Mississippi State University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <iostream>
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <stdlib.h>

//mavs sensor simulation
#include <simulation/sensor_sim.h>

int main(int argc, char *argv[]){
  int myid = 0;
#ifdef USE_MPI  
  int ierr = MPI_Init(&argc, &argv);
  MPI_Comm_rank(MPI_COMM_WORLD, &myid);
#endif
  
  if (argc<2){
    if(myid==0)std::cerr<<"No input file listed on command line arg 1"<<
		 std::endl;
    exit(1);
  }
  
  mavs::SensorSimulation sim;
  std::string infile(argv[1]);
  sim.Load(infile);

  if (!sim.IsValid()){
    if (myid==0)std::cerr<<"The simulation is not valid, exiting"<<std::endl;
    exit(1);
  }
#ifdef USE_MPI  
  MPI_Barrier(MPI_COMM_WORLD);
#endif
  sim.Run();
#ifdef USE_MPI  
  MPI_Finalize();
#endif  
  return 0;
}
