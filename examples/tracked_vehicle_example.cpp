// c++ includes
#include <iostream>
// project includes
#include "vehicles/tracked/tracked_vehicle.h"
#include "vehicles/tracked/tracked_render.h"

int main(int argc, char** argv) {

    std::string sim_input_file = std::string(argv[1]);

    mavs::vehicle::tracked::TrackedVehicle sim(sim_input_file);

    mavs::vehicle::tracked::TrackedRender render(&sim);

    while (render.DisplayOpen()) {
        
        mavs::vehicle::tracked::TrackSpeeds cmd = render.GetKeyboardDrivingCommand();

        sim.Step(sim.GetSimulationDt(), cmd);

        render.Update();

    }

}
