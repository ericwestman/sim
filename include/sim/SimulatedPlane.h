/* SimulatedPlane
This class contains the data structures and functions required to perform plane simulation.
Each object instantiated will be considered one "plane" in the system. */

#ifndef SIMULATED_PLANE_H
#define SIMULATED_PLANE_H

#define MAXIMUM_TURNING_ANGLE 22.5 //degrees

#include "sim/standardDefs.h"
#include "sim/Command.h"
#include "sim/TelemetryUpdate.h"
#include "sim/CreateSimulatedPlane.h"

namespace sim {
	class SimulatedPlane {
        private:
            //last received command info
            sim::Command lastCommand;
            
            //current information (used mostly in update)
            long long int planeID;
            
            sim::waypoint currentLocation;
            sim::waypoint currentDest;
            
            //these two values are sent in the telemetry update
            double groundSpeed;
            double bearing;
            
            //this is stored as part of the UAV info
            double actualBearing;
            
            long long int currentWaypointIndex;
            double distanceToDestination;
            
            //index of sent message
            int updateIndex;
            
        public:
            //dummy constructor, shouldn't really be used
            SimulatedPlane();
            
            //primary constructor
            SimulatedPlane(long long int planeID, sim::CreateSimulatedPlane::Request &requestFromUser);
        
            //function for handling a command from the coordinator
            bool handleNewCommand(sim::Command newCommand);
            
            //periodic function for filling in a new telemetry update for this UAV
            bool fillTelemetryUpdate(sim::TelemetryUpdate *tUpdate);
	};
}

#endif