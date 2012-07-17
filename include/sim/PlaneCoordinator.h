/*
PlaneCoordinator
Class responsible for storing:
A) The latest update from the plane
B) The waypoints to go to (aka path)
C) Any collision avoidance waypoints
*/

#ifndef PLANE_COORDINATOR_H
#define PLANE_COORDINATOR_H

//normal headers
#include <stdio.h>
#include <list>
#include <string>

//ROS headers
#include "sim/standardDefs.h"
#include "sim/TelemetryUpdate.h"
#include "sim/Command.h"

namespace sim
{
	class PlaneCoordinator
	{
	private:
		//the most recent update received from a plane
		sim::TelemetryUpdate latestUpdate;
		
		//two queues related to where the plane should go next, note that avoidance takes priority
		std::list<struct sim::waypoint> normalPath;
		std::list<struct sim::waypoint> avoidancePath;
		
		//index of the next command to send, starts at 0
		int commandIndex;
		
	public:
		//this is set if the coordinator has allocated this UAV to somewhere
		bool isActive;
		
		//constructors
		PlaneCoordinator();
		
		//command related functions
		bool goToPoint(struct sim::waypoint receivedPoint, bool isAvoidanceManeuver, bool isNewQueue);
		struct sim::waypoint getWaypointOfQueue(bool isAvoidanceQueue, int position);
		sim::Command getPriorityCommand();
		
		//update related functions
		bool handleNewUpdate(sim::TelemetryUpdate update, sim::Command *newCommand);
	};
}

#endif