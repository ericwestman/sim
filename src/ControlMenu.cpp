/* Controls Menu
This will be the primary UI for now just to control the simulator and coordinator. */

//standard headers
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <ctype.h>

//ROS headers
#include "ros/ros.h"
#include "ros/package.h"
#include "sim/standardDefs.h"
#include "sim/CreateSimulatedPlane.h"
#include "sim/DeleteSimulatedPlane.h"
#include "sim/GoToWaypoint.h"
#include "sim/LoadPath.h"
#include "sim/LoadCourse.h"
#include "sim/SaveFlightData.h"

//services to the simulator
ros::ServiceClient createSimulatedPlaneClient;
ros::ServiceClient deleteSimulatedPlaneClient;

//services to the coordinator
ros::ServiceClient goToWaypointClient;
ros::ServiceClient loadPathClient;
ros::ServiceClient loadCourseClient;

//services to the KMLCreator
ros::ServiceClient saveFlightDataClient;

/* Takes a filename, parses it to determine how many UAVs there are, and creates them as needed. */
bool createCourseUAVs(std::string filename) {
	//open our file
	FILE *fp;
	fp = fopen((ros::package::getPath("sim")+"/courses/"+filename).c_str(), "r");
	
	//check for a good file open
	if(fp != NULL) {
		char buffer[256];
		
		std::map<int, bool> isFirstPoint;
		
		while(fgets(buffer, sizeof(buffer), fp)) {
			if(buffer[0] == '#' || isBlankLine(buffer)) {
				//this line is a comment
				continue;
			}
			else {
				//set some invalid defaults
				int planeID = -1;
				struct sim::waypoint temp;
				temp.latitude = temp.longitude = temp.altitude = -1000;
				
				//parse the string
				sscanf(buffer, "%d %lf %lf %lf\n", &planeID, &temp.latitude, &temp.longitude, &temp.altitude);
				
				//check for the invalid defaults
				if(planeID == -1 || temp.latitude == -1000 || temp.longitude == -1000 || temp.altitude == -1000) {
					//this means we have a bad file somehow
					ROS_ERROR("Bad file parse");
					return false;
				}
				
				//check our map for an entry, if we dont have one then this is the first time
				//that this plane ID has been referenced so it's true
				if(isFirstPoint.find(planeID) == isFirstPoint.end()) {
					isFirstPoint[planeID] = true;
					
					//this is the first time we've seen this ID in the file, attempt to create it
					sim::CreateSimulatedPlane srv;
					srv.request.startingLatitude = temp.latitude;
					srv.request.startingLongitude = temp.longitude;
					srv.request.startingAltitude = temp.altitude;
					srv.request.startingBearing = 0;
					srv.request.requestedID = planeID;
				
					//send the service request
					printf("\nRequesting to create new plane with ID #%d...\n", planeID);
					if(createSimulatedPlaneClient.call(srv))
						printf("New plane with ID #%d has been created!\n", srv.response.planeID);
					else
						ROS_ERROR("Did not receive a response from simulator");
				}
				
				//only clear the queue with the first point
				if(isFirstPoint[planeID]) isFirstPoint[planeID] = false;
			}
		}
	}
	else {
		ROS_ERROR("Invalid filename or location: %s", filename.c_str());
		return false;
	}
}

/* This is a menu for simulator related options. Specifically, it will be used for UAV creation and deletion. */
void simulatorMenu()
{
	//simple menu setup
	int choice = 0;
	
	//loop inside this menu until we go back
	while(choice != 3)
	{
		choice = 0;
		printf("\nSimulator Menu:\n");
		printf("1-Add plane\n");
		printf("2-Delete plane\n");
		printf("3-Back\n");
		printf("Choice:");
		scanf("%d", &choice);
		system("clear");
		
		switch(choice) {
			//Add a simulated plane
			case 1: {
				//values we need to get started
				double latitude, longitude, altitude;
				double bearing;
				
				//Get user input data
				printf("\nEnter starting latitude, longitude, and altitude (format \"1 2 3\"):");
				scanf("%lf %lf %lf", &latitude, &longitude, &altitude);
				printf("\nEnter starting bearing:");
				scanf("%lf", &bearing);
				
				//construct the service request
				sim::CreateSimulatedPlane srv;
				srv.request.startingLatitude = latitude;
				srv.request.startingLongitude = longitude;
				srv.request.startingAltitude = altitude;
				srv.request.startingBearing = bearing;
				srv.request.requestedID = -1;
				
				//send the service request
				printf("\nRequesting to create new plane...\n");
				if(createSimulatedPlaneClient.call(srv))
					printf("New plane with ID #%d has been created!\n", srv.response.planeID);
				else
					ROS_ERROR("Did not receive a response from simulator");
				
				break;
			}
			
			//Delete a simulated plane
			case 2: {
				//only value we need
				int planeID;
				
				//get user input
				printf("\nEnter the plane ID to delete:");
				scanf("%d", &planeID);
				
				//construct the service request
				sim::DeleteSimulatedPlane srv;
				srv.request.planeID = planeID;
				
				//send the request
				printf("\nRequesting to delete plane...\n");
				if(deleteSimulatedPlaneClient.call(srv))
					printf("Plane with ID #%d has been deleted!\n", planeID);
				else
					ROS_ERROR("Did not receive a response from simulator");
				
				break;
			}
			
			case 3: {
				//nothing to do but leave
				break;
			}
			
			default: {
				printf("Invalid choice.\n");
				break;
			}
		}
	}
}

/* This menu is used for when the user wishes to perform some actions on the path planning aspect of the
coordinator.  This includes simply sending a UAV to a waypoint and loading paths or courses for the UAV. */
void pathMenu()
{
	//simple menu setup
	int choice = 0;
	
	//loop until the user asks to go back
	while(choice != 4) {
		printf("\nPath Planning Menu:\n");
		printf("1-Go to waypoint\n");
		printf("2-Load path\n");
		printf("3-Load course\n");
		printf("4-Back\n");
		printf("Choice:");
		scanf("%d", &choice);
		system("clear");
		
		switch(choice) {
			//go to a specified waypoint, this will clear the entire queue for the UAV
			case 1: {
				//create service
				sim::GoToWaypoint srv;
				
				//get user inputs
				printf("\nEnter the plane ID:");
				scanf("%d", &(srv.request.planeID));
				
				printf("\nEnter latitude, longitude, and altitude (format \"1 2 3\"):");
				scanf("%lf %lf %lf", &(srv.request.latitude), &(srv.request.longitude), &(srv.request.altitude));
				
				//we want to wipe the current queue and go straight to that waypoint
				srv.request.isAvoidanceManeuver = false;
				srv.request.isNewQueue = true;
				
				//call the service
				printf("Sending go to waypoint...\n");
				if(goToWaypointClient.call(srv))
					printf("Waypoint sent successfully!\n");
				else
					ROS_ERROR("Did not receive response from coordinator");
				
				break;
			}
			
			//load a path for a plane
			case 2: {
				//create service
				sim::LoadPath srv;
				
				//get user inputs
				char filename[256];
				printf("\nEnter the filename:");
				scanf("%s", filename);
				
				int planeID;
				printf("\nEnter the plane ID:");
				scanf("%d", &planeID);
				
				srv.request.planeID = planeID;
				srv.request.filename = filename;
				
				if(loadPathClient.call(srv))
					printf("Path loaded successfully!\n");
				else
					ROS_ERROR("Error loading path");
				break;
			}
			
			//load a course for our UAVs
			case 3: {
				//create the service
				sim::LoadCourse srv;
				
				//get the file input
				char filename[256];
				char createNewPlanes[256] = "blah";
				printf("\nEnter the filename:");
				scanf("%s", filename);
				
				//get whether we should create planes or not
				while(!isValidYesNo(createNewPlanes[0])) {
					printf("\nDo you want to automatically create any non-existent planes (y/n)?");
					scanf("%s", createNewPlanes);
				}
				
				//check if we need to create some simulated UAVs
				if(tolower(createNewPlanes[0]) == 'y')
					createCourseUAVs(filename);
				
				srv.request.filename = filename;
				
				if(loadCourseClient.call(srv))
					printf("Course loaded successfully!\n");
				else
					ROS_ERROR("Error loading course");
				break;
			}
			
			//we don't have to do anything for the go back case
			case 4: {
				break;
			}
			
			//someone failed to read or type
			default: {
				printf("Invalid choice.\n");
				break;
			}
		}
	}
}

/* This just sets up all the ROS stuff and serves as the primary top level menu. */
int main(int argc, char **argv)
{
	//Standard ROS startup
	ros::init(argc, argv, "ControlsMenu");
	ros::NodeHandle n;
	
	//setup client services
	createSimulatedPlaneClient = n.serviceClient<sim::CreateSimulatedPlane>("create_simulated_plane");
	deleteSimulatedPlaneClient = n.serviceClient<sim::DeleteSimulatedPlane>("delete_simulated_plane");
	goToWaypointClient = n.serviceClient<sim::GoToWaypoint>("go_to_waypoint");
	loadPathClient = n.serviceClient<sim::LoadPath>("load_path");
	loadCourseClient = n.serviceClient<sim::LoadCourse>("load_course");
	saveFlightDataClient = n.serviceClient<sim::SaveFlightData>("save_flight_data");
	
	//set up the menu
	int choice = 0;
	
	//loop until the user asks to quit
	while(choice != 4) {
		printf("\nStandard Controller Menu:\n");
		printf("1-Simulator Controls\n");
		printf("2-Path Planning\n");
		printf("3-Save all data\n");
		printf("4-Exit Program\n");
		printf("Choice:");
		scanf("%d", &choice);
		system("clear");
		
		switch(choice) {
			case 1: {
				simulatorMenu();
				break;
			}
			case 2: {
				pathMenu();
				break;
			}
			case 3: {
				char filename[256];
				printf("\nEnter the filename to save to:");
				scanf("%s", filename);
				
				sim::SaveFlightData srv;
				srv.request.filename = filename;
				
				if(saveFlightDataClient.call(srv))
					printf("Flight data saved successfully!\n");
				else
					ROS_ERROR("Error saving flight data");
				
				break;
			}
			case 4: {
				//nothing to do but leave
				break;
			}
			default: {
				printf("Invalid choice.\n");
				break;
			}
		}
	}
	
	return 0;
}