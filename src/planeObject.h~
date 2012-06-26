/*PlaneObject

*/

#ifndef PLANE_OBJECT_H
#define PLANE_OBJECT_H

#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/standardDefs.h"

namespace AU_UAV_ROS {

	class PlaneObject {

	public:
		/*
		Explicit value constructor: Takes a collision radius and a telemetry update and creates a new PlaneObject.
		*/
		PlaneObject(double cRadius, const AU_UAV_ROS::TelemetryUpdate &msg);

		/*mutator functions*/
		void setID(int id);
		void setPreviousLoc(double lat, double lon, double alt);
		void setCurrentLoc(double lat, double lon, double alt);
		void setTargetBearing(double tBearing);		/* set bearing to destination */
		void setCurrentBearing(double cBearing); 	/* set current bearing in the air */
		void setSpeed(double speed);
		void setDestination(const AU_UAV_ROS::waypoint &destination);
		void updateTime(void); //changed function name from "update" to "updateTime"

		/* Update the plane's data members with the information contained within the telemetry update */
		void update(const AU_UAV_ROS::TelemetryUpdate &msg);

		/* Accessor functions: Allow the client to access the plane's id, altitude, bearing, spped, etc. */
		int getID(void) const;
		AU_UAV_ROS::coordinate getPreviousLoc(void) const;
		AU_UAV_ROS::coordinate getCurrentLoc(void) const;
		double getTargetBearing(void) const;		/* get target bearing */
		double getCurrentBearing(void) const;		/* get current bearing in the air */
		double getSpeed(void) const;
		double getLastUpdateTime(void) const;
		AU_UAV_ROS::waypoint getDestination(void) const;

		/* Find distance between this plane and another plane */
		double findDistance(const PlaneObject& pobj) const;
		/* Find distance between this plane and another plane's latitude/longitude */
		double findDistance(double lat, double lon) const;

		/* Find Cartesian angle between this plane and another plane */
		double findAngle(const PlaneObject& pobj) const;
		/* Find Cartesian angle between this plane and another plane's latitude/longitude */
		double findAngle(double lat, double lon) const;

		/* Overloaded equality operator */
		PlaneObject& operator=(const PlaneObject& pobj);

		/* Returns true if a plane object is within the cRadius meters of this plane object, false otherwise */
		bool isColliding(const PlaneObject& planeObj) const;

	private:
		/* Private data members */
		int id;
		double collisionRadius;
		double targetBearing;		/* get bearing to destination */
		double currentBearing;		/* get current bearing in the air */
		double speed;
		double lastUpdateTime;
		AU_UAV_ROS::coordinate previousLoc;	/*used to calculate currentBearing*/
		AU_UAV_ROS::coordinate currentLoc;
		AU_UAV_ROS::waypoint destination;
		
	};
};


#endif
