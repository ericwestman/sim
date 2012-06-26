/*
Implementation of standardFuncs.h.  For information on how to use these functions, visit standardFuncs.h.  Comments in this file
are related to implementation, not usage.
*/

#include <cmath>
#include <stdlib.h>
#include "AU_UAV_ROS/standardFuncs.h"


/* Convert Cardinal direction to an angle in the Cartesian plane */
double toCartesian(double UAVBearing){
	UAVBearing = manipulateAngle(UAVBearing); /* get angle on the interval [-180, 180] */

	if (UAVBearing < 180.0 && UAVBearing >= 0) /* UAV bearing is in the first or fourth quadrant */
		return 90 - UAVBearing;
	else if (UAVBearing < 0 && UAVBearing >= -90) /* UAV bearing is in the second quadrant */
		return -1*UAVBearing + 90;
	else if (UAVBearing < -90 && UAVBearing > -180.0) /* UAV bearing is in the third quadrant */
		return -1*(UAVBearing + 180.0) - 90;
	else if (UAVBearing == 180.0 || UAVBearing == -180.0)
		return -90;
	else
		return -999; /* should never happen in current setup */
}

/* Convert angle in the Cartesian plane to a Cardinal direction */
double toCardinal(double angle){
	angle = manipulateAngle(angle); /* get angle on the interval [-180, 180] */

	if (angle <= 90 && angle >= -90) /* angle is in the first or fourth quadrant */
		return 90 - angle;
	else if (angle >= 90 && angle <= 180.0) /* angle is in the second quadrant */
		return -1*angle + 90;
	else if (angle <= -90 && angle >= -180.0) /* angle is in third quadrant */
		return -180.0 + -1*(90 + angle);
	else 
		return -999; /* should never happen in current setup */ 
}

/* Modify the angle so that it remains on the interval [-180, 180] */
double manipulateAngle(double angle){
	while (angle > 180.0){
		/* decrease angle by one 360 degree cycle */
		angle-=360;
	}

	while (angle < -180.0){
		/* increase angle by one 360 degree cycle cycle */
		angle+=360;
	}

	return angle;
}

/* 
Returns the distance between two points of latitude and longitude in meters.  The first two parameters
are the latitude and longitude of the starting point, and the last two parameters are the latitude and
longitude of the ending point.
*/

double findDistance(double lat1, double long1, double lat2, double long2){
	
	/* Get difference in radians */
	double latDiff = (lat2 - lat1)*DELTA_LAT_TO_METERS;
	double lonDiff = (long2 - long1)*DELTA_LON_TO_METERS;

	/* Return result in meters */
	return sqrt(pow(latDiff, 2) + pow(lonDiff, 2));
}

/* 
Returns the Cartesian angle between two points of latitude and longitude in degrees.  The starting point is given
by lat1 and long1 (the first two parameters), and the final point is given by lat2 and long2 (the final two parameters).
The value returned is on the interval [-180, 180].
*/
double findAngle(double lat1, double long1, double lat2, double long2){
	
	/* Get difference in radians */
	double latDiff = (lat2 - lat1)*DELTA_LAT_TO_METERS;
	double lonDiff = (long2 - long1)*DELTA_LON_TO_METERS;

	/* Return result in degrees */
	return atan2(latDiff,lonDiff)*180.0/PI;
}

/* Returns the sign of the double*/
double findSign(double number){
	if (number > 0) return 1;
	if (number < 0) return -1;
	return 0;
}


/* Calculates the signed supplement of an angle*/
double calculateSupplement(double theta){
	return findSign(theta)*(PI-abs(theta));	
}


