#include "robo.h"
#include <math.h>

#define PI 3.14159265

// Given the initial vector of polygons (obstacles), grow each one
// individually (all the way around) by some pretermined region
// in order to determine the workspace of the robot without collisions
// Inputs:
//   obstacles:  Original polygons consisting of the coordinates
//                               which make up their vertices
// Outputs:
//   obstacles:  The same polygons with coordinates added to each.
//                       polygon accounting for their growth regions

int robo::init(char *device)
{


	if( biscInit(device) != BISC_SUCCESS ){
		fprintf(stderr, "could not connect to the create. \n");
		return 1;
	}

	//full mode
	biscChangeMode(BISC_MODE_FULL);
	//drive forward 1 meter
	//biscDriveDistance(BISC_DRIVE_FORWARD_HALF_SPEED, 300, 1.0 * 1000);

	//biscDisconnect();

	return 0;
}

void robo::moveNextCoord(vector<float> dest) {
	//turn
	biscWaitAngle( (int) dest[1]);
	//move
	biscDriveDistanceStraight(0.1, dest[0] * 1000);
	//turn back
	biscWaitAngle( -(int) dest[1]);

}


