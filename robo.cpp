#include "robo.h"
#include <math.h>

#define PI 3.14159265

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


	return 0;
}

void robo::moveNextCoord(vector<float> dest) {
	//turn
	biscWaitAngle( (int) dest[1]);
	//move; way to improve: see how long it takes for the robot to stop (distance-wise) during calibration
	//decrease the dest[0] by that much; same with angle
	biscDriveDistanceStraight(0.1, (int) dest[0] * 1000);
	//turn back
	biscWaitAngle( -(int) dest[1]); // do we want to turn back?

}

int robo::disconnect(){

	biscDisconnect();

	return 0;
}