/*
 * ctrl_system.cpp
 *
 *  Created on: Jun 9, 2014
 *      Author: franz
 */

#include <ctrl_system.h>
#include <math.h>
#include <stdio.h>
#include <string.h>


int ControlSystem::getCurrentPosition(int which)
{
	return currentPos[which];
}

void ControlSystem::setCurrentPosition(int which, int val)
{
	currentPos[which] = val;
}

void ControlSystem::update2DMap(float objectDistance)
{
	/* These values are identified by measuring - see master thesis */
	#define PIXEL_FACTOR_ZERO 0.403061224f
	#define PIXEL_FACTOR_ZERO_OFFSET 195
	#define PIXEL_GROW_FACTOR 0.00104393f
	#define REFERENCE_DISTANCE 94.5

	float distanceB = 0;

	/* Calc the distanceB - see master thesis */
	distanceB = sqrt((objectDistance*objectDistance)-(height*height));
	printf("distanceB %f cm\n", distanceB);

	/* Calculate the distance in cm from the middle of the picture */
	x_2D_map = PIXEL_FACTOR_ZERO-((currentPos[Y_POS]-PIXEL_FACTOR_ZERO_OFFSET)*PIXEL_GROW_FACTOR);
	x_2D_map = x_2D_map*(currentPos[X_POS]-(pixelWidth/2));

	/* OK, now we have the X value, now we can also calculate the Y value */
	/* B is the hypothenuse */
	y_2D_map = sqrt((distanceB*distanceB)-(x_2D_map*x_2D_map));

	fprintf( stderr,"2D: X %f  Y %f\n", x_2D_map, y_2D_map);

	return;
}

void ControlSystem::hexaWalk(HEXA_MOVE_t DIRECTION, char steps)
{
	char buf[5];

	if(hexaWrite == NULL)
	{
		return;
	}

	if(DIRECTION < HEXA_WALK_END)
	{
		switch(DIRECTION)
		{
			case WALK_BACK:
				buf[0] = 'b';
				break;
			case WALK_FORWARD:
				buf[0] = 'f';
				break;
			case WALK_LEFT:
				buf[0] = 'l';
				break;
			case WALK_RIGHT:
				buf[0] = 'r';
				break;
			default:
				break;
		}
		buf[1] = steps;
		buf[2] = '\n';
		hexaWrite(buf, 3, hexaWriteUserData);
	}
}

void ControlSystem::hexaControl(int setPointX, int setPointY)
{
	int distance = 0;

	/* calc distance */
	distance = x_2D_map - setPointX;

	if(distance < -10)
	{
		hexaWalk(WALK_LEFT, 1);
	}
	else if (distance > 10)
	{
		hexaWalk(WALK_RIGHT, 1);
	}

	distance = y_2D_map - setPointY;
	if(distance < -10)
	{
		hexaWalk(WALK_BACK, 1);
	}
	else if (distance > 10)
	{
		hexaWalk(WALK_FORWARD, 1);
	}
}

