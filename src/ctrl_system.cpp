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

void ControlSystem::update2DMap(float objectDistance, float xCorrectionVal, float yCorrectionVal)
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
	x_2D_map += xCorrectionVal;

	/* OK, now we have the X value, now we can also calculate the Y value */
	/* B is the hypothenuse */
	y_2D_map = sqrt((distanceB*distanceB)-(x_2D_map*x_2D_map));
	y_2D_map += yCorrectionVal;

	fprintf( stderr,"2D: X %f  Y %f  correctionX %f correctionY %f\n", x_2D_map, y_2D_map, xCorrectionVal, yCorrectionVal);
	return;
}

void ControlSystem::hexaWalk(HEXA_MOVE_t DIRECTION, char steps)
{
	char buf[5];

	if(hexaWrite == NULL)
	{
		return;
	}

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
		case ROTATE_RIGHT:
			buf[0] = 'y';
			break;
		case ROTATE_LEFT:
			buf[0] = 'x';
			break;
		default:
			break;
	}

	buf[1] = steps;
	buf[2] = '\n';
	hexaWrite(buf, 3, hexaWriteUserData);
}

void ControlSystem::hexaSetAngle(float angleRad)
{
	angle = angleRad;
}

void ControlSystem::hexaControl(int setPointX, int setPointY)
{
	static int pingPong = 0;
	int distance = 0;

	if(!pingPong)
	{
		pingPong = 1;
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
	}
	else
	{
		pingPong = 0;
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
}

void ControlSystem::hexaControlRotate(int setPointX, int setPointY)
{
	int distance = 0;

	float dirVectShould[2];
	float dirVectIs[2];
	float skalarProd;
	float crossProd;

	HEXA_MOVE_t leftRight;

	float normVectShould;

	float angleDegShould;
	float angleDegIs;

	float angleBetween;

	/* Get the direction vector from the hexapod to the target point */
	dirVectShould[0] = setPointX - x_2D_map;
	dirVectShould[1] = setPointY - y_2D_map;

	/* Normalize the direction vector */
	normVectShould = sqrt((dirVectShould[0]*dirVectShould[0]) + (dirVectShould[1]*dirVectShould[1]));

	dirVectShould[0] /= normVectShould;
	dirVectShould[1] /= normVectShould;

	dirVectIs[0] = cos(angle-(M_PI/2));
	dirVectIs[1] = sin(angle-(M_PI/2));

	angleDegShould = (57.2958*atan2(dirVectShould[0],dirVectShould[1]));
	angleDegIs = (57.2958*atan2(dirVectIs[0],dirVectIs[1]));

	printf("DIRECTION VECTOR SHOULD: %f  %f  %f\n", dirVectShould[0], dirVectShould[1], angleDegShould);
	printf("DIRECTION VECTOR IS:     %f  %f  %f\n", dirVectIs[0], dirVectIs[1], angleDegIs);

	/* Get signed angle between is and should - using reference vector [1;0] */
	skalarProd = (dirVectIs[0]*dirVectShould[0]) + (dirVectIs[1]*dirVectShould[1]);
	angleBetween = acos(skalarProd)*57.2958;

	/* Get the fastest way to the desired direction via the crossprodukt */
	crossProd = (dirVectIs[1]*dirVectShould[0])-(dirVectIs[0]*dirVectShould[1]);

	if(crossProd > 0)
	{
		leftRight = ROTATE_RIGHT;
	}
	else
	{
		leftRight = ROTATE_LEFT;
	}


	/* Get the distance between these points */
	distance = sqrt(((setPointX - x_2D_map)*(setPointX - x_2D_map)) + ((setPointY - y_2D_map)*(setPointY - y_2D_map)));

	if(distance > 15)
	{
		if(angleBetween > 20)
		{
			hexaWalk(leftRight, 1);
		}
		else
		{
			hexaWalk(WALK_FORWARD, 1);
		}
	}

//	angleBetween = skalarProd;
//
//	angleBetween = acos(angleBetween);
//
	printf("ANGLEBETWEEN = %f  crossprod: %f  distance: %d\n", angleBetween, crossProd, distance);

	/* OK now we got all we need turn to the right direction*/
//	if(angleDegIs < 0 && angleDegShould > 0)
//	{
//		distance = 180 + angleDegIs;
//		distance += 180 - angleDegShould;
//	}
//	else if(angleDegIs > 0 && angleDegShould < 0)
//	{
//		distance = 180 + angleDegShould;
//		distance += 180 - angleDegIs;
//		distance *= (-1);
//	}
//	else
//	{
//		distance = angleDegIs - angleDegShould;
//	}
//
//	if(distance < 20 && distance > -20)
//	{
//		return;
//	}
//
//	else
//	{
//		hexaWalk(ROTATE_RIGHT, 1);
//	}

}

