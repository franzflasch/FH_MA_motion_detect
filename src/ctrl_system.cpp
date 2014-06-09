/*
 * ctrl_system.cpp
 *
 *  Created on: Jun 9, 2014
 *      Author: franz
 */

#include <ctrl_system.h>


int ControlSystem::getCurrentPosition(int which)
{
	return currentPos[which];
}

void ControlSystem::setCurrentPosition(int which, int val)
{
	currentPos[which] = val;
}


