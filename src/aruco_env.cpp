/*
 * aruco_env.cpp
 *
 *  Created on: Jun 9, 2014
 *      Author: franz
 */
#include <aruco_env.h>

vector<Marker> arucoEnv::getMarker(void)
{
	return Markers;
}

int arucoEnv::searchForMarkerId(int markerIdToSearch)
{
	for (unsigned int i=0;i<Markers.size();i++)
	{
		if(Markers[i].id == markerIdToSearch)
		{
			markerObjectToTrack = i;
			return ARUCO_ENV_TRUE;
		}
	}
	markerObjectToTrack = -1;
	return ARUCO_ENV_FALSE;
}

void arucoEnv::processSingle(Mat image)
{
	currentImage = image;
	CamParam.resize( currentImage.size());
	MDetector.detect(currentImage,Markers,CamParam,MarkerSize);
	//for each marker, draw info and its boundaries in the image
	for (unsigned int i=0;i<Markers.size();i++) {
		cout<<Markers[i]<<endl;
		Markers[i].draw(currentImage,Scalar(0,0,255),2);
	}
}

