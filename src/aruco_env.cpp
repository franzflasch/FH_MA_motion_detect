/*
 * aruco_env.cpp
 *
 *  Created on: Jun 9, 2014
 *      Author: franz
 */
#include <aruco_env.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <math.h>

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
			return ARUCO_ENV_TRUE;
		}
	}
	return ARUCO_ENV_FALSE;
}

int arucoEnv::searchForMarkerIdFromList(void)
{
	unsigned int i = 0;
	unsigned int j = 0;

	for (i=0;i<Markers.size();i++)
	{
		j = 0;
		while(true)
		{
			if(Markers[i].id == multipleMarkerList[j])
			{
				markerObjectToTrack = i;
				return ARUCO_ENV_TRUE;
			}
			if(multipleMarkerList[j] == 0)
			{
				break;
			}
			j++;
		}
	}
	markerObjectToTrack = -1;
	return ARUCO_ENV_FALSE;
}

float arucoEnv::getObjectToTrackAngle(void)
{
	return angleRad;
}

void arucoEnv::processSingle(Mat image)
{
	Mat rotMatrix;
	currentImage = image;
	CamParam.resize( currentImage.size());
	MDetector.detect(currentImage,Markers,CamParam,MarkerSize);
	float angleDeg = 0;

	//for each marker, draw info and its boundaries in the image
	for (unsigned int i=0;i<Markers.size();i++)
	{
		cout<<Markers[i]<<endl;
		Markers[i].draw(currentImage,Scalar(0,0,255),2);

		cv::Rodrigues(Markers[i].Rvec, rotMatrix);

//		printf("%f  %f  %f\n", rotMatrix.ptr<float>(0)[0], rotMatrix.ptr<float>(0)[1], rotMatrix.ptr<float>(0)[2]);
//		printf("%f  %f  %f\n", rotMatrix.ptr<float>(1)[0], rotMatrix.ptr<float>(1)[1], rotMatrix.ptr<float>(1)[2]);
//		printf("%f  %f  %f\n", rotMatrix.ptr<float>(2)[0], rotMatrix.ptr<float>(2)[1], rotMatrix.ptr<float>(2)[2]);

		fprintf( stderr,"%f  %f  %f\n", rotMatrix.ptr<float>(0)[0], rotMatrix.ptr<float>(1)[0], rotMatrix.ptr<float>(2)[0]);
		fprintf( stderr,"%f  %f  %f\n", rotMatrix.ptr<float>(0)[1], rotMatrix.ptr<float>(1)[1], rotMatrix.ptr<float>(2)[1]);
		fprintf( stderr,"%f  %f  %f\n", rotMatrix.ptr<float>(0)[2], rotMatrix.ptr<float>(1)[2], rotMatrix.ptr<float>(2)[2]);

		if(Markers[i].id==705)
		{
			angleDeg = (atan2(rotMatrix.ptr<float>(0)[2], rotMatrix.ptr<float>(0)[0]));
			angleRad = angleDeg;
			angleDeg *= 57.2958;
			fprintf( stderr,"Deg: %f Rad: %f ID = %d correcting direction\n", angleDeg, angleRad, Markers[i].id);
		}
		else if(Markers[i].id==706)
		{
			angleDeg = -((M_PI/2)-atan2(rotMatrix.ptr<float>(0)[2], rotMatrix.ptr<float>(0)[1]));
			angleRad = angleDeg;
			angleDeg *= 57.2958;
			fprintf( stderr,"Deg: %f Rad: %f ID = %d\n", angleDeg, angleRad, Markers[i].id);
		}
		else if(Markers[i].id==707)
		{
			angleDeg = (atan2(rotMatrix.ptr<float>(0)[2], rotMatrix.ptr<float>(0)[1]));
			if(angleDeg < 0)
			{
				angleDeg=((M_PI)+angleDeg);
			}
			else
			{
				angleDeg=((-M_PI)+angleDeg);
			}
			angleRad = angleDeg;
			angleDeg *= 57.2958;
			fprintf( stderr,"Deg: %f Rad: %f ID = %d\n", angleDeg, angleRad, Markers[i].id);
		}
		if(Markers[i].id==708)
		{
			angleDeg = (((M_PI/2))+atan2(rotMatrix.ptr<float>(0)[2], rotMatrix.ptr<float>(0)[0]));
			angleRad = angleDeg;
			angleDeg *= 57.2958;
			fprintf( stderr,"Deg: %f Rad: %f ID = %d correcting direction\n", angleDeg, angleRad, Markers[i].id);
		}

		//fprintf( stderr,"%f\n", 57.2958*atan2(rotMatrix.ptr<float>(0)[0], rotMatrix.ptr<float>(0)[1]));

		/* We just need one marker so break here*/
		break;

//		printf("MARKER X: %f\n", Markers[i].Rvec.ptr<float>(0)[0]);
//		printf("MARKER Y: %f\n", Markers[i].Rvec.ptr<float>(0)[1]);
//		printf("MARKER Z: %f\n", Markers[i].Rvec.ptr<float>(0)[2]);
//		printf("MARKER Z: %f\n", Markers[i].Tvec.ptr<float>(0)[2]);
	}
}

