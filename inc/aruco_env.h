/*
 * aruco_env.h
 *
 *  Created on: Jun 9, 2014
 *      Author: franz
 */

#ifndef ARUCO_ENV_H_
#define ARUCO_ENV_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <aruco/aruco.h>
#include <aruco/boarddetector.h>

using namespace cv;
using namespace aruco;

class arucoEnv
{
	#define ARUCO_ENV_TRUE	1
	#define ARUCO_ENV_FALSE 0

	private:
		CameraParameters CamParam;
		MarkerDetector MDetector;
		BoardConfiguration TheBoardConfig;
		BoardDetector TheBoardDetector;
		Board TheBoardDetected;
		vector<Marker> Markers;
		float MarkerSize;

	public:
		int markerObjectToTrack;
		Mat currentImage;

		vector<Marker> getMarker(void);
		int searchForMarkerId(int markerIdToSearch);
		void processSingle(Mat image);

		arucoEnv(char *boardConfig, char* cameraParam, char *markerSize)
		{
			markerObjectToTrack = -1;
			TheBoardConfig.readFromFile(boardConfig);
			CamParam.readFromXMLFile(cameraParam);
			MarkerSize=atof(markerSize);
		}

#if 0
		/* Constructor */
		MotionDetection(int imageHeight, int imageWidth, unsigned int storages, string objName, int minRectSize, int erodeValue, int dilateValue):
		usedStorages(storages),
		minRectSizeForDetection(minRectSize),
		erodeVal(erodeValue),
		dilateVal(dilateValue)
		{
			objectName = objName;
			storageCount = 0;
			imageStorages = new Mat[usedStorages];
			for(unsigned int i=0;i<storages;i++)
			{
				imageStorages[i] = cv::Mat::zeros(imageHeight, imageWidth, CV_8UC1);
			}
		}

		/* Destructor */
		~MotionDetection()
		{
			delete[] imageStorages;
		}
#endif
};

#endif /* ARUCO_ENV_H_ */