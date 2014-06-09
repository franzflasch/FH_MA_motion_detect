/*
 * motion_tracking.h
 *
 *  Created on: Feb 19, 2014
 *      Author: franz
 */

#ifndef MOTION_TRACKING_H_
#define MOTION_TRACKING_H_

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

using namespace cv;
using namespace std;

string intToString(int number);

class MotionDetection
{
	private:
		unsigned int storageCount;
		const unsigned int usedStorages;
		Mat *imageStorages;
		string objectName;
		int minRectSizeForDetection;
		int erodeVal;
		int dilateVal;
		Mat currentImageToProcess;
		Mat compareImage;

	public:
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

		/* Methods */
		Mat detectMotion(cv::Mat src);
		void drawRotatedContourLines(vector<RotatedRect> minRect, size_t contourSize, int minDrawSize, Scalar color, String name);
		Mat getMergedMotion(vector<Rect> minRect, size_t contourSize, int minDrawSize, Scalar color, String name);
		void detectSift(cv::Mat src);
};

#endif /* MOTION_TRACKING_H_ */
