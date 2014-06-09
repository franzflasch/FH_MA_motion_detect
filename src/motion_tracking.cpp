/*
 * motion_tracking.c
 *
 *  Created on: Feb 19, 2014
 *      Author: franz
 */

#include <motion_tracking.h>
#include <stdio.h>
#include <string>
#include <iostream>

string intToString(int number)
{
   stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}

Mat getHsvHist(Mat src, String name)
{
	  Mat histSrc;

	  src.copyTo(histSrc);

	  //cvtColor( histSrc, histSrc, CV_RGB2HSV_FULL );
	  cvtColor( histSrc, histSrc, CV_BGR2HSV_FULL );

	  imshow(name+"HsvImage", histSrc);

	  /// Separate the image in 3 places ( B, G and R )
	  vector<Mat> bgr_planes;
	  split( histSrc, bgr_planes );

	  /// Establish the number of bins
	  int histSize = 256;

	  /// Set the ranges ( for B,G,R) )
	  float range[] = { 30, 200 } ;
	  const float* histRange = { range };

	  bool uniform = true; bool accumulate = false;

	  Mat b_hist, g_hist, r_hist;

	  /// Compute the histograms:
	  calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
	  calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
	  calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

	  // Draw the histograms for B, G and R
	  int hist_w = 512; int hist_h = 400;
	  int bin_w = cvRound( (double) hist_w/histSize );

	  Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

	  /// Normalize the result to [ 0, histImage.rows ]
	  normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
	  normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
	  normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

	  /// Draw for each channel
	  for( int i = 1; i < histSize; i++ )
	  {
	      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
	                       Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
	                       Scalar( 255, 0, 0), 2, 8, 0  );
//	      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
//	                       Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
//	                       Scalar( 0, 255, 0), 2, 8, 0  );
//	      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
//	                       Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
//	                       Scalar( 0, 0, 255), 2, 8, 0  );
	  }

	  /// Display
	  namedWindow(name, CV_WINDOW_AUTOSIZE );
	  imshow(name, histImage );
	  return b_hist;

}

void featureCompare(Mat imgA, int minHessian)
{
	// Load images
//	Mat imgA = imread("/home/franz/Desktop/workspace/opencv_featureDetector/Debug/stm32discovery.jpg", CV_LOAD_IMAGE_GRAYSCALE );
//	if( !imgA.data ) {
//		std::cout<< " --(!) Error reading image " << std::endl;
//		return;
//	}

	Mat imgB = imread("/home/franz/Desktop/workspace/opencv_featureDetector/Debug/stm32f4discovery.jpg", CV_LOAD_IMAGE_GRAYSCALE );
	if( !imgB.data ) {
		std::cout<< " --(!) Error reading image " << std::endl;
		return;
	}

	//cvtColor(imgA, imgA,CV_RGB2GRAY);

    //Detect the keypoints using SURF Detector
    //int minHessian = 500;
    SurfFeatureDetector detector( minHessian );
    std::vector<KeyPoint> kp_object;

    detector.detect( imgA, kp_object );

    //Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;
    Mat des_object;

    extractor.compute( imgA, kp_object, des_object );

    FlannBasedMatcher matcher;

    //VideoCapture cap(0);

    namedWindow("Good Matches");

    std::vector<Point2f> obj_corners(4);

    //Get the corners from the object
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( imgA.cols, 0 );
    obj_corners[2] = cvPoint( imgA.cols, imgA.rows );
    obj_corners[3] = cvPoint( 0, imgA.rows );

	Mat des_image, img_matches;
	std::vector<KeyPoint> kp_image;
	std::vector<vector<DMatch > > matches;
	std::vector<DMatch > good_matches;
	std::vector<Point2f> obj;
	std::vector<Point2f> scene;
	std::vector<Point2f> scene_corners(4);
	Mat H;

	double t = (double)getTickCount();
	detector.detect( imgB, kp_image );
	extractor.compute( imgB, kp_image, des_image );

	matcher.knnMatch(des_object, des_image, matches, 2);

	for(int i = 0; i < min(des_image.rows-1,(int) matches.size()); i++) //THIS LOOP IS SENSITIVE TO SEGFAULTS
	{
		if((matches[i][0].distance < 0.6*(matches[i][1].distance)) && ((int) matches[i].size()<=2 && (int) matches[i].size()>0))
		{
			good_matches.push_back(matches[i][0]);
		}
	}

	//Draw only "good" matches
	drawMatches( imgA, kp_object, imgB, kp_image, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	if (good_matches.size() >= 4)
	{
		for( unsigned int i = 0; i < good_matches.size(); i++ )
		{
			//Get the keypoints from the good matches
			obj.push_back( kp_object[ good_matches[i].queryIdx ].pt );
			scene.push_back( kp_image[ good_matches[i].trainIdx ].pt );
		}

		H = findHomography( obj, scene, CV_RANSAC );

		perspectiveTransform( obj_corners, scene_corners, H);

		//Draw lines between the corners (the mapped object in the scene image )
		line( img_matches, scene_corners[0] + Point2f( imgA.cols, 0), scene_corners[1] + Point2f( imgA.cols, 0), Scalar(0, 255, 0), 4 );
		line( img_matches, scene_corners[1] + Point2f( imgA.cols, 0), scene_corners[2] + Point2f( imgA.cols, 0), Scalar( 0, 255, 0), 4 );
		line( img_matches, scene_corners[2] + Point2f( imgA.cols, 0), scene_corners[3] + Point2f( imgA.cols, 0), Scalar( 0, 255, 0), 4 );
		line( img_matches, scene_corners[3] + Point2f( imgA.cols, 0), scene_corners[0] + Point2f( imgA.cols, 0), Scalar( 0, 255, 0), 4 );
	}

	t = ((double)getTickCount() - t)/getTickFrequency();
	std::cout << "detection time [s]: " << t << std::endl;

	//Show detected matches
	imshow( "Good Matches", img_matches );
}

void MotionDetection::drawRotatedContourLines(vector<RotatedRect> minRect, size_t contourSize, int minDrawSize, Scalar color, String name)
{
	Point2f rect_points[4];
	Mat tmpImage = currentImageToProcess.clone();
	tmpImage.setTo(cv::Scalar(255,255,255));
	for( unsigned int i = 0; i< contourSize; i++ )
	{
		minRect[i].points( rect_points );

		if((minRect[i].boundingRect().height > minDrawSize) && (minRect[i].boundingRect().width > minDrawSize))
		{
			/* Crop image */
			Mat croppedImage;
			//croppedImage = src(minRect[largestContourIndex].boundingRect()).clone();
			if(((minRect[i].boundingRect().y + minRect[i].boundingRect().height) < currentImageToProcess.rows) &&
			   ((minRect[i].boundingRect().x + minRect[i].boundingRect().width) < currentImageToProcess.cols) &&
			   (minRect[i].boundingRect().x > 0) &&
			   (minRect[i].boundingRect().y > 0))
			{
				currentImageToProcess(minRect[i].boundingRect()).copyTo(croppedImage);
				//imshow( name +"Crop" + intToString(i), croppedImage );

				printf("Center: %f  %f\n", minRect[i].center.x, minRect[i].center.y);
				printf("Points: %d  %d\n", minRect[i].boundingRect().x, minRect[i].boundingRect().y);

				Rect roi = Rect(minRect[i].boundingRect().x, minRect[i].boundingRect().y, croppedImage.cols, croppedImage.rows);
				cv::Mat subView = tmpImage(roi);
				croppedImage.copyTo(subView);
				imshow("tempimage", tmpImage);
			}

			for( int j = 0; j < 4; j++ )
			{
				  line( currentImageToProcess, rect_points[j], rect_points[(j+1)%4], color, 3, 8 );
			}
		}
	}
	imshow( name +"Contours", currentImageToProcess );
}

Mat MotionDetection::getMergedMotion(vector<Rect> minRect, size_t contourSize, int minDrawSize, Scalar color, String name)
{
	Mat croppedImage;
	Mat drawing = currentImageToProcess.clone();
	drawing.setTo(cv::Scalar(255,255,255));

	for( unsigned int i = 0; i< contourSize; i++ )
	{
		if(minRect[i].height > minRectSizeForDetection && minRect[i].width > minRectSizeForDetection)
		{
			currentImageToProcess(minRect[i]).copyTo(croppedImage);
			//rectangle( drawing, minRect[i].tl(), minRect[i].br(), color, 2, 8, 0 );
			Rect roi = Rect(minRect[i].x, minRect[i].y, croppedImage.cols, croppedImage.rows);
			cv::Mat subView = drawing(roi);
			croppedImage.copyTo(subView);
		}
	}
	imshow( "merged image", drawing );
	return drawing;
}

Mat MotionDetection::detectMotion(cv::Mat src)
{
	Mat depth_gray;
	Mat depth_diff;
	Mat threshold_output;
	Mat detected_edges;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	currentImageToProcess = src;

	if(storageCount >= usedStorages)
	{
		storageCount = 0;
	}

	cvtColor(currentImageToProcess, depth_gray, CV_RGB2GRAY);

	absdiff(imageStorages[storageCount], depth_gray, depth_diff);
	imshow( objectName+"diff", depth_diff );

	// Detect edges using Threshold
	threshold( depth_diff, threshold_output, 20, 255, THRESH_BINARY );
	imshow( objectName+"threshold", threshold_output );

	/* Erosion */
	Mat element = getStructuringElement( MORPH_RECT, Size(2*erodeVal+1, 2*erodeVal+1), Point(erodeVal, erodeVal));
	Mat erode_out;
	erode( threshold_output, erode_out, element );
	imshow( objectName+"erosion", erode_out );

	/* Dilation */
	element = getStructuringElement( MORPH_RECT, Size(2*dilateVal+1, 2*dilateVal+1), Point(dilateVal, dilateVal));
	Mat dilate_out;
	dilate( erode_out, dilate_out, element );
	imshow( objectName+"dilation", dilate_out );

	/* Find the contours within the dilated picture */
	findContours(dilate_out, contours, hierarchy, CV_RETR_EXTERNAL,  CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	Scalar color = Scalar(255,0,255);

	/* Find rotated rectangle */
	vector<vector<Point> > contours_poly( contours.size() );

//	vector<RotatedRect> minRect( contours.size() );
//	for( unsigned int i = 0; i < contours.size(); i++ )
//	{
//		approxPolyDP( Mat(contours[i]), contours_poly[i], 15, true );
//		minRect[i] = minAreaRect( Mat(contours[i]) );
//	}
//	drawRotatedContourLines(minRect, contours.size(), minRectSizeForDetection, color, objectName);

	vector<Rect> minRect( contours.size() );
	for( unsigned int i = 0; i < contours.size(); i++ )
	{
		approxPolyDP( Mat(contours[i]), contours_poly[i], 15, true );
		minRect[i] = boundingRect( Mat(contours[i]) );
	}

	imageStorages[storageCount++] = depth_gray.clone();
	return getMergedMotion(minRect, contours.size(), minRectSizeForDetection, color, objectName);
}
