/*
 ============================================================================
 Name        : kinect_testing.c
 Author      : Franz Flasch
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <sys/ipc.h>
#include <sys/shm.h>

#include <libfreenect.h>
#include <unistd.h>
#include <math.h>
#include <iostream>
#include <kinect_opencv.h>
#include <motion_tracking.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>
#include <aruco_env.h>
#include <ctrl_system.h>

extern "C"
{
	#include <websocket.h>
}

using namespace cv;
using namespace std;

int X_global_setpoint = 0;
int Y_global_setpoint = 120;

/* freenect lowlevel global variables */
freenect_context *f_ctx;
freenect_device *f_dev;
pthread_mutex_t gl_backbuf_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t gl_frame_cond = PTHREAD_COND_INITIALIZER;

/* This might be some convert value table */
uint16_t t_gamma[2048];

/* global arrays for the depth information */
uint8_t *depth_mid, *depth_front;
uint16_t *depthValues_u16;

uint8_t *rgb_back, *rgb_mid, *rgb_front;

/* global variables for synchronizing */
int got_depth = 0;
int got_rgb = 0;
int stopKinectThread = 0;

/* Kinect depth callback, that's the reason we need so many global variables :( */
void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
	int i;
	uint16_t *depth = (uint16_t*)v_depth;

	pthread_mutex_lock(&gl_backbuf_mutex);
	for (i=0; i<PIXEL_WIDTH*PIXEL_HEIGHT; i++) {
		int pval = t_gamma[depth[i]];
		int lb = pval & 0xff;
		switch (pval>>8) {
			case 0:
				depth_mid[3*i+0] = 255;
				depth_mid[3*i+1] = 255-lb;
				depth_mid[3*i+2] = 255-lb;
				break;
			case 1:
				depth_mid[3*i+0] = 255;
				depth_mid[3*i+1] = lb;
				depth_mid[3*i+2] = 0;
				break;
			case 2:
				depth_mid[3*i+0] = 255-lb;
				depth_mid[3*i+1] = 255;
				depth_mid[3*i+2] = 0;
				break;
			case 3:
				depth_mid[3*i+0] = 0;
				depth_mid[3*i+1] = 255;
				depth_mid[3*i+2] = lb;
				break;
			case 4:
				depth_mid[3*i+0] = 0;
				depth_mid[3*i+1] = 255-lb;
				depth_mid[3*i+2] = 255;
				break;
			case 5:
				depth_mid[3*i+0] = 0;
				depth_mid[3*i+1] = 0;
				depth_mid[3*i+2] = 255-lb;
				break;
			default:
				depth_mid[3*i+0] = 0;
				depth_mid[3*i+1] = 0;
				depth_mid[3*i+2] = 0;
				break;
		}
	}
	got_depth++;
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}

void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
	pthread_mutex_lock(&gl_backbuf_mutex);

	// swap buffers
	assert (rgb_back == rgb);
	rgb_back = rgb_mid;
	freenect_set_video_buffer(dev, rgb_back);
	rgb_mid = (uint8_t*)rgb;

	got_rgb++;
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}

/* Freenect thread */
void *freenect_threadfunc(void *arg)
{
	printf("Kinect camera test\n");

	int i;
	for (i=0; i<2048; i++)
	{
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}

	if (freenect_init(&f_ctx, NULL) < 0)
	{
		printf("freenect_init() failed\n");
		return NULL;
	}

	freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
	freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

	int nr_devices = freenect_num_devices (f_ctx);
	printf ("Number of devices found: %d\n", nr_devices);

	if (nr_devices < 1)
	{
		freenect_shutdown(f_ctx);
		return NULL;
	}

	if (freenect_open_device(f_ctx, &f_dev, 0) < 0)
	{
		printf("Could not open device\n");
		freenect_shutdown(f_ctx);
		return NULL;
	}

	freenect_set_tilt_degs(f_dev, -25);
	freenect_set_led(f_dev,LED_RED);

	freenect_set_depth_callback(f_dev, depth_cb);
	freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));

	freenect_set_video_callback(f_dev, rgb_cb);
	freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
	freenect_set_video_buffer(f_dev, rgb_back);


	freenect_start_video(f_dev);
	freenect_start_depth(f_dev);

	while (!stopKinectThread && freenect_process_events(f_ctx) >= 0)
	{
	}

	printf("\nshutting down streams...\n");

	freenect_stop_depth(f_dev);
	freenect_stop_video(f_dev);

	freenect_close_device(f_dev);
	freenect_shutdown(f_ctx);

	printf("-- done!\n");
	return NULL;
}


static void writeDepthFile(const char *fileName, uint16_t *depthValues, uint32_t pixelHeight, uint32_t pixelWidth)
{
	uint32_t i = 0;
	FILE *fDepthValues;

	fDepthValues = fopen(fileName, "w");
	if (fDepthValues == NULL)
	{
	    printf("Error opening file!\n");
	    exit(1);
	}
	/* Write depth file */
	rewind(fDepthValues);
	/* Write depth values to the file */
	for (i=0; i<pixelWidth*pixelHeight; i++)
	{
		if((i>0) && (i%pixelWidth) == 0)
		{
			fprintf(fDepthValues, "\n");
		}
		fprintf(fDepthValues, "%d;", depthValues_u16[i]);
	}

	fclose(fDepthValues);
}

double measureTimeSince(timeval *time)
{
   timeval t;
   double elapsedTime;

   // stop timer
   gettimeofday(&t, NULL);

   // compute and print the elapsed time in millisec
   elapsedTime = (t.tv_sec - time->tv_sec) * 1000.0;      // sec to ms
   elapsedTime += (t.tv_usec - time->tv_usec) / 1000.0;   // us to ms
   return elapsedTime;
}

#define MAXMYMEM 30

int attachSharedMem(void)
{
	int shID;
	int32_t *shrdMemPtr;

	shID = shmget(2404, MAXMYMEM, IPC_CREAT | 0666);
	if (shID >= 0)
	{
		shrdMemPtr = (int32_t *)shmat(shID, 0, 0);
		if (shrdMemPtr==(void *)-1)
		{
			return -1;
		}
		else
		{
			/* Set default values */
			shrdMemPtr[0] = X_global_setpoint;
			shrdMemPtr[1] = Y_global_setpoint;
			shmdt(shrdMemPtr);
			return shID;
		}
	}
	return -1;
}

int updateCoordinatesFromSharedMem(int shID)
{
	int32_t *shrdMemPtr;

	shrdMemPtr = (int32_t *)shmat(shID, 0, 0);

	if (shrdMemPtr==(void *)-1)
	{
		return -1;
	}
	else
	{
		X_global_setpoint = shrdMemPtr[0];
		Y_global_setpoint = shrdMemPtr[1];
		shmdt(shrdMemPtr);
		return 0;
	}
}

void detachSharedMem(int shID)
{
	/* Shared Memory erzeugen */
	shID = shmget(2404, MAXMYMEM, 0666);
	if (shID >= 0)
	{
		/* zerstöre den Shared Memory */
		shmctl(shID, IPC_RMID, 0);
	}
	else
	{
		/* shmctl lief schief */
		perror("shmctl");
	}
}

int main( int argc, char** argv )
{
	/* Input argument check */
	if(argc<3) {cerr<<"Usage: boardConfig.yml [cameraParams.yml] [markerSize]  [outImage]"<<endl;exit(0);}

	int retVal = 1;
	int shId = 0;

	if((shId = attachSharedMem()) < 0)
	{
		printf("Error could not attach shared memory\n");
		return retVal++;
	}

	pthread_t freenect_thread;
	uint8_t depth_ready = 0;
	uint8_t rgb_ready = 0;

	/* get some memory */
	depth_mid = (uint8_t*)malloc(PIXEL_WIDTH*PIXEL_HEIGHT*3);
	depth_front = (uint8_t*)malloc(PIXEL_WIDTH*PIXEL_HEIGHT*3);
	//depthValues_u16 = (uint16_t*)malloc(PIXEL_WIDTH*PIXEL_HEIGHT*sizeof(uint16_t));

	/* map the depth image array to an opencv Matrix */
	Mat depthImage_u8 = Mat(PIXEL_HEIGHT, PIXEL_WIDTH, CV_8UC3, depth_front);
	//Mat depthImage_u16 = Mat(PIXEL_HEIGHT, PIXEL_WIDTH, CV_16UC1, depthValues_u16);

	rgb_back = (uint8_t*)malloc(PIXEL_WIDTH*PIXEL_HEIGHT*3);
	rgb_mid = (uint8_t*)malloc(PIXEL_WIDTH*PIXEL_HEIGHT*3);
	rgb_front = (uint8_t*)malloc(PIXEL_WIDTH*PIXEL_HEIGHT*3);

	/* map the depth image array to an opencv Matrix */
	Mat rgbImage_u8 = Mat(PIXEL_HEIGHT, PIXEL_WIDTH, CV_8UC3, rgb_front);

	/* Initialize applicaton specific objects */
	int outPutCnt = 0;
	timeval currTime;    /* To measure the execution time */
	double elapsedTime;  /* To measure the execution time */
	int key = 0;

	static webSocketState_s hexaWebSocketState;
	if(initWebsocketContext(&hexaWebSocketState))
	{
		printf("Error when creating the websocket context\n");
		return retVal++;
	}

	arucoEnv arucoObject(argv[1], argv[2], argv[3]);
	MotionDetection hexaMotionRgb(PIXEL_HEIGHT, PIXEL_WIDTH, 15, "Rgb", 25, 3, 40);
	MotionDetection hexaMotionDepth(PIXEL_HEIGHT, PIXEL_WIDTH, 5, "Depth", 50, 10, 20);
	ControlSystem hexapodRobot(PIXEL_WIDTH,
							   PIXEL_HEIGHT,
							   1.1102/*~63°*/,
							   55,
							   &writeWebsocket,
							   &hexaWebSocketState);

	/* Start the kinect thread */
	if(pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL))
	{
		printf("pthread_create failed\n");
		freenect_shutdown(f_ctx);
		return retVal++;
	}

	do
	{
		/* image grabbing context */
		uint8_t *tmp;
		pthread_mutex_lock(&gl_backbuf_mutex);
		if (got_depth)
		{
			tmp = depth_front;
			depth_front = depth_mid;
			depth_mid = tmp;
			got_depth = 0;
			depth_ready = 1;
		}
		if (got_rgb)
		{
			tmp = rgb_front;
			rgb_front = rgb_mid;
			rgb_mid = tmp;
			got_rgb = 0;
			rgb_ready = 1;
		}
		pthread_mutex_unlock(&gl_backbuf_mutex);

		/* Image processing */
		if(depth_ready)
		{
			depthImage_u8 = Mat(PIXEL_HEIGHT, PIXEL_WIDTH, CV_8UC3, depth_front);
			cvtColor(depthImage_u8, depthImage_u8, CV_BGR2RGB);
			imshow( "Kinect DepthImage", depthImage_u8 );
			//hexaMotionDepth.detectMotion(depthImage_u8);
			depth_ready = 0;
		}

		if(rgb_ready)
		{
			rgbImage_u8 = Mat(PIXEL_HEIGHT, PIXEL_WIDTH, CV_8UC3, rgb_front);
			cvtColor(rgbImage_u8, rgbImage_u8, CV_BGR2RGB);

			key = waitKey(33);
			if(key == 10)
			{
				imwrite("output"+intToString(outPutCnt++) + ".png", rgbImage_u8);
			}

			imshow( "Kinect RgbImage", rgbImage_u8 );

			gettimeofday(&currTime, NULL);

			//arucoObject.processSingle(hexaMotionRgb.detectMotion(rgbImage_u8));
			arucoObject.processSingle(rgbImage_u8);
			//imshow( "aruco Object", arucoObject.currentImage );

			if(arucoObject.searchForMarkerIdFromList() == ARUCO_ENV_TRUE )
			{
				/* update current positions */
				hexapodRobot.setCurrentPosition(X_POS,
						(int)arucoObject.getMarker()[arucoObject.markerObjectToTrack].getCenter().x);
				hexapodRobot.setCurrentPosition(Y_POS,
						(int)arucoObject.getMarker()[arucoObject.markerObjectToTrack].getCenter().y);
				printf("GOT OBJECT: current position: x: %d  y:%d\n", hexapodRobot.getCurrentPosition(X_POS),
																	  hexapodRobot.getCurrentPosition(Y_POS));
				hexapodRobot.update2DMap(arucoObject.getMarker()[arucoObject.markerObjectToTrack].Tvec.ptr<float>(0)[2],
										 arucoObject.getCorrVal(GET_X),
										 arucoObject.getCorrVal(GET_Y));

				/* Update the current object angle */
				hexapodRobot.hexaSetAngle(arucoObject.getObjectToTrackAngle());

				imshow( "aruco Object", arucoObject.currentImage );

				/*
				if(hexaWebSocketState.writeable)
					hexapodRobot.hexaControl(40, 100);
					*/

				hexapodRobot.hexaControlRotate(X_global_setpoint, Y_global_setpoint);

				updateCoordinatesFromSharedMem(shId);
				printf("Coordinates from shared mem: X:%d Y: %d\n", X_global_setpoint, Y_global_setpoint);
			}

			elapsedTime = measureTimeSince(&currTime);
			printf("Execution Time: %f\n", elapsedTime);

			rgb_ready = 0;
		}

	} while((char)key != 27);

	stopKinectThread = 1;

	detachSharedMem(shId);

	//imwrite("depthout.jpg", depthImage_u8);
	//writeDepthFile("kinectDepthValues.csv", depthValues_u16, PIXEL_HEIGHT, PIXEL_WIDTH);

	sleep(1);

	printf("All jobs done...\n");

	return EXIT_SUCCESS;
}
