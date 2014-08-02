/*
 * ctrl_system.h
 *
 *  Created on: Jun 9, 2014
 *      Author: franz
 */

#ifndef CTRL_SYSTEM_H_
#define CTRL_SYSTEM_H_

typedef enum POS_e
{
	X_POS = 0,
	Y_POS,
	Z_POS,
	POS_MAX

}POS_t;

typedef enum HEXA_MOVE_e
{
	WALK_LEFT = 0,
	WALK_RIGHT,
	WALK_FORWARD,
	WALK_BACK,
	HEXA_WALK_END,
	ROTATE_LEFT,
	ROTATE_RIGHT,
	HEXA_MOVE_END,

}HEXA_MOVE_t;

class ControlSystem
{
	private:
		int currentPos[POS_MAX];
		int pixelWidth;
		int pixelHeight;
		float x_2D_map;
		float y_2D_map;
		float angle;
		float height;

		typedef void (*hexaWriteFunction)(char *data, int size, void *userData);
		hexaWriteFunction hexaWrite;
		void *hexaWriteUserData;

	public:

		int getCurrentPosition(int which);
		void setCurrentPosition(int which, int val);
		void update2DMap(float objectDistance, float xCorrectionVal = 0, float yCorrectionVal = 0);
		void hexaWalk(HEXA_MOVE_t DIRECTION, char steps);
		void hexaControl(int setPointX, int setPointY);
		void hexaControlRotate(int setPointX, int setPointY);

		void hexaSetAngle(float angleRad);

		ControlSystem(int pixWidth,
					  int pixHeight,
					  float angleRad,
					  float cameraHeight,
					  hexaWriteFunction hexaWriteFnc,
					  void *userData)
		{
			int i;
			for (i=0;i<POS_MAX;i++)
			{
				currentPos[i] = 0;
			}

			pixelWidth = pixWidth;
			pixelHeight = pixHeight;
			x_2D_map = 0;
			y_2D_map = 0;
			angle = angleRad;
			height = cameraHeight;
			hexaWrite = hexaWriteFnc;
			hexaWriteUserData = userData;
		}
};


#endif /* CTRL_SYSTEM_H_ */
