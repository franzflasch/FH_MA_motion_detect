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

class ControlSystem
{
	private:
		int currentPos[POS_MAX];

	public:
		int getCurrentPosition(int which);
		void setCurrentPosition(int which, int val);

		ControlSystem()
		{
			int i;
			for (i=0;i<POS_MAX;i++)
			{
				currentPos[i] = 0;
			}
		}
};


#endif /* CTRL_SYSTEM_H_ */
