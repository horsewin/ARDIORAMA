#ifndef KEYBOARD_CONTROLSCLIENT_H
#define KEYBOARD_CONTROLSCLIENT_H

#define SIM_MICROMACHINE 1

#include <iostream>

//OpenCV
#include "opencv/cv.h"

extern int collide_counter;
extern double prev_collide_clock;
const float CUBE_SIZE = 4;
const float SPHERE_SIZE = 2;//FULL SIZE

using namespace std;

class KeyboardController_client{
public:
	KeyboardController_client(){};
	int check_input() {

#ifdef SIM_MICROMACHINE
		if (getKey(78)) {//N
			return 78;
		} else if (getKey(83)) {//S
			return 83;
		} else if (getKey(65)) {//A
			return 65;
		} else if (getKey(68)) {//D
			return 68;
		}
#endif /*SIM_MICROMACHINE*/

		//A 65 S 83 D 68 W 87 F 70 V 86
		return 0;
	}

	void set_input(const int & key) {

#ifdef SIM_MICROMACHINE
		switch(key){
			case 78://N
				osgAddObjectNode(osgSphereNode(SPHERE_SIZE));
				Virtual_Objects_Count++;
				break;
				
			case 65:
				break;

			case 66: //B
				osgAddObjectNode(osgBoxNode(CUBE_SIZE));
				Virtual_Objects_Count++;
				break;

			case 68:
				break;
			case 83:
				break;
		}
#endif /*SIM_MICROMACHINE*/

		//A 65 S 83 D 68 W 87 F 70 V 86
	}

private:
	inline bool getKey(int key) {
//		return GetAsyncKeyState(key)& 0x8000;
		return false;
	}
};

#endif
