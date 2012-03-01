#ifndef ARMM_VRPN_H
#define ARMM_VRPN_H

#include <stdio.h>
//#include <tchar.h>

#include <math.h>

#include "vrpn_Text.h"
#include "vrpn_Tracker.h"
#include "vrpn_Connection.h"

#include <iostream>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Texture2D>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osgShadow/ShadowedScene>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/io_utils>

using namespace std;
const int NUMBER_CAR = 2;
const int NUM_WHEEL = 4;

namespace ARMM{
	/////////////////////// ARMM_COMMUNICATOR /////////////////////////////

	class ARMM_Communicator : public vrpn_Tracker
{
public:
    ARMM_Communicator( vrpn_Connection *c = 0);
    virtual ~ARMM_Communicator();
	void VRPN_Update_Client_Cars(osg::Quat *q,osg::Vec3d  *v, osg::Quat wq[][4], osg::Vec3d wv[][4], float *ground_grid);
    virtual void mainloop();

protected:
    struct timeval _timestamp;
	float* HeightfieldData;
	osg::Vec3d *CarsArrayPos;
	osg::Quat *CarsArrayQuat;
	osg::Vec3d **WheelsArrayPos;
	osg::Quat **WheelsArrayQuat;

};
}
#endif
