/*
 * PointgreyCamera.h
 *
 *  Created on: 2012/01/25
 *      Author: umakatsu
 */

#ifndef POINTGREYCAMERA_H_
#define POINTGREYCAMERA_H_

#include "CaptureLibrary.h"
#include "PTAMTracking/ATANCamera.h"
#include "VideoSource/VideoSource.h"

//#include <opencv/cv.h>
//#include <opencv/highgui.h>

namespace ARMM{
	//class PointgreyCamera : public Capture
	class PointgreyCamera : public PTAMM::ATANCamera, public Capture{
	  public:
		PointgreyCamera( std::string sName, const char* parametersFile, PTAMM::VideoSource * video = NULL);
		~PointgreyCamera( void );

		IplImage* getFrame();

      private:
 		PTAMM::VideoSource * mVideoSource;
	};
}

#endif /* POINTGREYCAMERA_H_ */
