/*
 * PointgreyCamera.cc
 *
 *  Created on: 2012/01/25
 *      Author: umakatsu
 */

#include "ARMM/PointgreyCamera.h"
#include "myCVmethod.h"

namespace ARMM {
	PointgreyCamera::PointgreyCamera(std::string sName, const char* parametersFile, PTAMM::VideoSource * video)
	: PTAMM::ATANCamera(sName), Capture(), mVideoSource(video)
	{
		if (video == NULL) {
			std::cout << "Create video source for PGR Camera" << std::endl;
			mVideoSource = new PTAMM::VideoSource();
		}

		// get camera parameter memory from an external file
		bool loadedParams = false;
		//If we're using the parameter files load them
		if (parametersFile && strlen(parametersFile) > 0)
			loadedParams = loadCaptureParams(parametersFile);

//		SetImageSize(mVideoSource->Size());

		//	params.width  = mVideoSource.Size()[0];
		//	params.height = mVideoSource.Size()[1];

		//set intrinsics matrix of the camera
		//PTAM Camera.cfg REF:"www.cagylogic.com/archives/2009/02/26000100.php"
		//	cvmSet(params.intrinsics, 0, 0, (*mgvvCameraParams)[0]);
		//	cvmSet(params.intrinsics, 0, 2, (*mgvvCameraParams)[2]);
		//	cvmSet(params.intrinsics, 1, 1, (*mgvvCameraParams)[1]);
		//	cvmSet(params.intrinsics, 1, 2, (*mgvvCameraParams)[3]);

		//	//set distortion co-efficient value
		//	params.distortion->data.db[0] = (*mgvvCameraParams)[4];
		//	params.distortion->data.db[1] = mdWinv;
		//	params.distortion->data.db[2] = md2Tan;
		//	params.distortion->data.db[3] = mdOneOver2Tan;

		params.principalX = params.intrinsics->data.db[2];
		params.principalY = params.intrinsics->data.db[5];
		//Initialize Undistortion Maps
		cvInitUndistortMap(params.intrinsics, params.distortion, params.undistortX,
				params.undistortY);

		//Turn undistortion on
		setUndistort(true);

		if (params.width == 0 || params.height == 0)
			std::cerr << "Invalid Camera Resolution" << std::endl;
	}

	PointgreyCamera::~PointgreyCamera(void) {
		delete mVideoSource;
	}

	IplImage* PointgreyCamera::getFrame(void) {
		CVD::Image<CVD::Rgb<CVD::byte> > imRGB;
		CVD::Image<CVD::byte> imBW;

		imBW.resize(mVideoSource->Size());
		imRGB.resize(mVideoSource->Size());
		mVideoSource->GetAndFillFrameBWandRGB(imBW, imRGB);

		IplImage* newFrame;
		unsigned char * frame = new unsigned char[params.width * params.height * 3];
		myReadRawimage(imRGB, frame);
		newFrame = mycvConvertIplImage(frame, params.width, params.height, 3);

		if (undistort) {
			IplImage *unDistortedFrame = cvCreateImage(cvGetSize(newFrame),
					newFrame->depth, newFrame->nChannels);
			cvRemap(newFrame, unDistortedFrame, params.undistortX,
					params.undistortY);
			cvReleaseImage(&newFrame);
			newFrame = unDistortedFrame;
		}

		delete frame;
		return newFrame;
	}
}
