/*
 * Image/ForegroundContents.h
 *
 *  Created on: 2010/11/21
 *      Author: umakatsu
 */

#ifndef FOREGROUNDCONTENTS_H_
#define FOREGROUNDCONTENTS_H_

#include <vector>
#include <deque>

#include <cv.h>
#include <highgui.h>
#include "OpenGL.h"

#include <TooN/helpers.h>
#include "Window/GLWindow2.h"
#include "PTAMTracking/ATANCamera.h"
#include "ImageType.h"
#include "common.h"

#include <boost/array.hpp>

namespace PTAMM{
	class Stroke;

	struct SegmentationInfo{
		uchar* 			silhouette;	// foreground silhouette data
		TooN::SE3<> 		camerapose;	// camera's pose data
		ImageType 			imageRGB;		// foreground color data

		SegmentationInfo( const int & img_width , const int & img_height){
			silhouette = new uchar[ img_width * img_height ];
		}
		~SegmentationInfo(void){
			delete[] silhouette;
		}
	};

	// This data is projected from a set of voxels
	struct ProjectedData{
		double *projected;	// projected 2D data from a set of voxel
		state 	state_proj;			// if "projected" is used by some thread, this variable becomes WAIT
	};

	/*
	 * ビデオ中の前景情報を保持するクラス
	 */
	class ForegroundContents{
	private:
		int width; 				// video image width
		int height; 				// video image height
		double **rgb_data;	// RGB information
		bool update;			// update flag

	public:
		std::deque<Stroke *>	contour_stroke_list;
		std::deque<Stroke *>	silhouette_stroke_list;
		uchar* silhouette;
		uchar* contour;
		ProjectedData pData;
		ProjectedData pData_pre;

		ForegroundContents( int size_w , int size_h );
		~ForegroundContents();

		void DrawSilhouette( const TooN::SE3<> & currentPose , const TooN::Matrix< 4 > & projection ) const;
		void DrawContour   ( const TooN::SE3<> & currentPose , const TooN::Matrix< 4 > & projection ) const;

		void outputSilhouettePPM(const char *filename);
		void writeBitmapfile(const char *filename, const int & w, const int & h, uchar*result );
		void pushSegmentationInfo(uchar* s , const TooN::SE3<> & current , const ImageType & img_color);
		SegmentationInfo * popSegmentationInfoRef( void ) const;
		void popSegmentationInfo( void );
		uchar* 	popSilhouette() const;
		TooN::SE3<> 	popCameraPose() const;
		uchar* 	popSilhouette(int num) const;
		TooN::SE3<> 	popCameraPose(int num) const;
		ImageType 	popColor() const;
		ImageType	popColor(int num) const;
		bool isEmpty( void ) const;

		std::vector< TooN::Vector< 3 > > LabelingBorder(short *result , const int & i, const int & j, const int & code);
		uint getSeginfo( void ) const;
		bool getUpdate() const
		{
			return update;
		}

		void setUpdate(bool update)
		{
			this->update = update;
		}

		double **getRgb_data() const
		{
			return rgb_data;
		}

		void setRgb_data(double **rgb_data)
		{
			this->rgb_data = rgb_data;
		}

	private:
		std::deque< SegmentationInfo * > seginfo;
		bool rangeChecker(const int & is , const int & js);
		void searchPixelSet(int & js, int & is, const int & jc, const int & ic, const int & code);
	};
}
#endif /* FOREGROUNDCONTENTS_H_ */
