/*
 * Segmentation2D.h
 *
 *  Created on: 2011/06/24
 *      Author: umakatsu
 */

#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

#include "common.h"

#include <deque>
/* Data , Thetaの定義を行っているヘッダ */
#include "parameter.h"

#include "Stroke/Stroke.h"
#include "PTAMTracking/TrackerDrawable.h"
#include "PTAMTracking/MapPoint.h"
#include "Image/ForegroundContents.h"

//#include <boost/scoped_ptr.hpp>
//#include "FrameBufferObject.h"
//#include "PixelBufferObject.h"

#include "ImageType.h"
#include "ImageSize.h"
#include "PTAMTracking/ATANCamera.h"

#include <cvd/thread.h>

namespace PTAMM {

	//EMアルゴリズムを適応するサンプルの最大
	const unsigned int MAX_EM_SAMPLES = 50000;
	//Dynamic segmentation結果を保持する個数
	const unsigned int MAXRESERVE = 5;
	//距離変換を正規化するときの最大・最小値・広さを与えるもう1つの条件式
	const double DIST_MAX = 100;
	const double DIST_MIN = 0;
	const double DIST_SIGMA2 = 0.01;
	const double DIST_MU = 0;
	//距離変換の正規分布を計算する時の定数部
	const int ratio = 2;
	const int A = 5;

	class Stroke;
	class StrokeGenerator;
	class VideoData;
	class GMM;
	class Segmentation : protected CVD::Thread {
	protected:
		int w; //image width
		int h; //image height
		int num_of_dynamics;
		char **image_map;
		double * Pr_object;
		double * Pr_back;

//		TooN::SE3<> 			currentPose;
		VideoData 	* 			vd;
		Stroke * 				stroke; //objective stroke contents
		StrokeGenerator *		mpStrokeGenerator; //creating stroke
		ForegroundContents * fContents; //foreground object information
		ATANCamera 			camera; // Same as the tracker's camera
		ImageType 				mimFrameRGB;
		TrackerDrawable *  	tracker;

		// メインスレッドと動作の同期をとるための論理変数
		state state_segment;
		bool interrupt;

		/* GMM */
		GaussDistribution ** gauss_fore;
		GaussDistribution ** gauss_back;

	public:
		Segmentation(const ImageSize &  img_size ,
				ForegroundContents *f,
				const ATANCamera & cam,
				TrackerDrawable * td);
		~Segmentation(void);
		void Start( void );
		void Reset(void);

		void SetStroke(Stroke * stroke) {
			this->stroke = stroke;
		}

		void SetMimFrameRGB(ImageType mimFrameRGB) {
			this->mimFrameRGB = mimFrameRGB;
		}

		bool GetInterrupt() const {
			return interrupt;
		}

		void SetInterrupt(bool interrupt) {
			this->interrupt = interrupt;
		}

		void SetStateSegment(state state_segment);
		state GetStateSegment(void) const;

	protected:
		virtual void run();

	private:
		void Allocate(void);
		void Deallocate(void);

		void InitialSegmentation();
		void DynamicSegmentation();
		void ExcludeSegmentation();
		void IncludeSegmentation();

		void ExtractStrokePart(int & x, int & y, const char label, double *color);
		void EMAlgorithmPart(void);
		template<class T > void PreLabelAssign(T & fore, T & back, T & check , const int & img_width , const int & img_height);
		bool SegmentReserve(unsigned char *tmpbinary, short int *result, const int x, const int y);
		void PreDynamic(void);
		void PreAccurateDynamic(void);
	};
}

#endif /* SEGMENTATION2D_H_ */
