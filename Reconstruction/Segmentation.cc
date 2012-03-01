/*
 * Segmentation2D.cc
 *
 *  Created on: 2011/06/24
 *      Author: umakatsu
 *
 *  This file is created to describe a various of segmentaton order
 *  and segmentation part are acting as this class thread
 *  グラフカット終了後、デストラクトされているがなぜ？
 *  勉強する必要アリ。
 *  ポインタにしたらデストラクタが呼ばれなくなった。ポインタやと呼ばれない？
 *  TODO 画像ファイルをスマートポインタで扱う
*/

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>

#include "Reconstruction/Segmentation.h"
#include "Image/VideoData.h"
#include "Stroke/Stroke.h"
#include "Stroke/StrokeGenerator.h"
#include "Reconstruction/GMM.h"
#include "Image/ImageLabeling.h"
#include "Reconstruction/GraphCut.h"
#include "Image/ForegroundContents.h"

#include <boost/foreach.hpp>
#include "PTAMTracking/TrackerData.h"

//Brushの動作を記述
#include "Reconstruction/BrushAction.h"

namespace PTAMM{
	#define CHECK 0			//処理時間を出力するorしない
	#define EMCHECK 0		//処理時間を出力するorしない
	#define CHECKSUM 0 		//処理時間を出力するorしない
	#define CHANGESIZE 1		//EMの前処理で画像の縮退をするorしない
	#define OUTPUT_DEST 0	//距離画像をPPM出力するorしない

	//namespace definition
	using namespace CVD;
	using namespace std;
	namespace
	{
		template< typename T >
		void createBinaryImage(char **image_map , T * binary , const int width , const int height , int m){
			T max = static_cast< T >(m);
			REP(i , width) REP(j , height)
				if(image_map[i][j] == FOREGROUND )	binary[i + j*width] = max;
				else 									binary[i + j*width] = 0;
		}

		template< class T >
		void createFrame(T * binary , int m , int w , int h , int narrow){
			  T replace = static_cast<T>(m);
			  REP(n,narrow){
				  REP(i,w){
					  binary[i + n*w] = replace;
					  binary[i + (h-1-n)*w] = replace;
				  }
				  REP(j,h){
					  binary[n + j*w] = replace;
					  binary[j*w + (w-n-1)] = replace;
				  }
			  }
		}

		template< class T >
		vector< T > shrinkVector(vector< T > & src , const int & d){
			  assert(d>0);
			  assert(src.size()>0);
			  vector < T > det;
			  int count = 0;
			  int limit = src.size();
			  while(count < limit){
				  T tmp = src.at(count);
				  det.push_back(tmp);
				  count += d;
			  }
			  return det;
		}
		double N(const double & x , const double & mu , const double & sigma2){
			return( exp( -(x-mu)*(x-mu)/ (2 * sigma2) ) / sqrt(2*M_PI*sigma2));
		}

		//距離変換に用いる累積分布関数の式 edited by Umakatsu 2011.1.19
		double CDF(const double & x){
			return( (1 + erf((x-DIST_MU)/sqrt(2*DIST_SIGMA2)))/2);
		}
	}

	/*
	 * constructor
	 * @param width	: image size
	 * @param height	: image size
	 * @param f 		: foreground information
	 * @param cam		: camera information
	 */
	Segmentation::Segmentation(const ImageSize &  img_size , ForegroundContents *f , const ATANCamera & cam , TrackerDrawable * td)
	: w(img_size.width) , h(img_size.height) , fContents(f) , camera(cam), tracker(td)
	//,	fbo( FrameBufferObject::create( img_size ) )
	//,	pbo( PixelBufferObject< GLint >::create( img_size ) )
	{
		mpStrokeGenerator = StrokeGenerator::create( *td , camera );
		state_segment = NEW;
		interrupt		 = false;
		num_of_dynamics = 0;
		Allocate( );
	}

	Segmentation::~Segmentation( void ){
		Deallocate();
	}

	void Segmentation::run( void ){
		while(!shouldStop())  // ShouldStop is a CVD::Thread func which return true if the thread is told to exit.
		{
			if( state_segment == INIT ){
				InitialSegmentation();
				state_segment = READY;
			}
			else if(state_segment == EXCLUDE ){
				ExcludeSegmentation();
				state_segment = READY;
				interrupt = false;
			}
			else if( state_segment == INCLUDE ){
				IncludeSegmentation();
				state_segment = READY;
				interrupt = false;
			}
			else if( state_segment == DYNAMIC ){
	#if CHECKSUM
					double start = TIME();
	#endif
					DynamicSegmentation();
	#if CHECKSUM
					cout << " Sum of Time : " << PROCESSTIME(start) << "sec" << endl;
	#endif
				state_segment = READY;
				interrupt = false;
			}
			else{}
		}
	}

	void Segmentation::Start( void ){
		start(); // This CVD::thread func starts the map-maker thread with function run()
	}

	void Segmentation::Reset( void ){
		if(!shouldStop()){
			interrupt = true;
			//他の処理が中断されるまで待つ
			sleep(0.5);
			//領域の開放と再確保
			Deallocate();
			state_segment = NEW;
			interrupt = false;
			num_of_dynamics = 0;
			stop();
		}
//		allocate();
	}

	void Segmentation::InitialSegmentation()
	{
		//initializing Image Map
		REP(i,w) REP(j,h) image_map[i][j] = BACKGROUND;
		uchar *binary = new uchar[w * h];

		/* Extracting stroke part */
		int foreground_coord_x = 0;
		int foreground_coord_y = 0;
		double *color = new double[3];
		color[0]=255.0; color[1] = 255.0; color[2] = 255.0; //color : white
		/* ストローク部の抽出 */
		ExtractStrokePart( foreground_coord_x , foreground_coord_y , FOREGROUND , color);

		/* ストローク部の2値画像生成 */
		createBinaryImage(image_map , binary , this->w , this->h ,255);

		/* Classification for EM Algorithm */
		EMAlgorithmPart();

		/* Classification between foreground and background using Graph Cuts */
		GraphCut *graph = new GraphCut( w , h , CLASSNUMBER , vd);
		graph->initFirst( gauss_fore , gauss_back );
		graph->firstRun(image_map);
		delete graph;

		/* creating the binary image */
		createBinaryImage(image_map , binary , this->w , this->h ,255);
		createFrame(binary , 255 , w , h , 1);

		/* labeling */
		ImageLabeling *imglabel = new ImageLabeling(w , h , binary);
		imglabel->run();
		short *result = imglabel->getDet();

		/* ストロークがあった部分のラベリング領域を抽出する */
		short int **region_of_label = new short int*[w];
		REP(i,w) region_of_label[i] = new short int[h];
		/* ストローク領域のラベル番号を取得 */
		int result_label = result[foreground_coord_x + (foreground_coord_y) * w];
		double **rgb_data = vd->getRGB();
		fContents->setRgb_data(rgb_data);
		REP(i , h){
			REP(j , w){
				short int tmp = 0;
				/*　ストロークあるラベリング領域のみ表示 */
				if( result[j + i*w] == result_label){
					tmp = 255;
					binary[j + i*w] = 255;
				}else
					binary[j + i*w] = 0;
				/* 抽出領域のみを2値画像に書き戻す */
				region_of_label[j][i] = tmp;
			}
		}
		createFrame(binary , 0 , w , h , 2);

		/* 前景の重心座標を抽出 */
		IplImage *src_img =mycvConvertIplImage(binary , w , h , 1);
		mycvGetGravity(src_img , foreground_coord_x , foreground_coord_y);

		/* ラプラシアンを実行 */
		mycvLaplace(src_img , binary);
		cvReleaseImage(&src_img);

		/* ラベリング対象画像を再設定 */
		imglabel->setSrc(binary);
		imglabel->run();
		result = imglabel->getDet();

		/* セグメントデータを保存して、画面上に射影するための変換を行う */
		SegmentReserve(binary , result , foreground_coord_x , foreground_coord_y);

		// set to the foreground object about segmentation data
		// 変数binaryはこのメソッド内で生成されているが，ポインタをそのままプッシュするので
		// 格納先の処理でdeleteはされる必要がある．
		fContents->pushSegmentationInfo( binary , tracker->GetCurrentPose() , mimFrameRGB);

		REP(i,w) delete region_of_label[i];
		delete[] region_of_label;
		delete[] binary;
		delete imglabel;
	}

	/*
	 * content : to run automatic segmentation
	 */
	void Segmentation::DynamicSegmentation(){
	//	preAccurateDynamic();
		PreDynamic();
		if(interrupt) return;

		/* Getting raw image from current camera */
		uchar *colbuf = new uchar[w * h * 3];
		myReadRawimage(mimFrameRGB , colbuf);

		/* EM Algorithm part */
		vd->setLabel(colbuf , image_map);
		// 2011.1.23 Xフレームに1回の頻度で色分布を更新する
		const int X = 3;
		if( (num_of_dynamics % X) == 0 ){
			EMAlgorithmPart();
		}

		/* Graph Cuts */
	#if CHECK
		double start = TIME();
	#endif
		GraphCut *graph = new GraphCut( w , h , CLASSNUMBER , vd);
		graph->initRegular(gauss_fore , gauss_back , Pr_object , Pr_back);
		graph->regularRun(image_map);
		delete graph;
	#if CHECK
		cout << "  Graph Cuts\t: " << PROCESSTIME(start) << "sec"<< endl;
	#endif

		/* creating the binary image (Process Time = 0.001sec) */
		uchar *binary = new uchar[ w * h];
		createBinaryImage(image_map , binary , w , h , 255);
		createFrame(binary , 255 , w , h , 1);

		/* Labeling (Process Time = 0.002sec) */
		ImageLabeling *imglabel = new ImageLabeling( w , h , binary);
		imglabel->run();

		/* ラベリング後の面積最大領域を取得 */
		short int *_binary = imglabel->getDet();
//		Fill out the edge of image (Process Time = 5.0e-6 sec)
		createFrame(_binary , 0 , w , h , 2);
		REP(i,w*h)
			if(_binary[i] == 1) binary[i] = 255;
			else binary[i] = 0;
		IplImage * output = mycvConvertIplImage(binary , w , h , 1);

//		重心座標を取得
		int x,y;
		mycvGetGravity(output , x , y);

		//Confirm whether there is a interrupting request
		if(!interrupt){
	#if CHECK
		double segment = TIME();
	#endif
			uchar *damy = new uchar[w * h];
			if( SegmentReserve( damy , _binary , x , y) ){
				fContents->pushSegmentationInfo(binary , tracker->GetCurrentPose() , mimFrameRGB);
				fContents->setUpdate(true);
			}
			delete[] damy;
	#if CHECK
		cout << "  Segment Reserve\t:\t" << PROCESSTIME(segment) << "sec"<< endl;
	#endif
		}
		num_of_dynamics ++;

		// memory release
		cvReleaseImage(&output);
		delete[] colbuf;
		delete[] binary;
		delete imglabel;
	}

	void Segmentation::ExcludeSegmentation(){
		BrushAction *brush = new BrushAction( w , h , vd);
		double *color = new double[3];
		int dx,dy;
		// color : yellow
		color[0]=255.0; color[1] = 255.0; color[2] = 0.0;

		/* ストローク部の抽出 */
		ExtractStrokePart( dx , dy , CHECKREGION , color);
		brush->excludeAction( tracker->GetCurrentPose() , fContents->silhouette_stroke_list.back() , camera);

		// 修正後のセグメンテーション情報の保存
		fContents->pushSegmentationInfo( brush->getBinary() , tracker->GetCurrentPose() , mimFrameRGB);
		SegmentReserve(brush->getBinary() , brush->getResult() , brush->getForeground_coord_x() , brush->getForeground_coord_y());

		// EMの前処理
		PreAccurateDynamic();
		uchar *colbuf = new uchar[w * h * 3];
		myReadRawimage(mimFrameRGB , colbuf);
		this->vd->setLabel(colbuf , image_map);
		// Running EM Algorithm
		EMAlgorithmPart();
		//memory release
		delete colbuf;
		delete brush;
	}

	void Segmentation::IncludeSegmentation(){
		BrushAction *brush = new BrushAction( w , h , vd);
		double *color = new double[3];
		int dx,dy;
		// color : purple
		color[0]=255.0; color[1] = 0.0; color[2] = 255.0;

		/* ストローク部の抽出 */
		ExtractStrokePart(dx , dy , CHECKREGION , color);
		brush->includeAction( tracker->GetCurrentPose()  , fContents->silhouette_stroke_list.back() , camera);

		// 修正後のセグメンテーション情報の保存
		fContents->pushSegmentationInfo( brush->getBinary() , tracker->GetCurrentPose()  , mimFrameRGB);
		SegmentReserve(brush->getBinary() , brush->getResult() , brush->getForeground_coord_x() , brush->getForeground_coord_y());

		// EMの前処理
		PreAccurateDynamic();
		uchar *colbuf = new uchar[w * h * 3];
		myReadRawimage(mimFrameRGB , colbuf);
		vd->setLabel(colbuf , image_map);
		// Running EM Algorithm
		EMAlgorithmPart();
		//memory release
		delete colbuf;
		delete brush;
	}

	/*********************************************/
	/************* 以下，補助メソッド *************/
	/*********************************************/

	void Segmentation::ExtractStrokePart(int & x , int & y , const char label , double *color){
		const int W = w;
		const int H = h;
		uchar *buf	= new uchar[W * H * 3 ];
		uchar *sbuf	= new uchar[W * H * 3 ];
		// read color image
		myReadRawimage(mimFrameRGB , buf);
		// preparing stroke image
		IplImage * src = cvLoadImage("screen.bmp", CV_LOAD_IMAGE_COLOR);
		cvDilate(src , src , NULL , 1);
		cvDilate(src , src , NULL , 1);
		cvErode(src , src , NULL , 1);
		cvErode(src , src , NULL , 1);
		mycvConvertRegularType(src , sbuf , 3);
		// extracting stroke part
		vd->strokeDiff( sbuf , image_map , &x , &y , label , color );
		// Label Assignment
		vd->setLabel(buf , image_map);
		// memory release]
		cvReleaseImage(&src);
		delete[] sbuf;
		delete[] buf;
	}

	void Segmentation::EMAlgorithmPart(){
	#if EMCHECK
		double start = TIME();
	#endif
		bool bFore = true;
		bool bBack = true;
	#if CHANGESIZE
		/* 画像サイズを縮小する */
		double **origin_rgb = new double*[w*h];
		REP(ik,w*h) origin_rgb[ik] = new double[3];
		const int ratio = 2;
		vd->cloneRGB(origin_rgb);
		vd->imageSizeShrink(ratio);
	#endif
		vector< Vector< 3 > > col;	// 各ストローク点のRGB値
		vector< Vector< 2 > > crd;	// 各ストローク点のi,j値
		/* EM Algorithm for foreground */
		unsigned long int foreground_count = vd->extractLabel(col , crd , FOREGROUND);
		if(col.size() > MAX_EM_SAMPLES){
			int d = 1 + (col.size() / MAX_EM_SAMPLES);
			col = shrinkVector(col,d);
			crd = shrinkVector(crd,d);
			foreground_count = col.size();
		}else if( static_cast<int>(col.size()) < CLASSNUMBER ){
			bFore = false;
		}

	#if EMCHECK
		double forestart = TIME();
	#endif
		if(bFore){
			GMM * gmm_fore = new GMM( CLASSNUMBER , D , foreground_count );
			gmm_fore->Run(col , crd);
			gmm_fore->setParameter(gauss_fore);
		#if EMCHECK
			cout << "    Update foreground colour("<<foreground_count<<"): " << PROCESSTIME(forestart) << "sec"<< endl;
		#endif
			delete gmm_fore;
		}
		/* EM Algorithm for background */
		unsigned long int background_count = vd->extractLabel(col , crd , BACKGROUND);
		if(col.size() > MAX_EM_SAMPLES){
			int d = 1 + (col.size() / MAX_EM_SAMPLES);
			col = shrinkVector(col,d);
			crd = shrinkVector(crd,d);
			background_count = col.size();
		}
		//標本数が分類クラスの数より少ないならばEMを実行しない
		else if( static_cast<int>(col.size()) < CLASSNUMBER  ){
			bBack = false;
		}
		if(bBack){
			GMM * gmm_back = new GMM( CLASSNUMBER , D , background_count );
		#if EMCHECK
			double backstart = TIME();
		#endif
			gmm_back->Run( col , crd );
			gmm_back->setParameter(gauss_back);
		#if EMCHECK
			cout << "    Update background colour("<<background_count<<"): " << PROCESSTIME(backstart) << "sec"<< endl;
		#endif
			delete gmm_back;
		}

	#if CHANGESIZE
		/* 元のサイズに戻す */
		vd->imageSizeExpand(ratio , origin_rgb);
	#endif

	#if EMCHECK
		cout << "  EM Algorithm \t: " << PROCESSTIME(start) << "sec"<< endl;
	#endif
	#if CHANGESIZE
		// memory release
		REP(ik,w*h) delete origin_rgb[ik];
		delete[] origin_rgb;
	#endif
	}

	template< class T >
	void Segmentation::PreLabelAssign(T & fore , T & back , T & check , const int & img_width , const int & img_height){
		IplImage * src = mycvConvertIplImage(check , img_width , img_height , 1);
		IplImage * det_fore = cvCreateImage(cvSize(img_width,img_height), IPL_DEPTH_8U , 1);
	//	IplImage * det_back = cvCreateImage(cvSize(img_width,img_height), IPL_DEPTH_8U , 1);
		IplImage * det_check = cvCreateImage(cvSize(img_width,img_height), IPL_DEPTH_8U , 1);
	//	IplImage * calc_check = cvCreateImage(cvSize(img_width,img_height), IPL_DEPTH_8U , 1);
		/* 自己位置姿勢から計算したシルエット位置の雑音を除去 */
		cvDilate(src , src);
		cvErode(src, det_fore);
		/* 前景/背景/注目領域ごとに処理を行う */
		//現在の前景のxピクセル内側を前景と仮定
	//	cvCopy(src_sil , det_fore);
		REP(i,5) cvErode(det_fore , det_fore);
	//	cvErode(det_fore , det_fore , NULL , 1);
		mycvConvertRegularType(det_fore, fore , 1);
		//仮定した前景以外の前景領域とその外側 xピクセルを注目領域と仮定
	//	cvNot(det_fore , det_fore);
		cvCopy(src , det_check);
	//	REP(i,2) cvDilate(det_check , det_check);
		REP(i,5) cvDilate(det_check , det_check);
	//	cvDilate(det_check , det_check , NULL , 2 );
	//	cvAnd(calc_check , det_fore , det_check);
		mycvConvertRegularType(det_check , check , 1);
		//注目領域のさらに外側xピクセルを背景領域と仮定
	//	REP(i,2) cvDilate(det_check , det_check );
		REP(i,5) cvDilate(det_check , det_check);
		mycvConvertRegularType(det_check, back , 1);
		cvReleaseImage(&det_check);
		cvReleaseImage(&det_fore);
		cvReleaseImage(&src);
	}

	bool Segmentation::SegmentReserve( uchar *tmpbinary , short int  * result  , const int x, const int y){
		int fx = x;
		int fy = y;
		REP(i,w*h) tmpbinary[i] = 0;
		/* ストローク領域の輪郭ラベルのみ抽出 */
		vector< Vector< 3 > > temp;
		temp.clear();
		bool first = false;
		REP(i , w*h) {
			if(result[i] == 1){
				int tx = i / w;
				int ty = i % w;
				temp = fContents->LabelingBorder(result , tx , ty  , 7);
				first = true;
				break;
			}
		}
		REP(i , temp.size()){
			Vector< 2 > v;
			Vector< 3 > v3;
			v3 = temp.at(i);
			v[0] = v3[0]; v[1] = v3[1];
			int num = static_cast<int>(v[0]) + static_cast<int>(v[1]) * w;
			tmpbinary[num] = 255;
			if(i==0){
//				mpStrokeGenerator->Begin( tracker->GetCurrentPose() , v, tracker->GetCurrentPose() );
				mpStrokeGenerator->Begin( tracker->GetCurrentPose() , v );
			}else{
				mpStrokeGenerator->Add( tracker->GetCurrentPose() , v);
			}
		}
		if(first) {
			if(fContents->contour_stroke_list.size() < MAXRESERVE){
				fContents->contour_stroke_list.push_back(mpStrokeGenerator->End());
			}else{
				Stroke * damy = fContents->contour_stroke_list.front();
				delete damy;
				fContents->contour_stroke_list.pop_front();
				fContents->contour_stroke_list.push_back(mpStrokeGenerator->End());
			}
		}else{
			return false;
		}

		REP(i,w*h){
			fContents->contour[i] = tmpbinary[i];
		}

		IplImage * contour;
		contour = mycvConvertIplImage(tmpbinary , w , h , 1);
		cvFloodFill( contour , cvPoint(fx,fy) , CV_RGB(255,255,255));
		mycvConvertRegularType(contour , tmpbinary , 1);
		// memory release
		cvReleaseImage(&contour);

		REP(i,w*h){
			fContents->silhouette[i] = tmpbinary[i];
		}

	//	FILE *f_stroke = fopen("result_after.ppm","w");
	//	fprintf(f_stroke,"P3\n%d\t%d\n255\n",w,h);
	//	REP(j , h*w){
	//			if(tmpbinary[j] == 255) fprintf(f_stroke , "255\t255\t255\n");
	//			else fprintf(f_stroke , "0\t0\t0\n");
	//	}
	//	fclose(f_stroke);
		first = false;
		REP(i,w*h) {
			if(tmpbinary[i] == 255){
				Vector< 2 > v; v[0] = i%w; v[1] = i/w;
				if(!first){
	//				mpStrokeGenerator->Begin( tracker->GetCurrentPose() , v, tracker->GetCurrentPose() );
					mpStrokeGenerator->Begin( tracker->GetCurrentPose() , v );
				}else{
					mpStrokeGenerator->Add( tracker->GetCurrentPose() , v);
				}
				first = true;
			}
		}
		if(first){
			if(fContents->silhouette_stroke_list.size() < MAXRESERVE){
				fContents->silhouette_stroke_list.push_back(mpStrokeGenerator->End());
			}else{
				Stroke * damy = fContents->silhouette_stroke_list.front();
				delete damy;
				fContents->silhouette_stroke_list.pop_front();
				fContents->silhouette_stroke_list.push_back(mpStrokeGenerator->End());
			}
		}
		else{
			return false;
		}
		return true;
	}

	/*
	 * Preprocessing of Dynamic segmentation (Process Time : about 0.15sec)
	 */
	void Segmentation::PreDynamic( void ){
	#if CHECK
		double start = TIME();
	#endif
		//画像サイズを縮退させる
		const int width = w;
		const int height= h;
		w /= ratio;
		h /= ratio;
		uchar * damy = new uchar[width * height];
		//Create the memory space
		uchar * fore = new uchar[w * h];
		uchar * back = new uchar[w * h];
		uchar * sbuf = new uchar[w * h];
		uchar * Pr_o = new uchar[w * h];
		uchar * Pr_b = new uchar[w * h];
	#if CHECK
		double start_pro = TIME();
	#endif
		//Get the current silhouette data
		vd->BackProjectionFromWorldToImage(damy , tracker->GetCurrentPose( ) , fContents->silhouette_stroke_list.back() , camera);
		//Copy silhouette data from original size to shrinked size
	#if CHECK
		cout << "  PreProcessing(Proj) \t: " << PROCESSTIME(start_pro) << "sec"<< endl;
	#endif
	#if OUTPUT_DEST
		FILE *fw= fopen("object.ppm","w");
		fprintf(fw,"P3\n%d\t%d\n255\n",width,height);
	#endif

	//	if( contents->pData_pre.state_proj == READY ){
		if( false ){
			IplImage * src = mycvConvertIplImage(damy , width , height, 1);
			cvDilate(src , src);
			cvErode(src , src );
			mycvConvertRegularType(src , damy , 1);
			cvReleaseImage(&src);
			uchar * _fore = new uchar[width * height];
			uchar * _back = new uchar[width * height];
			uchar * _Pr_o = new uchar[width * height];
			//Begin reading projected data from a set of voxels
			fContents->pData_pre.state_proj = RUN;
			PreLabelAssign( _fore , _back , damy , width , height );
			REP(i , width*height){
				// Label assign
				if( _fore[i] == 255){
					image_map[i%width][i/width] = FOREGROUND;
				}
				else if( damy[i] == 255) {
					image_map[i%width][i/width] = CHECKREGION;
				}
				else if(_back[i] == 255){
					image_map[i%width][i/width] = BACKGROUND;
				}
				else{
					image_map[i%width][i/width] = NOLABEL;
				}
				_Pr_o[i] = static_cast< uchar > (fContents->pData_pre.projected[i]*255);
			}
			//Finish reading projected data from a set of voxels
			fContents->pData_pre.state_proj = READY;
			IplImage *src_gauss = mycvConvertIplImage(_Pr_o , width , height , 1);
			IplImage *dst_gauss = cvCreateImage (cvSize (src_gauss->width, src_gauss->height), IPL_DEPTH_8U, 1);
			cvSmooth(src_gauss , dst_gauss , CV_GAUSSIAN , 11 , 0 , 0 , 0);
			mycvConvertRegularType(dst_gauss , _Pr_o , 1);
			cvReleaseImage(&src_gauss);
			cvReleaseImage(&dst_gauss);
			double tmp;
			REP(i,width*height){
				tmp = static_cast< double >(_Pr_o[i]) / 255;
				if( tmp > 1.0 ) 		Pr_object[i] = 1.0;
				else if( tmp < 0.0 ) Pr_object[i] = 0.0;
				else					Pr_object[i] = tmp;
				Pr_back[i] = 1 - Pr_object[i];
			}
	#if 0
			// ラベル割り当て結果を画像に出力
			FILE *f_stroke = fopen("PRERABEL.ppm","w");
			fprintf(f_stroke,"P3\n%d\t%d\n255\n",width,height);
			REP(j , height){
				REP(i , width){
					if(image_map[i][j] == FOREGROUND) fprintf(f_stroke , "255\t255\t255\n");
					else if(image_map[i][j] == CHECKREGION) fprintf(f_stroke , "255\t255\t0.0\n");
					else if(image_map[i][j] == BACKGROUND) fprintf(f_stroke , "255\t0.0\t0.0\n");
					else fprintf(f_stroke , "0\t0\t0\n");
				}
			}
			fclose(f_stroke);
			FILE *fo= fopen("Probability_object.ppm","w");
			fprintf(fo,"P3\n%d\t%d\n255\n",width,height);
			FILE *fb= fopen("Probability_background.ppm","w");
			fprintf(fb,"P3\n%d\t%d\n255\n",width,height);
			REP(address,width*height){
				REP(k,3) fprintf(fo,"%lf\t",Pr_object[address]*255);
				REP(k,3) fprintf(fb,"%lf\t",Pr_back[address]*255);
				fprintf(fo,"\n");
				fprintf(fb,"\n");
			}
			fclose(fo);
			fclose(fb);
			cout << " Probability map was created" <<	 endl;
			exit(1);
	#endif
			delete[] _fore;
			delete[] _back;
			delete[] _Pr_o;
		}else{
			int tmp;
			for(int j=0; j < height; j+=ratio){
				for(int i=0; i < width; i+=ratio){
					tmp = static_cast<int>(damy[i + j * width])
						 + static_cast<int>(damy[i + (j+1) * width])
						 + static_cast<int>(damy[(i+1) + j * width])
						 + static_cast<int>(damy[(i+1) + (j+1) * width] );
					tmp /= (ratio*ratio);
					sbuf[(i / ratio) + (j / ratio) * w] = static_cast<uchar>(tmp);
				}
			}
			IplImage * src = mycvConvertIplImage(sbuf , w , h , 1);
			cvDilate(src , src);
			cvErode(src , src );
			mycvConvertRegularType(src , sbuf , 1);
			cvReleaseImage(&src);
			// creating binary image to make distance image
			REP(i,w*h){
				if(sbuf[i] == 255) {
					Pr_o[i] = 255;
					Pr_b[i] = 0;
				}else{
					Pr_o[i] = 0;
					Pr_b[i] = 255;
				}
			}
		#if CHECK
			double start_assign = TIME();
		#endif
			//前景・探索領域・背景のそれぞれのラベル割り当て (Process Time : about 0.05sec)
			PreLabelAssign(fore , back , sbuf , w , h);
		#if CHECK
			cout << "  PreProcessing(Label)\t: " << PROCESSTIME(start_assign) << "sec"<< endl;
		#endif

		#if CHECK
			double start_dist = TIME();
		#endif
			//距離変換と正規化
			IplImage * src_img = mycvConvertIplImage(Pr_o , w , h , 1);
			IplImage * dst_img = cvCreateImage (cvSize (src_img->width, src_img->height), IPL_DEPTH_32F, 1);
			IplImage * dst_img_norm = cvCreateImage (cvSize (src_img->width, src_img->height), IPL_DEPTH_8U, 1);
			cvDistTransform (src_img , dst_img, CV_DIST_L2, 3, NULL, NULL);
			cvNormalize (dst_img, dst_img_norm, DIST_MIN , DIST_MAX , CV_MINMAX, NULL);
			mycvConvertRegularType(dst_img_norm , Pr_o , 1);
			cvReleaseImage(&src_img);
			src_img = mycvConvertIplImage(Pr_b , w , h , 1);
			cvDistTransform (src_img , dst_img, CV_DIST_L2, 3, NULL, NULL);
			cvNormalize (dst_img, dst_img_norm, DIST_MIN , DIST_MAX , CV_MINMAX, NULL);
			mycvConvertRegularType(dst_img_norm , Pr_b , 1);
			cvReleaseImage(&src_img);
			cvReleaseImage(&dst_img);
			cvReleaseImage(&dst_img_norm);

		#if CHECK
			cout << "  PreProcessing(Dist) \t: " << PROCESSTIME(start_dist) << "sec"<< endl;
		#endif

			//距離画像から確率分布を生成
			//同時にEMとGraph Cutsの領域の生成
		#if OUTPUT_DEST
			FILE *fo= fopen("Probability_object.ppm","w");
			fprintf(fo,"P3\n%d\t%d\n255\n",width,height);
			FILE *fb= fopen("Probability_background.ppm","w");
			fprintf(fb,"P3\n%d\t%d\n255\n",width,height);
		#endif
	#if CHECK
			double start_calc = TIME();
	#endif
			int address , address2 , x , y;
			double *po = new double[w*h];
			double *pb = new double[w*h];
			REP( j , h) REP( i , w){
				//確率分布生成
				address = i + j * w;
				po[address]	= CDF(static_cast<double>(Pr_o[address])/DIST_MAX);
				pb[address] 	= CDF(static_cast<double>(Pr_b[address])/DIST_MAX);
	//			po[address]	= sigmoid(static_cast<double>(Pr_o[address])/DIST_MAX);
	//			pb[address] 	= sigmoid(static_cast<double>(Pr_b[address])/DIST_MAX);
				if(po[address] > pb[address]);
				else po[address] = 1 - pb[address];
				pb[address] = 1 - po[address];
				REP(ii,ratio) REP(jj,ratio){
					x = (i*ratio + ii);
					y = (j*ratio + jj);
					//	領域生成
					if(fore[address] == 255){
						image_map[x][y] = FOREGROUND;
					}
					else if(sbuf[address] == 255) {
						image_map[x][y] = CHECKREGION;
					}
					else if(back[address] == 255){
						image_map[x][y] = BACKGROUND;
					}
					else{
						image_map[x][y] = NOLABEL;
					}
					address2 = x + y * width;
					Pr_object[address2]	= po[address];
					Pr_back[address2]		= pb[address];
				}
			}
			delete[] po;
			delete[] pb;
	#if CHECK
			cout << "  PreProcessing(Calc) \t: " << PROCESSTIME(start_calc) << "sec"<< endl;
	#endif
		#if OUTPUT_DEST
			REP(address,width*height){
				REP(k,3) fprintf(fo,"%lf\t",Pr_object[address]*255);
				REP(k,3) fprintf(fb,"%lf\t",Pr_back[address]*255);
				fprintf(fo,"\n");
				fprintf(fb,"\n");
			}
			fclose(fo);
			fclose(fb);
			cout << "Distance image end" << endl;
			exit(1);
		#endif
		}
		// ラベル割り当て結果を画像に出力
		w = width;
		h = height;
		// memory release
		delete[] fore;
		delete[] back;
		delete[] sbuf;
		delete[] Pr_o;
		delete[] Pr_b;
		delete[] damy;
	#if CHECK
		cout << "  PreProcessing(ALL) \t: " << PROCESSTIME(start) << "sec"<< endl;
	#endif
	}

	/*
	 * Preprocessing of Dynamic segmentation (Process Time : about 0.15sec)
	 */
	void Segmentation::PreAccurateDynamic( void ){
	#if CHECK
		double start = TIME();
	#endif
		//Create the memory space
		uchar * fore = new uchar[w * h];
		uchar * back = new uchar[w * h];
		uchar * sbuf = new uchar[w * h];
		uchar * Pr_o = new uchar[w * h];
		uchar * Pr_b = new uchar[w * h];
	#if CHECK
		double start_pro = TIME();
	#endif
		//Get the current silhouette data
		vd->BackProjectionFromWorldToImage(sbuf , tracker->GetCurrentPose(), fContents->silhouette_stroke_list.back() , camera);
		IplImage * src = mycvConvertIplImage(sbuf , w , h , 1);
		cvDilate(src , src);
		cvErode(src , src );
		mycvConvertRegularType(src , sbuf , 1);
	#if CHECK
		cout << "  PreProcessing(Proj) \t: " << PROCESSTIME(start_pro) << "sec"<< endl;
	#endif
	#if OUTPUT_DEST
		FILE *fw= fopen("object.ppm","w");
		fprintf(fw,"P3\n%d\t%d\n255\n", w , h );
	#endif

	//	if( contents->pData.state_proj == IDLE ){
		if( false ){
			//Begin reading projected data from a set of voxels
			fContents->pData.state_proj = RUN;
			PreLabelAssign(fore , back , sbuf , w , h);
	//		FILE *fw= fopen("object.ppm","w");
	//		fprintf(fw,"P3\n%d\t%d\n255\n",w,h);
			REP(i , w*h){
				// Label assign
				if(fore[i] == 255){
					image_map[i%w][i/w] = FOREGROUND;
				}
				else if(sbuf[i] == 255) {
					image_map[i%w][i/w] = CHECKREGION;
				}
				else if(back[i] == 255){
					image_map[i%w][i/w] = BACKGROUND;
				}
				else{
					image_map[i%w][i/w] = NOLABEL;
				}
				Pr_o[i] = static_cast< uchar > (fContents->pData.projected[i]*255);
	//			if( Pr_o[i] > 0) fprintf(fw ,"%lf\t%lf\t%lf\n", ((double)Pr_o[i]) * 255 , ((double)Pr_o[i]) * 255 , ((double)Pr_o[i]) * 255 );
	//			else fprintf(fw ,"0\t0\t0\n");
			}
	//		fclose(fw);
	//		exit(1);
			//Finish reading projected data from a set of voxels
			fContents->pData.state_proj = IDLE;
			IplImage *src_gauss = mycvConvertIplImage(Pr_o , w , h , 1);
			IplImage *dst_gauss = cvCreateImage (cvSize (src_gauss->width, src_gauss->height), IPL_DEPTH_8U, 1);
			cvSmooth(src_gauss , dst_gauss);
			mycvConvertRegularType(dst_gauss , Pr_o , 1);
			cvReleaseImage(&src_gauss);
			cvReleaseImage(&dst_gauss);
			REP(i,w*h){
				Pr_object[i] = static_cast< double >(Pr_o[i]) / 255;
				Pr_back[i] = 1 - Pr_object[i];
			}
		}else{
			// creating binary image to make distance image
			REP(i,w*h){
				if(sbuf[i] == 255) {
					Pr_o[i] = 255;
					Pr_b[i] = 0;
				}else{
					Pr_o[i] = 0;
					Pr_b[i] = 255;
				}
		#if OUTPUT_DEST
				if(sbuf[i] == 255) fprintf(fw ,"255\t255\t255\n");
				else fprintf(fw ,"0\t0\t0\n");
		#endif
			}
		#if OUTPUT_DEST
			fclose(fw);
		#endif
		#if CHECK
			double start_assign = TIME();
		#endif
			//前景・探索領域・背景のそれぞれのラベル割り当て (Process Time : about 0.05sec)
			PreLabelAssign(fore , back , sbuf , w , h);
		#if CHECK
			cout << "  PreProcessing(Label)\t: " << PROCESSTIME(start_assign) << "sec"<< endl;
		#endif

	//		// ラベル割り当て結果を画像に出力
	//		FILE *f_stroke = fopen("PRERABEL.ppm","w");
	//		fprintf(f_stroke,"P3\n%d\t%d\n255\n",w,h);
	//		REP(j , h){
	//			REP(i , w){
	//				if(image_map[i][j] == FOREGROUND) fprintf(f_stroke , "255\t255\t255\n");
	//				else if(image_map[i][j] == CHECKREGION) fprintf(f_stroke , "255\t255\t0.0\n");
	//				else if(image_map[i][j] == BACKGROUND) fprintf(f_stroke , "255\t0.0\t0.0\n");
	//				else fprintf(f_stroke , "0\t0\t0\n");
	//			}
	//		}
	//		fclose(f_stroke);

		#if CHECK
			double start_dist = TIME();
		#endif
			//距離変換と正規化
			IplImage * src_img = mycvConvertIplImage(Pr_o , w , h , 1);
			IplImage * dst_img = cvCreateImage (cvSize (src_img->width, src_img->height), IPL_DEPTH_32F, 1);
			IplImage * dst_img_norm = cvCreateImage (cvSize (src_img->width, src_img->height), IPL_DEPTH_8U, 1);
			cvDistTransform (src_img , dst_img, CV_DIST_L2, 3, NULL, NULL);
			cvNormalize (dst_img, dst_img_norm, DIST_MIN , DIST_MAX , CV_MINMAX, NULL);
			mycvConvertRegularType(dst_img_norm , Pr_o , 1);
			cvReleaseImage(&src_img);
			src_img = mycvConvertIplImage(Pr_b , w , h , 1);
			cvDistTransform (src_img , dst_img, CV_DIST_L2, 3, NULL, NULL);
			cvNormalize (dst_img, dst_img_norm, DIST_MIN , DIST_MAX , CV_MINMAX, NULL);
			mycvConvertRegularType(dst_img_norm , Pr_b , 1);
			cvReleaseImage(&src_img);
			cvReleaseImage(&dst_img);
			cvReleaseImage(&dst_img_norm);

		#if CHECK
			cout << "  PreProcessing(Dist) \t: " << PROCESSTIME(start_dist) << "sec"<< endl;
		#endif

			//距離画像から確率分布を生成
			//同時にEMとGraph Cutsの領域の生成
		#if OUTPUT_DEST
			FILE *fo= fopen("Probability_object.ppm","w");
			fprintf(fo,"P3\n%d\t%d\n255\n", w , h);
			FILE *fb= fopen("Probability_background.ppm","w");
			fprintf(fb,"P3\n%d\t%d\n255\n", w , h);
			for(int val = 0; val <= DIST_MAX; val+=50){
				printf("value%d = %lf\n",val , CDF(2*val/DIST_MAX));
			}
		#endif
	#if CHECK
			double start_calc = TIME();
	#endif
			REP(i , w*h){
		//		領域生成
				if(fore[i] == 255){
					image_map[i%w][i/w] = FOREGROUND;
				}
				else if(sbuf[i] == 255) {
					image_map[i%w][i/w] = CHECKREGION;
				}
				else if(back[i] == 255){
					image_map[i%w][i/w] = BACKGROUND;
				}
				else{
					image_map[i%w][i/w] = NOLABEL;
				}
				//確率分布生成
				Pr_object[i]	= CDF(static_cast<double>(Pr_o[i])/DIST_MAX);
				Pr_back[i] 	= CDF(static_cast<double>(Pr_b[i])/DIST_MAX);
				if(Pr_object[i] > Pr_back[i]);
				else Pr_object[i] = 1 - Pr_back[i];
				Pr_back[i] = 1 - Pr_object[i];
		#if OUTPUT_DEST
				REP(j,3) fprintf(fo,"%lf\t",Pr_object[i]*255);
				REP(j,3) fprintf(fb,"%lf\t",Pr_back[i]*255);
				fprintf(fo,"\n");
				fprintf(fb,"\n");
		#endif
			}
	#if CHECK
			cout << "  PreProcessing(Calc) \t: " << PROCESSTIME(start_calc) << "sec"<< endl;
	#endif
		#if OUTPUT_DEST
			fclose(fo);
			fclose(fb);
			cout << "Distance image end" << endl;
			exit(1);
		#endif
		}
		// memory release
		delete[] fore;
		delete[] back;
		delete[] sbuf;
		delete[] Pr_o;
		delete[] Pr_b;
		cvReleaseImage(&src);

	#if CHECK
		cout << "  PreProcessing(ALL) \t: " << PROCESSTIME(start) << "sec"<< endl;
	#endif
	}

	void Segmentation::Allocate( void ){
		image_map = new char*[w];
		REP(i,w) image_map[i] = new char[h];
		Pr_object = new double[w * h];
		Pr_back = new double[w * h];
		vd = new VideoData( w , h);
		gauss_fore = new GaussDistribution*[CLASSNUMBER];
		gauss_back = new GaussDistribution*[CLASSNUMBER];
		REP(i,CLASSNUMBER){
			gauss_fore[i] = new GaussDistribution(D);
			gauss_back[i] = new GaussDistribution(D);
		}
	}

	void Segmentation::Deallocate( void ){
		REP(i,CLASSNUMBER){
			delete gauss_fore[i];
			delete gauss_back[i];
		}
		delete[] gauss_back;
		delete[] gauss_fore;
		delete vd;
		delete[] Pr_back;
		delete[] Pr_object;
		REP(i,w) delete image_map[i];
		delete[] image_map;
	}

	/* **************** Getter , Setter *************** */
	void Segmentation::SetStateSegment( state state_segment ){
		this->state_segment = state_segment;
	}

	state Segmentation::GetStateSegment( void )const{
		return state_segment;
	}
}
