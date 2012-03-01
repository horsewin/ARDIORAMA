/*
 * myCVmethod.h
 *
 *  Created on: 2010/12/05
 *      Author: umakatsu
 */

#ifndef MYCVMETHOD_H_
#define MYCVMETHOD_H_

#include <cv.h>
#include <highgui.h>
#include "ImageType.h"
#include <cxtypes.h>
#include <assert.h>
#include <typeinfo>

using namespace CVD;

template< class T >
inline void mycvGetGravity(IplImage * src_img , T & gravity_x , T & gravity_y){
	CvMoments moments;
	cvMoments (src_img, &moments, 0);
	gravity_x = static_cast<T>(moments.m10/moments.m00);
	gravity_y = static_cast<T>(moments.m01/moments.m00);
}

template < class T >
inline void mycvLaplace(IplImage * src_img , T * binary , const int band = 2){
	IplImage *dst_img = cvCreateImage(cvSize(src_img->width , src_img->height) , IPL_DEPTH_16S , 1);
	IplImage *dst = cvCreateImage(cvSize(src_img->width , src_img->height) , IPL_DEPTH_8U , 1);
	cvCopy(src_img , dst);
	cvLaplace(src_img , dst_img , 1);
	cvConvertScaleAbs(dst_img , src_img);
	if( band == 1){
		cvAnd(src_img , dst , src_img);
	}
	mycvConvertRegularType(src_img , binary , 1);
	cvReleaseImage(&dst_img);
	cvReleaseImage(&dst);
}

inline void mycvshow(IplImage * object  , const char *window){
	cvNamedWindow (window , CV_WINDOW_AUTOSIZE);
	cvShowImage (window , object);
	cvWaitKey(0); // ０秒待つ => ずっと入力待ち
	cvDestroyWindow(window);
}

template<typename tname>
inline void myReadRawimage(ImageType mimFrameRGB , tname * dst_image){
//	ImageType mimFrameRGB_copy;	// frame clone
//	mimFrameRGB_copy.copy_from(mimFrameRGB);// copy original frame
//	const int image_width = mimFrameRGB_copy.size().x;
//	const int image_height = mimFrameRGB_copy.size().y;
	const int image_width = mimFrameRGB.size().x;
	const int image_height = mimFrameRGB.size().y;
	int datasize2 =  image_width * image_height * 3;  // ppm(カラー)用
	int line , row;
	for(int i = 0; i < datasize2; i+=3){  // カラー画像
		line = div(i/3, image_width).quot;
		row = div(i/3, image_width).rem;
//		dst_image[i] = mimFrameRGB_copy[line][row].red;
//		dst_image[i + 1] = mimFrameRGB_copy[line][row].green;
//		dst_image[i + 2] = mimFrameRGB_copy[line][row].blue;
		dst_image[i] = mimFrameRGB[line][row].red;
		dst_image[i + 1] = mimFrameRGB[line][row].green;
		dst_image[i + 2] = mimFrameRGB[line][row].blue;
	}
//	int num = 0;
//	for(int y=0; y < image_height; y++){
//		for(int x=0; x < image_width; x++){
//			num = x + y*image_width;
//			dst_image[num] 	  = static_cast< tname >(mimFrameRGB_copy[y][x].red);
//			dst_image[num + 1] = static_cast< tname >(mimFrameRGB_copy[y][x].green);
//			dst_image[num + 2] = static_cast< tname >(mimFrameRGB_copy[y][x].blue);
//		}
//	}
}

template<typename tname>
inline void myReadRawimageReverse(ImageType mimFrameRGB , tname * dst_image){
	const int image_width = mimFrameRGB.size().x;
	const int image_height = mimFrameRGB.size().y;
	int datasize2 =  image_width * image_height * 3;  // ppm(カラー)用
	int line , row;
	for(int i = 0; i < datasize2; i+=3){  // カラー画像
		line = div(i/3, image_width).quot;
		row = div(i/3, image_width).rem;
		dst_image[ (image_height - line ) * image_width + row + 2] = mimFrameRGB[line][row].red;
		dst_image[ (image_height - line ) * image_width + row + 1] = mimFrameRGB[line][row].green;
		dst_image[ (image_height - line ) * image_width + row + 0] = mimFrameRGB[line][row].blue;
	}
}

/**
 * @param input  : convert opencv data
 * @param dst_image : converted regular type data
 * @param image_width : image width
 * @param image_height : image height
 * @param channel : color channel
 */

template < class T >
inline void mycvConvertRegularType(IplImage * input , T * dst_image, const int channel){
	assert(input);
	assert(dst_image);
	const int image_width = input->width;
	const int image_height = input->height;
//	assert(input->imageSize == (image_width * image_height * channel));
	T p[3];
	int widthstep = input->widthStep;
	for(int y=0; y<image_height; y++) {
		for(int x=0; x<image_width; x++) {
			if(channel == 3){
				p[0] = input->imageData[ widthstep * y + x * channel + 2];    // R
				p[1] = input->imageData[ widthstep * y + x * channel + 1];    // G
				p[2] = input->imageData[ widthstep  * y + x * channel + 0];    // B
			}else if(channel == 1){
				p[0] = input->imageData[ widthstep * y + x * channel];    // intensity
			}
			/* 出力ファイルに書き込む */
			for(int id = 0; id < channel; id++){
				dst_image[x*channel + y * widthstep + id] = p[id];
			}
		}
	}
}

/**
 * @param input  : convert regular type data
 * @param dst_image : converted opencv data
 * @param image_width : image width
 * @param image_height : image height
 * @param channel : color channel
 */
inline IplImage* mycvConvertIplImage(const unsigned char * src , const int image_width , const int image_height , const int channel){
	assert( channel < 5);
	assert(src);
	IplImage * dst_image;
	dst_image = cvCreateImage(cvSize(image_width,image_height) , IPL_DEPTH_8U , channel);
//	const unsigned char * src = input;
//	const unsigned char * src = reinterpret_cast< unsigned char * >(input);
	assert(dst_image->imageSize == (image_width * image_height * channel));
	for(int y=0; y<image_height; y++) for(int x=0; x<image_width; x++){
		if( channel >= 3 ){
			dst_image->imageData[ ( y * dst_image->widthStep) + (x * channel) + 2] = src[ ( x + y * image_width) * channel + 0];	//R
			dst_image->imageData[ ( y * dst_image->widthStep) + (x * channel) + 1] = src[ ( x + y * image_width) * channel + 1];	//G
			dst_image->imageData[ ( y * dst_image->widthStep) + (x * channel) + 0] = src[ ( x + y * image_width) * channel + 2];	//B
			for(int d = 3; d < channel; d++)
				dst_image->imageData[ ( y * dst_image->widthStep) + (x * channel) + d] = src[ ( x + y * image_width) * channel + d];	//B
		}else if(channel == 1){
			dst_image->imageData[ ( y * dst_image->widthStep) + x] = src[ ( x + y * image_width)];
		}else{
			std::cout<< "Channel should take the value of 1 or 3 or 4" << std::endl;
		}
	}
	return dst_image;
}

#endif /* MYCVMETHOD_H_ */
