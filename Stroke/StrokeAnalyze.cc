/*
 * StrokeAnalyze.cc
 *
 *  Created on: 2011/03/26
 *      Author: umakatsu
 */

#include "Stroke/StrokeAnalyze.h"
#include "Image/VideoData.h"
#include "Image/BmpFileIO.h"

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

namespace PTAMM{

StrokeAnalyze::StrokeAnalyze(int width , int height ) :
w(width) , h(height)
{
	vd = new VideoData( w , h);
}

StrokeAnalyze::~StrokeAnalyze( void ){
	delete vd;
}

void StrokeAnalyze::checkLetter( Stroke * stroke ){
	assert(!valid);
	double *color = new double[3];
	color[0]=0; color[1] = 255.0; color[2] = 255;
	extractStrokePart(color);
	// 外部アプリ(Tesseract)を走らせる
	std::stringstream command( "" );
	command << "tesseract stroke.bmp result";
	if( system( command.str( ).c_str( )) ){
		std::cout << "System Error in StrokeAnalyze.cc" << std::endl;
		exit(1);
	}
	std::ifstream fi("result.txt");
	std::string buf;
	std::getline(fi, buf);
	letter = buf[0];
	std::cout << letter << std::endl;
	valid = true;
//	command.clear();
//	command << "rm result.txt";
//	if( system( command.str( ).c_str( ) )){
//		std::cout << "System Error in StrokeAnalyze.cc" << std::endl;
//		exit(1);
//	}
}

bool StrokeAnalyze::isValidAnalyze( void ) const{
	return valid;
}

char StrokeAnalyze::getResult( void ){
	valid = false;
	return letter;
}

void StrokeAnalyze::extractStrokePart( double *color){
	const int W = w;
	const int H = h;
	uchar *sbuf	= new uchar[W * H * 3];
	uchar *buf	= new uchar[W * H];
	// preparing stroke image
	IplImage * src = cvLoadImage("screen.bmp", CV_LOAD_IMAGE_COLOR);
	mycvConvertRegularType(src , sbuf , 3);
	cvReleaseImage(&src);
	// extracting stroke part
	for (int i=0; i<w*h*3; i+=3) {
		if( (-(double)(sbuf[ 0 + i]) + color[0]) < 1.0
		&&	(-(double)(sbuf[ 1 + i ]) + color[1]) < 1.0
		&&	(-(double)(sbuf[ 2 +  i ]) + color[2]) <1.0
		){
			buf[i/3] = 255;
		}else{
			buf[i/3] = 0;
		}
	}
	const char * filename = "stroke.bmp";
	writeMonoBitmapfile(filename,W,H,buf);
	delete sbuf;
	delete buf;
}
}
