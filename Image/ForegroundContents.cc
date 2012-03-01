/*
 * ForegroundContents.cc
 *
 *  Created on: 2010/11/21
 *      Author: umakatsu
 *
 *	This program defines image axis below
 *	a[i][j] -> i:width , j:height
 */

#include "Image/ForegroundContents.h"
#include "Image/BmpFileIO.h"
#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <iostream>

namespace PTAMM{

	//const definition
	const int	OFFSET		=  5;  //境界線追跡の時に次に探索する方向を決めるのに使う
	const char	CHECK		= 't';
	const char	FIN			= 'f';
	const uint	MAXRESERVE	= 20;

	//ForegroundContents::ForegroundContents(const int & size_w , const int & size_h)
	ForegroundContents::ForegroundContents( int size_w , int size_h )
	: width(size_w) , height(size_h)
	{
		contour_stroke_list.clear();
		silhouette_stroke_list.clear();

		silhouette	= new uchar[ width * height ];
		contour	= new uchar[ width * height ];

		update = false;
		pData.projected = new double[ width * height ];
		pData.state_proj = NEW;
		pData_pre.projected = new double[ width * height ];
		pData_pre.state_proj = NEW;
	}

	ForegroundContents::~ForegroundContents( void ){
		contour_stroke_list.clear();
		silhouette_stroke_list.clear();
		seginfo.clear();
		delete[] pData.projected;
		delete[] pData_pre.projected;
		delete[] contour;
		delete[] silhouette;
	}

	void ForegroundContents::DrawSilhouette( const TooN::SE3<> &  currentPose , const TooN::Matrix< 4 > & projection ) const{
		glEnable(GL_POINT_SMOOTH);
		glEnable(GL_BLEND);
		glColor3f(0.0 , 1.0 , 0.0);
		glPointSize(5);
		glBegin(GL_POINTS);{
			REP(i,width*height){
				if( silhouette[i] > 0 ){
					glVertex2d( i%width , i/width);
				}
			}
		}glEnd( );
		glDisable(GL_BLEND);
		glDisable(GL_POINT_SMOOTH);
	}

	void ForegroundContents::DrawContour( const TooN::SE3<> &  currentPose , const TooN::Matrix< 4 > & projection ) const{
		glEnable(GL_POINT_SMOOTH);
		glEnable(GL_BLEND);
		glColor3f(0.0 , 1.0 , 0.0);
		glPointSize(5);
		glBegin(GL_POINTS);{
			REP(i,width*height){
				if( contour[i] > 0 ){
					glVertex2d( i%width , i/width);
				}
			}
		}glEnd( );
		glDisable(GL_BLEND);
		glDisable(GL_POINT_SMOOTH);
	}

	/*
	 * セグメンテーションデータの保持
	 */
	void ForegroundContents::pushSegmentationInfo(uchar* s , const TooN::SE3<> & current , const ImageType & img_color){
		if(seginfo.size() >= MAXRESERVE){
			SegmentationInfo * tmp = seginfo.front();
			delete tmp;
			seginfo.pop_front();
		}
		SegmentationInfo * info = new SegmentationInfo( width , height );
		REP(i,WIDTH*HEIGHT) info->silhouette[i] = s[i];
		info->camerapose	= current;
		info->imageRGB	= img_color;
		seginfo.push_back(info);
	}

	SegmentationInfo * ForegroundContents::popSegmentationInfoRef( void ) const{
		if( !seginfo.empty() ) return seginfo.back();
		else return NULL;
	}

	void ForegroundContents::popSegmentationInfo( void ){
		if( !seginfo.empty() ) seginfo.pop_back();
	}

	void ForegroundContents::outputSilhouettePPM(const char *filename)
	{
		if( seginfo.empty() ) return;
		uchar* s = seginfo.back()->silhouette;
		FILE *fw;
		fw = fopen(filename , "w");
		fprintf(fw,"P3\n%d\t%d\n255\n", width , height);
		REP(i,width * height){
			if( s[i] == 255){
				REP(data,D) fprintf(fw,"%lf\t",rgb_data[i][data]);
			}else{
				REP(data,D) fprintf(fw,"0.0\t");
			}
			fprintf(fw,"\n");
		}
		fclose(fw);
	}

	void ForegroundContents::writeBitmapfile(const char *filename, const int & w, const int & h, uchar*result )
	{
		/* RGB形式で書き込み */
		unsigned char	*result_image = new unsigned char[ w * h * 3 ];
		REP(j,h){
			REP(i,w){
				float	r, g, b;
				int n = i + (h-1-j)*w;
				if ( result[ i + j*w ] == 255) {
					r = g = b = 1.0f;
				} else {
					r = g = b = 0.0f;
				}
				result_image[ n * 3 ] = ( r * 255.0f );
				result_image[ n * 3 + 1 ] = ( g * 255.0f );
				result_image[ n * 3 + 2 ] = ( b * 255.0f );
			}
		}

		BmpFileIO	b;
		b.WriteRGB( filename, w, h, result_image );
		delete [] result_image;
	}

	TooN::SE3<> ForegroundContents::popCameraPose() const{
		assert(!seginfo.empty());
		return seginfo.back()->camerapose;
	}

	TooN::SE3<> ForegroundContents::popCameraPose(int num) const{
		assert( static_cast<int>(seginfo.size()) > num);
		return seginfo.at(num)->camerapose;
	}

	uchar* ForegroundContents::popSilhouette() const{
		if( !seginfo.empty() )	return seginfo.back()->silhouette;
		else						return NULL;
	}


	uchar* ForegroundContents::popSilhouette(int num) const{
		assert( static_cast<int>(seginfo.size()) > num);
		return seginfo.at(num)->silhouette;
	}

	ImageType 	ForegroundContents::popColor() const{
		assert(!seginfo.empty());
		return seginfo.back()->imageRGB;
	}

	ImageType	ForegroundContents::popColor(int num) const{
		assert( static_cast<int>(seginfo.size()) > num);
		return seginfo.at(num)->imageRGB;
	}

	bool ForegroundContents::isEmpty( void ) const{
		return seginfo.empty();
	}
	/*
	 * @param result : binary image (0 or 1)
	 * @param tmp : reserve contour data
	 * @param i : image height axis
	 * @param j : image width axis
	 * @param code
	 */
	std::vector< TooN::Vector< 3 > > ForegroundContents::LabelingBorder(short *result , const int & i, const int & j, const int & code)
	{
		int Istart,Jstart;	//start point
		int ic,jc;		//current point
		int is,js;		//search point
		char p[8];

		Istart = i;
		ic		= i;
		Jstart	= j;
		jc		= j;
		is	= Istart+1;
		js	= Jstart+1;

		std::vector< TooN::Vector< 3 > > tmp;
		tmp.clear();

		REP(x,8) p[x]=CHECK;
		int direction = code;
		while( (is != Istart) || (js!=Jstart) ){
			searchPixelSet(js , is , jc , ic , direction);
			if(rangeChecker(is , js)){
				int check = js + is * width;
				// Discover 1-value point
				if(result[check]==1) {
					p[direction] = CHECK;
					direction = ( direction + OFFSET) % 8;
					TooN::Vector< 3 > vertmp;
					vertmp[0] = (double)js;
					vertmp[1] = (double)is;
					vertmp[2] = 0.0;
					tmp.push_back(vertmp);
					ic = is;
					jc = js;
					REP(x,8) p[x]=CHECK;
				}
				// Discover 0-value point is not checked
				else if(result[check] != 1 && p[direction] == CHECK) {
					p[direction] = FIN;
					direction = (direction + 1 ) % 8;
				}
				// All around pixels were checked
				else{
					TooN::Vector< 3 > vertmp;
					vertmp[0] = (double)jc;
					vertmp[1] = (double)ic;
					vertmp[2] = 0.0;
					tmp.push_back(vertmp);
					return tmp;
				}
			}else{
				p[code] = FIN;
				direction = (direction + 1 ) % 8;
			}
		}
		return tmp;
	}

	void ForegroundContents::searchPixelSet(int & js , int & is , const int & jc , const int & ic , const int & code){
		// setting js value
		switch(code){
			case 0: case 4:
				js = jc; break;
			case 1: case 2: case 3:
				js = jc+1; break;
			case 5: case 6: case 7:
				js = jc-1; break;
			default:
				std::cerr << "No direction are set in J" << std::endl; break;
		}

		// setting is value
		switch(code){
			case 2: case 6:
				is = ic; break;
			case 0: case 1: case 7:
				is = ic+1; break;
			case 3: case 4: case 5:
				is = ic-1; break;
			default:
				std::cerr << "No direction are set in I" << std::endl; break;
		}
	}

	bool ForegroundContents::rangeChecker(const int & is , const int & js){
		return( is >= 0 && is < height && js >= 0 && js < width);
	}

	uint ForegroundContents::getSeginfo( void ) const{
		return seginfo.size();
	}
}
