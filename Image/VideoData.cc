/*
 * VideoData.cc
 *
 *  Created on: 2010/11/12
 *      Author: umakatsu
 */

#include "Image/VideoData.h"
#include "Stroke/Stroke.h"

#include <stdio.h>
#include <math.h>
#include <boost/foreach.hpp>
#include <boost/array.hpp>
#include "myVector.h"

#include <iostream>

namespace PTAMM{
	using namespace std;

	VideoData::VideoData(const int & w , const int & h)
	:width(w) , height(h)
	{
		int datasize = width * height;
		RGB	= new double*[datasize];
		REP(i,datasize){
			RGB[i]	= new double[3];
		}
		img = new char[datasize];
	}

	VideoData::~VideoData(){
		delete[] img;
		int datasize = width * height;
		REP(ik , datasize){
			delete RGB[ik];
		}
		delete[] RGB;
		cout << "Video data Delete" << endl;
	}

	/* 静止画像からRGB値を読み取るメソッド */
	void VideoData::store(unsigned char *buf , GLenum e , const char * filename)
	{
		/* 画面のRGB値の読み取り */
	//	unsigned long int datasize = width * height;
		glReadBuffer(e);
		glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, buf);
		/* 座標系をY軸方向で反転させる */
		REP(i , height/2){
			REP(j,width){
				REP(k,3){
					swap(buf[(i*width + j)*3 + k] ,
							buf[( ((height-1) - i ) * width + j ) * 3 + k]);
				}
			}
		}
		if(filename != NULL){
			ofstream fo(filename);
			REP(j,height){
				REP(i,width){
					REP(k,3) fo << (double)buf[(j*width+i)*3+k] << " ";
					fo << endl;
				}
			}
		}
	}

	/*
	 * 画像データを読み込んで、メンバ変数にラベルと、RGB値を格納する
	 */
	void VideoData::setLabel(unsigned char *buf , char **image_map)
	{
		assert(buf);
		assert(image_map);
		REP( i , height){
			REP ( j, width ){
				/* RGB値の格納 */
				REP(k,3) RGB[i*width + j ][k] = (double)(buf[ k + (i*width + j) * 3 ]);
				/* 前景/背景情報の格納 */
				img[i*width + j] = image_map[j][i];
			}
		}
	}

	void VideoData::strokeDiff(unsigned char *buf , char **image_map,  int *x , int *y , const char label , double *color){
		if(color == NULL){
			color = new double[3];
			color[0] = 255.0; color[1] = 255.0; color[2] = 255.0;
		}
		REP( i, height){
			REP( j , width) {
				if( (-(double)(buf[ 0 + (i*width + j) * 3 ]) + color[0]) < 1.0
				&&	(-(double)(buf[ 1 + (i*width + j) * 3 ]) + color[1]) < 1.0
				&&	(-(double)(buf[ 2 + (i*width + j) * 3 ]) + color[2]) <1.0
				){
					image_map[j][i] = label;
					*x = j ; *y = i;
				}else{
					image_map[j][i] = BACKGROUND;
				}
			}
		}
		delete[] color;
	}

	/*
	 * col : RGB値
	 * crd : ピクセル座標値
	 */
	int VideoData::extractLabel(
			vector< Vector< 3 > > & col,
			vector< Vector< 2 > > & crd,
			char label)
	{
		col.clear();
		crd.clear();
		int count = 0;
		REP( j , height){
			REP ( i, width ){
				if(this->img[j*width + i] == label){
					Vector< 3 > col_tmp;
					Vector< 2 > crd_tmp;
					REP( k , 3 ) col_tmp[k] = RGB[j * width + i][k];
					crd_tmp[0]=i; crd_tmp[1]=j;
					col.push_back(col_tmp);
					crd.push_back(crd_tmp);
					count++;
				}
			}
		}
		return count;
	}

	//template< class T >
	//void VideoData::replaceLabel(T op , T * buf , char replabel , char exlabel){
	void VideoData::replaceLabel(unsigned char  op , unsigned char  * buf , char replabel , char exlabel){
		REP(n , width * height){
			if(img[n] != exlabel) if(buf[n] == op) img[n] =replabel;
		}
	}

	void VideoData::imageSizeShrink(const int ratio){
		assert(RGB);
		assert( (width  % ratio) == 0);
		assert( (height % ratio) == 0);


		double **tmp_rgb;
		int datasize = width * height / (ratio*ratio);
		tmp_rgb	= new double*[datasize];
		REP(i,datasize){
			tmp_rgb[i]	= new double[3];
		}

		/* the pixel have the ratio x ratio pixel mean */
		REP( j , height/ratio){
			REP( i , width/ratio){
				REP(c,3) tmp_rgb[ i + j*width/ratio][c] = 0.0;
				REP(jj , ratio){
					REP(ii, ratio){
						REP( c , 3 ) tmp_rgb[ i + j*width/ratio][c] += RGB[ (i + j*width)*ratio + ii + jj*width][c];
					}
				}
				REP(c,3) tmp_rgb[ i + j*width/ratio][c] /= (ratio * ratio);
			}
		}
		/* memory release */
		REP(ik,width * height) delete RGB[ik];
		delete[] RGB;
	//	RGB	= new double*[datasize];
	//	REP(i,datasize){
	//		RGB[i]	= new double[3];
	//	}
		/* setting pointer */
		RGB = tmp_rgb;
		labelSizeShrink(ratio);
	}

	void VideoData::labelSizeShrink(const int ratio){
		assert(img);
		char *tmp_img;
		int datasize = width * height / (ratio*ratio);
		tmp_img = new char[datasize];

		/* the pixel have the ratio x ratio pixel mean */
		/* the pixel have the ratio x ratio pixel mean */
		REP( j , height/ratio){
			REP( i , width/ratio){
				tmp_img[ i + j*width/ratio] = BACKGROUND;
				bool label = false;
				for(int jj=0; jj<ratio && !label; jj++){
					for(int ii=0; ii<ratio && !label; ii++){
						if( img[ (i + j*width)*ratio + ii + jj*width] == FOREGROUND){
							tmp_img[ i + j*width/ratio] = FOREGROUND;
							label = true;
						}else if( img[ (i + j*width)*ratio + ii + jj*width] == CHECKREGION){
							tmp_img[ i + j*width/ratio] = CHECKREGION;
							label = true;
						}else if( img[ (i + j*width)*ratio + ii + jj*width] == NOLABEL){
							tmp_img[ i + j*width/ratio] = NOLABEL;
							label = true;
						}
					}
				}
			}
		}

		/* size shrink */
		width 	/= ratio;
		height /= ratio;
		delete[] img;
	//	img	= new char[datasize];
		img = tmp_img;
	}

	void VideoData::imageSizeExpand(const int ratio , double **origin_RGB){
		assert(RGB);
		assert(origin_RGB);
		double **tmp_rgb;
		int datasize = width * height * (ratio * ratio);
		tmp_rgb	= new double*[datasize];
		REP(i,datasize){
			tmp_rgb[i]	= new double[3];
		}
		/* コピーしたいRGBデータがないならば、ratio x ratio の領域は元の領域の値をコピーする */
		if(origin_RGB == NULL){
			/* the pixel have the ratio x ratio pixel mean */
			REP( j , height){
				REP( i , width){
					REP(jj , ratio){
						REP(ii, ratio){
							REP( c , 3 ) tmp_rgb[i*ratio + j*width*ratio*ratio+ ii + jj*width*ratio][c] = RGB[i + j*width][c];
						}
					}
				}
			}
		}else{
			REP(j,height*ratio){
				REP(i,width*ratio){
					REP(c,3) tmp_rgb[i + j*width*ratio][c] = origin_RGB[i + j*width * ratio][c];
				}
			}
		}
		REP(ik,width * height) delete RGB[ik];
		delete[] RGB;
	//	RGB	= new double*[datasize];
	//	REP(i,datasize){
	//		RGB[i]	= new double[3];
	//	}
		RGB = tmp_rgb;
		labelSizeExpand(ratio);
	}

	void VideoData::labelSizeExpand(const int ratio){

		assert(img);
		char *tmp_img;
		int datasize = width * height * ratio * ratio;
		tmp_img = new char[datasize];

		/* the pixel have the ratio x ratio pixel mean */
		REP( j , height){
			REP( i , width){
				if(img[i + j*width] == FOREGROUND)
					REP(jj , ratio) REP(ii, ratio)
						tmp_img[i*ratio + j*width*ratio*ratio+ ii + jj*width*ratio] = FOREGROUND;
				else if(img[i + j*width] == CHECKREGION)
					REP(jj , ratio) REP(ii, ratio)
						tmp_img[i*ratio + j*width*ratio*ratio+ ii + jj*width*ratio] = CHECKREGION;
				else if(img[i + j*width] == NOLABEL)
					REP(jj , ratio) REP(ii, ratio)
						tmp_img[i*ratio + j*width*ratio*ratio+ ii + jj*width*ratio] = NOLABEL;
				else
					REP(jj , ratio) REP(ii, ratio)
						tmp_img[i*ratio + j*width*ratio*ratio+ ii + jj*width*ratio] = BACKGROUND;
			}
		}
		/* size expand */
		width 	*= ratio;
		height *= ratio;
		delete[] img;
	//	img	= new char[datasize*ratio*ratio];
		img = tmp_img;
	}

	void VideoData::cloneRGB(double ** clone_rgb){
		assert(sizeof(clone_rgb) == sizeof(RGB));
		REP(j,height){
			REP(i,width){
				REP(c,3) clone_rgb[i + j*width][c] = RGB[i + j*width][c];
			}
		}
	}

	template< class T>
	void VideoData::extractStrokeDependColour( const T src , T & dst , const double colour[3] , const int channel){
		assert(src);
		assert(dst);
		double c[3];
		REP(i,channel) c[i] = colour[i]*255;		//0-255にレンジを補正
		REP( j, height){
			REP( i , width) {
				int num = i + j*width;
				if(src[num * channel + 0] == static_cast<unsigned char>(c[0])
				&& src[num * channel + 1] == static_cast<unsigned char>(c[1])
				&& src[num * channel + 2] == static_cast<unsigned char>(c[2])
				){
					REP(k , channel) dst[num * channel + k] = src[num * channel + k] ;
				}
			}
		}
	}

	/**
	 * @param buf
	 * @param se3CfromW:camera pose
	 * @param region : stroke information
	 * @param camera : camera information
	 */
	void VideoData::BackProjectionFromWorldToImage(unsigned char * buf , const TooN::SE3<> &  se3CfromW , Stroke * region , ATANCamera & camera)
	{
		REP(i,width*height) buf[i] = 0;	//初期化
		BOOST_FOREACH(Vertex & v , region->vertex){
			TooN::Vector< 3 > ver;
			REP(i,3) ver[i] = v[i];
			// Projetion from World-Coord to Window-Coord
			TooN::Vector< 3 > v3Cam = se3CfromW * ver;
			TooN::Vector< 2 > v2ImPlane = project(v3Cam);
			TooN::Vector< 2 > v2Image = camera.Project(v2ImPlane);

			int address = static_cast< int >(v2Image[0]) + static_cast< int >(v2Image[1]) * width;
			if(address >=0 && address < width * height){
				buf[address] = 255;
			}
		}
	}

}
