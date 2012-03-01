/*
 * Image/VideoData.h
 *
 *  Created on: 2010/11/12
 *      Author: umakatsu
 */

#ifndef VIDEODATA_H_
#define VIDEODATA_H_

#include <vector>
#include "OpenGL.h"
#include <TooN/numhelpers.h>
#include <TooN/helpers.h>
#include "PTAMTracking/ATANCamera.h"
/* 共通のパラメータの設定ヘッダ 2010/11/15*/
#include "common.h"

namespace PTAMM{

class Stroke;

class VideoData{
private:
	double **RGB;		/* RGB値 */
	char 	*img;
	int		width;
	int		height;

public:
	VideoData(const int & w , const int & h);
	~VideoData();
	void store(unsigned char * , GLenum e , const char * = NULL);
	void setLabel(unsigned char *buf , char **image_map);
	void strokeDiff(unsigned char *buf , char **image_map ,  int * x , int * y , const char label , double *color);
	int extractLabel(std::vector< Vector< 3 > > & col , std::vector< Vector< 2 > > & crd , char label);
	void replaceLabel(unsigned char op , unsigned char *buf , char replabel , char exlabel);
	void imageSizeShrink(const int ratio);
	void labelSizeShrink(const int ratio);
	void imageSizeExpand(const int ratio , double **origin_RGB = NULL);
	void labelSizeExpand(const int ratio);
	double **getRGB() const { return RGB; }
	void setRGB(double **RGB){ this->RGB = RGB; }
	void cloneRGB(double **clone_rgb);
	template< class T>
	void extractStrokeDependColour(const T src , T & dst , const double colour[3], const int channel);
	void BackProjectionFromWorldToImage(unsigned char * buf , const TooN::SE3<> &  se3CfromW , Stroke * region , ATANCamera & camera);
    char *getImg() const
    {
        return img;
    }

    void setImg(char *img)
    {
        this->img = img;
    }

    int getHeight() const
    {
        return height;
    }

    int getWidth() const
    {
        return width;
    }

    void setHeight(int height)
    {
        this->height = height;
    }

    void setWidth(int width)
    {
        this->width = width;
    }
};
}
#endif /* VIDEODATA_H_ */
