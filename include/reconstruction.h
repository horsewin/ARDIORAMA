	/*
 * reconstruction.h
 *
 *  Created on: 2011/03/01
 *      Author: umakatsu
 */

#ifndef RECONSTRUCTION_H_
#define RECONSTRUCTION_H_

#include "Reconstruction/Texture.h"
#include <TooN/TooN.h>
#include <TooN/numerics.h>
#include <TooN/numhelpers.h>
#include "ImageType.h"
#include "ImageSize.h"

struct Point {
	TooN::Vector< 3 > coord;
	double tex[2];
};

struct Mesh{
	Point point[3];
};

struct Triangle {
	Point point[3];
	Point normal;
	TooN::Vector< 3 > g;
	int texture_number;
	double check_value;
};

struct TextureInfo{
	Texture *tex;
	uchar *silhouette;
	TooN::SE3<> camerapose;
	TooN::Vector< 3 > camera_ref;
	ImageType imRGB;
	int texture_number;
	TextureInfo(){}
	~TextureInfo(){ delete tex; }
};
#endif /* RECONSTRUCTION_H_ */
