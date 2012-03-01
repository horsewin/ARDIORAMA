/*
 * Model.h
 *
 * This is Interface to control each model
 *
 *  Created on: 2011/02/28
 *      Author: umakatsu
 */
#ifndef MODEL_H_
#define MODEL_H_

#include "common.h"
#include "reconstruction.h"
#include <deque>

namespace PTAMM{

class Model{
public:
	Model( void ){};
	virtual ~Model( void ){};
	virtual void copy( std::deque<Triangle> * mTriangles , std::deque<TextureInfo *> t, const int & texnum) = 0;
	virtual void copy( std::deque<Mesh> * mTriangles , std::deque<TextureInfo *> t, const int & texnum) = 0;
	virtual void draw( void ) = 0;
	virtual void translate( const TooN::Vector< 3 > offset ) = 0;
	virtual void scale( const double & magnification ) = 0;

	virtual std::deque< Mesh > * getMeshes( void ) const = 0;
	virtual std::deque< TextureInfo * > getTexture( void ) const = 0;
	virtual int	getTextureSum( void ) const = 0;

	static Model * create(int tnum);
};

}
#endif
