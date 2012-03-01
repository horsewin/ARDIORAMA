/*
 * Model/ModelImplement.h
 *
 *  Created on: 2011/04/27
 *      Author: umakatsu
 */

#ifndef MODELIMPLEMENT_H_
#define MODELIMPLEMENT_H_

#include "Model.h"
namespace PTAMM{

class ModelImplement : public Model{
public:
	ModelImplement(int tnum);
	~ModelImplement();
	void copy( std::deque<Triangle> * mTriangles , std::deque<TextureInfo *> t, const int & texnum);
	void copy( std::deque<Mesh> * mTriangles , std::deque<TextureInfo *> t, const int & texnum);
	void translate( const TooN::Vector< 3 > offset );
	void draw( void );
	void scale( const double & magnification );

	/* Getter and Setter */
	std::deque< Mesh > * 		getMeshes		( void ) const;
	std::deque< TextureInfo * > getTexture		( void ) const;
	int								getTextureSum	( void ) const;

private:
	std::deque<Mesh> 				*meshes;
	std::deque<TextureInfo *> 	texture;
	mState state_model;
	int 	texture_sum;
	double scale_factor;
	TooN::Vector< 3 > gravity;
};
}
#endif /* MODELIMPLEMENT_H_ */
