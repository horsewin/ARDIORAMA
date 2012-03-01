/*
 * ModelImplement.cc
 *
 *  Created on: 2011/02/28
 *      Author: umakatsu
 */

#include "Model/ModelImplement.h"
#include "myVector.h"
#include <TooN/numerics.h>

namespace PTAMM{

using namespace std;
Model * Model::create( int num ){
	return new ModelImplement( num );
}

ModelImplement::ModelImplement( int tnum )
: texture_sum(tnum){
	meshes = new deque<Mesh>[ ( texture_sum + 1 ) ];
	scale_factor = 1.0;
	state_model = DISABLE;
	gravity = Zeros;
}

ModelImplement::~ModelImplement(void){
	delete[] meshes;
	REP(i,texture.size()){
		TextureInfo * buf = texture.front();
		delete buf;
		texture.pop_front();
	}
}

void ModelImplement::copy( deque<Triangle> * mTriangles , deque<TextureInfo *> t, const int & texnum){
	state_model = COPY;
	assert( texnum == texture_sum );
	texture.clear();
	REP(tnum,texnum+1){
		meshes[tnum].clear();
		deque<Triangle>::iterator i;
		for (i = mTriangles[tnum].begin(); i != mTriangles[tnum].end(); i++) {
			Mesh mesh;
			for (int j = 0; j < 3; j++) {
				// copy the triangle 3D coordinates
				mesh.point[j].coord = (*i).point[j].coord;
				// copy the texture coordinates
				REP(id,2) mesh.point[j].tex[id] = (*i).point[j].tex[id];
			}
			meshes[tnum].push_back(mesh);
		}
		if( tnum < texture_sum){
			TextureInfo *tmp_texture = new TextureInfo();
			tmp_texture->tex = new Texture(t.at(tnum)->imRGB);
			tmp_texture->silhouette = t.at(tnum)->silhouette;
			tmp_texture->camerapose = t.at(tnum)->camerapose;
			tmp_texture->imRGB = t.at(tnum)->imRGB;
			tmp_texture->texture_number = t.at(tnum)->texture_number;
			texture.push_back(tmp_texture);
		}
	}
}

void ModelImplement::copy( deque<Mesh> * mTriangles , deque<TextureInfo *> t, const int & texnum){
	state_model = COPY;
	assert( texnum == texture_sum );
	texture.clear();
	long int mesh_number = 0;
	REP(tnum,texnum+1){
		meshes[tnum].clear();
		deque<Mesh>::iterator i;
		mesh_number += mTriangles[tnum].size();
		for (i = mTriangles[tnum].begin(); i != mTriangles[tnum].end(); i++) {
			Mesh mesh;
			for (int j = 0; j < 3; j++) {
				// copy the triangle 3D coordinates
				mesh.point[j].coord = (*i).point[j].coord;
				// copy the texture coordinates
				REP(id,2) mesh.point[j].tex[id] = (*i).point[j].tex[id];
				// calculation of the gravity
				gravity += mesh.point[j].coord;
			}
			meshes[tnum].push_back(mesh);
		}
		if( tnum < texture_sum){
			TextureInfo *tmp_texture = new TextureInfo();
			tmp_texture->tex = new Texture(t.at(tnum)->imRGB);
			tmp_texture->silhouette = t.at(tnum)->silhouette;
			tmp_texture->camerapose = t.at(tnum)->camerapose;
			tmp_texture->imRGB = t.at(tnum)->imRGB;
			tmp_texture->texture_number = t.at(tnum)->texture_number;
			texture.push_back(tmp_texture);
		}
	}
	gravity /= static_cast< double > ( mesh_number);
	cout << gravity << endl;
}

// Change an original coordinate of each mesh
// add translation offset to a original coord
void ModelImplement::translate( const TooN::Vector< 3 > offset ){
	const TooN::Vector< 3 > x = offset - meshes[0].front().point[0].coord;
	REP(tnum,texture_sum+1){
		deque<Mesh>::iterator i;
		for (i = meshes[tnum].begin(); i != meshes[tnum].end(); i++) {
			for (int j = 0; j < 3; j++) {
				REP(id,3) (*i).point[j].coord[id] += x[id];
//				REP(id,3) (*i).point[j].coord[id] += offset[id];
			}
		}
	}
	gravity += x;
}

// Change an original coordinate of each mesh
// Calculation about scaling ratio
void ModelImplement::scale( const double & magnification ){
	REP(tnum,texture_sum+1){
		deque<Mesh>::iterator i;
		for (i = meshes[tnum].begin(); i != meshes[tnum].end(); i++) {
			for (int j = 0; j < 3; j++) {
				const TooN::Vector< 3 > diff = (*i).point[j].coord - gravity;
				const TooN::Vector< 3 > diff_ =  ( magnification / scale_factor) * diff;
				(*i).point[j].coord = gravity + ( magnification / scale_factor)  * diff_;
//				(*i).point[j].coord *= ( magnification / scale_factor) ; // scaling from the center of origin
			}
		}
	}
	scale_factor = magnification;
}

void ModelImplement::draw( void ){
	if( texture.size() < 1 ){
		cerr << "No Texture are set" << endl;
		cerr << "You have to set at least one" << endl;
		return;
	}
	// All mesh data drawing
	REP(tnum,texture_sum+1){
		deque<Mesh>::iterator i;
		// Texture Bind
		if( tnum < texture_sum ) texture.at(tnum)->tex->bind( );
		glBegin(GL_TRIANGLES);
		for (i = meshes[tnum].begin(); i != meshes[tnum].end(); i++) {
				Mesh mesh = (*i);
				for (int j = 0; j < 3; j++) {
					const double * tex = (double*)&(*i).point[j].tex;
					// Assign adequate coord if textures can be found
					if( tnum < texture_sum) glTexCoord2d( tex[ 0 ], tex[ 1 ] );
					// Assign (probably) background coord if textures cannot be found
					else glTexCoord2d( WIDTH-1 , HEIGHT-1 );
					glVertexVector( mesh.point[j].coord );
				}
		}
		glEnd();
	}
	texture.back()->tex->unbind( );
}

std::deque< Mesh > * ModelImplement::getMeshes( void ) const{
	return meshes;
}

std::deque< TextureInfo * > ModelImplement::getTexture( void ) const{
	return texture;
}

int	ModelImplement::getTextureSum( void ) const{
	return texture_sum;
}
}
