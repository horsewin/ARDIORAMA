#include <cassert>
#include <cmath>
#include "Stroke/Stroke.h"
#include "PTAMTracking/ATANCamera.h"
#include "PTAMTracking/MapPoint.h"

#include <GL/glut.h>
#include <cvd/utility.h>
#include <cvd/gl_helpers.h>
#include <TooN/helpers.h>
#include <boost/foreach.hpp>
#include <boost/serialization/vector.hpp>

#include <iostream>

namespace PTAMM{

//using TooN;
using namespace std;

namespace{
	template < class V >
	double normSquare( const V & vect ){
		double ret = 0;
		for( int i = 0; i < 3; i++){
			ret += vect[ i ] * vect[ i ];
		}
		return ret;
	}

	template < class V >
	double norm( const V & vect ){
		double norm_square = normSquare( vect );
		return std::sqrt( norm_square );
	}
	template < class V >
	double innerProduct( const V & v1, const V & v2 ){
		double ret = 0;
		for( int i = 0; i < 3; i++){
			ret += v1[ i ] * v2[ i ];
		}	
		return ret;
	}

	void getBaseCenter( const TooN::Vector< 3 > & normal, const TooN::Vector< 3 > & start, const GLdouble * center, GLdouble * base_center ){
		assert( center != NULL );
		assert( base_center != NULL );
		
		GLdouble d = 0;
		d = innerProduct( normal, start ) / normSquare( normal );
		
		for( int i = 0; i < 3; i++){
			base_center[ i ] = d * normal[ i ] + center[ i ];
		}
	}
}

Stroke::Stroke( void ){
	BOOST_FOREACH( GLdouble &  d, colour ){
		d = 1.0;
	}
	line_width = 10.0;
	reset( );
}

Stroke::Stroke(const	 GLuint & lwidth , const GLdouble c[3]){
	assert( c );
	REP(i,3) colour[i] = c[i];
	line_width = lwidth;
	reset( );
}

Stroke::~Stroke( void ){
//	cout << "Stroke delete" << endl;
}

void  Stroke::add( const TooN::Vector< 3 > &  newVertexFromC , const TooN::SE3<> &  se3CfromW){
	const TooN::SE3<> &  cam = se3CfromW;
	TooN::Vector< 3 >        drawFromC;
	{
		TooN::Vector< 3 >  normalFromC = cam * this->normal;
		TooN::Vector< 3 >  startFromC  = cam * this->start;
		double  inner_product = normalFromC * startFromC;
		double  depth         = inner_product / ( newVertexFromC * normalFromC );
		drawFromC             = depth * newVertexFromC;
	}
	TooN::Vector< 3 >  drawFromW = cam.inverse( ) * drawFromC;
	{
		Vertex  ver;
		for( int  i( 0 ); i < 3; i ++){
			ver[ i ] = drawFromW[ i ];
		}
		vertex.push_back( ver );
	}
}

void Stroke::set_plane( const TooN::Vector< 3 > &  n, const TooN::Vector< 3 > &  v ){
	Vertex ver;
	for( int  i = 0; i < 3; i ++){
		this->normal[ i ] = n[ i ];
		this->start[ i ]  = v[ i ];
		ver[ i ] = v[ i ];
	}
	vertex.push_back( ver );
}


unsigned  Stroke::getVertexCount( void ) const{
	return vertex.size( );
}


void Stroke::getTranslated( TooN::Vector< 3 > & ret, unsigned index ) const{
	assert( 0 <= index );
	assert( index < getVertexCount( ) );

	if( index == 0 ){
		for( int i = 0; i < 3; i += 1 ){
			ret[ i ] = 0.0;
		}
		return;
	}

	const Vertex &  v_prev( vertex[ index - 1 ] );
	const Vertex &  v( vertex[ index ] );
	for( unsigned i( 0 ); i < 3; i += 1 ){
		ret[ i ] = v[ i ] - v_prev[ i ];
	}
}

//void  Stroke::draw( const TooN::SE3<> &  se3CfromW, const TooN::Matrix< 4 > & proj ) const{
//	glColor3dv( colour );
//	glLineWidth( line_width);
//
////	GLint mat_mode;
////	glGetIntegerv( GL_MATRIX_MODE, & mat_mode );
//
//	glMatrixMode( GL_PROJECTION );
////	glPushMatrix( );
//	glLoadIdentity( );
//	CVD::glMultMatrix( proj  );
//
//	glMatrixMode( GL_MODELVIEW );
////	glPushMatrix( );
//	glLoadIdentity( );	// Add 2011.2.27 by Umakatsu
//	CVD::glMultMatrix(se3CfromW);
//
//	glBegin( GL_LINE_STRIP );{
//		BOOST_FOREACH( const Vertex &  v, vertex){
//			glVertexVector(v);
//		}
//	}glEnd( );
////
////	glPopMatrix( );
////	glMatrixMode( GL_PROJECTION );
////	glPopMatrix( );
////	glMatrixMode( mat_mode );		// modify (TEST) 2011.2.27 by Umakatsu
//	glMatrixMode( GL_MATRIX_MODE );
//}

void  Stroke::draw( const TooN::SE3<> &  se3CfromW, const TooN::Matrix< 4 > & proj ) const{
	glColor3dv( colour );
	glLineWidth( line_width);

	GLint mat_mode;
	glGetIntegerv( GL_MATRIX_MODE, & mat_mode );

	glMatrixMode( GL_PROJECTION );
	glPushMatrix( );
	glLoadIdentity( );
	CVD::glMultMatrix( proj );

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix( );
	glLoadIdentity( );	// Add 2011.2.27 by Umakatsu
	CVD::glMultMatrix(SE3<>());
	  CVD::glMultMatrix( se3CfromW );

	glBegin( GL_LINE_STRIP );{
		BOOST_FOREACH( const Vertex &  v, vertex){
			glVertexVector(v);
		}
	}glEnd( );

	glPopMatrix( );
	glMatrixMode( GL_PROJECTION );
	glPopMatrix( );
//	glMatrixMode( mat_mode );		// modify (TEST) 2011.2.27 by Umakatsu
	glMatrixMode( GL_MATRIX_MODE );
	glLoadIdentity();
}

void  Stroke::fill( const TooN::SE3<> &  se3CfromW, const TooN::Matrix< 4 > & proj ) const{
	glColor3dv( colour );
	glLineWidth( line_width);

	GLint mat_mode;
	glGetIntegerv( GL_MATRIX_MODE, & mat_mode );
	glMatrixMode( GL_PROJECTION );
	glPushMatrix( );
	glLoadIdentity( );

	CVD::glMultMatrix( proj  );
	glMatrixMode( GL_MODELVIEW );
	glPushMatrix( );
	CVD::glMultMatrix( se3CfromW );

	glBegin(GL_POLYGON);{
		BOOST_FOREACH( const Vertex &  v, vertex){
			glVertex3d( v[ 0 ], v[ 1 ], v[ 2 ] );
		}
		glVertex3d( vertex.front()[ 0 ], vertex.front()[ 1 ], vertex.front()[ 2 ] );
	}glEnd( );

	glPopMatrix( );
	glMatrixMode( GL_PROJECTION );
	glPopMatrix( );
	glMatrixMode( GL_MATRIX_MODE );
}


void  Stroke::stencil( int  write, const TooN::SE3<> &  se3CfromW, const TooN::Matrix< 4 > & proj ) const{
	glEnable( GL_STENCIL_TEST );
  //ステンシルに書き込む値のセット

	glStencilFunc( GL_ALWAYS, write, ~0 );
	glStencilOp( GL_REPLACE, GL_REPLACE, GL_REPLACE );

  //カラーバッファ　デプスバッファへの書き込みを禁止
	glColorMask( GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE );
	glDepthMask( GL_FALSE );

	GLint mat_mode;
	glGetIntegerv( GL_MATRIX_MODE, & mat_mode );

	glMatrixMode( GL_PROJECTION );
	glPushMatrix( );
	glLoadIdentity( );
	CVD::glMultMatrix( proj );

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix( );
	CVD::glMultMatrix( se3CfromW );

	glBegin( GL_POLYGON );{
	BOOST_FOREACH( const Vertex &  v, vertex){
		glVertex3d( v[ 0 ], v[ 1 ], v[ 2 ] );
	}
	}glEnd( );
	glPopMatrix( );

	glMatrixMode( GL_PROJECTION );

  //カラーバッファ　デプスバッファへの書き込み禁止を解除
	glColorMask( GL_TRUE,GL_TRUE, GL_TRUE, GL_TRUE );
	glDepthMask( GL_TRUE );
	glPopMatrix( );
	glMatrixMode( mat_mode );
	glDisable( GL_STENCIL_TEST );
}

void  Stroke::fix( void ){
	is_fixed = true;
}

void  Stroke::reset( void ){
	vertex.clear( );
	is_fixed = false;
}

void Stroke::setLineProperty(const GLuint & line_width , const GLdouble colour[3] ){
	this->line_width = line_width;
	this->colour[0]  = colour[0];
	this->colour[1]  = colour[1];
	this->colour[2]  = colour[2];
}
}
