#include "Stroke/StrokeGenerator.h"

#include "PTAMTracking/TrackerDrawable.h"
#include "Stroke/Stroke.h"
#include "OpenGL.h"

#include <list>
#include <assert.h>
#include <boost/foreach.hpp>
#include <iostream>
#include "Reconstruction/Background.h"

namespace PTAMM{

class StrokeGeneratorImpl : public StrokeGenerator
{
public:
	explicit StrokeGeneratorImpl( TrackerDrawable &  tkr, ATANCamera &  cam ):
		tracker( tkr ),
		camera( cam ),
		stroke( NULL ),
		is_stroke_on( false )
	{ };

	virtual ~StrokeGeneratorImpl( void ){
		clear( );
	};

	void strokeContentsSet( const GLdouble c[3], const GLuint & line){
		stroke->setLineProperty( line , c );
	}

	void Begin( const TooN::SE3<> &  cam_pose, const TooN::Vector< 2 > &  s){
		assert( ! is_stroke_on );
		stroke       = new Stroke( );
		is_stroke_on = true;

		TooN::SE3<> camInverse    = cam_pose.inverse( );
		TooN::SO3<> camInverseRot = camInverse.get_rotation( );
    
		Vector< 3 > n;//イメージ平面の法線ベクトル;
		n[ 0 ] = 0; n[ 1 ] = 0; n[ 2 ] = 1.0;
		n = camInverseRot * n;

		Vector< 3 > v;{
			Vector< 3 > neighbor;
			tracker.GetDrawedPoint( neighbor );
			GetDrawPointFromClick( v, cam_pose, s, neighbor );
		}
		stroke->set_plane( n, v);
		pLast = v;
	}
	void BeginWithBackground( const TooN::SE3<> &  cam_pose, const TooN::Vector< 2 > &  s,
		const TooN::SE3<> &  c, const Background &  bg)
	{
//		stroke       = new Stroke( );
//		is_stroke_on = true;
//
//		TooN::SE3<> camInverse( c.inverse( ) );
//		TooN::SO3 camInverseRot( camInverse.get_rotation( ) );
//
//		TooN::Vector< 3 > n;
//		bg.getNormal( n );
//
//		TooN::Vector< 3 > u;
//		TooN::Vector< 3 > v;
//		{
//			TooN::Vector< 3 > center;
//			bg.getCenter( center );
//			GetDrawPointFromClick( v, cam_pose, s, center );
//		}
//   		stroke->set_plane( n, v);
//   		pLast = v;
	}

	Stroke *  End( void ){
		assert( is_stroke_on );
		is_stroke_on = false;
		stroke->fix( );
		return stroke;
	}

  
	void  Add( const TooN::SE3<> &  cam_pose, TooN::Vector< 2 > &  mouse){
//		assert( is_stroke_on );
		if(!is_stroke_on){
			std::cout << "INTERRUPT" << std::endl;
			Begin(cam_pose , mouse);
		}
		Vector< 3 > v;
		{
			Vector< 2 > w;
			w = this->camera.UnProject( mouse );
			v[ 0 ] = w[ 0 ];
			v[ 1 ] = w[ 1 ];
			v[ 2 ] = 1.0;
		}
		stroke->add( v, cam_pose);
	}


	void  Render( const TooN::SE3<> &  cam_pose ) const{
		if( ! is_stroke_on ){
			return;
		}
		stroke->draw( cam_pose, camera.MakeUFBLinearFrustumMatrix( 0.005, 100 ) );
	}

	TooN::Vector< 3 > lastPoint( void ) const{
		return pLast;
	}

	bool getIsStroke( void ) const{
		return is_stroke_on;
	}

private:
	TrackerDrawable &  tracker;
	ATANCamera      &  camera;
	Stroke *           stroke;

	TooN::Vector< 3 > pLast;

	bool is_stroke_on;

	void  clear( void ){
		if( stroke != NULL ){
			delete stroke;
		}
		stroke = NULL;
	}


	void  GetDrawPointFromClick( Vector< 3 > &  ret, const TooN::SE3<> &  cam_pose,
		const Vector< 2 > &  mouse, const Vector< 3 > &  neighbor)
	{
		const SE3<> &  se3CFromW( cam_pose );
		Vector< 3 >  clicked_from_camera;
		{
			Vector< 2 > v;
			v = this->camera.UnProject( mouse );
			clicked_from_camera[ 0 ] =  v[ 0 ];
			clicked_from_camera[ 1 ] =  v[ 1 ];
			clicked_from_camera[ 2 ] =  1.0;
		}

		//点と直線の距離から深さの計算
		double  depth = 0.0;
		{
			Vector< 3 >  feature_from_camera = se3CFromW * neighbor;
			depth = ( feature_from_camera * clicked_from_camera ) / ( clicked_from_camera * clicked_from_camera );
		}
		Vector< 3 > draw_from_camera = depth * clicked_from_camera;

		Vector< 3 > draw_from_world  = se3CFromW.inverse( ) * draw_from_camera;
		for( int i = 0; i < 3; i ++){
			ret[ i ] = draw_from_world[ i ];
		}
	}
};


StrokeGenerator::StrokeGenerator( void )
{ }


StrokeGenerator::~StrokeGenerator( void )
{ }

StrokeGenerator *  StrokeGenerator::create( TrackerDrawable &  tkr, ATANCamera &  cam ){
	return new StrokeGeneratorImpl( tkr, cam );
}

}
