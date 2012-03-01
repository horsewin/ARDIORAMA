#ifndef INCLUDED__STROKE_DRAWER_H_
#define INCLUDED__STROKE_DRAWER_H_

#include <boost/scoped_ptr.hpp>
#include <TooN/se3.h>
//#include <TooN/numerics.h>
#include <TooN/numhelpers.h>
#include <boost/array.hpp>
#include "OpenGL.h"
#include <vector>

namespace CVD
{
	class ImageRef;
}

namespace PTAMM{
class TrackerDrawable;
class ATANCamera;

/* 画面サイズ */
/* 固定配列を使ってしまっているので動的配列にできればしたい */
#define WIDTH		640
#define HEIGHT 	480

//namespace TooN{
//	class SE3<>;
//}


class Stroke;
class Background;

class StrokeGenerator{
public:
	StrokeGenerator( void );
	virtual ~StrokeGenerator( void ) = 0;

	virtual void strokeContentsSet( const GLdouble  c[3] , const GLuint & line=10) = 0;
	virtual void Begin( const TooN::SE3<> &  cam_pose, const TooN::Vector< 2 > &  start) = 0;
	virtual void BeginWithBackground( const TooN::SE3<> &  cam_pose, const TooN::Vector< 2 > &  start, const TooN::SE3<> &  camera, const Background &  background ) = 0;
	virtual void Add( const TooN::SE3<> &  cam_pose, TooN::Vector< 2 > &  mouse_pos) = 0;
	virtual Stroke *  End( void ) = 0;
	virtual void  Render( const TooN::SE3<> &  cam_pose ) const = 0;
	virtual TooN::Vector< 3 > lastPoint( void ) const = 0;
	virtual bool getIsStroke( void ) const = 0;
	static StrokeGenerator *  create( TrackerDrawable &  tkr, ATANCamera &  cam );

};
}
#endif
