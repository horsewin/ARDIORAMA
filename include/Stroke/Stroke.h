#ifndef INCLUDED__3DSTROKE_H_
#define INCLUDED__3DSTROKE_H_
#include "common.h"
#include <list>
#include <deque>
//#include <TooN/numerics.h>
#include <TooN/numhelpers.h>
#include <fstream>

#include <boost/serialization/serialization.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/list.hpp>

//namespace TooN{
//	class SE3<double>;
//	template< int i >
//	class TooN::Vector;
// }
namespace CVD{
  class ImageRef;
}

namespace PTAMM{

class ATANCamera;
//namespace boost{
//	namespace {
//		template < class Archive >
//			void  serialization( Archive &  ar, TooN::Vector< 3 > &   v, const unsigned  version ){
//			ar & v[0] & v[1] & v[2];
//		}
//	}
//}

class Stroke{
public:
	explicit Stroke( void );
	Stroke(const GLuint & lwidth, const GLdouble colour[3]);
	virtual ~Stroke( void );

	void  draw( const SE3<> &  se3CFromW, const TooN::Matrix< 4 > &  proj ) const;
	void  fill( const SE3<> &  se3CfromW, const TooN::Matrix< 4 > &  proj ) const;
	void  stencil( int  write, const SE3<> &   se3CfromW, const TooN::Matrix< 4 > &  proj ) const; //ステンシルバッファにストローク内部のみ開けるマスクをかける
	void  add( const TooN::Vector< 3 > & , const SE3<> & );
	void  set_plane( const TooN::Vector< 3 > &  normal, const TooN::Vector< 3 > &  start);
	void  fix( void );

	unsigned  getVertexCount( void ) const;
	void  getTranslated( TooN::Vector< 3 > &  ret, unsigned  index ) const;
	TooN::Vector<3> getStart()  const { return start;  }
	TooN::Vector<3> getNormal() const { return normal; }
	std::deque< Vertex >	getVertex( void ) const { return vertex; }
	void setLineProperty(const GLuint & line_width , const GLdouble colour[3] );

	std::deque< Vertex >  vertex;	// 描画点群

private:
  //描画の色と太さ
	GLdouble  colour[ 3 ];
	GLuint    line_width;

	TooN::Vector< 3 >  normal;    //描画を行う平面の法線
	TooN::Vector< 3 >  start;     //描画を行う平面の始点
  
	bool  is_fixed;         //描画が終了したらtrue
  
	friend class boost::serialization::access;

//	template < class Archive >
//	void  serialize( Archive &  ar, const unsigned version ){
//		ar & colour;
//		ar & line_width;
//	}
  
	void reset( void );
	void set_color( const GLdouble *  color );
};
}
#endif
