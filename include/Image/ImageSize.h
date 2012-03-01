/**************************************************
2次元画像の大きさを記憶する構造体

2009.12.16 立石 昂裕
**************************************************/
#ifndef INCLUDED__IMAGE_SIZE_H_
#define INCLUDED__IMAGE_SIZE_H_
#include <cstdio>
#include <boost/serialization/serialization.hpp>
#include <iostream>
namespace PTAMM{
class ImageSize{
public:
	size_t  width;
	size_t  height;

	size_t  total_size( void ) const{
		return width * height;
	}

	ImageSize( void ):
		width( 0 ),
		height( 0 )
	{ }
	
	ImageSize( size_t w, size_t h ):
		width( w ),
		height( h )
	{ };
	

	~ImageSize( void ){ }

private:
	friend class boost::serialization::access;
	template< class Archive >
	void  serialize( Archive &  ar, unsigned   ver ){
		ar & width;
		ar & height;
	}
};

std::ostream & operator<<( std::ostream & os, const ImageSize & size );
}
#endif

