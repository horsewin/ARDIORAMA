#ifndef INCLUDED__TEXTURE_H_
#define INCLUDED__TEXTURE_H_

#include <cvd/rgb.h>
#include <cvd/byte.h>
#include <cvd/image.h>
#include <TooN/se3.h>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/scoped_ptr.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/split_member.hpp>
#include <vector>
#include <string>
#include "ImageType.h"

//namespace TooN{
//  class SE3<>;
//}


class Texture{
public:
	typedef CVD::Rgb< CVD::byte > Buftype;

	explicit Texture( const CVD::SubImage< Texture::Buftype > &  src );
	Texture( const std::vector< Buftype > & data, size_t width, size_t height );
	~Texture( void );

	const Buftype &  operator[ ]( unsigned i ) const;

	size_t  getWidth( void ) const;
	size_t  getHeight( void ) const;
	const std::vector< Buftype > & getData( void ) const;

	void  bind( void ) const;
	void  unbind( void ) const;
  
	void  draw( void ) const;//for debug
	int getTex_id( void ) const{ return (int)tex_id; }
private:
	unsigned  tex_width;
	unsigned  tex_height;
	std::vector< Buftype >  data;
  
	unsigned  tex_id; 

	friend class boost::serialization::access;
	BOOST_SERIALIZATION_SPLIT_MEMBER( );

	template < class Archive >
    void save( Archive &  ar, const unsigned version ) const{
		ar & tex_width;
		ar & tex_height;
		ar & data;
		ar & tex_id;
	}

	template <class Archive>
	void load( Archive &  ar, const unsigned  version ){
		ar & tex_width;
		ar & tex_height;
		ar & data;
		ar & tex_id;

//		glGenTextures( 1, & tex_id );
//		glBindTexture( GL_TEXTURE_RECTANGLE_ARB, tex_id );
//		glTexParameteri( GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
//		glTexParameteri( GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
//		glTexImage2D( GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGB, tex_width, tex_height,
//			0, GL_RGB, GL_UNSIGNED_BYTE, & ( data[ 0 ] ) );
//		glBindTexture( GL_TEXTURE_RECTANGLE_ARB, 0 );
	}
};


namespace boost{
	namespace serialization{
		template < class Archive >
		void serialize( Archive &  ar, CVD::Rgb< CVD::byte > &  rgb, const unsigned  version ){
			ar & rgb.red & rgb.green & rgb.blue;
		}

		template < class Archive >
		inline void save_construct_data( Archive & ar, const Texture * tex, const unsigned version ){
			size_t width = tex->getWidth( );
			ar << width;
			size_t height = tex->getHeight( );
			ar << height;
			const std::vector< Texture::Buftype > & data = tex->getData( );
			ar << data;
		}

		template < class Archive >
		inline void load_construct_data( Archive & ar, Texture * tex, const unsigned version ){
			size_t width;
			ar >> width;
			size_t height;
			ar >> height;
			std::vector< Texture::Buftype > data;
			ar >> data;
			
			::new( tex ) Texture( data, width, height );
		}
	}
}
#endif
