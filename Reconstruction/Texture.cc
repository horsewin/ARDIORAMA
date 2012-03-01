#include <GL/glew.h>
#include "Reconstruction/Texture.h"
#include <TooN/se3.h>
#include "OpenGL.h"
#include <GL/glut.h>
#include <GL/glu.h>
#include <cstdlib>
#include <cstring>
#include <boost/scoped_array.hpp>
#include <vector>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/base_object.hpp>
#include <fstream>
#include <algorithm>

namespace{
	inline void sendDataToTex( const Texture::Buftype * data, size_t width, size_t height ){
		assert( data != NULL );
		glTexImage2D( GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGB, width,
			height, 0, GL_RGB, GL_UNSIGNED_BYTE, data );  
	}

	GLuint createGLTexture( const Texture::Buftype * data, size_t width, size_t height ){
		assert( data != NULL );
			
		unsigned texture_id;
		glGenTextures( 1, &texture_id );
		glBindTexture( GL_TEXTURE_RECTANGLE_ARB, texture_id );
		glTexParameteri( GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
		sendDataToTex( & data[ 0 ], width, height );
		glBindTexture( GL_TEXTURE_RECTANGLE_ARB, 0 );
		
		return texture_id;
	}
}



Texture::Texture( const CVD::SubImage< CVD::Rgb< CVD::byte > > &  src ):
	tex_width( src.size( ).x ),
	tex_height( src.size( ).y ),
	data( src.totalsize( ) )
{
//std::memcpy(data.get(), src.data(), sizeof(Buftype) * src.totalsize());
	std::copy( src.data( ), src.data( ) + src.totalsize( ) - 1, data.begin( ) );
	tex_id = createGLTexture( & data[ 0 ], tex_width, tex_height );
}

Texture::Texture( const std::vector< Texture::Buftype > & data_, size_t width, size_t height ):
	tex_width( width ),
	tex_height( height )
{
	this->data.resize( data_.size( ) );
	std::copy( data_.begin( ), data_.end( ), data.begin( ) );
	tex_id = createGLTexture( & data[ 0 ], width, height );
}

Texture::~Texture( void ){
	glDeleteTextures( 1, & tex_id );
}
  

const Texture::Buftype & Texture::operator[ ]( unsigned  i ) const{
	return data[i];
}

size_t  Texture::getWidth( void ) const{
	return this->tex_width;
}

size_t  Texture::getHeight( void )const{
	return this->tex_height;
}  
  
const std::vector< Texture::Buftype > & Texture::getData( void ) const {
	return this->data;
}
void Texture::draw( void ) const{
	bind( );

	glEnd( );
	unbind( );
	return;
}

void Texture::bind( void ) const{
	glEnable( GL_TEXTURE_RECTANGLE_ARB );
	glBindTexture( GL_TEXTURE_RECTANGLE_ARB, tex_id );
}

void Texture::unbind( void ) const{
	glBindTexture( GL_TEXTURE_RECTANGLE_ARB, 0 );
	glDisable( GL_TEXTURE_RECTANGLE_ARB );
}
