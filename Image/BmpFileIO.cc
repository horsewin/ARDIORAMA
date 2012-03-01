/*
 * BmpFileIO.cc
 *
 *  Created on: 2010/11/21
 *      Author: umakatsu

 * Copyright (c) 2010, IMURA Masataka.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include	<fstream>
#include	<iostream>
#include	<string>
#include	<vector>

#include	"Image/BmpFileIO.h"

using namespace	std;

// constructor and destructor

BmpFileIO::BmpFileIO( const int n_origin )
	: origin( n_origin )
{
}

BmpFileIO::~BmpFileIO()
{
}



// utility functions

static int
GetInt( unsigned char *p )
{
    int r = 0;

    p += 3;
    for ( int i = 0; i < 4; i++ ) {
        r <<= 8;
        r += static_cast<int>( *p );
        p--;
    }

	return r;
}

static void
SetInt( unsigned char *p, int n )
{
	for ( int i = 0; i < 4; i++ ) {
		*p++ = n % 256;
		n >>= 8;
	}
}

static void
SetShort( unsigned char *p, int n )
{
	for ( int i = 0; i < 2; i++ ) {
		*p++ = n % 256;
		n >>= 8;
	}
}



// read

int
BmpFileIO::Read( const string& filename,
				 int& width, int& height, vector<unsigned char>& image )
{
	return ReadRGB( filename, width, height, image );
}

int
BmpFileIO::ReadRGB( const string& filename,
					int& width, int& height, vector<unsigned char>& image )
{
	ifstream	f_in( filename.c_str(), ios::binary );

	if ( f_in.fail()) {
		cerr << "BmpFileIO::ReadRGB(): cannot open " << filename << endl;
		return -1;
	}

	// read header

	unsigned char	header[ HEADER_SIZE ];
	f_in.read( reinterpret_cast<char *>( header ), HEADER_SIZE );

	width = GetInt( header + WIDTH_OFFSET );
	height = GetInt( header + HEIGHT_OFFSET );

	// read body

	int	line_bytes = (( width * 3 + 3 ) / 4 ) * 4;

	image.resize( width * height * 3 );

	int	y, y_step;

	if ( origin == ORIGIN_LEFT_TOP ) {
		y = height - 1;
		y_step = -1;
	} else {
		y = 0;
		y_step = 1;
	}

	for ( int i = 0; i < height; i++ ) {
		f_in.read( reinterpret_cast<char *>( &image[ y * width * 3 ] ),
				   width * 3 );
		if ( line_bytes - width * 3 > 0 ) {
			char	dummy[ 3 ];
			f_in.read( dummy, line_bytes - width * 3 );
		}
		y += y_step;
	}

	f_in.close();

	// BGR -> RGB

	unsigned char	*p = &image[ 0 ];
	for ( int i = 0; i < width * height; i++ ) {
		unsigned char	tmp;
		tmp = *p;
		*p = *( p + 2 );
		*( p + 2 ) = tmp;
		p += 3;
	}

	return 0;
}

int
BmpFileIO::ReadRGBA( const string& filename,
					 int& width, int& height, vector<unsigned char>& image )
{
	int	r;

	r = ReadRGB( filename, width, height, image );

	if ( !r ) {
		image.resize( width * height * 4 );

		unsigned char	*src = &image[ width * height * 3 ];
		unsigned char	*dst = &image[ 0 ] + ( width * height * 4 );

		for ( int i = 0; i < width * height; i++ ) {
			*(--dst) = 255;
			*(--dst) = *(--src);
			*(--dst) = *(--src);
			*(--dst) = *(--src);
		}
	}

	return r;
}

unsigned char *
BmpFileIO::Read( const string& filename, int& width, int& height )
{
	return ReadRGB( filename, width, height );
}

unsigned char *
BmpFileIO::ReadRGB( const string& filename, int& width, int& height )
{
	vector<unsigned char>	image;

	if ( ReadRGB( filename, width, height, image )) {
		return 0;
	}

	unsigned char	*r = new unsigned char[ width * height * 3 ];

	unsigned char	*src = &image[ 0 ];
	unsigned char	*dst = r;

	for ( int i = 0; i < width * height * 3; i++ ) {
		*dst++ = *src++;
	}

	return r;
}

unsigned char *
BmpFileIO::ReadRGBA( const string& filename, int& width, int& height )
{
	vector<unsigned char>	image;

	if ( ReadRGBA( filename, width, height, image )) {
		return 0;
	}

	unsigned char	*r = new unsigned char[ width * height * 4 ];

	unsigned char	*src = &image[ 0 ];
	unsigned char	*dst = r;

	for ( int i = 0; i < width * height * 4; i++ ) {
		*dst++ = *src++;
	}

	return r;
}



// write

int
BmpFileIO::Write( const string& filename,
				  const int width, const int height,
				  const vector<unsigned char>& image )
{
	return Write( filename, width, height, &image[ 0 ] );
}

int
BmpFileIO::WriteRGB( const string& filename,
					 const int width, const int height,
					 const vector<unsigned char>& image )
{
	return WriteRGB( filename, width, height, &image[ 0 ] );
}

int
BmpFileIO::WriteRGBA( const string& filename,
					  const int width, const int height,
					  const vector<unsigned char>& image )
{
	return WriteRGBA( filename, width, height, &image[ 0 ] );
}

int
BmpFileIO::Write( const string& filename,
				  const int width, const int height,
				  const unsigned char * const image )
{
	return WriteRGB( filename, width, height, image );
}

int
BmpFileIO::WriteRGB( const string& filename,
					 const int width, const int height,
					 const unsigned char * const image )
{
	ofstream	f_out( filename.c_str(), ios::binary );

	if ( f_out.fail()) {
		cerr << "BmpFileIO::WriteRGB(): cannot open " << filename << endl;
		return -1;
	}

	// write header

    int	line_bytes = (( width * 3 + 3 ) / 4 ) * 4;

	unsigned char	header[ HEADER_SIZE ];
	for ( int i = 0; i < HEADER_SIZE; i++ ) {
		header[ i ] = 0;
	}

	header[ 0 ] = 'B';
	header[ 1 ] = 'M';
	SetInt( header + SIZE_OFFSET, line_bytes * height + HEADER_SIZE );
	SetInt( header + OFFBITS_OFFSET, HEADER_SIZE );
	SetInt( header + INFO_HEADER_SIZE_OFFSET, HEADER_SIZE - FILE_HEADER_SIZE );
	SetInt( header + WIDTH_OFFSET, width );
	SetInt( header + HEIGHT_OFFSET, height );
	SetShort( header + PLANE_OFFSET, 1 );
	SetShort( header + DEPTH_OFFSET, 24 );

	f_out.write( reinterpret_cast<char *>( header ), HEADER_SIZE );

	// convert

	vector<unsigned char>	tmp( line_bytes * height );

	int	y, y_step;

	if ( origin == ORIGIN_LEFT_TOP ) {
		y = height - 1;
		y_step = -1;
	} else {
		y = 0;
		y_step = 1;
	}

	const unsigned char	*src;
	unsigned char	*dst = &tmp[ 0 ];
	for ( int i = 0; i < height; i++ ) {
		src = image + y * width * 3;
		for ( int x = 0; x < width; x++ ) {
			*dst++ = *( src + 2 );
			*dst++ = *( src + 1 );
			*dst++ = *( src );
			src += 3;
		}
		if ( line_bytes - width * 3 > 0 ) {
			for ( int j = 0; j < line_bytes - width * 3; j++ ) {
				*dst++ = 0;
			}
		}
		y += y_step;
	}

	// write body

	f_out.write( reinterpret_cast<char *>( &tmp[ 0 ] ), line_bytes * height );

	f_out.close();

	return 0;
}

int
BmpFileIO::WriteRGBA( const string& filename,
					  const int width, const int height,
					  const unsigned char * const image )
{
	vector<unsigned char>	tmp( width * height * 3 );

	const unsigned char	*src = image;
	unsigned char	*dst = &tmp[ 0 ];
	for ( int i = 0; i < width * height; i++ ) {
		*dst++ = *src++;
		*dst++ = *src++;
		*dst++ = *src++;
		src++;
	}

	int	r = WriteRGB( filename, width, height, tmp );

	return r;
}



// test routine

#ifdef	TEST_BMP_FILE_IO

#include	<iomanip>

int
main( int argc, char **argv )
{
	if ( argc != 3 ) {
		cerr << "usage: " << argv[ 0 ] << " <input bmp file> <output bmp file>"
			 << endl;
		return -1;
	}

	BmpFileIO	b( BmpFileIO::ORIGIN_LEFT_TOP );
	int	width, height;
	vector<unsigned char>	image;

	cout << "reading " << argv[ 1 ] << "..." << endl;
	b.ReadRGBA( argv[ 1 ], width, height, image );
//	unsigned char	*image = b.ReadRGBA( argv[ 1 ], w, h );
	cout << "done" << endl;

	cout << "width: " << width << endl;
	cout << "height: " << height << endl;
	cout << "image: ";

	ios::fmtflags	f = cout.flags();
	for ( int i = 0; i < 16; i++ ) {
		cout << hex << setw( 2 ) << setfill( '0' )
			 << static_cast<int>( image[ i ] ) << ",";
	}
	cout.setf( f );
	cout << "..." << endl;

	cout << "writing " << argv[ 2 ] << "..." << endl;
	b.WriteRGBA( argv[ 2 ], width, height, image );
//	b.WriteRGBA( argv[ 2 ], w, h, image );
	cout << "done" << endl;

//	delete [] image;

	return 0;
}

#endif
