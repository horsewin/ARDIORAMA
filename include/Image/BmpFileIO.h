/*
 * Image/BmpFileIO.h
 *
 *  Created on: 2010/11/21
 *      Author: umakatsu
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

#ifndef BMPFILEIO_H_
#define BMPFILEIO_H_

#include	<string>
#include	<vector>

class BmpFileIO {
protected:
	static const int	HEADER_SIZE             = 54;
	static const int	FILE_HEADER_SIZE        = 14;

	static const int	SIZE_OFFSET             = 2;
	static const int	OFFBITS_OFFSET          = 10;
	static const int	INFO_HEADER_SIZE_OFFSET = 14;
	static const int	WIDTH_OFFSET            = 18;
	static const int	HEIGHT_OFFSET           = 22;
	static const int	PLANE_OFFSET            = 26;
	static const int	DEPTH_OFFSET            = 28;

	int	origin;

public:
	enum {
		ORIGIN_LEFT_TOP = 0,
		ORIGIN_LEFT_BOTTOM = 1
	};

	BmpFileIO( const int n_origin = BmpFileIO::ORIGIN_LEFT_BOTTOM );
	virtual ~BmpFileIO();

	// read

	virtual int	Read( const std::string& filename,
					  int& width, int& height,
					  std::vector<unsigned char>& image );
	virtual int	ReadRGB( const std::string& filename,
						 int& width, int& height,
						 std::vector<unsigned char>& image );
	virtual int	ReadRGBA( const std::string& filename,
						  int& width, int& height,
						  std::vector<unsigned char>& image );

	virtual unsigned char	*Read( const std::string& filename,
								   int& width, int& height );
	virtual unsigned char	*ReadRGB( const std::string& filename,
									  int& width, int& height );
	virtual unsigned char	*ReadRGBA( const std::string& filename,
									   int& width, int& height );

	// write

	virtual	int Write( const std::string& filename,
					   const int width, const int height,
					   const std::vector<unsigned char>& image );
	virtual int	WriteRGB( const std::string& filename,
						  const int width, const int height,
						  const std::vector<unsigned char>& image );
	virtual int	WriteRGBA( const std::string& filename,
						   const int width, const int height,
						   const std::vector<unsigned char>& image );

	virtual	int Write( const std::string& filename,
					   const int width, const int height,
					   const unsigned char * const image );
	virtual int	WriteRGB( const std::string& filename,
						  const int width, const int height,
						  const unsigned char * const image );
	virtual int	WriteRGBA( const std::string& filename,
						   const int width, const int height,
						   const unsigned char * const image );
};

#endif /* BMPFILEIO_H_ */
