#ifndef DioramaScreenShot_H_INCLUDED
#define DioramaScreenShot_H_INCLUDED


#include "ImageType.h"
#include "ImageSize.h"
#include "common.h"

// ビットマップのヘッダ
struct BITMAPFILEHEADER_ {
  //char bfType1;
  //char bfType2;
  unsigned long  bfSize;
  unsigned short bfReserved1;
  unsigned short bfReserved2;
  unsigned long  bfOffBits;
};

struct BITMAPINFOHEADER_ {
    unsigned long  biSize;
    long           biWidth;
    long           biHeight;
    unsigned short biPlanes;
    unsigned short biBitCount;
    unsigned long  biCompression;
    unsigned long  biSizeImage;
    long           biXPixPerMeter;
    long           biYPixPerMeter;
    unsigned long  biClrUsed;
    unsigned long  biClrImporant;
};


class DioramaScreenShot{
public:
	DioramaScreenShot( void );
	~DioramaScreenShot( void );

//----------------------------------------------------
//	filename : hoge.bmp
//	bpp      : 24 (NOT include alpha) or 32 (include alpha)
//----------------------------------------------------
bool screenshot(const char* filename, int bpp);
bool rawdata(const char* filename, const char* filename2, ImageType mimFrameRGB, const char thresh, const int erodeIterations,const int dilateIterations);
};

#endif
