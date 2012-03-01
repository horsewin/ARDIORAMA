#include <iostream>
#include <fstream>
#include <vector>
#include "Image/DioramaScreenShot.h"
#include <GL/glut.h>

#include <cv.h> // 2010/11/16
#include <highgui.h>
#include <cstdio> //2010/11/18
#include <cstdlib>
#include <string>


using namespace std;
using namespace cv; //2010/11/20

DioramaScreenShot::DioramaScreenShot(){}
DioramaScreenShot::~DioramaScreenShot(){}

// 画面の内容をビットマップファイルに出力
//   filename: 出力ファイル名
//   bpp: ピクセルあたりのビット数（24, 32）
bool DioramaScreenShot::screenshot(const char* filename, int bpp)
{

	const int width    = 640;   // 画像の大きさ
	const int height   = 480;
	// ピクセルデータ全体のサイズ
	const int datasize = height*((((width*bpp/8) + 3) >> 2) << 2);
	// ファイルサイズ
	const int filesize = 2 + sizeof(BITMAPFILEHEADER_) + sizeof(BITMAPINFOHEADER_) + datasize;

	// ビットマップのヘッダ
	BITMAPFILEHEADER_ bmfh = {filesize, 0, 0, 54,};
	BITMAPINFOHEADER_ bmih = {40, width, height, 1, bpp, 0, 0, 0, 0, 0, 0,};

	// データのフォーマット
	int format;
	if (bpp == 24) {
		format = GL_RGB;
	}
	else if (bpp == 32) {
		format = GL_RGBA;
	}
	else {
		std::cerr << "invalid parameter 'bpp'" << std::endl;
		return false;
	}

	// データをもらう
	std::vector<unsigned char> buf(datasize);
	//glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadBuffer(GL_FRONT);
	glReadPixels(0, 0, width, height, format, GL_UNSIGNED_BYTE, &buf[0]);

	// RGB → BGR
	for (int i=0; i<datasize; i+=bpp/8) {
		std::swap(buf[i], buf[i+2]);
	}

	// 出力
	std::ofstream fs(filename , std::ios::out | std::ios::trunc | std::ios::binary);
	if (!fs) {
		std::cerr << "fstream::open() failed." << std::endl;
		return false;
	}

	fs.write("BM", 2);
	fs.write(reinterpret_cast<const char*>(&bmfh), sizeof(BITMAPFILEHEADER_));
	fs.write(reinterpret_cast<const char*>(&bmih), sizeof(BITMAPINFOHEADER_));
	fs.write(reinterpret_cast<const char*>(&buf[0]), datasize);
	fs.close();
	return true;
//	uchar *buf = new uchar[datasize];
//	glReadBuffer(GL_FRONT);
//	glReadPixels(0, 0, width, height, format, GL_UNSIGNED_BYTE, buf);
//	writeColorBitmapfile("screen.bmp", width , height , buf);
//	delete buf;
}

bool DioramaScreenShot::rawdata(const char* filename, const char* filename2, ImageType mimFrameRGB, const char thresh, const int erodeIterations, const int dilateIterations)
{ // raw データで画像処理すると速いか？　2010/11/19 -> そんなことないっぽい 2010/11/20

	const int width    = 640;   // 画像の大きさ
	const int height   = 480;

	ImageType mimFrameRGB_copy;//コピー先

	mimFrameRGB_copy.copy_from(mimFrameRGB);//コピー

	// データサイズ
	int datasize  = width * height;       // pgm(グレースケール)用  **pbm(白黒)用だとうまく出力できても変な画像にしかならない!
	int datasize2 =  width * height * 3;  // ppm(カラー)用

	std::vector<unsigned char> Sbuf(datasize);// HSVのS（彩度）
	unsigned char max, min;

	//画像の左上から右下へと画素値を読み取る, Sを求める
	for(int i = 0; i < mimFrameRGB_copy.size().y ; i++){
		for(int j = 0; j < mimFrameRGB_copy.size().x ; j++){
			// max(R,G,B)を求める
			max = mimFrameRGB_copy[i][j].red;
			if(max < mimFrameRGB_copy[i][j].green) max = mimFrameRGB_copy[i][j].green;
			if(max < mimFrameRGB_copy[i][j].blue) max = mimFrameRGB_copy[i][j].blue;
			// min(R,G,B)を求める
			min = mimFrameRGB_copy[i][j].red;
			if(min > mimFrameRGB_copy[i][j].green) min = mimFrameRGB_copy[i][j].green;
			if(min > mimFrameRGB_copy[i][j].blue) min = mimFrameRGB_copy[i][j].blue;
			if(!max){
				Sbuf[i*width + j]=0;
			} else {
				Sbuf[i*width + j]=(unsigned char)(255*(max - min)/(double)max);
			}
		}
	}

	// データをもらう (グレースケール)
	std::vector<unsigned char> buf(datasize);
	for (int i=0; i<datasize; i++) { // 白黒(マスク)画像
		if(Sbuf[i] < thresh){
			buf[i] = 0;
		}
		else{
			buf[i] = 255;
		}
	}
	// データをもらう (カラー)
	std::vector<unsigned char> buf2(datasize2);
	for(int i = 0; i < datasize2; i+=3){  // カラー画像
		int line = div(i/3, width).quot;
		int row = div(i/3, width).rem;
		buf2[i] = mimFrameRGB_copy[line][row].red;
		buf2[i + 1] = mimFrameRGB_copy[line][row].green;
		buf2[i + 2] = mimFrameRGB_copy[line][row].blue;
	}


	// 出力　グレー(.pgm)
	std::ofstream fs(filename ,/* std::ios::out | std::ios::trunc |*/ std::ios::binary);

	if (!fs) {
		std::cerr << "fstream::open() failed." << std::endl;
		return false;
	}
	// pgm
	fs.write("P5\n640 480\n255\n",15);

	fs.write(reinterpret_cast<const char*>(&buf[0]), datasize);
	fs.close();

	cout << "PGM output complete" << endl;


	// 出力　カラー (.ppm)
	std::ofstream fs2(filename2 ,/* std::ios::out | std::ios::trunc |*/ std::ios::binary);

	if (!fs2) {
		std::cerr << "fstream::open() failed." << std::endl;
		return false;
	}
	// ppm
	fs2.write("P6\n640 480\n255\n",15);

	//fs.write(reinterpret_cast<const char*>(&mimFrameRGB_copy[0][0]), datasize);
	fs2.write(reinterpret_cast<const char*>(&buf2[0]), datasize2);
	fs2.close();

	cout << "PPM output complete" << endl;


	//---------------------------------------------------------------------------------
	// 画像処理(膨張・収縮)
	//---------------------------------------------------------------------------------

	char* cvfilename = const_cast<char *>(filename); //OpenCVで処理する画像(膨張・収縮処理)
	cout << "cvfilename is " << cvfilename << endl;

	IplImage *InputImage = cvLoadImage(cvfilename, -1); // 入力画像
	IplImage *OutputImage = cvCloneImage(InputImage); // 出力画像


//　cvSmooth(OutputImage, OutputImage, CV_GAUSSIAN, 3); //ぼかし
// cvErode, cvDilate 関数内で処理回数を指定することもできるが実際のピクセルとずれてしまうことがあるとかないとか
// とりあえず，そのまま使うが，まずければ for 文で繰り返す感じに変更する
// 膨張処理は収縮処理より少し多めに
//	cvErode(OutputImage, OutputImage, NULL, 6); // 収縮処理 * 6
//	cvSaveImage("MaskplusCVErode.pgm", OutputImage); // 保存
//
//	cvDilate(OutputImage, OutputImage, NULL, 8); // 膨張処理　* 8
//	cvSaveImage("MaskplusCVErodeplusDilate.pgm", OutputImage); // 保存


	for(int i=0; i < erodeIterations; i++){
	 cvErode(OutputImage, OutputImage, NULL, 1); // 収縮処理
	}
	cvSaveImage("MaskplusCVErode.pgm", OutputImage); // 保存
	for(int i=0; i < dilateIterations; i++){
	 cvDilate(OutputImage, OutputImage, NULL, 1); // 膨張処理
	}
	cvSaveImage("MaskplusCVErodeplusDilate.pgm", OutputImage); // 保存

	cvReleaseImage(&InputImage); // メモリ領域開放
	cvReleaseImage(&OutputImage);

	cout << "Dilate and erode processed output complete" <<endl;



	/*	画像の読み込み (テスト用)*/
//	IplImage *InputImage2 = cvLoadImage("MaskplusCVErodeplusDilate.pgm",4);
//    IplImage *BufImage = cvCloneImage(InputImage2);
//
//    std::ofstream fs5("pixLog.txt" /* std::ios::out | std::ios::trunc |*/);
//
//    int num0, num255;
//    num0 = 0; num255 = 0;
//
//    for (int y = 0; y < 480; y++){
//    	for(int x = 0; x < 640; x++){
//    		unsigned char *line = (unsigned char*)BufImage->imageData + y * BufImage->widthStep;
//    		unsigned char *pixel = (uchar* )(line + x * BufImage->depth/8 * BufImage->nChannels);
//
//    		if (0 == (int)pixel[0] ) num0++;
//    		else if ( (int)pixel[0] == 255) num255++;
//    		else cout << "err = not 0 and not 255" << endl;
//    	}
//    }
//
//    cout << "num0 =" << num0 << ", num255 =" << num255 << ", sum = " << num0+num255 << endl;
//    fs5.close();
//
//    cout << "BufImage->depth/8 = " << BufImage->depth/8 << endl;
//    cout << "BufImage->nChannels = " << BufImage->nChannels << endl;
//
//    cvReleaseImage(&InputImage2);
//    cvReleaseImage(&BufImage);
//
	/*	画像の読み込み (テスト用) ここまで*/


	return true;
}
