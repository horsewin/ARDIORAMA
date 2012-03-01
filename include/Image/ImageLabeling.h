/*
 * Image/ImageLabeling.h
 *
 *  Created on: 2010/11/18
 *      Author: umakatsu
 *      2-dimensional binary imageに対してのラベリングを行うためのクラスのヘッダ
 *		またメソッドに輪郭抽出を行う処理のメソッドも備えている
 */

#ifndef IMAGELABELING_H_
#define IMAGELABELING_H_

#include "common.h"
#include "Labeling.h"
#include <vector>

namespace PTAMM{

class ImageLabeling{
private:
	int w;					// 画像横サイズ
	int h;					// 画像縦サイズ
	unsigned char *src;		// 入力画像
	short *det;			// 処理結果画像
	LabelingBS	labeling;
public:
	ImageLabeling( void );
	ImageLabeling(int size_x , int size_y , unsigned char *input);
	~ImageLabeling( void );
	void laplacian( short **result , short int **region);
	void run( void );
    unsigned char *getSrc() const
    {
        return src;
    }

    void setSrc(unsigned char *src)
    {
        this->src = src;
    }

    short *getDet() const{ return det;}
};
}
#endif /* IMAGELABELING_H_ */
