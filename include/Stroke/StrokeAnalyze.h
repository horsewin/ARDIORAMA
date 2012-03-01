/*
 * Stroke/StrokeAnalyze.h
 *
 *  Created on: 2011/03/26
 *      Author: umakatsu
 */

#ifndef STROKEANALYZE_H_
#define STROKEANALYZE_H_

#include "common.h"
#include "Stroke/Stroke.h"

namespace PTAMM{

class VideoData;
class StrokeAnalyze{
public:
	StrokeAnalyze( int width , int height  );
	~StrokeAnalyze( void );

	void checkLetter( Stroke * stroke);
	bool isValidAnalyze( void ) const;
	char getResult( void );

private:
	int w; //image width
	int h; //image height
	char letter;
	bool valid;
	VideoData 	*vd;

	void extractStrokePart(double *color);
};
}
#endif /* STROKEANALYZE_H_ */
