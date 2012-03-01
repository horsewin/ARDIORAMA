/*
 * BrushAction.h
 *
 *  Created on: 2010/12/14
 *      Author: umakatsu
 */

#ifndef BRUSHACTION_H_
#define BRUSHACTION_H_

#include "Stroke/Stroke.h"

#include "PTAMTracking/ATANCamera.h"

//MatrixとSE3を使う
//#include <TooN/numerics.h>
#include <TooN/numhelpers.h>
#include <TooN/helpers.h>
#include <cvd/gl_helpers.h>

namespace PTAMM{
class VideoData;

class BrushAction{
public:
	BrushAction(int width , int height , VideoData *video);
	~BrushAction();
	void excludeAction(const TooN::SE3<> & currentPose , Stroke *silhouette , ATANCamera & camera);
	void includeAction(const TooN::SE3<> & currentPose , Stroke *silhouette , ATANCamera & camera);
    unsigned char *getBinary() const
    {
        return binary;
    }

    int getForeground_coord_x() const
    {
        return foreground_coord_x;
    }

    int getForeground_coord_y() const
    {
        return foreground_coord_y;
    }

    short int *getResult() const
    {
        return result;
    }

private:
	int w;
	int h;
	int foreground_coord_x;
	int foreground_coord_y;
	unsigned char * 	binary;
	short int * result;
	VideoData *vd;
};
}
#endif /* BRUSHACTION_H_ */
