#ifndef INCLUDED__TRACKER_DRAWABLE_H_
#define INCLUDED__TRACKER_DRAWABLE_H_
#include "PTAMTracking/Tracker.h"
#include <vector>
#include <utility>

namespace PTAMM{

class TrackerDrawable : public Tracker {
 public:
    TrackerDrawable(CVD::ImageRef irVideoSize, const ATANCamera &c, std::vector<Map*> &maps, Map *m, MapMaker &mm); // 変更前は Map &mだった
    ~TrackerDrawable();
    
    void TrackFrame(CVD::Image<CVD::byte> &imFrame, bool bDraw, bool bDrawStroke, Vector<2> &mouse);
    void GetDrawedPoint(TooN::Vector<3> &ret);

    const std::vector<std::pair<double, double> >& getFeaturePoint(void);
    const std::vector<TooN::Vector<3> >& getFeaturePoint3d(void);
    const std::vector<MapPoint*>&        getMapPoint(void);
 private:
    void TrackMap(bool bDrawStroke, Vector<2> &mouse);
    Vector<3> mvDrawedPoint;
    bool mbDrawed;

    //カメラの平面上二投影された特徴点
    std::vector<std::pair<double, double> > featureProjected;
    std::vector<TooN::Vector<3> > featureProjected3d;//上の奴の3d座標
    std::vector<MapPoint*> featureMapPoint;
};
}
#endif
