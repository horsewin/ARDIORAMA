// -*- c++ -*-
// Copyright 2009 Isis Innovation Limited
//
// PTAMTracking/MapViewer.h
//
// Defines the MapViewer class
//
// This defines a simple map viewer widget, which can draw the 
// current map and the camera/keyframe poses within it.
//
#ifndef __MAP_VIEWER_H
#define __MAP_VIEWER_H

#include "PTAMTracking/Map.h"
#include "common.h"
#include <TooN/TooN.h>
#include <TooN/se3.h>
//#include <TooN/numerics.h>
#include <TooN/numhelpers.h>
#include <sstream>
#include "Window/GLWindow2.h"

#include <vector>
#include <boost/multi_array.hpp>

namespace PTAMM {

using namespace TooN;

class Map;
class MarchingCubes;
class Stroke;
class Construction;

class MapViewer
{
  public:
    MapViewer(std::vector<Map*> &maps, Map *map, GLWindow2 &glw);
    void DrawMap(SE3<> se3CamFromWorld);
    void DrawMap(SE3<> se3CamFromWorld , MarchingCubes * mc , const int & mode);
    void DrawMap(SE3<> se3CamFromWorld , MarchingCubes * mc);
    std::string GetMessageForUser();
    void SwitchMap( Map * map, bool bForce = false );

    void ViewNextMap();
    void ViewPrevMap();
    void ViewCurrentMap();
    
  protected:
    std::vector<Map*> & mvpMaps;     // Reference to all of the maps
    Map * mpMap, *mpViewingMap;      // the active tracking map, and the map being viewed
    GLWindow2 &mGLWindow;
    
    void DrawGrid();
    void DrawMapDots();
    void DrawCamera(SE3<> se3, bool bSmall=false);
    void SetupFrustum();
    void SetupModelView(SE3<> se3WorldFromCurrent = SE3<>());

    //edited by Umakatsu 2011/1/14
    void DrawVoxel( MarchingCubes * mc);
    void DrawVoxel( MarchingCubes * mc , const int & mode);

    Vector<3> mv3MassCenter;
    SE3<> mse3ViewerFromWorld;

    std::ostringstream mMessageForUser;
    bool mbBrowseMode;                            // Map browsing mode enabled?
};

}

#endif
