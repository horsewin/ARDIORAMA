#include "PTAMTracking/TrackerDrawable.h"
#include "OpenGL.h"
#include "PTAMTracking/TrackerData.h"

#include <cvd/gl_helpers.h>
#include <gvars3/instances.h>
#include <gvars3/GStringUtil.h>

#include "PTAMTracking/MEstimator.h"

#include <fstream>
#include <iostream>

namespace PTAMM{

using namespace GVars3;
TrackerDrawable::TrackerDrawable(CVD::ImageRef irVideoSize, const ATANCamera &c, std::vector<Map*> &maps, Map *m, MapMaker &mm)
  :Tracker(irVideoSize, c, maps, m, mm)
{}


TrackerDrawable::~TrackerDrawable()
{}


void TrackerDrawable::TrackFrame(CVD::Image<CVD::byte> &imFrame, bool bDraw,
				 bool bDrawStroke, Vector<2> &mouse)
{
  mbDraw = bDraw;
  mMessageForUser.str("");   // Wipe the user message clena

  // Take the input video image, and convert it into the tracker's keyframe struct
  // This does things like generate the image pyramid and find FAST corners
  mCurrentKF.mMeasurements.clear();
  mCurrentKF.MakeKeyFrame_Lite(imFrame);

  // Update the small images for the rotation estimator
  static gvar3<double> gvdSBIBlur("Tracker.RotationEstimatorBlur", 0.75, SILENT);
  static gvar3<int> gvnUseSBI("Tracker.UseRotationEstimator", 1, SILENT);
  mbUseSBIInit = *gvnUseSBI;
  if(!mpSBIThisFrame)
    {
      mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
      mpSBILastFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
    }
  else
    {
      delete  mpSBILastFrame;
      mpSBILastFrame = mpSBIThisFrame;
      mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
    }
  
  // From now on we only use the keyframe struct!
  mnFrame++;
  
  if(mbDraw)
    {
      CVD::glDrawPixels(mCurrentKF.aLevels[0].im);
      if(GV2.GetInt("Tracker.DrawFASTCorners",0, SILENT))
	{
	  glColor3f(1,0,1);  glPointSize(1); glBegin( GL_POINTS );
	  for(unsigned int i=0; i<mCurrentKF.aLevels[0].vCorners.size(); i++)
	    {
	      CVD::glVertex(mCurrentKF.aLevels[0].vCorners[i]);
	    }
	  glEnd();
	}
    }
  // Decide what to do - if there is a map, try to track the map ...
  if(mpMap->IsGood())
    {
      if(mnLostFrames < 3)  // .. but only if we're not lost!
	{
	  if(mbUseSBIInit)
	    CalcSBIRotation();
	  ApplyMotionModel();       // 
	  TrackMap(bDrawStroke, mouse);               //  These three lines do the main tracking work.
	  UpdateMotionModel();      // 
	  
	  AssessTrackingQuality();  //  Check if we're lost or if tracking is poor.
	  
	  { // Provide some feedback for the user:
	    mMessageForUser << "Tracking Map, quality ";
	    if(mTrackingQuality == GOOD)  mMessageForUser << "good.";
	    if(mTrackingQuality == DODGY) mMessageForUser << "poor.";
	    if(mTrackingQuality == BAD)   mMessageForUser << "bad.";
	    mMessageForUser << " Found:";
	    for(int i=0; i<LEVELS; i++) mMessageForUser << " " << manMeasFound[i] << "/" << manMeasAttempted[i];
	    //	    mMessageForUser << " Found " << mnMeasFound << " of " << mnMeasAttempted <<". (";
	    mMessageForUser << " Map: " << mpMap->vpPoints.size() << "P, " << mpMap->vpKeyFrames.size() << "KF";
	  }
	  
	  // Heuristics to check if a key-frame should be added to the map:
	  if(mTrackingQuality == GOOD &&
	     mMapMaker.NeedNewKeyFrame(mCurrentKF) &&
	     mnFrame - mnLastKeyFrameDropped > 20  &&
	     mMapMaker.QueueSize() < 3)
	    {
	      mMessageForUser << " Adding key-frame.";
	      AddNewKeyFrame();
	    };
	}
      else  // what if there is a map, but tracking has been lost?
	{
	  mMessageForUser << "** Attempting recovery **.";
	  if(AttemptRecovery())
	    {
	      TrackMap(bDrawStroke, mouse);
	      AssessTrackingQuality();
	    }
	}
      if(mbDraw){
	//RenderGrid();
      }
    } 
  else // If there is no map, try to make one.
   TrackForInitialMap(); 
  
  // GUI interface
  while(!mvQueuedCommands.empty())
    {
      GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
      mvQueuedCommands.erase(mvQueuedCommands.begin());
    }
};

namespace LOCAL{
  inline double distance(Vector<2> &src, Vector<2> &dst)
  {
    return ( (dst[0] - src[0]) * (dst[0] - src[0]) + (dst[1] - (src[1]) ) * (dst[1] - (src[1]) ) );
  }
}
// TrackMap is the main purpose of the Tracker.
// It first projects all map points into the image to find a potentially-visible-set (PVS);
// Then it tries to find some points of the PVS in the image;
// Then it updates camera pose according to any points found.
// Above may happen twice if a coarse tracking stage is performed.
// Finally it updates the tracker's current-frame-KeyFrame struct with any
// measurements made.
// A lot of low-level functionality is split into helper classes:
// class TrackerData handles the projection of a MapPoint and stores intermediate results;
// class PatchFinder finds a projected MapPoint in the current-frame-KeyFrame.
void TrackerDrawable::TrackMap(bool bDrawStroke, Vector<2> &mouse)
{
  // Some accounting which will be used for tracking quality assessment:
  for(int i=0; i<LEVELS; i++){
    manMeasAttempted[i] = manMeasFound[i] = 0;
  }
  
  // The Potentially-Visible-Set (PVS) is split into pyramid levels.
  std::vector<TrackerData*> avPVS[LEVELS]; 
  for(int i=0; i<LEVELS; i++){
    avPVS[i].reserve(500);
  }

  double min_distance = 1.0e20;//十分大きな数字
  int min = 0;

  
  featureProjected.clear();
  featureProjected3d.clear();
  featureMapPoint.clear();

  // For all points in the map..
  for(unsigned int i=0; i<mpMap->vpPoints.size(); i++){
    MapPoint &p= *(mpMap->vpPoints[i]);
    // Ensure that this map point has an associated TrackerData struct.
    if(!p.pTData){
      p.pTData = new TrackerData(&p);
    }
    TrackerData &TData = *p.pTData;
      
    // Project according to current view, and if it's not in the image, skip.
    TData.Project(mse3CamFromWorld, mCamera);
    if(!TData.bInImage){
      continue;
    }

    featureProjected.push_back(std::make_pair(TData.v2Image[0], TData.v2Image[1]));
    featureProjected3d.push_back(p.v3WorldPos);
    featureMapPoint.push_back(mpMap->vpPoints[i]);
    

    if(bDrawStroke){
      double d = LOCAL::distance(mouse, TData.v2Image);
      if(min_distance > d){
	min_distance = d;
	min = i;
      }
      
    }
    
    
    // Caculate camera projection derivatives of this point.
    TData.GetDerivsUnsafe(mCamera);
    
    // And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..
    TData.nSearchLevel = TData.Finder.CalcSearchLevelAndWarpMatrix(TData.Point, mse3CamFromWorld, TData.m2CamDerivs);
    if(TData.nSearchLevel == -1){
      continue;   // a negative search pyramid level indicates an inappropriate warp for this view, so skip.
    }
    
    // Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
    TData.bSearched = false;
    TData.bFound = false;
    avPVS[TData.nSearchLevel].push_back(&TData);
  };
  

  if(bDrawStroke){
    for(int i = 0; i < 3; i++){
      this->mvDrawedPoint[i] = mpMap->vpPoints[min]->v3WorldPos[i];
    }
  }
  
  // Next: A large degree o faffing about and deciding which points are going to be measured!
  // First, randomly shuffle the individual levels of the PVS.
  for(int i=0; i<LEVELS; i++){
    random_shuffle(avPVS[i].begin(), avPVS[i].end());
  }
  // The next two data structs contain the list of points which will next 
  // be searched for in the image, and then used in pose update.
  std::vector<TrackerData*> vNextToSearch;
  std::vector<TrackerData*> vIterationSet;
  
  // Tunable parameters to do with the coarse tracking stage:
  static gvar3<unsigned int> gvnCoarseMin("Tracker.CoarseMin", 20, SILENT);   // Min number of large-scale features for coarse stage
  static gvar3<unsigned int> gvnCoarseMax("Tracker.CoarseMax", 60, SILENT);   // Max number of large-scale features for coarse stage
  static gvar3<unsigned int> gvnCoarseRange("Tracker.CoarseRange", 30, SILENT);       // Pixel search radius for coarse features
  static gvar3<int> gvnCoarseSubPixIts("Tracker.CoarseSubPixIts", 8, SILENT); // Max sub-pixel iterations for coarse features
  static gvar3<int> gvnCoarseDisabled("Tracker.DisableCoarse", 0, SILENT);    // Set this to 1 to disable coarse stage (except after recovery)
  static gvar3<double> gvdCoarseMinVel("Tracker.CoarseMinVelocity", 0.006, SILENT);  // Speed above which coarse stage is used.
  
  unsigned int nCoarseMax = *gvnCoarseMax;
  unsigned int nCoarseRange = *gvnCoarseRange;
  
  mbDidCoarse = false;

  // Set of heuristics to check if we should do a coarse tracking stage.
  bool bTryCoarse = true;
  if(*gvnCoarseDisabled || 
     mdMSDScaledVelocityMagnitude < *gvdCoarseMinVel  ||
     nCoarseMax == 0)
    bTryCoarse = false;
  if(mbJustRecoveredSoUseCoarse)
    {
      bTryCoarse = true;
      nCoarseMax *=2;
      nCoarseRange *=2;
      mbJustRecoveredSoUseCoarse = false;
    };
      
  // If we do want to do a coarse stage, also check that there's enough high-level 
  // PV map points. We use the lowest-res two pyramid levels (LEVELS-1 and LEVELS-2),
  // with preference to LEVELS-1.
  if(bTryCoarse && avPVS[LEVELS-1].size() + avPVS[LEVELS-2].size() > *gvnCoarseMin )
    {
      // Now, fill the vNextToSearch struct with an appropriate number of 
      // TrackerDatas corresponding to coarse map points! This depends on how many
      // there are in different pyramid levels compared to CoarseMin and CoarseMax.
      
      if(avPVS[LEVELS-1].size() <= nCoarseMax) 
	{ // Fewer than CoarseMax in LEVELS-1? then take all of them, and remove them from the PVS list.
	  vNextToSearch = avPVS[LEVELS-1];
	  avPVS[LEVELS-1].clear();
	}
      else
	{ // ..otherwise choose nCoarseMax at random, again removing from the PVS list.
	  for(unsigned int i=0; i<nCoarseMax; i++)
	    vNextToSearch.push_back(avPVS[LEVELS-1][i]);
	  avPVS[LEVELS-1].erase(avPVS[LEVELS-1].begin(), avPVS[LEVELS-1].begin() + nCoarseMax);
	}
      
      // If didn't source enough from LEVELS-1, get some from LEVELS-2... same as above.
      if(vNextToSearch.size() < nCoarseMax)
	{
	  unsigned int nMoreCoarseNeeded = nCoarseMax - vNextToSearch.size();
	  if(avPVS[LEVELS-2].size() <= nMoreCoarseNeeded)
	    {
	      vNextToSearch = avPVS[LEVELS-2];
	      avPVS[LEVELS-2].clear();
	    }
	  else
	    {
	      for(unsigned int i=0; i<nMoreCoarseNeeded; i++)
		vNextToSearch.push_back(avPVS[LEVELS-2][i]);
	      avPVS[LEVELS-2].erase(avPVS[LEVELS-2].begin(), avPVS[LEVELS-2].begin() + nMoreCoarseNeeded);
	    }
	}
      // Now go and attempt to find these points in the image!
      unsigned int nFound = SearchForPoints(vNextToSearch, nCoarseRange, *gvnCoarseSubPixIts);
      vIterationSet = vNextToSearch;  // Copy over into the to-be-optimised list.
      if(nFound >= *gvnCoarseMin)  // Were enough found to do any meaningful optimisation?
	{
	  mbDidCoarse = true;
	  for(int iter = 0; iter<10; iter++) // If so: do ten Gauss-Newton pose updates iterations.
	    {
	      if(iter != 0)
		{ // Re-project the points on all but the first iteration.
		  for(unsigned int i=0; i<vIterationSet.size(); i++){
		    if(vIterationSet[i]->bFound){
		      vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
		    }
		  }
		}
	      for(unsigned int i=0; i<vIterationSet.size(); i++)
		if(vIterationSet[i]->bFound)
		  vIterationSet[i]->CalcJacobian();
	      double dOverrideSigma = 0.0;
	      // Hack: force the MEstimator to be pretty brutal 
	      // with outliers beyond the fifth iteration.
	      if(iter > 5)
		dOverrideSigma = 1.0;
	      
	      // Calculate and apply the pose update...
	      Vector<6> v6Update = 
		CalcPoseUpdate(vIterationSet, dOverrideSigma);
	      mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
	    };
	}
    };
  
  // So, at this stage, we may or may not have done a coarse tracking stage.
  // Now do the fine tracking stage. This needs many more points!
  
  int nFineRange = 10;  // Pixel search range for the fine stage. 
  if(mbDidCoarse)       // Can use a tighter search if the coarse stage was already done.
    nFineRange = 5;
  
  // What patches shall we use this time? The high-level ones are quite important,
  // so do all of these, with sub-pixel refinement.
  {
    int l = LEVELS - 1;
    for(unsigned int i=0; i<avPVS[l].size(); i++)
      avPVS[l][i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
    SearchForPoints(avPVS[l], nFineRange, 8);
    for(unsigned int i=0; i<avPVS[l].size(); i++)
      vIterationSet.push_back(avPVS[l][i]);  // Again, plonk all searched points onto the (maybe already populate) vIterationSet.
  };
  
  // All the others levels: Initially, put all remaining potentially visible patches onto vNextToSearch.
  vNextToSearch.clear();
  for(int l=LEVELS - 2; l>=0; l--)
    for(unsigned int i=0; i<avPVS[l].size(); i++)
      vNextToSearch.push_back(avPVS[l][i]);
  
  // But we haven't got CPU to track _all_ patches in the map - arbitrarily limit 
  // ourselves to 1000, and choose these randomly.
  static gvar3<int> gvnMaxPatchesPerFrame("Tracker.MaxPatchesPerFrame", 1000, SILENT);
  int nFinePatchesToUse = *gvnMaxPatchesPerFrame - vIterationSet.size();
  if((int) vNextToSearch.size() > nFinePatchesToUse)
    {
      random_shuffle(vNextToSearch.begin(), vNextToSearch.end());
      vNextToSearch.resize(nFinePatchesToUse); // Chop!
    };
  
  // If we did a coarse tracking stage: re-project and find derivs of fine points
  if(mbDidCoarse)
    for(unsigned int i=0; i<vNextToSearch.size(); i++)
      vNextToSearch[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
  
  // Find fine points in image:
  SearchForPoints(vNextToSearch, nFineRange, 0);
  // And attach them all to the end of the optimisation-set.
  for(unsigned int i=0; i<vNextToSearch.size(); i++)
    vIterationSet.push_back(vNextToSearch[i]);
  
  // Again, ten gauss-newton pose update iterations.
  Vector<6> v6LastUpdate;
  v6LastUpdate = Zeros;
  for(int iter = 0; iter<10; iter++)
    {
      bool bNonLinearIteration; // For a bit of time-saving: don't do full nonlinear
                                // reprojection at every iteration - it really isn't necessary!
      if(iter == 0 || iter == 4 || iter == 9)
	bNonLinearIteration = true;   // Even this is probably overkill, the reason we do many
      else                            // iterations is for M-Estimator convergence rather than 
	bNonLinearIteration = false;  // linearisation effects.
      
      if(iter != 0)   // Either way: first iteration doesn't need projection update.
        {
	  if(bNonLinearIteration)
	    {
	      for(unsigned int i=0; i<vIterationSet.size(); i++)
		if(vIterationSet[i]->bFound)
		  vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
	    }
	  else
	    {
	      for(unsigned int i=0; i<vIterationSet.size(); i++)
		if(vIterationSet[i]->bFound)
		  vIterationSet[i]->LinearUpdate(v6LastUpdate);
	    };
	}
      
      if(bNonLinearIteration)
	for(unsigned int i=0; i<vIterationSet.size(); i++)
	  if(vIterationSet[i]->bFound)
	    vIterationSet[i]->CalcJacobian();

      // Again, an M-Estimator hack beyond the fifth iteration.
      double dOverrideSigma = 0.0;
      if(iter > 5)
	dOverrideSigma = 16.0;
      
      // Calculate and update pose; also store update vector for linear iteration updates.
      Vector<6> v6Update = 
	CalcPoseUpdate(vIterationSet, dOverrideSigma, iter==9);
      mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
      v6LastUpdate = v6Update;
    };
  
  if(mbDraw)
    {
      glPointSize(6);
      glEnable(GL_BLEND);
      glEnable(GL_POINT_SMOOTH);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glBegin( GL_POINTS );
      for(std::vector<TrackerData*>::reverse_iterator it = vIterationSet.rbegin();
	  it!= vIterationSet.rend();
	  it++)
	{
	  if(! (*it)->bFound){
	    continue;
	  }
	  CVD::glColor(gavLevelColors[(*it)->nSearchLevel]);
	  CVD::glVertex((*it)->v2Image);
	}
      glEnd();
      glDisable(GL_BLEND);
    }
  
  // Update the current keyframe with info on what was found in the frame.
  // Strictly speaking this is unnecessary to do every frame, it'll only be
  // needed if the KF gets added to MapMaker. Do it anyway.
  // Export pose to current keyframe:
  mCurrentKF.se3CfromW = mse3CamFromWorld;
  
  // Record successful measurements. Use the KeyFrame-Measurement struct for this.
  mCurrentKF.mMeasurements.clear();
  for(std::vector<TrackerData*>::iterator it = vIterationSet.begin();
      it!= vIterationSet.end(); 
      it++)
    {
      if(! (*it)->bFound)
	continue;
      Measurement m;
      m.v2RootPos = (*it)->v2Found;
      m.nLevel = (*it)->nSearchLevel;
      m.bSubPix = (*it)->bDidSubPix; 
      mCurrentKF.mMeasurements[& ((*it)->Point)] = m;
    }
  
  // Finally, find the mean scene depth from tracked features
  {
    double dSum = 0;
    double dSumSq = 0;
    int nNum = 0;
    for(std::vector<TrackerData*>::iterator it = vIterationSet.begin();
	it!= vIterationSet.end(); 
	it++)
      if((*it)->bFound)
	{
	  double z = (*it)->v3Cam[2];
	  dSum+= z;
	  dSumSq+= z*z;
	  nNum++;
	};
    if(nNum > 20)
      {
	mCurrentKF.dSceneDepthMean = dSum/nNum;
	mCurrentKF.dSceneDepthSigma = sqrt((dSumSq / nNum) - (mCurrentKF.dSceneDepthMean) * (mCurrentKF.dSceneDepthMean));
      }
  }
};

void TrackerDrawable::GetDrawedPoint(Vector<3> &ret){
  for(int i = 0; i < 3; i++){
    ret[i] = mvDrawedPoint[i];
  }
}


const std::vector<std::pair<double, double> >& TrackerDrawable::getFeaturePoint(void){
  return this->featureProjected;
}

const std::vector<TooN::Vector<3> > & TrackerDrawable::getFeaturePoint3d(void){
  return this->featureProjected3d;
}

const std::vector<MapPoint*>& TrackerDrawable::getMapPoint(void){
  return this->featureMapPoint;
}
}
