// -*- c++ -*-
// Copyright 2009 Isis Innovation Limited
//
// System.h
//
// Defines the System class
//
// This stores the main functional classes of the system, like the
// mapmaker, map, tracker etc, and spawns the working threads.
//
#ifndef __SYSTEM_H
#define __SYSTEM_H
#include "VideoSource/VideoSource.h"
#include "PTAMTracking/MapPoint.h"
#include "Window/GLWindow2.h"
#include "ImageType.h"
#include "ImageSize.h"

#include <gvars3/instances.h>

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>

#include "Button.h"
/* 共通のパラメータの設定ヘッダ 2010/11/15 */
#include "common.h"

#include "ARMM/PointgreyCamera.h"
namespace PTAMM {

class ATANCamera;
class Map;
class MapMaker;
class Tracker;
class TrackerDrawable;
class ARDriver;
class MapViewer;
class MapSerializer;

/* Added by Tateishi's */
class Stroke;
class StrokeGenerator;
class Switcher;
class TrackerDrawable;

/* added by Umakatsu */
class GraphCut;
class ForegroundContents;
class VideoData;
class GMM;
class Segmentation;
class Construction;
class StrokeAnalyze;

class System
{
  public:
    System();
    ~System();
    void Run();
  
  private:
    static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);  //process a console command
    bool GetSingleParam(int &nAnswer, std::string sCommand, std::string sParams);          //Extract an int param from a command param
    bool SwitchMap( int nMapNum, bool bForce = false );                                    // Switch to a particular map.
    void NewMap();                                  // Create a new map and move all elements to it
    bool DeleteMap( int nMapNum );                  // Delete a specified map
    void ResetAll();                                // Wipes out ALL maps, returning system to initial state
    void StartMapSerialization(std::string sCommand, std::string sParams);   //(de)serialize a map
    void DrawMapInfo();                             // draw a little info box about the maps
    void SaveFIFO();                                // save the video out to a FIFO (save to disk)

	void reset( void );
	void setCaption(std::string & sCaption , bool bDrawMap);
	void flagReset( void * tmp );
	void stopOtherThread( void * tmp);
	void readyOtherThread( void * tmp);
    
  private:
    VideoSource mVideoSource;                       // The video image source
    GLWindow2 mGLWindow;                            // The OpenGL window
    CVD::Image<CVD::Rgb<CVD::byte> > mimFrameRGB;   // The RGB image used for AR
    CVD::Image<CVD::byte> mimFrameBW;               // The Black and white image for tracking/mapping

    std::vector<Map*> mvpMaps;                      // The set of maps
    Map *mpMap;                                     // The current map
    MapMaker *mpMapMaker;                           // The map maker
//    Tracker *mpTracker;                           // The tracker
    TrackerDrawable * mpTracker;					   // The tracker added by Umakatsu
    ARMM::PointgreyCamera *mpCamera;                // The camera model
    ARDriver *mpARDriver;                           // The AR Driver
    MapViewer *mpMapViewer;                         // The Map Viewer
    MapSerializer *mpMapSerializer;                 // The map serializer for saving and loading maps

    bool mbDone;                                    // Kill?
    
    GVars3::gvar3<int> mgvnLockMap;                 // Stop a map being edited - i.e. keyframes added, points updated
    GVars3::gvar3<int> mgvnDrawMapInfo;             // Draw map info on the screen
    
#ifdef _LINUX
    GVars3::gvar3<int> mgvnSaveFIFO;                // Output to a FIFO (make a video)
    GVars3::gvar3<int> mgvnBitrate;                 // Bitrate to encode at
#endif
    //以下,追加部分 by Tateishi
    	StrokeGenerator *		mpStrokeGenerator;	//モデルを生成する
    	Switcher *				mpSwitcher;			//配置と生成の切り変え
    	uint 					modelCount;			//生成したモデルの総数
    	uint					modelTarget;			//現在落とすのためのモデル
    	double					magnification;		//モデルの倍率

    	/* マウスの状態を取得 */
    	Button mouseLeftButton;
    	Button mouseRightButton;
    	Button mouseMiddleDown;
    	Button mouseMiddleUp;
    	Button mouseMiddleButton;

    	std::vector< Stroke *>  strokes;	//生成されたストロークの群

    //	ODERoot 			ode;		//ODEエンジン
    	ImageSize			img_size;	//カメラの解像度

    //	static void ODECallBackCollision( void *  data, dGeomID geomA, dGeomID geomB );
    /* 以上,ARDioramaオリジナル */

    /*	以下からは 追加部分 */
    	bool bIgnoreMouseAction;	//map表示の際にはマウスアクションを無視する(true時)     	//yasuhara edited 2010/10/27 -

    	ForegroundContents *fContents;
    	bool	create_object;				// 3Dオブジェクトが生成されている途中かどうか 2010/11/22
    	bool	bgraph;
    	bool	bBrush;
    	bool	bStartSCM;
    	bool	bRecovery;

    	bool	autosegment;
    	bool	viewmode;

    	bool	run_other_command;
    	// cache variable
    	mode	sysmode;
    	state	sSegment;

    	// テクスチャ画像閲覧用コマンド
    	bool	bShowTexture;
    	unsigned int	count_texture;
    	MapPoint * visualPoint;

    	Stroke *draw_frame;	//edited by umakatsu 2011/1/17
    	std::deque<Vertex> vertex;

    	//for the segmentation thread
    	Segmentation *segment;
    	//for the construction thread
    	Construction *construct;
    	StrokeAnalyze * input_s;

    	std::ostringstream mMessageForUser;
	};
}

#endif
