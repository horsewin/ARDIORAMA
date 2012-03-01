// Copyright 2009 Isis Innovation Limited
/*
 * System.cc
 *
 *      Author: Atsushi Umakatsu
 *      PTAM's System.cc -> Tateishi's System.cc -> Umakatsu's System.cc
 *      2011.5.13 PTAMMに移行
 */

#include "System.h"
#include "OpenGL.h"
#include <gvars3/GStringUtil.h>

#include <boost/foreach.hpp>
#include <sys/types.h>
#include <dirent.h>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <vector>

#include "PTAMTracking/ATANCamera.h"
#include "PTAMTracking/MapMaker.h"
#include "PTAMTracking/Tracker.h"
#include "PTAMTracking/ARDriver.h"
#include "PTAMTracking/MapViewer.h"
#include "PTAMTracking/MapSerializer.h"
#include "Demo/Games.h"

#ifdef _LINUX
#include <fcntl.h>
#endif
#ifdef WIN32
#include <Windows.h>
#endif

//Tateishi's original code
#include "PTAMTracking/TrackerDrawable.h"
#include "Stroke/StrokeGenerator.h"
#include "Stroke/StrokeAnalyze.h"
#include "Stroke/Stroke.h"
#include "Model/Model.h"
#include "Image/Bitmap.h"
#include "Image/Switcher.h"

/* created by Umakatsu 2010/10/28 */
#include "Image/ForegroundContents.h"
#include "Reconstruction/Tables.h"
#include "Reconstruction/GraphCut.h"		// グラフカットに関するヘッダ 2010/11/3
#include "Image/VideoData.h"		// ビデオ情報を格納するヘッダ（#TODO 削除）2010/11/12
#include "Reconstruction/BrushAction.h"		// Brush関連の処理をするヘッダ
#include "Reconstruction/Segmentation.h" 	// Segmentation partを実現するヘッダ
#include "Reconstruction/Construction.h" 	// Construction partを実現するヘッダ
#include "Reconstruction/MarchingCubes.h"	// Marching Cubes Methodを実行するオブジェクトに関するヘッダ
#include "ARMM/ARMM_Client.h"

namespace PTAMM {
using namespace CVD;
using namespace std;
using namespace GVars3;

//const definition
#define DEBUG 0
#define READPIXELTEST 0
#define LETTER 0
#define DRAW_STROKE 0

const double SCALE_CONST = 0.05;
ARMM::ARMM *armm;

System::System() :
	mGLWindow(mVideoSource.Size(), "AR Diorama") {
	GUI.RegisterCommand("exit", GUICommandCallBack, this);
	GUI.RegisterCommand("quit", GUICommandCallBack, this);
	//PTAMM commands
	GUI.RegisterCommand("SwitchMap", GUICommandCallBack, this);
	GUI.RegisterCommand("NewMap", GUICommandCallBack, this);
	GUI.RegisterCommand("DeleteMap", GUICommandCallBack, this);
	GUI.RegisterCommand("ResetAll", GUICommandCallBack, this);

	GUI.RegisterCommand("LoadMap", GUICommandCallBack, this);
	GUI.RegisterCommand("SaveMap", GUICommandCallBack, this);
	GUI.RegisterCommand("SaveMaps", GUICommandCallBack, this);

	GV2.Register(mgvnLockMap, "LockMap", 0, SILENT);
	GV2.Register(mgvnDrawMapInfo, "MapInfo", 0, SILENT);

#ifdef _LINUX
	GV2.Register(mgvnSaveFIFO, "SaveFIFO", 0, SILENT);
	GV2.Register(mgvnBitrate, "Bitrate", 15000, SILENT);
#endif

	GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
	// Umakatsu's original part
	GUI.RegisterCommand("mapview", GUICommandCallBack, this); // Edited by yasuhara 2010/10/27
	GUI.RegisterCommand("screenshot", GUICommandCallBack, this); // Edited by umakatsu 2010/11/8
	GUI.RegisterCommand("Reset", GUICommandCallBack, this); // Edited by umakatsu 2010/11/8
	GUI.RegisterCommand("Save", GUICommandCallBack, this); // Edited by umakatsu 2011.1.26
	GUI.RegisterCommand("MapInit", GUICommandCallBack, this); // Edited by umakatsu 2011.3.16

	// Segmentation
	GUI.RegisterCommand("GraphCuts", GUICommandCallBack, this); // Edited by umakatsu 2011.5.12

	// Reconstruction
	GUI.RegisterCommand("SpaceCarvingMethod", GUICommandCallBack, this); // Edited by umakatsu 2011.5.12
	GUI.RegisterCommand("TextureAdd", GUICommandCallBack, this); // Edited by umakatsu 2010/11/8
	GUI.RegisterCommand("ViewReconstructModel", GUICommandCallBack, this);// Edited by umakatsu 2011.1.25
	GUI.RegisterCommand("Reconstruct", GUICommandCallBack, this); // Edited by umakatsu 2011.2.13
	GUI.RegisterCommand("Recovery", GUICommandCallBack, this); // Edited by umakatsu 2011.3.16

	// Authoring
	GUI.RegisterCommand("SetObject", GUICommandCallBack, this); // Edited by umakatsu 2011.3.16

	// Viewing reserved textures
	GUI.RegisterCommand("retrieve", GUICommandCallBack, this); // Edited by umakatsu 2011.1.27
	GUI.RegisterCommand("next", GUICommandCallBack, this); // Edited by umakatsu 2011.1.27
	GUI.RegisterCommand("back", GUICommandCallBack, this); // Edited by umakatsu 2011.1.27

	// Toggle between PTAMM and ARMM
	GUI.RegisterCommand("Toggle", GUICommandCallBack, this); // Edited by umakatsu 2012.2.9

	mimFrameBW.resize(mVideoSource.Size());
	mimFrameRGB.resize(mVideoSource.Size());
	// First, check if the camera is calibrated.
	// If not, we need to run the calibration widget.
	Vector<NUMTRACKERCAMPARAMETERS> vTest;

	vTest = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters",
			ATANCamera::mvDefaultParams, HIDDEN);
	//	mpCamera = new ATANCamera("Camera");
	const string CAM_FILENAME = "ARMM/Data/Cameras/camera_pointgrey.yml";
	mpCamera = new ARMM::PointgreyCamera("Camera", CAM_FILENAME.c_str(), &mVideoSource);
	mpCamera->SetImageSize(mVideoSource.Size());
	armm = new ARMM::ARMM( mpCamera );
	if (vTest == ATANCamera::mvDefaultParams) {
		cout << endl;
		cout
				<< "! Camera.Parameters is not set, need to run the CameraCalibrator tool"
				<< endl;
		cout
				<< "  and/or put the Camera.Parameters= line into the appropriate .cfg file."
				<< endl;
		exit(1);
	}

	// Setting the window size here
	img_size.width = mVideoSource.Size().x;
	img_size.height = mVideoSource.Size().y;
	fContents = new ForegroundContents(img_size.width, img_size.height); // Edited by Umakatsu 2010/11/21

	//create the first map
	mpMap = new Map();
	mvpMaps.push_back(mpMap);
	mpMap->mapLockManager.Register(this);

	mpMapMaker = new MapMaker(mvpMaps, mpMap);
	//  mpTracker = new Tracker(mVideoSource.Size(), *mpCamera, mvpMaps, mpMap, *mpMapMaker); // PTAMM
	mpTracker = new TrackerDrawable(mVideoSource.Size(), *mpCamera, mvpMaps,
			mpMap, *mpMapMaker); // Added by Umakatsu
	mpARDriver
			= new ARDriver(*mpCamera, mVideoSource.Size(), mGLWindow, *mpMap);
	mpMapViewer = new MapViewer(mvpMaps, mpMap, mGLWindow);
	mpMapSerializer = new MapSerializer(mvpMaps);

	// Tateishi's original part
	mpStrokeGenerator = StrokeGenerator::create(*mpTracker, *mpCamera);
	mpSwitcher = Switcher::create();
	// Umakatsu's original part
	segment = new Segmentation(img_size, fContents, *mpCamera, mpTracker);
	construct = new Construction(img_size, fContents, *mpCamera);
	input_s = new StrokeAnalyze(img_size.width, img_size.height);

	//These commands have to be registered here as they call the classes created above
	GUI.RegisterCommand("NextMap", GUICommandCallBack, mpMapViewer);
	GUI.RegisterCommand("PrevMap", GUICommandCallBack, mpMapViewer);
	GUI.RegisterCommand("CurrentMap", GUICommandCallBack, mpMapViewer);

	GUI.RegisterCommand("LoadGame", GUICommandCallBack, mpARDriver);
	GUI.RegisterCommand("Mouse.Click", GUICommandCallBack, mpARDriver);

	//create the menus by Umakatsu
	GUI.ParseLine("GLWindow.AddMenu Menu Menu");
	GUI.ParseLine("Menu.ShowMenu Root");
#if !LETTER
	// Initialize "Toggle" button parameters
	GUI.ParseLine("GraphCuts=0");
	GUI.ParseLine("Brush=0");
	GUI.ParseLine("scm=0");
	GUI.ParseLine("recovery=0");

	//Basic Menu
	GUI.ParseLine("Menu.AddMenuButton Root Save Save Root");
	// Segmentation menu
	GUI.ParseLine("Menu.AddMenuButton Root Segment GraphCuts Root");
	GUI.ParseLine("Menu.AddMenuToggle Root Brush Brush Root");
	// Reconstruction menu
	GUI.ParseLine("GLWindow.AddMenu ReMenu Reconstruction");
	GUI.ParseLine("ReMenu.AddMenuButton Root SCM SpaceCarvingMethod Root");
	GUI.ParseLine("ReMenu.AddMenuButton Root Recovery Recovery Root");
	GUI.ParseLine("ReMenu.AddMenuButton Root TextureAdd TextureAdd Root");
	//	GUI.ParseLine("ReMenu.AddMenuButton Root Reconstruct Reconstruct Root");
	GUI.ParseLine("ReMenu.AddMenuButton Root ObjView ViewReconstructModel Root");
	// Authoring menu
	GUI.ParseLine("GLWindow.AddMenu AusMenu Authoring ");
	GUI.ParseLine("AusMenu.AddMenuButton Root SetObject SetObject Root");
#endif

	//create the menus by PTAMM
	// GUI.ParseLine("GLWindow.AddMenu Menu Menu"); // invalidate by Umakatsu
	// GUI.ParseLine("Menu.ShowMenu Root");// invalidate by Umakatsu
	GUI.ParseLine("Menu.AddMenuButton Root \"Reset All\" ResetAll Root");
	//  GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
	//  GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
	GUI.ParseLine("Menu.AddMenuButton Root Demos \"\" Demos");
	GUI.ParseLine("DrawAR=0");
	GUI.ParseLine("DrawMap=0");
	//  GUI.ParseLine("Menu.AddMenuToggle Root \"Draw AR\" DrawAR Root");

	// Toggle between PTAMM and ARMM
	GUI.ParseLine("Toggle=0");
	GUI.ParseLine("Menu.AddMenuToggle Root Toggle Toggle Root");

	//Games. This function can be found in Games.cc. Add your games to it.
	InitializeGameMenu();

	//	GUI.ParseLine("GLWindow.AddMenu MapsMenu Maps");
	//	GUI.ParseLine("MapsMenu.AddMenuButton Root \"New Map\" NewMap Root");
	//	GUI.ParseLine("MapsMenu.AddMenuButton Root \"Serialize\" \"\" Serial");
	//	GUI.ParseLine("MapsMenu.AddMenuButton Serial \"Save Maps\" SaveMaps Root");
	//	GUI.ParseLine("MapsMenu.AddMenuButton Serial \"Save Map\" SaveMap Root");
	//	GUI.ParseLine("MapsMenu.AddMenuButton Serial \"Load Map\" LoadMap Root");
	//#ifdef _LINUX
	//	GUI.ParseLine("MapsMenu.AddMenuToggle Serial \"Save Video\" SaveFIFO Serial");
	//	GUI.ParseLine("MapsMenu.AddMenuSlider Serial Bitrate Bitrate 100 20000 Serial");
	//#endif
	GUI.ParseLine("LockMap=0");
	//	GUI.ParseLine("MapsMenu.AddMenuToggle Root \"Lock Map\" LockMap Root");
	//	GUI.ParseLine("MapsMenu.AddMenuButton Root \"Delete Map\" DeleteMap Root");
	GUI.ParseLine("MapInfo=0");
	//	GUI.ParseLine("MapsMenu.AddMenuToggle Root \"Map Info\" MapInfo Root");

	//	GUI.ParseLine("GLWindow.AddMenu MapViewerMenu Viewer");
	//	GUI.ParseLine("MapViewerMenu.AddMenuToggle Root \"View Map\" DrawMap Root");
	//	GUI.ParseLine("MapViewerMenu.AddMenuButton Root Next NextMap Root");
	//	GUI.ParseLine("MapViewerMenu.AddMenuButton Root Previous PrevMap Root");
	//	GUI.ParseLine("MapViewerMenu.AddMenuButton Root Current CurrentMap Root");

	// Added by Umakatsu
	modelCount = 0;
	modelTarget = 0;
	magnification = 1.0;
	mbDone = false;
	bIgnoreMouseAction = false; //map表示の際にはマウスアクションを無視する(trutodo管理 フリーe時)
	/* Below initialization Edited by Umakatsu */
	bgraph = false;
	bBrush = false;
	create_object = false;
	autosegment = false;
	viewmode = false;
	bStartSCM = false;
	bRecovery = false;
	bShowTexture = false;
	run_other_command = false;
	sysmode = RECONSTRUCT;
	count_texture = 0;
}

/**
 * Destructor
 */
System::~System(void) {
	if (mpMap != NULL) {
		mpMap->mapLockManager.UnRegister(this);
	}
	// Added by Umakatsu
	BOOST_FOREACH( Stroke * s, strokes )
	{
		delete s;
	}

//	const char * leftover = "rm screen.bmp";
//	system(leftover);
}

/**
 * Run the main system thread.
 * This handles the tracker and the map viewer.
 */
void System::Run() {
	while (!mbDone) {
		//Check if the map has been locked by another thread, and wait for release.
		bool bWasLocked = mpMap->mapLockManager.CheckLockAndWait(this, 0);

		/* This is a rather hacky way of getting this feedback,
		 but GVars cannot be assigned to different variables
		 and each map has its own edit lock bool.
		 A button could be used instead, but the visual
		 feedback would not be as obvious.
		 */
		mpMap->bEditLocked = *mgvnLockMap; //sync up the maps edit lock with the gvar bool.

		// Grab new video frame...
		mVideoSource.GetAndFillFrameBWandRGB(mimFrameBW, mimFrameRGB);
		static bool bFirstFrame = true;
		if (bFirstFrame) {
			mpARDriver->Init();
			bFirstFrame = false;
		}

		mGLWindow.SetupViewport();
		mGLWindow.SetupVideoOrtho();
		mGLWindow.SetupVideoRasterPosAndZoom();

		if (!mpMap->IsGood()) {
			mpARDriver->Reset();
		}

		if (bWasLocked) {
			mpTracker->ForceRecovery();
		}
		/* GUI.ParseLine()で追加した処理を行う部分の定義 */
		static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN | SILENT);
		static gvar3<int> gvnDrawAR("DrawAR", 0, HIDDEN | SILENT);
#if !LETTER
		static gvar3<int> gvnBrush("Brush", 0, HIDDEN | SILENT);
#endif
		//for toggle tracking mode
		static gvar3<int> gvnTracking("Toggle", 0, HIDDEN | SILENT);

		/* Determine mouse behaviors TODO Controlling mouse states by Hex number */
		// Added by Umakatsu 2011.5.13
		Vector<2> vMousePosition = mGLWindow.GetMousePosition();
		int mouse_state;
		mouse_state = mGLWindow.GetMouseState();
		bool bLeftClicked = (mouse_state & CVD::GLWindow::BUTTON_LEFT);
		mouseLeftButton.set(bLeftClicked);
		bool bRightClicked = (mouse_state & CVD::GLWindow::BUTTON_RIGHT);
		mouseRightButton.set(bRightClicked);
		bool bMiddleDown = (mouse_state & CVD::GLWindow::BUTTON_WHEEL_DOWN);
		mouseMiddleDown.set(bMiddleDown);
		bool bMiddleUp = (mouse_state & CVD::GLWindow::BUTTON_WHEEL_UP);
		mouseMiddleUp.set(bMiddleUp);
		bool bMiddleClicked = (mouse_state & CVD::GLWindow::BUTTON_MIDDLE);
		mouseMiddleButton.set(bMiddleClicked);

		bool bDrawMap = mpMap->IsGood() && bIgnoreMouseAction; // yasuhara Edited 2010/10/27 gvnDrawMap -> bIgnoreMouseAction
		//bool bDrawMap = mpMap->IsGood() && *gvnDrawMap;
		bool bDrawAR = mpMap->IsGood() && *gvnDrawAR;
		// Added by Umakatsu
#if !LETTER
		bool bBrushButton = *gvnBrush;
#endif
		//    /* カメラ位置の更新 */
		bool is_neccesary_clicked_point = mouseLeftButton.isDown();

		//	PTAMM
		//	mpTracker->TrackFrame(mimFrameBW, !bDrawAR && !bDrawMap);
		//	Umakatsu Atsushi
		mpTracker->TrackFrame(mimFrameBW, !bDrawAR && !bDrawMap,
				is_neccesary_clicked_point, vMousePosition);
		// Inserting Huge source code by Umakatsu
		// until if(bDrawMap){}
		const TooN::SE3<> currentPose(mpTracker->GetCurrentPose());
		const TooN::Matrix<4> projection =
				mpCamera->MakeUFBLinearFrustumMatrix(0.005, 100);
		State state = mpSwitcher->get();

		// Acquisition from the segmentation part
		sSegment = segment->GetStateSegment();

		//display color camera image
		if (state == COMPOSE) {
			mpARDriver->Render(mimFrameRGB, currentPose, mpTracker->IsLost());
		}

		switch (sysmode) {
			case MAPINIT:
				break;
			case RECONSTRUCT: case COMMAND:
				// Display the extract region  Edited by Umakatsu 2010/11/24
				if (sSegment != NEW) {
					if (fContents->contour_stroke_list.size() > 0) {
						draw_frame = fContents->contour_stroke_list.back();
						draw_frame->setLineProperty(LINEWIDTH, GREEN);
						draw_frame->draw(currentPose, projection);
					}
					//				fContents->DrawContour( currentPose , projection );
				}
				break;
			case AUTHORING:
				// Overlay objects in the real world
				construct->DrawObjects(currentPose, projection);
				if (vertex.size() > 0) {
					construct->translateModel(vertex.front(), modelTarget);
					vertex.pop_front();
				}
				break;

			case INTERACTION: // interaction mode in ARMM


			default:
				break;
		}

		if (autosegment) {
			construct->setStateSCM(READY);
		}

		if (!bIgnoreMouseAction && !autosegment && !viewmode && !bShowTexture && (*gvnTracking == 0) ) {
			if (mouseLeftButton.isPressed()) {
				//impose segmentation thread to stop running
				segment->SetInterrupt(true);
				sleep(0.5);
				mpStrokeGenerator->Begin(currentPose, vMousePosition);
			} else if (mouseLeftButton.isDown()) {
				// input stroke data
				if (create_object) {
					if (!bBrush) {
						mpStrokeGenerator->strokeContentsSet(PURPLE, 10); //exclusion brush color
					} else {
						mpStrokeGenerator->strokeContentsSet(YELLOW, 10); //inclusion brush color
					}
				}
				if (create_object) {
					if (bBrush) {
						sSegment = EXCLUDE;
					} else {
						sSegment = INCLUDE;
					}
				}
				mpStrokeGenerator->Add(currentPose, vMousePosition);
			}
			Stroke * s = NULL;
			if (mouseLeftButton.isReleased()
					&& mpStrokeGenerator->getIsStroke()) {//ストロークを書き終えた時
				s = mpStrokeGenerator->End();
				if (run_other_command) {
					run_other_command = false;
				} else {
					WriteBitmapFromGL("screen.bmp", img_size.width, img_size.height);
					strokes.push_back(s);
					vertex = strokes.back()->getVertex();
				}
				//Segmentation partに対する割り込み信号をOFFにする
				segment->SetInterrupt(false);
			}

#if LETTER
			if( mouseRightButton.isPressed() && state == COMPOSE && ( sysmode != COMMAND ) ) {
				//impose segmentation thread to stop running
				segment->SetInterrupt(true);
				sleep(0.5);
				mpStrokeGenerator->Begin( currentPose, vMousePosition, currentPose);
				sysmode = COMMAND;
			} else if( mouseRightButton.isDown( ) ) {
				if(create_object) {
					if(bBrush) sSegment = EXCLUDE;
					else sSegment = INCLUDE;
				}
				mpStrokeGenerator->strokeContentsSet( CIAN );
				mpStrokeGenerator->Add( currentPose, vMousePosition);
			}

			if( mouseRightButton.isReleased( ) && mpStrokeGenerator->getIsStroke() ) {//ストロークを書き終えた時
				s = mpStrokeGenerator->End( );
				WriteBitmapFromGL("screen.bmp",img_size.width,img_size.height);
				input_s->checkLetter(s);
				//Segmentation partに対する割り込み信号をOFFにする
				segment->SetInterrupt(false);
				sysmode = RECONSTRUCT;
			}
#endif
			/* Copy the reconstructed object */
			if (sysmode == AUTHORING && state == COMPOSE) {
				// Define the position of the copy object
				if (mouseMiddleButton.isPressed() && !mouseLeftButton.isDown()
						&& !mouseRightButton.isDown()) {
					mpStrokeGenerator->Begin(currentPose, vMousePosition);
					mpStrokeGenerator->End();
				} else if (mouseMiddleButton.isReleased()
						&& !mouseLeftButton.isDown()
						&& !mouseRightButton.isDown()) {
					construct->createModel(modelTarget);
					construct->translateModel(mpStrokeGenerator->lastPoint(),
							modelCount);
					modelCount++;
				}
				if (mouseMiddleUp.isReleased() && !mouseLeftButton.isDown()
						&& !mouseRightButton.isDown()) {
					if (modelCount != 0) {
						if (modelTarget < modelCount - 1) {
							modelTarget += 1;
						} else {
							modelTarget = 0;
						}
					} else {
						modelTarget = 0;
					}
					magnification = 1.0;
					std::cout << "target: " << modelTarget << std::endl;
				}
				if (mouseMiddleDown.isReleased() && !mouseLeftButton.isDown()
						&& !mouseRightButton.isDown()) {
					if (modelCount != 0) {
						if (modelTarget >= 1) {
							modelTarget -= 1;
						} else {
							modelTarget = modelCount - 1;
						}
					} else {
						modelTarget = 0;
					}
					magnification = 1.0;
					std::cout << "target: " << modelTarget << std::endl;
				}
				if (mouseMiddleUp.isReleased() && mouseLeftButton.isDown()
						&& !mouseRightButton.isDown()) {
					magnification += SCALE_CONST;
					construct->scalingModel(magnification, modelTarget);
					cout << "Magnification : " << magnification << endl;
				}
				if (mouseMiddleDown.isReleased() && mouseLeftButton.isDown()
						&& !mouseRightButton.isDown()) {
					if (magnification > SCALE_CONST) {
						magnification -= SCALE_CONST;
						construct->scalingModel(magnification, modelTarget);
					} else {
					}
					cout << "Magnification : " << magnification << endl;
				}
			}
			if (mouseMiddleButton.isReleased() && sysmode != AUTHORING) {
				//復元と配置の切り変え
				mpSwitcher->toggle();
				state = mpSwitcher->get();
			}
			//できれば特徴点に対して、ドロネーを行いたい
			// Turn on the Initial segmentation
			//		if( input_s->isValidAnalyze() ){
			if (strokes.size() != 0 && bgraph && !create_object) {
				segment->SetMimFrameRGB(mimFrameRGB);
				segment->SetStroke(strokes.back());
				segment->SetStateSegment(INIT); //Run Initial segmentation in the segment thread
				sSegment = INIT;
				Stroke * last = strokes.front();
				Vector<3> v;
				Vector<2> t;
				vector<Vector<3> > p;
				Vector<3> normalFromC = currentPose * last->getNormal();
				Vector<3> startFromC = currentPose * last->getStart();
				for (int i = 0; i <= WIDTH; i += WIDTH) {
					for (int j = 0; j <= WIDTH; j += WIDTH) {
						t[0] = i;
						t[1] = j;
						{
							Vector<2> w;
							w = mpCamera->UnProject(t);
							v[0] = w[0];
							v[1] = w[1];
							v[2] = 1.0;
						}
						Vector<3> drawFromC;
						{
							double inner_product = normalFromC * startFromC;
							double depth = inner_product / (v * normalFromC);
							drawFromC = depth * v;
						}
						Vector<3> drawFromW = currentPose.inverse() * drawFromC;
						p.push_back(drawFromW);
					}
				}
				//Setting edge length in the voxel
				const Vector<3> tmp_v = p.front();
				const Vector<3> dx = (p.at(1) - p.at(0)) / size;
				const Vector<3> dy = (p.at(2) - p.at(0)) / size;
				Vector<3> n =
						outerProduct(p.at(0) - p.at(1), p.at(0) - p.at(2));
				n /= norm3(n);
				n *= (norm3(p.at(0) - p.at(1))) / size;
				const Vector<3> dz = n;
				//initializing voxel occupied and look-up table
				construct->setSideOfCube(dx, dy, dz, tmp_v);
				construct->setStateSCM(INIT);
				segment->SetInterrupt(false);
				strokes.clear();
				create_object = true;
				//			}
			}

#if LETTER
			if( input_s->isValidAnalyze() ) {
				if( input_s->getResult() == 'B' || input_s->getResult() == 'b' )
				bBrush = !bBrush;
				else if( input_s->getResult() == 'G' || input_s->getResult() == 'g' )
				GUI.ParseLine("GraphCuts");
				else if( input_s->getResult() == 'S' || input_s->getResult() == 's' )
				bStartSCM = !bStartSCM;
				else if( input_s->getResult() == 'V' || input_s->getResult() == 'v' )
				viewmode = !viewmode;
				else if( input_s->getResult() == 'A' || input_s->getResult() == 'a' )
				GUI.ParseLine("SETOBJECT");
				else if( input_s->getResult() == 'T' || input_s->getResult() == 't' )
				GUI.ParseLine("TEXTURE");
				else if( input_s->getResult() == 'R' || input_s->getResult() == 'r' )
				GUI.ParseLine("RECOVERY");
				else
				cerr << "No process to this letter" << endl;
			}
#endif
			// Brush ON
			if (strokes.size() > 0 && create_object && sysmode == RECONSTRUCT) {
				// Exclusion Brush
				if (bBrush && segment->GetStateSegment() != EXCLUDE) {
					construct->setInterrupt(true);
					// stop running Dynamic Segmentation
					segment->SetInterrupt(true);
					// set up for Exclusion Brush
					segment->SetMimFrameRGB(mimFrameRGB);
					;
					segment->SetStroke(strokes.back());
					segment->SetStateSegment(EXCLUDE);
					sSegment = EXCLUDE;
				}
				// Inclusion Brush
				if (!bBrush && segment->GetStateSegment() != INCLUDE) {
					construct->setInterrupt(true);
					// stop running Dynamic Segmentation
					segment->SetInterrupt(true);
					// set up for Inclusion Brush
					segment->SetMimFrameRGB(mimFrameRGB);
					;
					segment->SetStroke(strokes.back());
					segment->SetStateSegment(INCLUDE);
					sSegment = INCLUDE;
				}
				strokes.clear();
			}
			// Dynamic Segmentation Part
			if (!mouseLeftButton.isDown() && !mouseLeftButton.isPressed()
			//		 	&& ( mpTracker->getTrackingQuality() == GOOD || mpTracker->getTrackingQuality() == DODGY)
			) {
				if (create_object && sSegment != INIT //Initial Segmentationが動いていない
						&& sSegment != DYNAMIC //Dynamic Segmentationが動いていない
						&& !(segment->GetInterrupt()) //Interruption信号が入力されていない
				) {
					segment->SetMimFrameRGB(mimFrameRGB);
					segment->SetStateSegment(DYNAMIC);
					sSegment = DYNAMIC;
				}
			}
			// Construction Part
			if (bStartSCM && sysmode == RECONSTRUCT
			//			&& ( mpTracker->getTrackingQuality() == GOOD || mpTracker->getTrackingQuality() == DODGY)
			) {
				//前景情報が更新された
				if (fContents->getUpdate()) {
					if (construct->getStateSCM() != RUN) {
						//					//前景情報をconstruction側の情報に格納
						//					construct->pushForeground(
						//							fContents->popSilhouette() ,
						//							fContents->popCameraPose() ,
						//							fContents->popColor()
						//					);
						fContents->setUpdate(false);
					}
				}
				if (construct->getStateSCM() == READY && sSegment != EXCLUDE
						&& sSegment != INCLUDE) {
					if (construct->getSizeTexture() == 0) {
						ImageType mimFrameRGB_copy; // frame clone
						mimFrameRGB_copy.copy_from(mimFrameRGB);// copy original frame
						construct->createTexture(mimFrameRGB_copy, currentPose);
					}
					if (sSegment != DYNAMIC) {
						//					segment->setInterrupt(true);
						construct->setInterrupt(false);
						construct->setStateSCM(RUN);
					} else {
						//Space Carving を実行するタイミング

						//2011.1/8 Dynamic segmentationが5回実行されたら実行する
						//					if(segment->getNum_of_dynamics() % 5 == 0){
						//2011.1/10 Construction部に保持される情報があったら実行
						if (construct->sizeForeground() > 0) {
							//						segment->setInterrupt(true);
							construct->setInterrupt(false);
							construct->setStateSCM(RUN);
						}
					}
				}
			} else {
				if (construct->getStateSCM() != NEW)
					construct->setStateSCM(READY);
				if (construct->getStateMCM() != NEW) {
					construct->setStateMCM(READY);
					construct->setInterrupt(true);
				}
				if (construct->getForeground() > 0)
					construct->clearForeground();
			}

			//別スレッドで処理中はレンダリングを行わない
			//		if(!(segment->getInitial()) && !(segment->getDynamic()) ){
			mpStrokeGenerator->Render(currentPose);
			//		}
		}

		if (*mgvnDrawMapInfo) {
			DrawMapInfo();
		}

		if (bDrawMap) {
			mpMapViewer->DrawMap(mpTracker->GetCurrentPose());
		} else if (bDrawAR) {
			if (!mpTracker->IsLost()) {
				mpARDriver->AdvanceLogic();
			}
				mpARDriver->Render(mimFrameRGB, mpTracker->GetCurrentPose(), mpTracker->IsLost() );
		}

		// Added by Umakatsu
		else if (autosegment || viewmode) {
			//Marching Cubesの実行終了を待つ
			if (construct->getStateMCM() != RUN && construct->getStateSCM()
					!= NEW) {
				if (construct->getCubesData()) {
					//Map表示
					segment->SetInterrupt(true);
					construct->setInterrupt(true);
					if (autosegment) {
						mpMapViewer->DrawMap(currentPose,
								construct->getCubesData(), 0);
					} else {
						mpMapViewer->DrawMap(currentPose,
								construct->getCubesData(), 2);
					}
				} else {
					cerr << "No Cube Data" << endl;
					GUI.ParseLine("ViewModeTexture");
				}
			} else if (construct->getStateSCM() == NEW) {
				cerr << "No Construct Data" << endl;
				autosegment = false;
				viewmode = false;
			}
		}
		/* テクスチャ画像閲覧モード */
		else if (bShowTexture) {
			if (construct->getStateMCM() != NEW) {
				if (construct->getSizeTexture() > count_texture) {
					ImageType tmpRGB = construct->getImageRGBFromTexture(
							static_cast<int> (count_texture));
					TooN::SE3<> tmpPose = construct->getCamemraPoseFromTexture(
							static_cast<int> (count_texture));
					mpARDriver->Render(tmpRGB, tmpPose, mpTracker->IsLost());
				} else {
					cerr << "No Texture Data" << endl;
					bShowTexture = false;
					segment->SetInterrupt(false);
					construct->setInterrupt(false);
				}
			} else {
				cerr << "No Cubes Data" << endl;
				bShowTexture = false;
				segment->SetInterrupt(false);
				construct->setInterrupt(false);
			}
		}
#if !LETTER
		/* Brush mode を切り替える場合 */
		/* 状態が完全に遷移するまでmodeを切り替えない */
		else if ((bBrushButton != bBrush) && mouseLeftButton.isReleased()) {
			if (!bBrush) {
				strokes.clear();
			}
			bBrush = *gvnBrush;
			//Dynamic segmentationを止める割り込み信号
			segment->SetInterrupt(true);
		}
#endif

//		if (bStartSCM && construct->getStateSCM() != RUN && sSegment != EXCLUDE
//				&& sSegment != INCLUDE && !bShowTexture && sysmode
//				== RECONSTRUCT) {
//			if (fContents->pData.state_proj != NEW) {
//				construct->DrawProjectedRegion(mpTracker->GetCurrentPose(), projection);
//			}
//		}

		if (bStartSCM
		//	&& construct->getStateSCM() != RUN
				&& sSegment != EXCLUDE && sSegment != INCLUDE && !bShowTexture
				&& sysmode == RECONSTRUCT) {
			if (fContents->pData.state_proj != NEW) {
				construct->DrawProjectedRegion(mpTracker->GetCurrentPose(), projection);
			}
		}

		// Restore a texture image sync with PTAM's capturing key frame
		if (mpMapMaker->QueueSize() > 0 && construct->getStateMCM() != NEW) {
			cout << "Add Key Frame" << endl;
			GUI.ParseLine("TextureAdd");
		}

		// Display system state message
		string sCaption;
		if (bDrawMap) {
			sCaption = mpMapViewer->GetMessageForUser();
		} else {
			sCaption = mpTracker->GetMessageForUser();
		}
		mGLWindow.DrawCaption(sCaption);
		if (mpStrokeGenerator->getIsStroke()) {

		}
		else
		{
			mGLWindow.DrawMenus();
		}
#ifdef _LINUX
		if( *mgvnSaveFIFO )
		{
			SaveFIFO();
		}
#endif
		mGLWindow.swap_buffers();
		mGLWindow.HandlePendingEvents();

		if (*gvnTracking) {
			armm->Run();
		}
	}
	delete armm;
}

/**
 * Parse commands sent via the GVars command system.
 * @param ptr Object callback
 * @param sCommand command string
 * @param sParams parameters
 */
void System::GUICommandCallBack(void *ptr, string sCommand, string sParams) {
	if (sCommand == "quit" || sCommand == "exit") {
		static_cast<System*> (ptr)->mbDone = true;
	} else if (sCommand == "SwitchMap") {
		int nMapNum = -1;
		if (static_cast<System*> (ptr)->GetSingleParam(nMapNum, sCommand,
				sParams)) {
			static_cast<System*> (ptr)->SwitchMap(nMapNum);
		}
	} else if (sCommand == "ResetAll") {
		static_cast<System*> (ptr)->ResetAll();
		return;
	} else if (sCommand == "NewMap") {
		cout << "Making new map..." << endl;
		static_cast<System*> (ptr)->NewMap();
	} else if (sCommand == "DeleteMap") {
		int nMapNum = -1;
		if (sParams.empty()) {
			static_cast<System*> (ptr)->DeleteMap(
					static_cast<System*> (ptr)->mpMap->MapID());
		} else if (static_cast<System*> (ptr)->GetSingleParam(nMapNum,
				sCommand, sParams)) {
			static_cast<System*> (ptr)->DeleteMap(nMapNum);
		}
	} else if (sCommand == "NextMap") {
		static_cast<MapViewer*> (ptr)->ViewNextMap();
	} else if (sCommand == "PrevMap") {
		static_cast<MapViewer*> (ptr)->ViewPrevMap();
	} else if (sCommand == "CurrentMap") {
		static_cast<MapViewer*> (ptr)->ViewCurrentMap();
	} else if (sCommand == "SaveMap" || sCommand == "SaveMaps" || sCommand
			== "LoadMap") {
		static_cast<System*> (ptr)->StartMapSerialization(sCommand, sParams);
	} else if (sCommand == "LoadGame") {
		static_cast<ARDriver*> (ptr)->LoadGame(sParams);
	} else if (sCommand == "Mouse.Click") {
		vector<string> vs = ChopAndUnquoteString(sParams);

		if (vs.size() != 3) {
			return;
		}

		istringstream is(sParams);
		int nButton;
		ImageRef irWin;
		is >> nButton >> irWin.x >> irWin.y;
		static_cast<ARDriver*> (ptr)->HandleClick(nButton, irWin);

	} else if (sCommand == "KeyPress") {
		if (sParams == "q" || sParams == "Escape") {
			GUI.ParseLine("quit");
			return;
		}

		bool bUsed = static_cast<System*> (ptr)->mpTracker->HandleKeyPress(
				sParams);

		if (!bUsed) {
			static_cast<System*> (ptr)->mpARDriver->HandleKeyPress(sParams);
		}
	}

	// Added by Umakatsu
	else if (sCommand == "Save") {
		static_cast<System *> (ptr)->stopOtherThread(ptr);
		struct tm *newtime;
		time_t aclock;
		time(&aclock); /* 現在時刻の獲得 */
		newtime = localtime(&aclock); /* tm構造体への変換 */
		char name[50];
		sprintf(name, "Voxel_20%d_%d%d_%d%d", newtime->tm_year - 100,
				newtime->tm_mon + 1, newtime->tm_mday, newtime->tm_hour,
				newtime->tm_min);
		static_cast<System *> (ptr)->construct->Save(name);
		static_cast<System *> (ptr)->readyOtherThread(ptr);
	}

	else if (sCommand == "GraphCuts") {
		static_cast<System *> (ptr)->segment->Start();
		static_cast<System *> (ptr)->run_other_command = true;
		static_cast<System *> (ptr)->strokes.clear();
		static_cast<System *> (ptr)->bgraph = true;
	}

	else if (sCommand == "SpaceCarvingMethod") {
		static_cast<System *> (ptr)->construct->Start();
		static_cast<System *> (ptr)->strokes.clear();
		static_cast<System *> (ptr)->bStartSCM
				= ~static_cast<System *> (ptr)->bStartSCM;
	}

	else if (sCommand == "Reconstruct") {
		static_cast<System *> (ptr)->stopOtherThread(ptr);
		cout << "Begin Reconstrucing Model" << endl;
		static_cast<System *> (ptr)->construct->Reconstruct();
		cout << "Finish Reconstrucing Model" << endl;
	}

	else if (sCommand == "SetObject") {
		if (static_cast<System *> (ptr)->bStartSCM) {
			static_cast<System *> (ptr)->stopOtherThread(ptr);
			static_cast<System *> (ptr)->sysmode = AUTHORING;
			sleep(3.0);
			static_cast<System *> (ptr)->construct->New();
			static_cast<System *> (ptr)->modelCount = 1;
		} else {
			cerr << "No Voxel Data" << endl;
		}
	} else if (sCommand == "Recovery") {
		System * tmp = static_cast<System *> (ptr);
		tmp->sysmode = INTERACTION;
		//clear setting for AR Diorama
		//		tmp->ResetAll();
		static_cast<System*> (ptr)->ResetAll();
		return;
		//		//change image to color mode
		//		tmp->mpSwitcher->toggle();

		//		static_cast<System *> (ptr)->stopOtherThread(ptr);
		//		tmp->construct->pushForeground(tmp->fContents->popSilhouette(),
		//				tmp->fContents->popCameraPose(), tmp->fContents->popColor());
		//		cout << "Begin Recovering" << endl;
		//		static_cast<System *> (ptr)->construct->Recovery();
		//		cout << "Finish Recovering" << endl;
		return;
	} else if (sCommand == "mapview") {
		if (static_cast<System *> (ptr)->bIgnoreMouseAction) {
			static_cast<System *> (ptr)->flagReset(ptr);
		} else {
			static_cast<System *> (ptr)->flagReset(ptr);
			static_cast<System *> (ptr)->bIgnoreMouseAction = true;
			static_cast<System *> (ptr)->stopOtherThread(ptr);
		}
	}
	/* SCMにおいてどのようにテクスチャをはっているかの色分けを確認する 2011.1.30 */
	else if (sCommand == "screenshot") {
		if (static_cast<System *> (ptr)->autosegment) {
			static_cast<System *> (ptr)->flagReset(ptr);
		} else {
			static_cast<System *> (ptr)->flagReset(ptr);
			static_cast<System *> (ptr)->autosegment = true;
			static_cast<System *> (ptr)->stopOtherThread(ptr);
			sleep(2);
		}
	}
	//テクスチャマッピングモードに移行 2011.1.28
	else if (sCommand == "ViewReconstructModel") {
		if (static_cast<System *> (ptr)->viewmode) {
			static_cast<System *> (ptr)->flagReset(ptr);
		} else {
			static_cast<System *> (ptr)->flagReset(ptr);
			static_cast<System *> (ptr)->viewmode = true;
			static_cast<System *> (ptr)->stopOtherThread(ptr);
			sleep(2);
		}
	}
	//手動でテクスチャ追加 2011.1.27
	else if (sCommand == "TextureAdd") {
		System * tmp = static_cast<System *> (ptr);
		if (tmp->construct->getStateMCM() != NEW) {
			const TooN::SE3<> tmpPose = tmp->mpTracker->GetCurrentPose();
			ImageType mimFrameRGB_copy; // frame clone
			mimFrameRGB_copy.copy_from(tmp->mimFrameRGB); // copy original frame
			//			if( tmp->fContents->isEmpty());
			//			else{
			//				uchar * tmpsilhouette = tmp->fContents->popSilhouette();
			tmp->construct->createTexture(mimFrameRGB_copy, tmpPose);
			//			}
		} else {
			cerr << "No Marching Cubes Data" << endl;
		}
	}
	/* 今まで取得したテクスチャ画面を見るモードへ移行 2011.1.30 */
	else if (sCommand == "retrieve") {
		if (static_cast<System *> (ptr)->bShowTexture) {
			static_cast<System *> (ptr)->flagReset(ptr);
		} else if (static_cast<System *> (ptr)->construct->getStateSCM() != NEW) {
			static_cast<System *> (ptr)->flagReset(ptr);
			static_cast<System *> (ptr)->bShowTexture = true;
			static_cast<System *> (ptr)->stopOtherThread(ptr);
			System * tmp = static_cast<System *> (ptr);
			unsigned int tsize = tmp->construct->getSizeTexture();
			if (tsize > 0) {
				tmp->mMessageForUser.str("");
				tmp->mMessageForUser << " Texture Number: "
						<< tmp->count_texture + 1 << "/" << tsize;
				tmp->mMessageForUser << setprecision(4);
				tmp->mMessageForUser << "   Camera Pos: "
						<< tmp->construct->getCamemraPoseFromTexture(
								tmp->count_texture).inverse().get_translation()
						<< endl;
				tmp->mMessageForUser << "Color cord (";
				REP(i,3)
					tmp->mMessageForUser << ColorMap[tmp->count_texture][i]
							<< " ";
				tmp->mMessageForUser << ")";
				tmp->mMessageForUser << setprecision(4);
			}
		} else if (sCommand == "MapInit") {
			//			static_cast< System *>( ptr )->sysmode = RECONSTRUCT;
		} else {
			cerr << "No Texture Data" << endl;
		}
	}
	/* 今まで取得したテクスチャ画面を見るモードにおいて別のテクスチャを見る 2011.1.30 */
	else if (sCommand == "next" || sCommand == "back") {
		if (static_cast<System *> (ptr)->construct->getStateSCM() != NEW) {
			System * tmp = static_cast<System *> (ptr);
			unsigned int tsize = tmp->construct->getSizeTexture();
			if (sCommand == "next") {
				if ((tsize - 1) == tmp->count_texture)
					tmp->count_texture = 0;
				else
					tmp->count_texture++;
			} else {
				if (tmp->count_texture == 0)
					tmp->count_texture = tsize - 1;
				else
					tmp->count_texture--;
			}
			tmp->mMessageForUser.str("");
			tmp->mMessageForUser << " Texture Number: " << tmp->count_texture
					+ 1 << "/" << tsize;
			tmp->mMessageForUser << setprecision(4);
			tmp->mMessageForUser << "   Camera Pos: "
					<< tmp->construct->getCamemraPoseFromTexture(
							tmp->count_texture).inverse().get_translation()
					<< endl;
			tmp->mMessageForUser << "Color cord (";
			REP(i,3)
				tmp->mMessageForUser << ColorMap[tmp->count_texture][i] << " ";
			tmp->mMessageForUser << ")";
			tmp->mMessageForUser << setprecision(4);
		} else {
			cerr << "No Texture Data" << endl;
		}
	} else {
	}
}

/**
 * Parse and allocate a single integer variable from a string parameter
 * @param nAnswer the result
 * @param sCommand the command (used to display usage info)
 * @param sParams  the parameters to parse
 * @return success or failure.
 */
bool System::GetSingleParam(int &nAnswer, string sCommand, string sParams) {
	vector<string> vs = ChopAndUnquoteString(sParams);

	if (vs.size() == 1) {
		//is param a number?
		bool bIsNum = true;
		for (size_t i = 0; i < vs[0].size(); i++) {
			bIsNum = isdigit(vs[0][i]) && bIsNum;
		}

		if (!bIsNum) {
			return false;
		}

		int *pN = ParseAndAllocate<int> (vs[0]);
		if (pN) {
			nAnswer = *pN;
			delete pN;
			return true;
		}
	}

	cout << sCommand << " usage: " << sCommand << " value" << endl;

	return false;
}

/**
 * Switch to the map with ID nMapNum
 * @param  nMapNum Map ID
 * @param bForce This is only used by DeleteMap and ResetAll, and is
 * to ensure that MapViewer is looking at a safe map.
 */
bool System::SwitchMap(int nMapNum, bool bForce) {

	//same map, do nothing. This should not actually occur
	if (mpMap->MapID() == nMapNum) {
		return true;
	}

	if ((nMapNum < 0)) {
		cerr << "Invalid map number: " << nMapNum << ". Not changing." << endl;
		return false;
	}

	for (size_t ii = 0; ii < mvpMaps.size(); ii++) {
		Map * pcMap = mvpMaps[ii];
		if (pcMap->MapID() == nMapNum) {
			mpMap->mapLockManager.UnRegister(this);
			mpMap = pcMap;
			mpMap->mapLockManager.Register(this);
		}
	}

	if (mpMap->MapID() != nMapNum) {
		cerr << "Failed to switch to " << nMapNum << ". Does not exist."
				<< endl;
		return false;
	}

	/*  Map was found and switched to for system.
	 Now update the rest of the system.
	 Order is important. Do not want keyframes added or
	 points deleted from the wrong map.

	 MapMaker is in its own thread.
	 System,Tracker, and MapViewer are all in this thread.
	 */

	*mgvnLockMap = mpMap->bEditLocked;

	//update the map maker thread
	if (!mpMapMaker->RequestSwitch(mpMap)) {
		return false;
	}

	while (!mpMapMaker->SwitchDone()) {
#ifdef WIN32
		Sleep(1);
#else
		usleep(10);
#endif
	}

	//update the map viewer object
	mpMapViewer->SwitchMap(mpMap, bForce);

	//update the tracker object
	//   mpARDriver->Reset();
	mpARDriver->SetCurrentMap(*mpMap);

	if (!mpTracker->SwitchMap(mpMap)) {
		return false;
	}

	return true;
}

/**
 * Create a new map and switch all
 * threads and objects to it.
 */
void System::NewMap() {

	*mgvnLockMap = false;
	mpMap->mapLockManager.UnRegister(this);
	mpMap = new Map();
	mpMap->mapLockManager.Register(this);
	mvpMaps.push_back(mpMap);

	//update the map maker thread
	mpMapMaker->RequestReInit(mpMap);
	while (!mpMapMaker->ReInitDone()) {
#ifdef WIN32
		Sleep(1);
#else
		usleep(10);
#endif
	}

	//update the map viewer object
	mpMapViewer->SwitchMap(mpMap);

	//update the tracker object
	mpARDriver->SetCurrentMap(*mpMap);
	mpARDriver->Reset();
	mpTracker->SetNewMap(mpMap);

	cout << "New map created (" << mpMap->MapID() << ")" << endl;

}

/**
 * Moves all objects and threads to the first map, and resets it.
 * Then deletes the rest of the maps, placing PTAMM in its
 * original state.
 * This reset ignores the edit lock status on all maps
 */
void System::ResetAll() {

	//move all maps to first map.
	if (mpMap != mvpMaps.front()) {
		if (!SwitchMap(mvpMaps.front()->MapID(), true)) {
			cerr << "Reset All: Failed to switch to first map" << endl;
		}
	}
	mpMap->bEditLocked = false;

	//reset map.
	mpTracker->Reset();

	//lock and delete all remaining maps
	while (mvpMaps.size() > 1) {
		DeleteMap(mvpMaps.back()->MapID());
	}
	segment->Reset();
	construct->Reset();
	//	Reset();

}

/**
 * Delete a specified map.
 * @param nMapNum map to delete
 */
bool System::DeleteMap(int nMapNum) {
	if (mvpMaps.size() <= 1) {
		cout << "Cannot delete the only map. Use Reset instead." << endl;
		return false;
	}

	//if the specified map is the current map, move threads to another map
	if (nMapNum == mpMap->MapID()) {
		int nNewMap = -1;

		if (mpMap == mvpMaps.front()) {
			nNewMap = mvpMaps.back()->MapID();
		} else {
			nNewMap = mvpMaps.front()->MapID();
		}

		// move the current map users elsewhere
		if (!SwitchMap(nNewMap, true)) {
			cerr << "Delete Map: Failed to move threads to another map."
					<< endl;
			return false;
		}
	}

	// find and delete the map
	for (size_t ii = 0; ii < mvpMaps.size(); ii++) {
		Map * pDelMap = mvpMaps[ii];
		if (pDelMap->MapID() == nMapNum) {

			pDelMap->mapLockManager.Register(this);
			pDelMap->mapLockManager.LockMap(this);
			delete pDelMap;
			mvpMaps.erase(mvpMaps.begin() + ii);

			///@TODO Possible bug. If another thread (eg serialization) was using this
			/// and waiting for unlock, would become stuck or seg fault.
		}
	}

	return true;
}

/**
 * Set up the map serialization thread for saving/loading and the start the thread
 * @param sCommand the function that was called (eg. SaveMap)
 * @param sParams the params string, which may contain a filename and/or a map number
 */
void System::StartMapSerialization(std::string sCommand, std::string sParams) {
	if (mpMapSerializer->Init(sCommand, sParams, *mpMap)) {
		mpMapSerializer->start();
	}
}

/**
 * Draw a box with information about the maps.
 */
void System::DrawMapInfo() {
	int nLines = static_cast<int> (mvpMaps.size()) + 2;
	int x = 5, y = 120, w = 160, nBorder = 5;

	mGLWindow.DrawBox(x, y, w, nLines, 0.7f);

	y += 17;

	glColor3f(1, 1, 1);
	std::ostringstream os;
	os << "Maps " << mvpMaps.size();
	mGLWindow.PrintString(ImageRef(x + nBorder, y + nBorder), os.str());
	os.str("");

	for (size_t i = 0; i < mvpMaps.size(); i++) {
		Map * pMap = mvpMaps[i];
		if (pMap == mpMap) {
			glColor3f(1, 1, 0);
		} else if (pMap->bEditLocked) {
			glColor3f(1, 0, 0);
		} else {
			glColor3f(1, 1, 1);
		}

		os << "M: " << pMap->MapID() << "  P: " << pMap->vpPoints.size()
				<< "  K: " << pMap->vpKeyFrames.size();
		mGLWindow.PrintString(
				ImageRef(x + nBorder, y + nBorder + (i + 1) * 17), os.str());
		os.str("");
	}
}

/**
 * Save the current frame to a FIFO.
 * This function is called on each frame to create a video.
 * The GVar SaveFIFO starts and stops the saving, and the GVar
 * Bitrate sets the quality.
 * Bitrate can only be set before the first call of SaveFIFO.
 */
void System::SaveFIFO() {
#ifdef _LINUX
	//Some static variables
	static CVD::byte* pcImage = NULL;
	static int fd = 0;
	static bool bFIFOInitDone = false;
	static ImageRef irWindowSize;

	if( !bFIFOInitDone )
	{
		irWindowSize = mGLWindow.size();

		ostringstream os;
		os << /*"/bin/bash\n" <<*/
		"file=\"`date '+%Y-%m-%d_%H-%M-%S'`.avi\"; " <<
		"if [ ! -e FIFO ]; then mkfifo FIFO; echo Made FIFO...; fi; " <<
		"echo Mencoding to $file....; " <<
		"cat FIFO |nice mencoder -flip -demuxer rawvideo -rawvideo fps=30:w=" <<
		irWindowSize.x << ":h=" << irWindowSize.y <<
		":format=rgb24 -o $file -ovc lavc -lavcopts vcodec=mpeg4:vbitrate=" << *mgvnBitrate <<
		":keyint=45 -ofps 30 -ffourcc DIVX - &";

		cout << "::" << os.str()<< "::" << endl;
		int i = system( os.str().c_str() );
		if( i != 0 ) {
			cerr << "ERROR: could not set up the FIFO!" << endl;
			return;
		}

		int damy = posix_memalign((void**)(&pcImage), 16, irWindowSize.x*irWindowSize.y*3);
		if(damy);
		string s = "FIFO";
		fd = open(s.c_str(), O_RDWR | O_ASYNC);

		bFIFOInitDone = true;
	}

	if( irWindowSize != mGLWindow.size() )
	{
		cerr << "ERROR: Aborting FIFO as window size has changed!!" << endl;
		*mgvnSaveFIFO = 0;
		return;
	}

	glReadBuffer(GL_BACK);
	glReadPixels(0,0,irWindowSize.x,irWindowSize.y,GL_RGB, GL_UNSIGNED_BYTE, pcImage);
	int damy = write(fd, (char*) pcImage, irWindowSize.x*irWindowSize.y*3);
	if(damy);

#else
	cout << "Video Saving using FIFOs is only available under Linux" << endl;
#endif

}

// Added by Umakatsu
void System::flagReset(void * ptr) {
	static_cast<System *> (ptr)->bShowTexture = false;
	static_cast<System *> (ptr)->viewmode = false;
	static_cast<System *> (ptr)->bIgnoreMouseAction = false;
	static_cast<System *> (ptr)->autosegment = false;
	static_cast<System *> (ptr)->readyOtherThread(ptr);
}

void System::stopOtherThread(void * ptr) {
	static_cast<System *> (ptr)->segment->SetInterrupt(true);
	static_cast<System *> (ptr)->construct->setInterrupt(true);
}

void System::readyOtherThread(void * ptr) {
	static_cast<System *> (ptr)->segment->SetInterrupt(false);
	static_cast<System *> (ptr)->construct->setInterrupt(false);
}

}
