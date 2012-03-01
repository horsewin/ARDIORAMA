# DO NOT DELETE THIS LINE -- make depend depends on it.

EXTRA_CFLAGS:= -DENABLE_MODELS_GAME
# Edit the lines below to point to any needed include and link paths
# Or to change the compiler's optimization flags
CC = g++

#Compile and Link Flag
OPIRALIB = -lOpiraLibrary -lOpiraLibraryMT -lRegistrationAlgorithms
OPIRAINCPATH = /home/umakatsu/desktop/Lab/M1/OPIRALIBRARY/include
OPENCVLIB = -lhighgui -lcvaux -lcv -lml -lcxcore
OPENGLLIB = -lGL -lGLU -lglut
OSGLIB = -losg -losgUtil -losgGA -losgViewer -losgText -losgDB -losgShadow -lOpenThreads
OSGINCPATH = /home/umakatsu/Download/OpenSceneGraph-3.0.1/include
VRPNINCPATH = /home/umakatsu/Download/vrpn
VRPNLIB = -lvrpn -lvrpnatmel -lvrpnserver
3DSLIB = -l3ds

ARMMINC=-I/$(OPIRAINCPATH) -I/$(OSGINCPATH) -I/$(VRPNINCPATH)
ARMMLIB=-L/home/umakatsu/desktop/Lab/M1/OPIRALIBRARY -L/home/umakatsu/Download/vrpn/pc_linux64 $(OPIRALIB) $(OSGLIB) $(VRPNLIB)

#COMPILEFLAGS = -I MY_CUSTOM_INCLUDE_PATH -D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3 -fno-strict-aliasing
#LINKFLAGS = -L MY_CUSTOM_LINK_PATH -lGVars3 -lcvd $(3DSLIB)
COMPILEFLAGS = -I MY_CUSTOM_INCLUDE_PATH -I./include -I/usr/include/octave -I/usr/include/opencv -I/usr/include/flycapture $(ARMMINC) -D_LINUX -D_REENTRANT -Wall -O3 -march=nocona -pipe -fno-strict-aliasing -mfpmath=sse -fomit-frame-pointer -msse3 -fno-tree-vectorize -g -fpermissive 
LINKFLAGS = -L MY_CUSTOM_LINK_PATH -L/usr/lib/octave-3.2.3 $(ARMMLIB) -Wl,-rpath /usr/lib/octave-3.2.3 -lblas -llapack -lGVars3 -lcvd -lGL -lGLU -lglut -lGLEW -lboost_serialization -loctave -lode -lflycapture -lgslcblas -lhighgui -lcvaux -lcv -lml -lcxcore  $(3DSLIB)

######## Object file #########
SUBDIRS = ./ARMM ./Demo ./Image ./Model ./PTAMTracking ./Reconstruction ./Stroke ./VideoSource ./Window
VPATH = $(SUBDIRS)

VIDEOSOURCE=Obj/VideoSource_Linux_DV.o	
SEGMENTATION=Obj/Segmentation.o

#3DS modelling game dependencies
3DSFILES =	Obj/ModelsGame.o\
						Obj/ModelBrowser.o\
						Obj/ModelControls.o\
						Obj/MGButton.o\
						Obj/Model3ds.o\
						Obj/ModelsGameData.o

RECONSTRUCTION = $(SEGMENTATION)\
										Obj/Construction.o\
										Obj/MarchingCubes.o\
										Obj/maxflow.o\
										Obj/graph.o\
										Obj/GraphCut.o\
										Obj/GMM.o\
										Obj/BrushAction.o

ARMM = Obj/ARMM_Client.o\
				Obj/ARMM_vrpn.o\
				Obj/MyShadowMap.o\
				Obj/PointgreyCamera.o
														
# Object files for an AR Diorama
ARDiorama = Obj/Authoring.o\
		Obj/StrokeAnalyze.o\
		$(RECONSTRUCTION)\
		Obj/TrackerDrawable.o\
		Obj/StrokeGenerator.o\
		Obj/Stroke.o\
		Obj/Texture.o\
		Obj/ModelImplement.o\
		Obj/Switcher.o\
		Obj/VideoData.o\
		Obj/ImageLabeling.o\
		Obj/ForegroundContents.o\
		Obj/BmpFileIO.o\
		Obj/Bitmap.o\
		Obj/Tables.o\
		Obj/Moment.o

OBJECTS=	Obj/main.o\
		Obj/GLWindow2.o\
		Obj/GLWindowMenu.o\
		$(VIDEOSOURCE)\
		Obj/System.o \
		$(ARDiorama)\
		$(ARMM)\
		Obj/ATANCamera.o\
		Obj/KeyFrame.o\
		Obj/MapPoint.o\
		Obj/Map.o\
		Obj/SmallBlurryImage.o\
		Obj/ShiTomasi.o \
		Obj/HomographyInit.o \
		Obj/MapMaker.o \
		Obj/Bundle.o \
		Obj/PatchFinder.o\
		Obj/Relocaliser.o\
		Obj/MiniPatch.o\
		Obj/MapViewer.o\
		Obj/ARDriver.o\
		Obj/EyeGame.o\
		Obj/Tracker.o\
		Obj/tinyxml.o\
		Obj/tinyxmlerror.o\
		Obj/tinyxmlparser.o\
		Obj/MapLockManager.o\
		Obj/MD5.o\
		Obj/MD5Wrapper.o\
		Obj/MapSerializer.o\
		Obj/Games.o\
		Obj/Utils.o\
		Obj/ShooterGame.o\
		Obj/ShooterGameTarget.o\
		$(3DSFILES)


CALIB_OBJECTS=Obj/GLWindow2.o\
		Obj/GLWindowMenu.o\
		$(VIDEOSOURCE)\
		Obj/CalibImage.o \
		Obj/CalibCornerPatch.o\
		Obj/ATANCamera.o \
		Obj/CameraCalibrator.o

all: ARDiorama CameraCalibrator

ARDiorama: $(OBJECTS)
	cd Obj
	$(CC) -g -o ARDiorama $(OBJECTS) $(LINKFLAGS)
	cd ..

CameraCalibrator:$(CALIB_OBJECTS)
	$(CC) -g -o CameraCalibrator $(CALIB_OBJECTS) $(LINKFLAGS)


Obj/%.o: %.cc
	$(CC) $< -o $@ -c $(COMPILEFLAGS) $(EXTRA_CFLAGS)

Obj/%.o: %.cpp
	$(CC) $< -o $@ -c $(COMPILEFLAGS) $(EXTRA_CFLAGS)

clean:
	rm Obj/*.o


depend:
	rm dependecies; touch dependencies
	makedepend -fdependencies $(INCLUDEFLAGS) $(MOREINCS) *.cc *.cpp *.h

-include dependencies

System.o: System.cc System.h ARMM_Client.cpp ARMM_Client.h osg_Client.h
ARMM_Client.o:ARMM_Client.cpp ARMM_Client.h osg_Client.h