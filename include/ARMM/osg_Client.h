#ifndef OSG_CLIENT_H
#define OSG_CLIENT_H

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Texture2D>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osgShadow/ShadowedScene>
#include "MyShadowMap.h"
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/io_utils>
#include <osg/Depth>
#include <map>
#include <sstream>
#include <assert.h>

#include "OPIRALibraryMT.h"
#include "RegistrationAlgorithms/OCVSurf.h"

#include "opencv/highgui.h"
#include "osg_geom_data.h"

#define SIM_MICROMACHINE 1
#define REP(i,n) for(int i=0;i<(int)n;++i)

typedef unsigned int uint;
typedef unsigned char uchar;

//window size
const int WINDOW_WIDTH  = 640;
const int WINDOW_HEIGHT = 480;

//to show terrain(this variable don't be needed)
bool WIREFRAME_MODE = false;

//marker origin
CvPoint2D32f center_trimesh;
float TrimeshScale = 1;

//for virtual objects
int Virtual_Objects_Count = 0;
const int MAX_NUM_VIR_OBJ = 5;

//for virtual hands
const int MAX_NUM_HANDS = 1;

vector<int> obj_ind;//to remove a lost object

std::vector<osg::PositionAttitudeTransform*> hand_object_global_array;
std::vector<osg::PositionAttitudeTransform*> hand_object_transform_array[MAX_NUM_HANDS];

void setOSGTrimeshScale(float scale) {
	TrimeshScale = scale;
}

void osg_inittracker(string markerName, int maxLengthSize, int maxLengthScale);
void osg_setHFNode(osg::Node* n);
void osgAddObjectNode(osg::Node* n);
void osg_createHand(int index, float world_scale, float ratio);
void osg_UpdateHand(int index, float *x, float *y, float *grid);

class ARTrackedNode : public osg::Group {

private:
	OPIRALibrary::Registration *r;
	typedef std::map<std::string, osg::ref_ptr<osg::MatrixTransform> > MarkerMap;
	MarkerMap mMarkers;
	osg::PositionAttitudeTransform* osgTrans;
	osg::Light* light;

public:
	ARTrackedNode(): osg::Group() {
		r = new RegistrationOPIRAMT(new OCVSurf());
		osg::Group* lightGroup = new osg::Group;  
		// Create a local light.
		light = new osg::Light();
		light->setLightNum(0);
		light->setPosition(osg::Vec4(0.0f, 0.0f, 1.0, 0.0f));
		light->setAmbient(osg::Vec4(0.4f,0.4f,0.4f,1.0f));
		light->setDiffuse(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
		light->setSpecular(osg::Vec4(0.4f,0.4f,0.4f,1.0f));
		light->setConstantAttenuation(0.2f);
		light->setLinearAttenuation(0.05f);
		light->setQuadraticAttenuation(0.05f);

		osg::LightSource* lightSource = new osg::LightSource;	
		lightSource->setLight(light);
		lightSource->setLocalStateSetModes(osg::StateAttribute::ON); 
		lightSource->setStateSetModes(*this->getOrCreateStateSet(), osg::StateAttribute::ON);
		lightGroup->addChild(lightSource);
	
		this->addChild(lightGroup);

	}

	osg::Light *getLight() {
		return light;
	}

	void stop() {
		delete r;
	}
	void setVisible(int index, bool visible) {
		osg::ref_ptr<osg::Switch> s = (osg::Switch*)this->getChild(index);
		if (visible) s->setAllChildrenOn(); else s->setAllChildrenOff();
	}
	void processFrame(IplImage *mFrame, CvMat *cParams, CvMat *cDistort) {
		for (MarkerMap::iterator iter = mMarkers.begin(); iter != mMarkers.end(); iter++) {
			osg::ref_ptr<osg::Switch> s = (osg::Switch*)iter->second->getChild(0); s.get()->setAllChildrenOff();
		}
		std::vector<MarkerTransform> mTransforms = r->performRegistration(mFrame, cParams, cDistort);
		for (std::vector<MarkerTransform>::iterator iter = mTransforms.begin(); iter != mTransforms.end(); iter++) {
			MarkerMap::iterator mIter = mMarkers.find(iter->marker.name);
			if (mIter != mMarkers.end()) {
				osg::ref_ptr<osg::MatrixTransform> m = mIter->second;
				m.get()->setMatrix(osg::Matrixd(iter->transMat));
				((osg::Switch*)m->getChild(0))->setAllChildrenOn();
			}
		}
		for (uint i=0; i<mTransforms.size(); i++) {
			free(mTransforms.at(i).viewPort); free(mTransforms.at(i).projMat); 	free(mTransforms.at(i).transMat);
		}
		mTransforms.clear();
	}

	int addMarkerContent(string imageFile, int maxLengthSize, int maxLengthScale, osg::Node *node) {
		r->removeMarker(imageFile);
		if (r->addResizedScaledMarker(imageFile, maxLengthSize, maxLengthScale)) {
			Marker m = r->getMarker(imageFile);

			osgTrans = new osg::PositionAttitudeTransform();
			osgTrans->setAttitude(osg::Quat(
						osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
						osg::DegreesToRadians(180.f), osg::Vec3d(0.0, 1.0, 0.0),
						osg::DegreesToRadians(180.f), osg::Vec3d(0.0, 0.0, 1.0)
						));
			osgTrans->setPosition(osg::Vec3d(0.0, 0.0, 0.0));
			osgTrans->getOrCreateStateSet()->setMode(GL_NORMALIZE, GL_TRUE);
			osgTrans->addChild(node);

			osg::ref_ptr<osg::Switch> foundObject = new osg::Switch();
			foundObject->addChild(osgTrans);

			osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform();
			mt->addChild(foundObject);

			osg::ref_ptr<osg::Switch> trackObject = new osg::Switch();
			trackObject->addChild(mt);
			this->addChild(trackObject);

			mMarkers[imageFile] = mt;
			return this->getChildIndex(trackObject);
		} else {
			return -1;
		}
	}

	osg::Node* getOsgTrans() {
		return osgTrans;
	}

	void addModel(osg::Node *node) {
		osgTrans->addChild(node);
	}

	void removeModel(osg::Node *node) {
		osgTrans->removeChild(node);
	}
};

/*********************************************************/
	osg::Image* mVideoImage;
	IplImage *mGLImage;
	bool captureFrame();
	osg::ref_ptr<ARTrackedNode> arTrackedNode; 
	osg::ref_ptr<osg::Node> HFNode;
	//std::vector<osg::ref_ptr<osg::Node>> world_sphere_array;

	//HeightField
	osg::Vec3Array* HeightFieldPoints = new osg::Vec3Array;
	osg::Geometry* HeightFieldGeometry_quad = new osg::Geometry();
	osg::Geometry* HeightFieldGeometry_line = new osg::Geometry();
	osg::Vec4Array* groundQuadColor = new osg::Vec4Array();
	osg::Vec4Array* groundLineColor = new osg::Vec4Array();

	//Shadowing Stuff
	osg::ref_ptr<osgShadow::ShadowedScene> shadowedScene;
	unsigned int rcvShadowMask = 0x1;
	unsigned int castShadowMask = 0x2;

	int celicaIndex;
	std::vector<osg::PositionAttitudeTransform*> car_transform;
	std::vector<osg::PositionAttitudeTransform*> wheel_transform[2];

	std::vector<osg::ref_ptr<osg::Node> > obj_node_array;
	std::vector<osg::PositionAttitudeTransform*> obj_transform_array;

	std::vector<osg::PositionAttitudeTransform*> world_sphere_transform_array;

	vector<CvPoint3D32f> carRect;
	CvPoint2D32f osg_carSize;
	osgViewer::Viewer viewer;
/*********************************************************/

inline void LoadCheck(void * ptr, const char * filename)
{
	if( ptr == NULL){
		std::cerr << "Error : No such file in the directory or incorrect path.  " << filename << std::endl;
	}
}

void osg_init(double *projMatrix) {
	mVideoImage = new osg::Image();
	mGLImage = cvCreateImage(cvSize(512,512), IPL_DEPTH_8U, 3);

	viewer.addEventHandler(new osgViewer::WindowSizeHandler());
	viewer.setUpViewInWindow(100, 100, WINDOW_WIDTH, WINDOW_HEIGHT);
	
	viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
	viewer.setKeyEventSetsDone(0);

	osg::ref_ptr<osg::Group> root = new osg::Group();
	viewer.setSceneData(root);


	HFNode = new osg::Node();
	HeightFieldPoints = new osg::Vec3Array;

	//Create Height Field
	for (int i=0; i<159; i++) {
		for (int j=0; j<119; j++) {
			HeightFieldPoints->push_back(osg::Vec3(i-159, j-119, 0));
			HeightFieldPoints->push_back(osg::Vec3(i-158, j-119, 0));
			HeightFieldPoints->push_back(osg::Vec3(i-158, j-118, 0));
			HeightFieldPoints->push_back(osg::Vec3(i-159, j-118, 0));
		}
	}

	// ----------------------------------------------------------------
	// Video background
	// ----------------------------------------------------------------
	osg::ref_ptr<osg::Camera> bgCamera = new osg::Camera();
	bgCamera->getOrCreateStateSet()->setRenderBinDetails(0, "RenderBin");
	bgCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	bgCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
	bgCamera->getOrCreateStateSet()->setMode(GL_LIGHTING, GL_FALSE);
	bgCamera->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, GL_FALSE);
	bgCamera->setProjectionMatrixAsOrtho2D(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT);
	
	osg::ref_ptr<osg::Geometry> geom = osg::createTexturedQuadGeometry(osg::Vec3(0, 0, 0), osg::X_AXIS * WINDOW_WIDTH, osg::Y_AXIS * WINDOW_HEIGHT, 0, 1, 1, 0);
	geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, new osg::Texture2D(mVideoImage));
	
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	geode->addDrawable(geom.get());
	bgCamera->addChild(geode.get());
	root->addChild(bgCamera.get());
	
	// ----------------------------------------------------------------
	// Foreground 3D content
	// ----------------------------------------------------------------
	osg::ref_ptr<osg::Camera> fgCamera = new osg::Camera();
	fgCamera->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
	fgCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT);
	fgCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
	fgCamera->setProjectionMatrix(osg::Matrix(projMatrix));
	root->addChild(fgCamera.get());

	arTrackedNode = new ARTrackedNode();
	fgCamera->addChild(arTrackedNode);

};


void osg_inittracker(string markerName, int maxLengthSize, int maxLengthScale) {
	static bool hasInit = false;

	if (hasInit) {
		arTrackedNode->removeChildren(0,arTrackedNode->getNumChildren());
		celicaIndex = arTrackedNode->addMarkerContent(markerName, maxLengthSize, maxLengthScale, shadowedScene);
		arTrackedNode->setVisible(celicaIndex, true);
		return;
	}

	arTrackedNode->removeChildren(0,arTrackedNode->getNumChildren());

#ifdef SIM_MICROMACHINE
	const char * file = "ARMM/Data/HatuneMiku.3ds";
	osg::Node *car1 = osgDB::readNodeFile(file);
//	osg::Node *car1 = osgDB::readNodeFile(CAR1_BODY_FILENAME);
	LoadCheck(car1, file);

	osg::PositionAttitudeTransform * car_trans1 = new osg::PositionAttitudeTransform();
	car_trans1->setAttitude(osg::Quat(
		osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
		osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0),
		osg::DegreesToRadians(180.f), osg::Vec3d(0.0, 0.0, 1.0)
		));
	car_trans1->setPosition(osg::Vec3d(0.0, 0.0, 3.0));//shift body higher 3 units
	car_trans1->addChild(car1);

	osg::ref_ptr<osg::Node> wheel1 = osgDB::readNodeFile(CAR1_WHEEL_FILENAME);
	std::vector<osg::PositionAttitudeTransform*> wheel_tmp_trans1;
	LoadCheck(wheel1, CAR1_WHEEL_FILENAME);

	for(int i = 0 ; i < 4; i++)  { 
		wheel_tmp_trans1.push_back(new osg::PositionAttitudeTransform); 
		wheel_tmp_trans1.at(i)->addChild(wheel1.get());
		if(i == 0 || i == 3) {
			wheel_tmp_trans1.at(i)->setAttitude(osg::Quat(osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0), osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 0.0, 1.0)));
		} else {
			wheel_tmp_trans1.at(i)->setAttitude(osg::Quat(osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0), osg::DegreesToRadians(180.f), osg::Vec3d(0.0, 0.0, 1.0)));
		}
		wheel_tmp_trans1.at(i)->setPosition(osg::Vec3d(0.0, 0.0, 0.0));
		wheel_transform[0].push_back(new osg::PositionAttitudeTransform); 
		wheel_transform[0].at(i)->setNodeMask(castShadowMask );
		//wheel_transform[0].at(i)->setNodeMask(rcvShadowMask );
		wheel_transform[0].at(i)->addChild(wheel_tmp_trans1.at(i));
	}

	//begin car 2	
	osg::Node *car2 = osgDB::readNodeFile(CAR2_BODY_FILENAME);
	LoadCheck(car2, CAR2_BODY_FILENAME);

	osg::PositionAttitudeTransform * car_trans2 = new osg::PositionAttitudeTransform();
	car_trans2->setAttitude(osg::Quat(
		osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
		osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0),
		osg::DegreesToRadians(180.f), osg::Vec3d(0.0, 0.0, 1.0)
		));
	car_trans2->setPosition(osg::Vec3d(0.0, 0.0, 2.0));
	car_trans2->addChild(car2);

	osg::ref_ptr<osg::Node> wheel2 = osgDB::readNodeFile(CAR2_WHEEL_FILENAME);
	std::vector<osg::PositionAttitudeTransform*> wheel_tmp_trans2;
	LoadCheck(wheel2, CAR2_WHEEL_FILENAME);

	for(int i = 0 ; i < 4; i++)  { 
		wheel_tmp_trans2.push_back(new osg::PositionAttitudeTransform); 
		wheel_tmp_trans2.at(i)->addChild(wheel2.get());
		if(i == 0 || i == 3) {
			wheel_tmp_trans2.at(i)->setAttitude(osg::Quat(osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0), osg::DegreesToRadians(180.f), osg::Vec3d(0.0, 0.0, 1.0)));
		} else {
			wheel_tmp_trans2.at(i)->setAttitude(osg::Quat(osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0), osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 0.0, 1.0)));
		}
		wheel_tmp_trans2.at(i)->setPosition(osg::Vec3d(0.0, 0.0, 0.0));
		wheel_transform[1].push_back(new osg::PositionAttitudeTransform); 

		wheel_transform[1].at(i)->setNodeMask(castShadowMask );
		//wheel_transform[1].at(i)->setNodeMask(rcvShadowMask );
		wheel_transform[1].at(i)->addChild(wheel_tmp_trans2.at(i));
	}
	//end car 2

	car_transform.push_back(new osg::PositionAttitudeTransform());
	car_transform.at(0)->setAttitude(osg::Quat(0,0,0,1));
	car_transform.at(0)->addChild(car_trans1);
	car_transform.at(0)->setNodeMask(castShadowMask );
	//car_transform.at(0)->setNodeMask(rcvShadowMask );

	car_transform.push_back(new osg::PositionAttitudeTransform());
	car_transform.at(1)->setAttitude(osg::Quat(0,0,0,1));
	car_transform.at(1)->addChild(car_trans2);
	car_transform.at(1)->setNodeMask(castShadowMask );
	//car_transform.at(1)->setNodeMask(rcvShadowMask );

#endif /*SIM_MICROMACHINE*/

	//osgAddObjectNode(osgNodeFromBtSphere(4));
	//cout << "OBJ" << endl;
	//Virtual_Objects_Count++;


	// Set shadow node
//V	osg::ref_ptr<osgShadow::ShadowTexture> sm = new osgShadow::ShadowTexture;
	osg::ref_ptr<osgShadow::MyShadowMap> sm = new osgShadow::MyShadowMap; //Adrian
	sm->setTextureSize( osg::Vec2s(1024, 1024) ); //Adrian
	sm->setTextureUnit( 1 );

	shadowedScene = new osgShadow::ShadowedScene;
	shadowedScene->setShadowTechnique( sm.get() );
	shadowedScene->setReceivesShadowTraversalMask( rcvShadowMask );
	shadowedScene->setCastsShadowTraversalMask( castShadowMask );
	shadowedScene->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON); //Adrian  

//V	osg::ref_ptr<osg::LightSource> source = new osg::LightSource;
//V	source->getLight()->setPosition( osg::Vec4(4.0, 4.0, 10.0, 0.0) );
//V	source->getLight()->setAmbient( osg::Vec4(0.2, 0.2, 0.2, 1.0) );
//V	source->getLight()->setDiffuse( osg::Vec4(0.8, 0.8, 0.8, 1.0) );
//V	shadowedScene->addChild(source);

#ifdef SIM_MICROMACHINE
	shadowedScene->addChild( car_transform.at(0) );
	shadowedScene->addChild( car_transform.at(1) );

	for(int i = 0 ; i < 2; i++)  { 
		for(int j = 0 ; j < 4; j++)  { 
			shadowedScene->addChild(wheel_transform[i].at(j));
		}
	} 
#endif /*SIM_MICROMACHINE*/
	celicaIndex = arTrackedNode->addMarkerContent(markerName, maxLengthSize, maxLengthScale, shadowedScene);
	arTrackedNode->setVisible(celicaIndex, true);


/*Adrian */
	{
		// Create the Heightfield Geometry
		HeightFieldGeometry_quad->setVertexArray(HeightFieldPoints); 
		HeightFieldGeometry_quad->addPrimitiveSet(new osg::DrawArrays( GL_QUADS, 0, HeightFieldPoints->size()));
		HeightFieldGeometry_quad->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);

		HeightFieldGeometry_line->setVertexArray(HeightFieldPoints); 
		HeightFieldGeometry_line->addPrimitiveSet(new osg::DrawArrays( GL_LINES, 0, HeightFieldPoints->size()));
		HeightFieldGeometry_line->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);		

		//Set the Heightfield to be alpha invisible
		HeightFieldGeometry_quad->setColorBinding(osg::Geometry::BIND_OVERALL); 
		HeightFieldGeometry_line->setColorBinding(osg::Geometry::BIND_OVERALL); 
		//osg::Vec4Array* col = new osg::Vec4Array(); 
		HeightFieldGeometry_quad->setColorArray(groundQuadColor); 
		HeightFieldGeometry_line->setColorArray(groundLineColor);
		groundQuadColor->push_back(osg::Vec4(1,1,1,0.0));
		groundLineColor->push_back(osg::Vec4(1,1,1,0.0));
		//Create the containing geode
		osg::ref_ptr< osg::Geode > geode = new osg::Geode(); 
		geode->addDrawable(HeightFieldGeometry_quad);
		geode->addDrawable(HeightFieldGeometry_line);

		//Create the containing transform
		float scale = 10; float x = 0; float y = 0; float z = 0;
		osg::ref_ptr< osg::PositionAttitudeTransform > mt = new osg::PositionAttitudeTransform();
		mt->setScale(osg::Vec3d(scale,scale,scale));
		mt->setAttitude(osg::Quat(0,0,0,1));       
		mt->setPosition(osg::Vec3d(x, y, z)); 
		mt->addChild( geode.get() );  

		//Set up the depth testing for the landscale
		osg::Depth * depth = new osg::Depth();
		depth->setWriteMask(true); depth->setFunction(osg::Depth::LEQUAL);
		mt->getOrCreateStateSet()->setAttributeAndModes(depth, osg::StateAttribute::ON);

		//Set up the shadow masks
		mt->setNodeMask( rcvShadowMask );
		mt->getOrCreateStateSet()->setRenderBinDetails(0, "RenderBin");
#ifdef SIM_MICROMACHINE
		car_transform.at(0)->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");
		car_transform.at(1)->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");
		for(int w=0; w<4; w++){
			wheel_transform[0].at(w)->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");
			wheel_transform[1].at(w)->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");
		}
#endif /*SIM_MICROMACHINE*/
		shadowedScene->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");

		//At the heightmap twice, once for shadowing and once for occlusion
		arTrackedNode->addModel(mt);
		shadowedScene->addChild(mt);


	}
/*Adrian*/

	//Added by Atsushi
	//this value depends on the state of Kinect Calibration
	REP(i,MAX_NUM_HANDS){
		osg_createHand(i, 0.39, 2.523809);
	}
	hasInit = true;
}

void osg_client_render(IplImage *newFrame, osg::Quat *q,osg::Vec3d  *v, osg::Quat wq[][4], osg::Vec3d wv[][4], CvMat *cParams, CvMat *cDistort) 
{
	assert( newFrame != NULL);
	cvResize(newFrame, mGLImage);
	cvCvtColor(mGLImage, mGLImage, CV_BGR2RGB);
	mVideoImage->setImage(mGLImage->width, mGLImage->height, 0, 3, GL_RGB, GL_UNSIGNED_BYTE, (unsigned char*)mGLImage->imageData, osg::Image::NO_DELETE);

#ifdef SIM_MICROMACHINE
	if(car_transform.at(0)) {
		for(int i = 0; i < 2; i++) {
			car_transform.at(i)->setAttitude(q[i]);
			car_transform.at(i)->setPosition(v[i]);
			for(int j = 0; j < 4; j++) {
				wheel_transform[i].at(j)->setAttitude(wq[i][j]);
				wheel_transform[i].at(j)->setPosition(wv[i][j]);
			}
		}
	}
#endif /*SIM_MICROMACHINE*/

	if (!viewer.done()) {
		if (CAPTURE_SIZE.width != REGISTRATION_SIZE.width || CAPTURE_SIZE.height != REGISTRATION_SIZE.height) {
			double scale = double(REGISTRATION_SIZE.width)/double(CAPTURE_SIZE.width);
			IplImage *scaledImage = cvCreateImage(cvSize(newFrame->width*scale, newFrame->height*scale), newFrame->depth, newFrame->nChannels); cvResize(newFrame, scaledImage);
			arTrackedNode->processFrame(scaledImage, cParams, cDistort);
			cvReleaseImage(&scaledImage);
		} else {
			arTrackedNode->processFrame(newFrame, cParams, cDistort);
		}
		viewer.frame();
	}
}

void osg_client_render(IplImage *newFrame, osg::Quat *q,osg::Vec3d  *v, osg::Quat wq[][4], osg::Vec3d wv[][4], CvMat *cParams, CvMat *cDistort, std::vector <osg::Quat> q_array,std::vector <osg::Vec3d>  v_array) 
{
	assert( newFrame != NULL);
	cvResize(newFrame, mGLImage);
	cvCvtColor(mGLImage, mGLImage, CV_BGR2RGB);
	mVideoImage->setImage(mGLImage->width, mGLImage->height, 0, 3, GL_RGB, GL_UNSIGNED_BYTE, (unsigned char*)mGLImage->imageData, osg::Image::NO_DELETE);

#ifdef SIM_MICROMACHINE
	//virtual cars part
	if(car_transform.at(0)) {
		for(int i = 0; i < 2; i++) {
			car_transform.at(i)->setAttitude(q[i]);
			car_transform.at(i)->setPosition(v[i]);
			for(int j = 0; j < 4; j++) {
				wheel_transform[i].at(j)->setAttitude(wq[i][j]);
				wheel_transform[i].at(j)->setPosition(wv[i][j]);
			}
		}
	}

	//virtual objects part
	for(uint i = 0; i < v_array.size(); i++) {
		obj_transform_array.at(i)->setAttitude(q_array.at(i));
		obj_transform_array.at(i)->setPosition(v_array.at(i));
	}

#endif /*SIM_MICROMACHINE*/

	if (!viewer.done()) {
		if (CAPTURE_SIZE.width != REGISTRATION_SIZE.width || CAPTURE_SIZE.height != REGISTRATION_SIZE.height) {
			double scale = double(REGISTRATION_SIZE.width)/double(CAPTURE_SIZE.width);
			IplImage *scaledImage = cvCreateImage(cvSize(newFrame->width*scale, newFrame->height*scale), newFrame->depth, newFrame->nChannels); cvResize(newFrame, scaledImage);
			arTrackedNode->processFrame(scaledImage, cParams, cDistort);
			cvReleaseImage(&scaledImage);
		} else {
			arTrackedNode->processFrame(newFrame, cParams, cDistort);
		}
		viewer.frame();
	}
}

void osg_uninit() {
	arTrackedNode->stop();
	cvReleaseImage(&mGLImage);
}

//void osg_UpdateHeightfieldTrimesh(float *ground_grid) {
//	int index =0;
//	for(int i = 0; i < 159; i++) {
//		for(int j = 0; j < 119; j++) {
//			float x = (float)(i- center_trimesh.x)*TrimeshScale; 
//			float y = (float)(j- (120-center_trimesh.y))*TrimeshScale;
//			HeightFieldPoints->at(index++) = osg::Vec3(x, y, ground_grid[j*160+i]); 
//			HeightFieldPoints->at(index++) = osg::Vec3(x+TrimeshScale, y, ground_grid[j*160+i+1]);
//			HeightFieldPoints->at(index++) = osg::Vec3(x+TrimeshScale, y+TrimeshScale, ground_grid[(j+1)*160+i+1]); 
//			HeightFieldPoints->at(index++) = osg::Vec3(x, y+TrimeshScale, ground_grid[(j+1)*160+i]);
//		}
//	}
//	HeightFieldGeometry_quad->dirtyDisplayList();
//	HeightFieldGeometry_line->dirtyDisplayList();
//}

void osgAddObjectNode(osg::Node* n) {
		obj_node_array.push_back(n);
		obj_transform_array.push_back(new osg::PositionAttitudeTransform());
		int index = obj_node_array.size()-1;
		//dynamic_cast<osg::Geode*>(n)->getDrawable(index)->
		obj_transform_array.at(index)->setAttitude(osg::Quat(
			osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 0.0, 1.0)
			));
		obj_transform_array.at(index)->setPosition(osg::Vec3d(0.0, 0.0, 0.0));
		//obj_transform_array.at(index)->setNodeMask(castShadowMask );
		//obj_transform_array.at(index)->setNodeMask(rcvShadowMask );
		obj_transform_array.at(index)->addChild(obj_node_array.at(index));
		shadowedScene->addChild( obj_transform_array.at(index) );

		obj_transform_array.at(index)->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
		shadowedScene->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");

		printf("Client Object number: %d added \n", index+1);
}

//----->Hand creating and updating
void osg_createHand(int index, float world_scale, float ratio) 
{
//	float sphere_size = 0.5;
	float sphere_size = world_scale * ratio;
	//sphere_size = 0.984286;
	const int scale = 10; 

	hand_object_global_array.push_back(new osg::PositionAttitudeTransform());
	for(int i = 0; i < MIN_HAND_PIX; i++) {
		for(int j = 0; j < MIN_HAND_PIX; j++) {
			//create a part of hands 
			osg::ref_ptr< osg::Sphere > sphere = new osg::Sphere(osg::Vec3d(0,0,0), sphere_size);
			osg::ShapeDrawable * shape = new osg::ShapeDrawable( sphere );
			shape->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			shape->setColor(osg::Vec4(0,0,0,0));
			osg::ref_ptr< osg::Geode> geode = new osg::Geode();
			geode->addDrawable( shape );

			hand_object_transform_array[index].push_back(new osg::PositionAttitudeTransform());
			int curr = hand_object_transform_array[index].size()-1;
			hand_object_transform_array[index].at(curr)->setScale(osg::Vec3d(scale,scale,scale));
			hand_object_transform_array[index].at(curr)->setPosition(osg::Vec3d(j*scale, i*scale, 1000));
			hand_object_transform_array[index].at(curr)->addChild( geode.get() );
			hand_object_global_array.at(index)->addChild( hand_object_transform_array[index].at(curr));
		}
	}
	hand_object_global_array.at(index)->setPosition(osg::Vec3d(0,0,0));
	shadowedScene->addChild( hand_object_global_array.at(index) );

	//set rendering order
	hand_object_global_array.at(index)->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
	shadowedScene->getOrCreateStateSet()->setRenderBinDetails(3, "RenderBin");
	cout << "Client Hand Create " << index << endl;
}

void osg_UpdateHand(int index, float *x, float *y, float *grid) 
{
	const int scale = 10; // to convert cm to mm scale
	const float HEIGHT_LIMITATION = 30;
	hand_object_global_array.at(index)->setPosition(osg::Vec3d(0, 0, 0));
	for(int i = 0; i < MIN_HAND_PIX; i++) {
		for(int j = 0; j < MIN_HAND_PIX; j++) {
			int curr = i*MIN_HAND_PIX+j;
			if(grid[curr] > 0 && grid[curr] < HEIGHT_LIMITATION ){
				hand_object_transform_array[index].at(curr)->setPosition(osg::Vec3d(x[curr]*scale, y[curr]*scale, grid[curr]*scale));
			}
			else{
				hand_object_transform_array[index].at(curr)->setPosition(osg::Vec3d(0,0,1000));
			}
		}
	}
}
#endif
