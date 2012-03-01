/*
 * Construction.cc
 *
 *  Created on: 2010/12/15
 *      Author: umakatsu
 */
#include "Reconstruction/Construction.h"

#include <boost/foreach.hpp>

#include <GL/glut.h>

#include <iostream>
#include <fstream>
#include <math.h>
#include <stdio.h>
#include <cstdio>

#include "Model/Model.h"

#include <sys/types.h>
#include <dirent.h>
#include "myCVmethod.h"

#include <cvd/image_io.h>

//const definition
namespace PTAMM{

#define CHECK 0 //処理時間を出力するorしない

//namespace definition
using namespace std;

namespace{
	int signumber(const double & obj){
		if( obj > 0.001) return 1;
		else if(obj < 0.001 && obj > -0.001) return 0;
		else return -1;
	}
}

/*
 * constructor
 */
Construction::Construction( void )
: camera("NULL")
{

}

Construction::Construction(const ImageSize &  img_size , ForegroundContents * f , const ATANCamera & cam)
: width(img_size.width) , height(img_size.height) , fContents(f) , camera(cam)
{
	state_scm = NEW;
	state_mcm = NEW;
	interrupt		= false;
	current_empty = 0;
	num_of_carving = 0;
	foreground.clear();
	cubesdata = NULL;

	start();
}

Construction::~Construction(){
	deallocateTables();
	delete cubesdata;
}

void Construction::run( ){
	while(!shouldStop())  // ShouldStop is a CVD::Thread func which return true if the thread is told to exit.
	{
		if(!interrupt){
			// run SCM when new segment data was reserved and SCM button have been pushed
			if( state_scm == RUN ){
				updateVoxel();
			}
			else if( state_scm == INIT ){
				Init();
			}
			if( state_mcm == RUN ){
				state_scm = IDLE;
				CreateMarchingCubes();
				state_scm = READY;
			}
		}else{
			if( state_scm != NEW) state_scm = READY;
			if( state_mcm != NEW) state_mcm = READY;
		}
	}
}

/*
 * 領域の生成
 */
void Construction::Init( void ){
	if ( voxels) {
		deallocateTables();
	}
	voxels = new GridPoint**[size + OFFSET];
	for (int i = 0; i < size + OFFSET; i++) {
		voxels[i] = new GridPoint*[size + OFFSET];
		for (int j = 0; j < size + OFFSET; j++) {
			voxels[i][j] = new GridPoint[size + OFFSET];
		}
	}
	initVoxel();
	//MCM running
	if( state_mcm == NEW ){
		state_mcm = INIT;
		cubesdata = new MarchingCubes(size , 0.0 , this->camera);
		cubesdata->initGridPoints(voxels);
		cubesdata->initGridCells();
		cubesdata->run();
		state_mcm = READY;
	}
	//Inform finishing Initializing
	state_scm = READY;
}

void Construction::initVoxel( void ){
	REP(i,size+1) REP(j,size+1) REP(k,size+1){
		if( i==0 || j==0 || k==0 || i==(size) || j==(size) || k==(size))
			voxels[i][j][k].val = EMPTY;
		else
			voxels[i][j][k].val = OCCUPIED;
		Vector< 3 > push = start_vector + dx * i + dy * j + (-dz) * (k-size/2);
		REP(id,3) voxels[i][j][k].coord[id] = push[id];
	}
}

void Construction::New( void ){
	if( voxels ){
		Model * model = cubesdata->modelCreate();
		models.push_back(model);
		cout << "Model : " << models.size() << endl;
	}else{
		cerr << "Failed to reconstruct " << endl;
	}
}

void Construction::Save(const char * filename){
	interrupt = true;
	if( voxels ){
		saveVoxelData(filename);
		cout << "Completed to save Voxel Data -> " << filename << endl;
	}else{
		cerr << "Failed to save Voxel Data" << endl;
	}
	interrupt = false;
}

void Construction::Start( void ){
//	if( state_scm == NEW ){
//		Init();
//	}
//	start();
}

/*
 * 状態のリセット
 */
void Construction::Reset( void ){
	if(!shouldStop()) {
		state_scm = NEW;
		state_mcm = NEW;
		interrupt		= false;
		current_empty = 0;
		foreground.clear();
		if ( voxels) deallocateTables();
		if ( cubesdata ) delete cubesdata;

		stop();
	}
}

void Construction::Reconstruct( void ){
	interrupt = true;
	if( voxels ){
		calcVoxelPosition();
		reconstructVoxel();
		cout << "Finished to reconstruct" << endl;
	}else{
		cerr << "Failed to reconstruct " << endl;
	}
	interrupt = false;
}

void Construction::Recovery( void ){
	interrupt = true;
	if( voxels ){
		recoveringVoxel();
	}else{
		cerr << "Failed to recover" << endl;
	}
	interrupt = false;
}

void Construction::reconstructVoxel( void ){
	const Vector< 3 > topleft = max;
	Vector< 3 > topright = max;
	Vector< 3 > bottomleft = max;
	Vector< 3 > topleftfar = max;
	topright[0] = min[0];
	bottomleft[1] = min[1];
	topleftfar[2] = min[2];
	const Vector< 3 > DX = ( topright -  topleft)/size;
	const Vector< 3 > DY = ( bottomleft - topleft)/size;
	const Vector< 3 > DZ = ( topleftfar -  topleft)/size;
	// Renew a set of voxels
	current_empty = 0;
	REP(i,size+1) REP(j,size+1) REP(k,size+1){
		if( i==0 || j==0 || k==0 || i==(size) || j==(size) || k==(size))
			voxels[i][j][k].val = EMPTY;
		else
			voxels[i][j][k].val = OCCUPIED;
		Vector< 3 > push = topleft + DX * i + DY * j + DZ * k;
		REP(id,3) voxels[i][j][k].coord[id] = push[id];
	}
	cout << "Finish renewing voxel data!" << endl;

	// Reconstruct above data by the reserved silhouette
//	int rS = static_cast< int >( cubesdata->getSizeTexture() );
//	REP(i,rS){
//		state_scm = RUN;
//		updateVoxel( cubesdata->getSilhouetteFromTexture(i) , cubesdata->getCamemraPoseFromTexture(i));
//	}
	cout << "Finish reconstructing voxel data!" << endl;
}

void Construction::recoveringVoxel( void ){
	//空ボクセル数の初期化
	ulint empty = 390152;
	//処理対象データがなければ実行しない
	if(foreground.size() < 1) return;
	//SCMが動作しているかどうか
	if( state_scm != RUN ) return;
	unsigned char *	silhouette = foreground.back()->silhouette;
	TooN::SE3<> 			se3CFromW  = foreground.back()->camerapose;
	// Renew a set of voxels
	for(int i = 1; i < size; i++) for(int j = 1; j < size; j++) for( int k = 1; k < size; k++){
		if(signumber(voxels[i][j][k].val) > EPSILON);
		else{
			// recovery of a set of voxels
			if( projection(i , j , k , silhouette , se3CFromW) > EPSILON ){
				voxels[i][j][k].val =  OCCUPIED;
			}else{
				empty++;
			}
		}
	}
	current_empty = empty;
}
void Construction::updateVoxel( void ){
#if CHECK
	double start = TIME();
#endif
	//空ボクセル数の初期化
	ulint empty = size*size*2+ (size - 2)*size*2 + (size - 2)*(size - 2)*2;
	//処理対象データがなければ実行しない
	if( fContents->getSeginfo() < 1) return;
	//SCMが動作しているかどうか
	if( state_scm != RUN ) return;
	//TODO foregroundメンバの削除
//	unsigned char *	silhouette = foreground.back()->silhouette;
//	TooN::SE3<> 			se3CFromW  = foreground.back()->camerapose;
	const unsigned char * silhouette = fContents->popSilhouette();
	const TooN::SE3<> 	 se3CFromW  = fContents->popCameraPose();
	while(fContents->pData.state_proj == RUN) state_scm = IDLE;
	state_scm = RUN;
	fContents->pData.state_proj = RUN;
	fContents->pData_pre.state_proj = RUN;
	REP(i,width*height){
		fContents->pData_pre.projected[i] = fContents->pData.projected[i];
		fContents->pData.projected[i] = 0.0;
	}
	if(current_empty > 0)
		fContents->pData_pre.state_proj = READY;
	else
		fContents->pData_pre.state_proj = NEW;
	for(int i = 1; i < size; i++) for(int j = 1; j < size; j++) for( int k = 1; k < size; k++){
		if(voxels[i][j][k].val <= EPSILON) empty++;
		else{
			//First Carving do not take votes, since then a number of votes are used
			if(current_empty > 0){
				voxels[i][j][k].val =  signumber(voxels[i][j][k].val) * ( voxels[i][j][k].val + (2 * projection(i , j , k , silhouette , se3CFromW) - 1) / NUMOFVOTES );
			}
			else{
				voxels[i][j][k].val = voxels[i][j][k].val * projection(i , j , k , silhouette , se3CFromW);
			}
			if(voxels[i][j][k].val > OCCUPIED ) voxels[i]	[j][k].val = OCCUPIED;
		}
	}
	if( current_empty <= 0 || ( cubesdata->getSizeTexture() < 3 ) ) fContents->pData.state_proj = IDLE;
	else fContents->pData.state_proj = READY;
	num_of_carving++;
	//Erase silhouette and camera pose
	if( state_scm == READY ) return;
	state_scm = READY;
	while(bDelete);
	deleteForegroundData();
	if( current_empty < empty){
		/* Free-spaceが増加したときにMarching Cubes法を実行する */
		if( num_of_carving > 5
				){
			state_mcm = RUN;
			num_of_carving = 0;
		}
		current_empty = empty;
	}
#if CHECK
	cout << "Construction END : " << PROCESSTIME(start) << endl;
#endif
}

//void Construction::updateVoxel( uchar * rsil , TooN::SE3<> cpose){
//#if CHECK
//	double start = TIME();
//#endif
//	//空ボクセル数の初期化
//	unsigned int empty = 390152;
//	//SCMが動作しているかどうか
//	if( state_scm != RUN ) return;
//	//Get the silhouette and camera pose
//	unsigned char *	silhouette = rsil;
//	TooN::SE3<> 			se3CFromW  = cpose;
////	ATANCamera			atancam	= this->camera;
//	while(contents->pData.state_proj == RUN);
//	contents->pData.state_proj = RUN;
//	REP(i,width*height) contents->pData.projected[i] = 0.0;
//	for(int i = 1; i < size; i++) for(int j = 1; j < size; j++) for( int k = 1; k < size; k++){
//		if(voxels[i][j][k].val <= EPSILON) empty++;
//		else{
//			voxels[i][j][k].val = voxels[i][j][k].val * projection(i , j , k , silhouette , se3CFromW );
//		}
//	}
//	state_scm = READY;
//	contents->pData.state_proj = IDLE;
//	if( current_empty < empty){
//		state_mcm = RUN;
//		current_empty = empty;
//	}
//#if CHECK
//	cout << "Construction END : " << PROCESSTIME(start) << endl;
//#endif
//}

/*
 * @param i : height
 * @param j : width
 * @param k : depth
 * @param silhouette
 * ボクセル(i,j,k)を渡してシルエット画像内部にあるかを判定するメソッド
 */
double Construction::projection(const int & i , const int & j , const int & k , const unsigned char * silhouette , const TooN::SE3<> & se3CFromW )
{
	//convert from the address of voxel to 3D Vector data
	REP(x,3) v3Pos[x] = voxels[i][j][k].coord[x];
	//3次元位置を2次元座標系に落とす
	v3Cam = se3CFromW * v3Pos;
	v2ImPlane = project(v3Cam);
	v2Ima = camera.Project(v2ImPlane);
	double address[2];
	address[0] = static_cast< double >(v2Ima[0]);	//width
	address[1] = static_cast< double >(v2Ima[1]);	//height
	//Check if the projected pixel is in the Image Plane
	if(address[0] >= 0.0 && address[0] < width && address[1] >= 0.0 && address[1] < height
	){
		wRatio = width / size;
		hRatio = height / size;
		wStart	= floor(address[0]);
		hStart	= floor(address[1]);
		wEnd	= ceil(address[0] + wRatio );
		hEnd	= ceil(address[1] + hRatio );
		bIn		= false;
		for(int x = wStart; x < wEnd; x++){
			for(int y = hStart; y < hEnd; y++){
//		for(int x = wStart; x <= wStart; x++){
//			for(int y = hStart; y <= hStart; y++){
//				if( silhouette[ x + y * width ] == 255 ) return 1;
				if( silhouette[ x + y * width ] == 255 ) {
					bIn = true;
				}
				if( fContents->pData.projected[ x + y * width] < voxels[i][j][k].val){
					fContents->pData.projected[ x + y * width] = voxels[i][j][k].val;
				}
			}
		}
		if(bIn)	return 1.0;
		else 		return 0.0;
	}else{
		return 0.0;
	}
}

/*
 * 領域の解放
 */
void Construction::deallocateTables(void) {
	if (voxels) {
		for (int i = 0; i < size + OFFSET; i++) {
			for (int j = 0; j < size + OFFSET; j++) {
				delete [] voxels[i][j];
			}
			delete [] voxels[i];
		}
		delete [] voxels;
		voxels = NULL;
	}
}

//#TODO スマートポインタを使う
void Construction::deleteForegroundData( void ) {
//	SegmentationInfo *buf = foreground.front();
//	delete buf;
//	foreground.pop_front();
	bDelete = true;
//	SegmentationInfo *buf = foreground.back();
//	SegmentationInfo *buf = foreground.front();
	SegmentationInfo *buf = fContents->popSegmentationInfoRef();
	delete buf;
	fContents->popSegmentationInfo();
//	foreground.pop_front();
	bDelete = false;
}

/*
 * MC法を実行してメッシュデータを生成する
 */
void Construction::CreateMarchingCubes( void ){
	if( state_mcm == RUN ){
#if CHECK
		double start = TIME();
		cout << "Marching Cubes Run" << endl;
#endif
		cubesdata->initGridPoints(voxels);
		cubesdata->run();
		state_mcm = READY;
#if CHECK
		cout << "Marching Cubes End : " << PROCESSTIME(start) << endl;
#endif
	}
}

void Construction::createModel( const uint & modeltarget ){
	if( models.size() > 0){
		if( models.size() <= modeltarget){
			cerr << " Overflow model" << endl;
			return;
		}
		Model * copy_model = models.at(modeltarget);
//		Model * model = new Model( copy_model->getTextureSum() );
		Model * model = Model::create( copy_model->getTextureSum() );
		model->copy( copy_model->getMeshes() , copy_model->getTexture() , copy_model->getTextureSum() );
		models.push_back(model);
		cout << "Model : " << models.size() << endl;
	}else{
		cerr << "No Model exist" << endl;
	}
}

void Construction::createModel( deque<Triangle> * mTriangles , deque<TextureInfo *> t, const int & texnum ){
	assert( mTriangles  );
	assert( t.size() > 0);
	assert( static_cast< int >( t.size() ) == texnum );
	Model * model = Model::create( texnum );
	model->copy(mTriangles , t , texnum);
	models.push_back(model);
	cout << "Model : " << models.size() << endl;
}

void Construction::translateModel( const TooN::Vector< 3 > offset , const int & mnum ){
	if( mnum < static_cast< int >(models.size()) ){
		models.at(mnum)->translate( offset );
	}else{
		cerr << "Error : Model Number " << endl;
	}
}

void Construction::scalingModel( const double & magnification , const int & mnum ){
	if( mnum < static_cast< int >(models.size()) ){
		models.at(mnum)->scale( magnification );
	}else{
		cerr << "Error : Model Number " << endl;
	}
}

void Construction::copyModel( void ){

}

void Construction::saveVoxelData(const char * filename){
//	cubesdata->saveOBJ(filename);
	cubesdata->save(filename);
}

/*
 * コンストラクション対象データの保存
 */
void Construction::pushForeground(unsigned char * s , const TooN::SE3<> & current , const ::ImageType & img_color){
	if( foreground.size() >= MAXSIZE){
		if(!bDelete) deleteForegroundData();
	}
//	if( foreground.size() >= MAXSIZE) return;
	SegmentationInfo * info = new SegmentationInfo( width , height );
	REP(i,width*height) info->silhouette[i]	= s[i];
	info->camerapose	= current;
	info->imageRGB	= img_color;
	foreground.push_back(info);
}

void Construction::calcVoxelPosition( void ){
	REP(i,3){
		max[i] = DBL_MIN;
		min[i] = DBL_MAX;
	}
	REP(i,size+1) REP(j,size+1) REP(k,size+1){
		if(voxels[i][j][k].val > EPSILON){
			REP(id,3){
				if( max[id] < voxels[i][j][k].coord[id]) max[id] = voxels[i][j][k].coord[id];
				if( min[id] > voxels[i][j][k].coord[id]) min[id] = voxels[i][j][k].coord[id];
			}
		}
	}
	cout << "Finish Calculating new position!" << endl;
}

void Construction::DrawProjectedRegion( const TooN::SE3<> &  se3CfromW, const TooN::Matrix< 4 > & proj ) const{
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);
	glColor4f(0.0 , 1.0 , 0.0 , 0.05);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glPointSize(5);
	glBegin(GL_POINTS);{
		REP(i,width*height){
			if( fContents->pData_pre.projected[i] > EPSILON ){
				glVertex2d( i%width , i/width);
			}
		}
	}glEnd( );
	glDisable(GL_BLEND);
	glDisable(GL_POINT_SMOOTH);
}

void Construction::DrawObjects( const TooN::SE3<> &  se3CfromW, const TooN::Matrix< 4 > & proj ) const {
	glClearDepth(1);
	glClear(GL_DEPTH_BUFFER_BIT);
	GLint mat_mode;
	glGetIntegerv( GL_MATRIX_MODE, & mat_mode );

	glMatrixMode( GL_PROJECTION );
	glPushMatrix( );
	glLoadIdentity( );
	CVD::glMultMatrix( proj );
	glEnable(GL_DEPTH_TEST);

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix( );
	glLoadIdentity( ); // Add date 2011.2.27 by Umakatsu
	CVD::glMultMatrix( se3CfromW );
	// Drawing phantom Object
	if( cubesdata ){
		glEnable(GL_BLEND);
		glColor4f(1.0 , 1.0 , 1.0 , 0.0);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		cubesdata->drawObject();
		glDisable(GL_BLEND);
	}
//	cubesdata->drawObject(offset);
	BOOST_FOREACH(Model * m, models){
		m->draw();
	}
	glPopMatrix( );
	glMatrixMode( GL_PROJECTION );
	glPopMatrix( );
	glMatrixMode( mat_mode );		// modify (TEST) 2011.2.27 by Umakatsu
//	glMatrixMode( GL_MATRIX_MODE );
	glDisable(GL_DEPTH_TEST);
}

/* **************** Getter , Setter *************** */
uint Construction::sizeForeground( void ) const{
	return fContents->getSeginfo();
}

void Construction::setSideOfCube(const TooN::Vector< 3 > & dx , const TooN::Vector< 3 > & dy , const TooN::Vector< 3 > & dz , const TooN::Vector< 3 > & start_vector){
	this->dx = dx;
	this->dy = dy;
	this->dz = dz * 2; //TODO 7.26変更してみた
	this->start_vector = start_vector;
}

bool Construction::getInterrupt() const{
    return interrupt;
}

void Construction::setInterrupt(bool interrupt){
    this->interrupt = interrupt;
}

void Construction::setCamera(ATANCamera camera){
	this->camera = camera;
}

void Construction::setStateSCM( state state_scm){
	this->state_scm = state_scm;
}

state Construction::getStateSCM( void )const{
	return state_scm;
}

void Construction::setStateMCM( state state_mcm){
	this->state_mcm = state_mcm;
}

state Construction::getStateMCM( void )const{
	return state_mcm;
}

uint Construction::getSizeTexture( void ) const{
	if(cubesdata)
		return cubesdata->getSizeTexture();
	else
		return 0;
}

void Construction::createTexture(const ::ImageType & imRGB , const TooN::SE3<> & se3CFromW){
	if(cubesdata)
		cubesdata->createTexture( imRGB , se3CFromW );
	else
		cerr << "No Cubes Data" << endl;
}

void Construction::createTexture(const ::ImageType & imRGB , const TooN::SE3<> & se3CFromW , uchar * silhouette){
	if( cubesdata )
		cubesdata->createTexture( imRGB , se3CFromW , silhouette);
	else
		cerr << "No Cubes Data" << endl;
}

/*
 * 生成したモデルのロード
 */
const ::ImageType Construction::loadTexture( const char * filename) {
	::ImageType TextureRGB = ( img_load(filename) );
	return (static_cast< const ::ImageType >(TextureRGB) );
}

MarchingCubes * Construction::getCubesData( void ) const {
	assert(cubesdata);
	return cubesdata;
}

::ImageType Construction::getImageRGBFromTexture( int num) const{
	assert(cubesdata);
	return cubesdata->checkImageRGB(num);
}

TooN::SE3<> Construction::getCamemraPoseFromTexture( int num )const{
	assert(cubesdata);
	return cubesdata->getCamemraPoseFromTexture(num);
}

ulint Construction::getCurrentVoxel( void ) const{
	return current_empty;
}

int Construction::getForeground( void ) const{
	return static_cast< int >( foreground.size() );
}

void Construction::clearForeground( void ){
	int size = foreground.size();
	REP(i,size) deleteForegroundData();
}
}
