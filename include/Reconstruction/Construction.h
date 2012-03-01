/*
 * Construction.h
 *
 *  Created on: 2010/12/15
 *      Author: umakatsu
 */

#ifndef CONSTRUCTION_H_
#define CONSTRUCTION_H_
#include <cvd/thread.h>

#include "common.h"

#include <vector>
#include <deque>
#include "PTAMTracking/ATANCamera.h"
#include "Reconstruction/MarchingCubes.h"
#include "Image/ForegroundContents.h"
#include "reconstruction.h"

namespace PTAMM{
	const double NUMOFVOTES = 8;
	const uint MAXSIZE = 5;

	class ForegroundContents;
	class Construction: protected CVD::Thread
	{
	public:
		Construction( const ImageSize &  img_size , ForegroundContents * f , const ATANCamera & cam);
		Construction( void );
		~Construction( void );
		void New( void );
		void Save( const char * filename);
		void Start( void );
		void Reset( void );
		void Reconstruct( void );
		void Recovery( void );
		void DrawProjectedRegion( const TooN::SE3<> &  se3CfromW, const TooN::Matrix< 4 > & proj ) const;
		void DrawObjects( const TooN::SE3<> &  se3CfromW, const TooN::Matrix< 4 > & proj ) const;
		void CreateMarchingCubes( void );
		void createTexture(const ImageType & imRGB , const TooN::SE3<> & se3CFromW);
		void createTexture(const ImageType & imRGB , const TooN::SE3<> & se3CFromW , uchar * silhouette);	//2011.2.13 23:03
		const ImageType loadTexture( const char * filename);
		void createModel( const uint & modeltarget );
		void createModel( std::deque<Triangle> * mTriangles , std::deque<TextureInfo *> t, const int & texnum );
		void translateModel( const TooN::Vector< 3 > offset , const int & mnum );
		void scalingModel( const double & magnification , const int & mnum );
		void copyModel( void );
		void pushForeground(unsigned char * s , const TooN::SE3<> & current , const ImageType & img_color);
		void calcVoxelPosition( void );

		/* Getter and Setter and similar method */
		uint	sizeForeground( void ) const;
		void	setInterrupt(bool interrupt);
		void	setCamera(ATANCamera camera);
		void	setSideOfCube(const TooN::Vector< 3 > & dx , const TooN::Vector< 3 > & dy , const TooN::Vector< 3 > & dz , const TooN::Vector< 3 > & start_vector);
		void	setStateSCM( state state_scm );
		void	setStateMCM( state state_mcm );
		bool	getInterrupt() const;
		state	getStateSCM( void )const;
		state	getStateMCM( void )const;
		uint	getSizeTexture( void ) const;
		ulint	getCurrentVoxel( void ) const;
		int		getForeground( void ) const;
		void	clearForeground( void );
		MarchingCubes *	getCubesData( void )const;
		ImageType 			getImageRGBFromTexture( int num ) const;
		TooN::SE3<> 			getCamemraPoseFromTexture( int num )const;

	protected:
		virtual void run();

	private:
		std::vector<Model *> models;
		GridPoint 		***voxels;				//voxel information
		MarchingCubes	*cubesdata;			//mesh information
		ulint current_empty;					//the number of free voxels

		int width;								//image width
		int height;							//image height
		short int num_of_carving;
		TooN::Vector< 3 > dx;				//cube x-side
		TooN::Vector< 3 > dy;				//cube y-side
		TooN::Vector< 3 > dz;				//cube z-side
		TooN::Vector< 3 > start_vector;
		state state_scm;						//running descriptor for Space Carving Method
		state state_mcm;						//running descriptor for Marching Cubes Method
		bool interrupt;
		bool bDelete;

		ForegroundContents	* fContents;	//foreground object information
		ATANCamera 			  camera;		// Same as the tracker's camera

		std::deque<SegmentationInfo * > foreground;

		// Cached from Last projection
		TooN::Vector< 3 >	v3Pos;
		TooN::Vector< 3 > v3Cam;
		TooN::Vector< 2 > v2ImPlane;
		TooN::Vector< 2 >	v2Ima;
		double 			wRatio;
		double				hRatio;
		int					wStart;
		int					wEnd;
		int					hStart;
		int					hEnd;
		bool				bIn;
		// Cached from Voxel position
		TooN::Vector< 3 > max;
		TooN::Vector< 3 > min;

		void 	Init(void);
		void 	initVoxel( void );
		void 	reconstructVoxel( void );
		void 	recoveringVoxel ( void );
		void 	deallocateTables(void);
		void 	updateVoxel( void );
		void 	updateVoxel( uchar * rsil , TooN::SE3<> cpose);
		double projection(const int & i , const int & j , const int & k , const unsigned char * silhouette , const TooN::SE3<> & se3CFromW );
		void 	deleteForegroundData( void );
		void 	saveVoxelData(const char * filename);
	};
}
#endif /* CONSTRUCTION_H_ */
