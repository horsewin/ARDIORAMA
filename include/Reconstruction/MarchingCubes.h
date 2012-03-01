/*
 * MarchingCubes.h
 *
 *  Created on: 2010/12/27
 *      Author: umakatsu
 */

#ifndef MARCHINGCUBES_H_
#define MARCHINGCUBES_H_

#include "common.h"
#include "reconstruction.h"

#include "Reconstruction/Texture.h"
#include "PTAMTracking/ATANCamera.h"
#include "ImageType.h"

#include <list>
#include <deque>

#include <lib3ds.h>

//#include "Construction.h"

namespace PTAMM{

	const uint		MAX_TEXTURE	= 30;
	const int		OFFSET			= 1;

	struct GridPoint {
		TooN::Vector< 3 > coord;
		double val;
	};

	struct GridCell {
		GridPoint* point[8];
	};

	class Model;

	class MarchingCubes {

	public:
		MarchingCubes(int size , double isolevel = 0.0);
		MarchingCubes(int size, double isolevel , const ATANCamera & acam);
		~MarchingCubes(void);

		int run();
		void reset();
		void save(const char * filename);
		void saveOBJ(const char * filename);
		std::list<Triangle>& getTriangles(void);

		void initGridPoints( GridPoint ***grid);
		void initGridCells( void );

		void draw(const int & mode = 0);
		void drawObject( void );
		void createTexture(const ImageType & imRGB , const TooN::SE3<> & se3CFromW);
		void createTexture(const ImageType & imRGB , const TooN::SE3<> & se3CFromW , uchar * silhouette); // 2011.2.13 23:03
		void checkTexturePose( void );	// for debug
		ImageType checkImageRGB( int num ) const; // for debug
		TooN::SE3<> getCamemraPoseFromTexture( int num = -1) const;
		uchar * getSilhouetteFromTexture(int num = -1) const;
		unsigned int getSizeTexture( void ) const;
	//	unsigned int getSizeTriangle()const { return mTriangles.size(); }
		void setMpCamera( ATANCamera mpCamera){
			this->mpCamera = mpCamera;
		}
		GridPoint *** getMGridPoints( ){
			return mGridPoints;
		}

		Model * modelCreate( void );

	private:
		int polygoniseAndInsert(const GridCell* grid, double isoLevel);
		Point interpolate(double isoLevel, const GridPoint* gp1, const GridPoint* gp2);

		void allocateTables(void);
		void deallocateTables(void);

		void calculateNormal(Triangle& tri);
		void calculateTexture(Triangle & tri);
		double calculateTriangleArea(const Triangle & tri );

		int checkOptimalTexture( Triangle & tri , const int & mode = 0);
		bool checkOptimalTextureNumber( Triangle & tri , const int & texture_num);

		void SaveXMLFile( const char * filename );

		int SIZE;
		double mIsoLevel;
		TooN::Vector< 3 > gravity;
		GridPoint*** mGridPoints;
		GridCell***  mGridCells;
		std::deque<Triangle> *mTriangles;
		std::deque<Triangle> *mTriangles_damy;
		std::deque<TextureInfo *> texture;
		ATANCamera mpCamera;

		int triangleSize;
		int texSize;
	};
}
#endif /* MARCHINGCUBES_H_ */
