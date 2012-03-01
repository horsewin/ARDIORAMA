#include <octave/config.h>
#include <octave/Matrix.h>
#include "DrawObject.h"

#include <boost/scoped_ptr.hpp>
#include <boost/foreach.hpp>
#include <cmath>

#include "ModelImpl.h"
#include "Reconstruction/Background.h"
#include "Window/Analysis.h"
#include "Model.h"
#include "ODE.h"

#include "PTAMTracking/Block.h"

#define USE_BLOCK 0

namespace PTAMM{
namespace{
::Matrix get_covariance( ::Matrix &  x ){
	const int n( x.rows( ) );
	x = x - ColumnVector( n, 1.0 ) * x.sum( 0 ).row( 0 ) / static_cast< double >( n );
	return x.transpose( ) * x / static_cast< double >( n - 1 );
}
}

class BackgroundImpl : public Background{
public:
	BackgroundImpl( unsigned  id, const ODERoot &    odeRoot_ ):
		modelImpl( id ),
		worldToPlaneMatrix( 3, 3 ),
		invWorldToPlaneMatrix( 3, 3 ),
		odeRoot( odeRoot_ ),
		block( NULL )
	{ 
		modelImpl.setODERoot( & odeRoot_ );
	}


	virtual ~BackgroundImpl( void ){
		if( block != NULL ){
			free( block );
		}
	} 

	void execPCA( ::Matrix &  vertexData ){
    //共分散行列を求める
		::Matrix  cov( get_covariance( vertexData ) );

    //特異値分解
		::SVD  svd( cov, SVD::economy );
		::Matrix  eigenVector( svd.left_singular_matrix( ) );
    //第3成分を法線の向きとする
		for( int  i( 0 ); i < 3; i ++){
			longAxis[ i ]  = eigenVector( 0, i );
			shortAxis[ i ] = eigenVector( 1, i );
			normal[ i ]    = eigenVector( 2, i );
		}
		TooN::normalize( longAxis );
		TooN::normalize( shortAxis );
		TooN::normalize( normal );

		worldToPlaneMatrix = eigenVector.inverse( );
		invWorldToPlaneMatrix = eigenVector;
	}

	bool removeOutLier( std::vector< int > & valid ){
		boost::scoped_ptr< Analysis > analysis( Analysis::create( ) );

		BOOST_FOREACH( int  i, valid ){
			const GLdouble *  v( modelImpl.getVertexAt( i ) );
			analysis->add( getDistance( v ) );
		}

		bool outlier_is_occur = analysis->smirnovGrubbsTest( valid );
		::Matrix data( valid.size( ), 3 );
		int  idx( 0 );
		BOOST_FOREACH( int j, valid ){
			const GLdouble *  v( modelImpl.getVertexAt( j ) );
			for( int  k( 0 ); k < 3; k ++){
				data( idx, k ) = v[ k ];
			}
			idx++;
		}
		execPCA( data );
		
		return outlier_is_occur;
	}


	void getAxis( void ){
		unsigned vCnt( modelImpl.getVertexCount( ) );
    	{
			::Matrix data( vCnt, 3 );
			for( unsigned  i( 0 ); i < vCnt; i ++){
					const GLdouble *  v( modelImpl.getVertexAt( i ) );
					for( int  j( 0 ); j < 3; j ++){
							data( i, j ) = v[ j ];
					}
			}
			execPCA( data );
		}

		bool  outlier_is_occur( false );
		std::vector< int >  valid;
		for( unsigned  i( 0 ); i < vCnt; i ++){
				valid.push_back( i );
		}
		do{
			outlier_is_occur = removeOutLier( valid );
		} while( outlier_is_occur );
	}

	void getNormal( TooN::Vector< 3 > &  ret )const{
			for( int  i( 0 ); i < 3; i ++){
					ret[ i ] = normal[ i ];
			}
	}

	void drawPlane( const TooN::SE3 &  pose, const TooN::Matrix< 4 > & proj )const{
		GLint mat_mode;
		glGetIntegerv( GL_MATRIX_MODE, & mat_mode );
		//投影行列の設定
		glMatrixMode( GL_PROJECTION );
		glPushMatrix( );
		glLoadIdentity( );
		CVD::glMultMatrix( proj );

		glMatrixMode( GL_MODELVIEW );
		glPushMatrix( );
		CVD::glMultMatrix( pose );
		CVD::glTranslate( center );

		//法線の描画
		glColor3d( 1, 0, 0 );
		glBegin( GL_LINES );{
			glVertex3d( 0, 0, 0 );
			glVertex3d(normal[ 0 ], normal[ 1 ], normal[ 2 ] );
		}glEnd();

		glColor3d( 1, 1, 1 );
		drawPlane( );
		glColor3d( 1, 1, 1 );
		glPopMatrix( );
		glMatrixMode( GL_PROJECTION );
		glPopMatrix( );
		glMatrixMode( mat_mode );
	}

	void drawNormal( void ){
		glColor3d( 1, 0, 0 );
		glBegin( GL_LINES );{
			glVertex3d( 0, 0, 0 );
			glVertex3d(normal[ 0 ], normal[ 1 ], normal[ 2 ] );
		}glEnd();
	}

	void drawPlane( void ) const{
		GLdouble square[ 4 ][ 3 ] = {
			{ -0.1,  0.1, 0 },
			{  0.1,  0.1, 0 },
			{  0.1, -0.1, 0 },
			{ -0.1, -0.1, 0 }
		};
		for( int k = 0; k < 5; k += 1 ){
		glBegin( GL_LINE_LOOP );{
			for( int  i( 0 ); i < 4; i ++){
				::ColumnVector v( 3 );
				::ColumnVector u( 3 );
				for( int  j( 0 ); j < 3; j ++){
					v( j ) = k * square[ i ][ j ];
				}
				u = worldToPlaneMatrix * v;
				glVertex3d( u( 0 ), u( 1 ), u( 2 ) );
			}
		}glEnd( );
		}
	}

	void drawGuide( void )const{
		unsigned vcnt( modelImpl.getVertexCount( ) );
		for( unsigned i( 0 ); i < vcnt; i ++){
			glBegin( GL_LINES );
			const GLdouble *  v( modelImpl.getVertexAt( i ) );
			glVertex3d( v[ 0 ], v[ 1 ], v[ 2 ] );
			::ColumnVector u( 3 );
			::ColumnVector proj_u( 3 );
			for( int  j( 0 ); j < 3; j ++){
				u( j ) = v[ j ];
			}
			proj_u    = this->invWorldToPlaneMatrix * u;
			proj_u( 2 ) = 0.0;
			u         = worldToPlaneMatrix * proj_u;
			glVertex3d( u( 0 ), u( 1 ), u( 2 ) );
			glEnd( );
		}
	}



	void drawBlock( const TooN::SE3 &  pose, const TooN::Matrix< 4 > & proj )const{
		if( block != NULL ){
			block->draw( pose, proj );
		}
	}


	void createODEGeom( void ){
		TooN::Vector< 3 >  c;
		modelImpl.getCenter( c );
		for( int i( 0 ); i <  3; i ++){
			this->center[ i ] = c[ i ];
		}
		odePlaneID = dCreatePlane( odeRoot.spaceID, normal[ 0 ], normal[ 1 ], normal[ 2 ],  center * normal );
	}

	void setODEGravity( void ){
		TooN::Vector< 3 > gravity;
		for( int  i( 0 ); i < 3; i ++){
			gravity[ i ] = normal[ i ];
		}
		gravity = -9.81 * normal;
		dWorldSetGravity( odeRoot.worldID, gravity[ 0 ], gravity[ 1 ], gravity[ 2 ] );
	}


	unsigned ID( void )const{
		return modelImpl.ID( );
	}

	void setODERoot( const ODERoot * ode_root ){
		modelImpl.setODERoot( ode_root );
	}

	void draw( void ) const{
		modelImpl.draw( );
	}

	void draw( const TooN::SE3 &  pose, const TooN::Matrix< 4 > & proj )const {
		drawPlane( pose, proj );
	}
	void draw( const TooN::SE3 &  pose, const TooN::Matrix< 4 > & proj, unsigned  id )const{
		drawPlane( pose, proj );
	}
	void stencil( const TooN::SE3 &  pose, const TooN::Matrix< 4 >  &  proj ) const{
		modelImpl.stencil( pose, proj );
	}

	void update( const ImageType &  imRGB, ATANCamera &  cam, const TooN::SE3 &  pose ){
		modelImpl.update( imRGB, cam, pose );
	}

	void disable( unsigned  tri_id ){
		modelImpl.disable( tri_id );
	}


	void move( const TooN::Vector< 3 > &  v ){
		modelImpl.move( v );
	}


	void moveTo( const TooN::Vector< 3 > &  v ){
		modelImpl.moveTo( v );
	}


	void rotate( const Rotation &  rot ){
		modelImpl.rotate( rot );
	}


	void scaling( GLdouble  scale ){
		modelImpl.scaling( scale );
	}


	void scaling( GLdouble  scale, const TooN::Vector< 3 > &  dir ){
		modelImpl.scaling( scale, dir );
	}


	GLuint getTriangleCount( void ) const{
		return modelImpl.getTriangleCount( );
	}


	GLuint getVertexCount( void ) const{
		return modelImpl.getVertexCount( );
	}

	void cross( const GLint *  buf, const TooN::SE3 &  pose, ATANCamera & cam ){
		modelImpl.cross( buf, pose, cam );
	}

	void getCenter( TooN::Vector< 3 > &  ret )const {
		for( int i( 0 ); i < 3; i ++){
			ret[ i ] = center[ i ];
		}
	}
	
	void addDrawObject( DrawObject * new_obj, const TooN::Vector< 3 > & trans ){	
		modelImpl.addDrawObject( new_obj, trans );
		getAxis( );
#if USE_BLOCK
		createBlocks( );
#endif
	}
	size_t getDrawObjectCount( void ) const{
		return modelImpl.getDrawObjectCount( );
	}

	void stencilDrawObject( const TooN::SE3 & pose, const TooN::Matrix< 4 > & proj, unsigned idx ) const{
		modelImpl.stencilDrawObject( pose, proj, idx );
	}
	
	void disableAt( unsigned d_obj_id, unsigned tri_id ){
		modelImpl.disableAt( d_obj_id, tri_id );
	}

private:
	ModelImpl modelImpl;

	TooN::Vector< 3 > center;
	TooN::Vector< 3 > longAxis;
	TooN::Vector< 3 > shortAxis;
	TooN::Vector< 3 > normal;

	//ワールド座標から平面座標に変換するための行列
	::Matrix worldToPlaneMatrix;
	//その逆行列
	::Matrix invWorldToPlaneMatrix;

	const ODERoot &   odeRoot;
	dGeomID           odePlaneID;

	Block *  block;


	double getDistance( const GLdouble *  v )const{
		::ColumnVector u( 3 );
		::ColumnVector proj_u( 3 );
		for( int j( 0 ); j < 3; j ++){
			u( j ) = v[ j ];
		}
		proj_u = invWorldToPlaneMatrix * u;
		return std::fabs( proj_u( 2 ) );
	}

	void transWorldToPlane( ::ColumnVector &  ret, const GLdouble *  v ) const{
		::ColumnVector u( 3 );
		for( int  i( 0 ); i < 3; i ++){
			u( i ) = v[ i ];
		}
		ret = worldToPlaneMatrix * u;
	}


	void createBlocks( void ){
		block = Block::create( modelImpl, odeRoot, worldToPlaneMatrix );
	}
};

Background *  Background::create( unsigned  id, const ODERoot &  odeRoot ){
	return new BackgroundImpl( id, odeRoot );
}


Background::Background( void ){ }



Background::~Background( void ){ }

}
