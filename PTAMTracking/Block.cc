#include <octave/config.h>
#include <octave/Matrix.h>
#include "PTAMTracking/Block.h"
#include "ODE.h"
#include "ModelImpl.h"
#include "TriangleMeshGenerator.h"

#include <TooN/se3.h>
#include <vector>
class BlockImpl : public Block{
public:
	BlockImpl( const ModelImpl & m_impl, const ODERoot & ode_root_, const ::Matrix & w ):
		model_impl( m_impl ),
		ode_root( ode_root_ ),
		worldToPlaneMatrix( w )
	{
		odeTriMeshID= 0;
		createBlocks(  );
	}

	~BlockImpl( void ){
		if( odeTriMeshID != 0 ){
			dGeomTriMeshDataDestroy( odeTriMeshID );
		}
	}

	void draw( const TooN::SE3 &  pose, const TooN::Matrix< 4 > &  proj ) const{
		GLint mat_mode;
		glGetIntegerv( GL_MATRIX_MODE, & mat_mode );

		glMatrixMode( GL_PROJECTION );
		glPushMatrix( );
		glLoadIdentity( );
		CVD::glMultMatrix( proj );
		glMatrixMode( GL_MODELVIEW );
		glPushMatrix( );
		CVD::glMultMatrix( pose );

		TooN::Vector< 3 > center;
		model_impl.getCenter( center );
		CVD::glTranslate( center );

		//法線の描画
		glEnableClientState( GL_VERTEX_ARRAY);
		glEnableClientState( GL_INDEX_ARRAY);
		glVertexPointer( 3, GL_FLOAT, sizeof( dReal ), & vertex[ 0 ] );
		glDrawElements( GL_LINES, index.size( ) / 3, GL_UNSIGNED_INT, & index[ 0 ] );
		glDisableClientState( GL_INDEX_ARRAY );
		glDisableClientState( GL_VERTEX_ARRAY );

		glPopMatrix( );
		glMatrixMode( GL_PROJECTION );
		glPopMatrix( );
		glMatrixMode( mat_mode );
	}
private:
	std::vector< GLdouble > vertex;
	std::vector< GLuint > index;

	const ModelImpl & model_impl;
	const ODERoot &  ode_root;
	const ::Matrix   & worldToPlaneMatrix;

	ODENode  ode;
	dTriMeshDataID  odeTriMeshID;

	void transWorldToPlane( ::ColumnVector &  ret, const GLdouble *  v ) const{
		::ColumnVector u( 3 );
		for( int  i( 0 ); i < 3; i ++){
			u( i ) = v[ i ];
		}
		ret = worldToPlaneMatrix * u;
	}


	void createBlocks( void ){
		//頂点データを作成
		for( unsigned i( 0 ); i < model_impl.getVertexCount( ); i ++){
			const GLdouble *  v( model_impl.getVertexAt( i ) );
			for( int j( 0 ); j < 3; j ++){
				vertex.push_back( v[ j ] );
			}
		}
		unsigned  vCnt( vertex.size( ) );
		for( unsigned  i( 0 ); i < vCnt; i ++){//頂点データの後ろに、点を平面上に投影したものを点を加える
		//そのためi番目の投影した点は (i + modelImpl.getVetexCount())番目
			if( i % 3 != 2 ){
				vertex.push_back( vertex[ i ] );
			} else{
				vertex.push_back( 0.0 );
			}
		}



		//三角形データの生成
		//平面に垂直な方向を基準としてドロネー分割
		std::vector< GLdouble > points;
		for( unsigned i( 0 ); i < model_impl.getVertexCount( ); i ++){
			const GLdouble *  v( model_impl.getVertexAt( i ) );
			::ColumnVector u( 3 );
			transWorldToPlane( u, v );
			points.push_back( u( 0 ) );
			points.push_back( u( 1 ) );
		}
		TriangleMesh::generate( points, index );
		createWall( );

		createBlocksODE( );
	}
	
	void createWall( void ){
		unsigned offset( model_impl.getVertexCount( ) );
		unsigned vCnt = index.size( );
		for( unsigned i( 0 ); i < vCnt; i += 3){
			index.push_back( index[ i ] );
			index.push_back( index[ i ] + offset );
			index.push_back( index[ i + 1] );

			index.push_back( index[ i + 1] );
			index.push_back( index[ i + 1] + offset );
			index.push_back( index[ i ] + offset );

			index.push_back( index[ i + 1 ] );
			index.push_back( index[ i + 1 ] + offset );
			index.push_back( index[ i + 2 ] );

			index.push_back( index[ i + 2 ] );
			index.push_back( index[ i + 2 ] + offset );
			index.push_back( index[ i + 1 ] + offset );

			index.push_back( index[ i + 2 ] );
			index.push_back( index[ i + 2 ] + offset );
			index.push_back( index[ i ]);

			index.push_back( index[ i ] );
			index.push_back( index[ i ] + offset );
			index.push_back( index[ i + 2 ] + offset );
		}
	}

	void createBlocksODE( void ){
		ode.bodyID   = dBodyCreate( ode_root.worldID );
		odeTriMeshID = dGeomTriMeshDataCreate( );

		dGeomTriMeshDataBuildSingle( odeTriMeshID, & vertex[ 0 ], 0, vertex.size( ) / 3, 
			&index[ 0 ], index.size( ), 0 );
		ode.geomID = dCreateTriMesh( ode_root.spaceID, odeTriMeshID, NULL, NULL, NULL );
	}
};

Block *  Block::create( const ModelImpl &  model_impl, const ODERoot &  ode_root, const ::Matrix & w ){
	return new BlockImpl( model_impl, ode_root, w );
}

Block::Block( void ){ }

Block::~Block( void ){ }
