
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/StateSet>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/PositionAttitudeTransform>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>

/*
//osgbullet
#include <osgbDynamics/MotionState.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbCollision/Utils.h>
*/

osg::Geode *createCube() {
	// vertex array
	osg::Vec3Array *vertexArray = new osg::Vec3Array();

	// bottom front left
	vertexArray->push_back(osg::Vec3(-1, -1, -1));
	vertexArray->push_back(osg::Vec3(-1, -1, -1));
	vertexArray->push_back(osg::Vec3(-1, -1, -1));
	// bottom front right
	vertexArray->push_back(osg::Vec3(+1, -1, -1));
	vertexArray->push_back(osg::Vec3(+1, -1, -1));
	vertexArray->push_back(osg::Vec3(+1, -1, -1));
	// bottom back right
	vertexArray->push_back(osg::Vec3(+1, +1, -1));
	vertexArray->push_back(osg::Vec3(+1, +1, -1));
	vertexArray->push_back(osg::Vec3(+1, +1, -1));
	// bottom back left
	vertexArray->push_back(osg::Vec3(-1, +1, -1));
	vertexArray->push_back(osg::Vec3(-1, +1, -1));
	vertexArray->push_back(osg::Vec3(-1, +1, -1));

	// top front left
	vertexArray->push_back(osg::Vec3(-1, -1,  1));
	vertexArray->push_back(osg::Vec3(-1, -1,  1));
	vertexArray->push_back(osg::Vec3(-1, -1,  1));
	// top front right
	vertexArray->push_back(osg::Vec3(+1, -1,  1));
	vertexArray->push_back(osg::Vec3(+1, -1,  1));
	vertexArray->push_back(osg::Vec3(+1, -1,  1));
	// top back right
	vertexArray->push_back(osg::Vec3(+1, +1,  1));
	vertexArray->push_back(osg::Vec3(+1, +1,  1));
	vertexArray->push_back(osg::Vec3(+1, +1,  1));
	// top back left
	vertexArray->push_back(osg::Vec3(-1, +1,  1));
	vertexArray->push_back(osg::Vec3(-1, +1,  1));
	vertexArray->push_back(osg::Vec3(-1, +1,  1));


	// face array
	osg::DrawElementsUInt *faceArray = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);

	// bottom
	faceArray->push_back(0); // face 1
	faceArray->push_back(9);
	faceArray->push_back(3);
	faceArray->push_back(9); // face 2
	faceArray->push_back(6);
	faceArray->push_back(3);
	// top
	faceArray->push_back(21);  //face 3
	faceArray->push_back(12);
	faceArray->push_back(18);
	faceArray->push_back(12);  //face 4
	faceArray->push_back(15);
	faceArray->push_back(18);
	// left
	faceArray->push_back(22);  //face 5
	faceArray->push_back(10);
	faceArray->push_back(13);
	faceArray->push_back(10);  //face 6
	faceArray->push_back(1);
	faceArray->push_back(13);
	// right
	faceArray->push_back(16);  //face 7
	faceArray->push_back(4);
	faceArray->push_back(19);
	faceArray->push_back(4);  //face 8
	faceArray->push_back(7);
	faceArray->push_back(19);
	// front
	faceArray->push_back(14);  //face 9
	faceArray->push_back(2);
	faceArray->push_back(17);
	faceArray->push_back(2);   //face 10
	faceArray->push_back(5);
	faceArray->push_back(17);
	// back
	faceArray->push_back(20);  //face 11
	faceArray->push_back(8);
	faceArray->push_back(23);
	faceArray->push_back(8);   //face 12
	faceArray->push_back(11);
	faceArray->push_back(23);

	// normal array
	osg::Vec3Array *normalArray = new osg::Vec3Array();
	normalArray->push_back(osg::Vec3(+1, 0, 0));
	normalArray->push_back(osg::Vec3(-1, 0, 0));
	normalArray->push_back(osg::Vec3(0, +1, 0));
	normalArray->push_back(osg::Vec3(0, -1, 0));
	normalArray->push_back(osg::Vec3(0, 0, +1));
	normalArray->push_back(osg::Vec3(0, 0, -1));

	// normal index
	osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType, 24, 4> *normalIndexArray;
	normalIndexArray = new osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType, 24, 4>();

	// bottom front left					
	normalIndexArray->push_back(5);
	normalIndexArray->push_back(3);
	normalIndexArray->push_back(0);
	// bottom front right
	normalIndexArray->push_back(5);
	normalIndexArray->push_back(2);
	normalIndexArray->push_back(0);
	// bottom back right
	normalIndexArray->push_back(5);
	normalIndexArray->push_back(2);
	normalIndexArray->push_back(1);
	// bottom back left
	normalIndexArray->push_back(5);
	normalIndexArray->push_back(3);
	normalIndexArray->push_back(1);

	// top front left					
	normalIndexArray->push_back(4);
	normalIndexArray->push_back(3);
	normalIndexArray->push_back(0);
	// top front right
	normalIndexArray->push_back(4);
	normalIndexArray->push_back(2);
	normalIndexArray->push_back(0);
	// top back right
	normalIndexArray->push_back(4);
	normalIndexArray->push_back(2);
	normalIndexArray->push_back(1);
	// top back left
	normalIndexArray->push_back(4);
	normalIndexArray->push_back(3);
	normalIndexArray->push_back(1);

	osg::Geometry *geometry = new osg::Geometry();
	geometry->setVertexArray(vertexArray);

	geometry->setNormalArray(normalArray);
	geometry->setNormalIndices(normalIndexArray);
	geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
	//geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
	geometry->addPrimitiveSet(faceArray);

	osg::Vec4Array* color = new osg::Vec4Array();     
	//color->push_back( osg::Vec4( std::rand(), std::rand(), std::rand(), 1. ) );    
	color->push_back( osg::Vec4( 1, 0, 0, 0.5 ) );    
	geometry->setColorArray( color );
	geometry->setColorBinding( osg::Geometry::BIND_OVERALL );

	osg::Geode *cube = new osg::Geode();
	cube->addDrawable(geometry);
	return cube;
}

osg::Node* osgSphereNode(float sphere_size) {
	float scale = 10;

	osg::Sphere * sphere = new osg::Sphere(osg::Vec3d(0,0,0), sphere_size*scale);
	osg::ref_ptr< osg::ShapeDrawable > shape = new osg::ShapeDrawable( sphere );
	//osg::ShapeDrawable * shape = new osg::ShapeDrawable( sphere );
	osg::ref_ptr< osg::Geode > geode = new osg::Geode();
	//osg::Geode  * geode = new osg::Geode();
  geode->addDrawable( shape.get() );
	return geode.release();
}

osg::Node* osgBoxNode(float box_size) {
	float scale = 10;

	osg::Box * box= new osg::Box(osg::Vec3d(0,0,0), box_size*scale);
	osg::ShapeDrawable * shape = new osg::ShapeDrawable( box );
  osg::Geode  * geode = new osg::Geode();
  geode->addDrawable( shape );
	return geode;
}