#include "GLViewOptimization.h"

#include "WorldList.h" //This is where we place all of our WOs
#include "ManagerOpenGLState.h" //We can change OpenGL State attributes with this
#include "Axes.h" //We can set Axes to on/off with this
#include "PhysicsEngineODE.h"

//Different WO used by this module
#include "WO.h"
#include "WOStatic.h"
#include "WOStaticPlane.h"
#include "WOStaticTrimesh.h"
#include "WOTrimesh.h"
#include "WOHumanCyborg.h"
#include "WOHumanCal3DPaladin.h"
#include "WOWayPointSpherical.h"
#include "WOLight.h"
#include "WOSkyBox.h"
#include "WOCar1970sBeater.h"
#include "Camera.h"
#include "CameraStandard.h"
#include "CameraChaseActorSmooth.h"
#include "CameraChaseActorAbsNormal.h"
#include "CameraChaseActorRelNormal.h"
#include "Model.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"
#include "ModelMeshDataShared.h"
#include "ModelMeshSkin.h"
#include "WONVStaticPlane.h"
#include "WONVPhysX.h"
#include "WONVDynSphere.h"
#include "AftrGLRendererBase.h"

//If we want to use way points, we need to include this.
#include "OptimizationWayPoints.h"
#include "MGLFrustum.h"
#include "Mat4.h"

using namespace Aftr;

GLViewOptimization* GLViewOptimization::New( const std::vector< std::string >& args )
{
   GLViewOptimization* glv = new GLViewOptimization( args );
   glv->init( Aftr::GRAVITY, Vector( 0, 0, -1.0f ), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE );
   glv->onCreate();
   return glv;
}


GLViewOptimization::GLViewOptimization( const std::vector< std::string >& args ) : GLView( args )
{
   //Initialize any member variables that need to be used inside of LoadMap() here.
   //Note: At this point, the Managers are not yet initialized. The Engine initialization
   //occurs immediately after this method returns (see GLViewOptimization::New() for
   //reference). Then the engine invoke's GLView::loadMap() for this module.
   //After loadMap() returns, GLView::onCreate is finally invoked.

   //The order of execution of a module startup:
   //GLView::New() is invoked:
   //    calls GLView::init()
   //       calls GLView::loadMap() (as well as initializing the engine's Managers)
   //    calls GLView::onCreate()

   //GLViewOptimization::onCreate() is invoked after this module's LoadMap() is completed.
}


void GLViewOptimization::onCreate()
{
   //GLViewOptimization::onCreate() is invoked after this module's LoadMap() is completed.
   //At this point, all the managers are initialized. That is, the engine is fully initialized.

   if( this->pe != NULL )
   {
      //optionally, change gravity direction and magnitude here
      //The user could load these values from the module's aftr.conf
      this->pe->setGravityNormalizedVector( Vector( 0,0,-1.0f ) );
      this->pe->setGravityScalar( Aftr::GRAVITY );
   }
   this->setActorChaseType( STANDARDEZNAV ); //Default is STANDARDEZNAV mode
   //this->setNumPhysicsStepsPerRender( 0 ); //pause physics engine on start up; will remain paused till set to 1
   
   this->cameraFrustum = WO::New();
   this->cameraFrustum->setModel(MGLFrustum::New(this->cameraFrustum, 1.0f, 60.0f, 60.0f, 1.0f));
   this->cameraFrustum->getModel()->setDisplayMatrix(this->cam->getDisplayMatrix());
   this->cameraFrustum->setPosition(this->cam->getPosition());
   worldLst->push_back(cameraFrustum);

   this->stationaryFrustum = WO::New();
   this->stationaryFrustum->setModel(MGLFrustum::New(this->stationaryFrustum, 1.0f, 60.0f, 60.0f, 1.0f));
   this->stationaryFrustum->setPosition(Vector(-50.0f, 75.0f, 50.0f));
   worldLst->push_back(this->stationaryFrustum);

   for (int i = 0; i < 15; i++) {
	   WO* wo;
	   if (i % 3 == 0) {
		   wo = WO::New("../mm/models/sphere.dae");
		   if (i % 2 == 0) {
			   wo->setPosition(Vector(25.0f + i * 3, 50.0f + i * 3, 50.0f + i*3));
		   }
		   else {
			   wo->setPosition(Vector(25.0f - i * 3, 50.0f + i * 3, 50.0f - i*3));
		   }
		}
	   else if (i % 3 == 1) {
		   wo = WO::New("../mm/models/rocket.dae");
		   Mat4 pose;
		   pose = Mat4::rotateIdentityMat(Vector(1, 0, 0), Aftr::PI / 2);
		   wo->getModel()->setDisplayMatrix(pose);
		   if (i % 2 == 0) {
			   wo->setPosition(Vector(-5.0f + i * 3, 250.0f + i * 3, 50.0f + i * 3));
		   }
		   else {
			   wo->setPosition(Vector(-5.0f - i * 3, 250.0f - i * 3,  50.0f - i * 3));
		   }
	   }
	   else {
		   wo = WO::New("../mm/models/frog/Tree_frog.obj");
		   if (i % 2 == 0) {
			   wo->setPosition(Vector(-5.0f + i * 2, 75.0f + i * 3, 110.0f + i * 3));
		   }
		   else {
			   wo->setPosition(Vector(-5.0f - i * 2, 75.0f - i * 3, 110.0f - i * 3));
		   }
	   }
	   worldLst->push_back(wo);
	   this->objects.insert(std::pair(wo, i));
   }
}


GLViewOptimization::~GLViewOptimization()
{
   //Implicitly calls GLView::~GLView()
}


void GLViewOptimization::updateWorld()
{
   GLView::updateWorld(); //Just call the parent's update world first.
                          //If you want to add additional functionality, do it after
                          //this call.
   this->cameraFrustum->getModel()->setDisplayMatrix(this->cam->getDisplayMatrix());
   this->cameraFrustum->setPosition(this->cam->getPosition());

   for (auto& i : this->objects) {
	   if (objectVisible(i.first))
		   i.first->isVisible = true;
	   else
		   i.first->isVisible = false;
	   Vector position = i.first->getPosition();
	   if (i.second % 3 == 0) {
		   if (position.x < -55.0f)
			   i.first->setPosition(Vector(40.0f, position.y, position.z));
		   else
			   i.first->setPosition(Vector(position.x-this->moveSpeed, position.y, position.z));
	   }
	   else if (i.second % 3 == 1) {
		   if (position.y < -30.0f)
			   i.first->setPosition(Vector(position.x, 250.0f, position.z));
		   else
			   i.first->setPosition(Vector(position.x, position.y - this->moveSpeed, position.z));
	   }
	   else {
		   if (position.z < -10.0f)
			   i.first->setPosition(Vector(position.x, position.y, 120.0f));
		   else
			   i.first->setPosition(Vector(position.x, position.y, position.z - this->moveSpeed));
	   }
   }

}


void GLViewOptimization::onResizeWindow( GLsizei width, GLsizei height )
{
   GLView::onResizeWindow( width, height ); //call parent's resize method.
}


void GLViewOptimization::onMouseDown( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseDown( e );
}


void GLViewOptimization::onMouseUp( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseUp( e );
}


void GLViewOptimization::onMouseMove( const SDL_MouseMotionEvent& e )
{
   GLView::onMouseMove( e );
}


void GLViewOptimization::onKeyDown( const SDL_KeyboardEvent& key )
{
   GLView::onKeyDown( key );
   if( key.keysym.sym == SDLK_0 )
      this->setNumPhysicsStepsPerRender( 1 );

   if( key.keysym.sym == SDLK_1 )
   {
	   this->cam->setPosition(Vector(52.5f, 131.8f, 39.1f));
	   this->cam->setCameraLookDirection(Vector(-0.993f, 0.075f, 0.088f));
   }
   if (key.keysym.sym == SDLK_2)
   {
	   this->cam->setPosition(Vector(29.4f, 25.2f, 39.7f));
	   this->cam->setCameraLookDirection(Vector(0.090f, 0.995f, 0.042f));
   }
   if (key.keysym.sym == SDLK_3)
   {
	   this->cam->setPosition(Vector(-15.7f, 87.7f, 127.9f));
	   this->cam->setCameraLookDirection(Vector(-0.012f, -0.231f, -0.773f));
   }
   if (key.keysym.sym == SDLK_x)
   {
	   this->cam->setCameraLookDirection(Vector(1.0f, 0.0f, 0.0f));
	   this->cam->setCameraNormalDirection(Vector(0.0f, 0.0f, 1.0f));
   }
   if (key.keysym.sym == SDLK_r)
   {
	   this->moveSpeed += 0.5f;
   }
   if (key.keysym.sym == SDLK_f)
   {
	   this->moveSpeed -= 0.5f;
   }
   if (key.keysym.sym == SDLK_c)
   {
	   std::cout << this->cam->getPosition() << " " << this->cam->getLookDirection() << std::endl;
   }
}


void GLViewOptimization::onKeyUp( const SDL_KeyboardEvent& key )
{
   GLView::onKeyUp( key );
}


void Aftr::GLViewOptimization::loadMap()
{
   this->worldLst = new WorldList(); //WorldList is a 'smart' vector that is used to store WO*'s
   this->actorLst = new WorldList();
   this->netLst = new WorldList();

   ManagerOpenGLState::GL_CLIPPING_PLANE = 1000.0;
   ManagerOpenGLState::GL_NEAR_PLANE = 0.1f;
   ManagerOpenGLState::enableFrustumCulling = false;
   Axes::isVisible = true;
   this->glRenderer->isUsingShadowMapping( false ); //set to TRUE to enable shadow mapping, must be using GL 3.2+

   this->cam->setPosition( 15,15,10 );

   std::string shinyRedPlasticCube( ManagerEnvironmentConfiguration::getSMM() + "/models/cube4x4x4redShinyPlastic_pp.wrl" );
   std::string wheeledCar( ManagerEnvironmentConfiguration::getSMM() + "/models/rcx_treads.wrl" );
   std::string grass( ManagerEnvironmentConfiguration::getSMM() + "/models/grassFloor400x400_pp.wrl" );
   std::string human( ManagerEnvironmentConfiguration::getSMM() + "/models/human_chest.wrl" );
   
   //SkyBox Textures readily available
   std::vector< std::string > skyBoxImageNames; //vector to store texture paths

   skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_deepsun+6.jpg" );
   
   float ga = 0.1f; //Global Ambient Light level for this module
   ManagerLight::setGlobalAmbientLight( aftrColor4f( ga, ga, ga, 1.0f ) );
   WOLight* light = WOLight::New();
   light->isDirectionalLight( true );
   light->setPosition( Vector( 0, 0, 100 ) );
   //Set the light's display matrix such that it casts light in a direction parallel to the -z axis (ie, downwards as though it was "high noon")
   //for shadow mapping to work, this->glRenderer->isUsingShadowMapping( true ), must be invoked.
   light->getModel()->setDisplayMatrix( Mat4::rotateIdentityMat( { 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD ) );
   light->setLabel( "Light" );
   worldLst->push_back( light );

   //Create the SkyBox
   WO* wo = WOSkyBox::New( skyBoxImageNames.at( 0 ), this->getCameraPtrPtr() );
   wo->setPosition( Vector( 0,0,0 ) );
   wo->setLabel( "Sky Box" );
   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   worldLst->push_back( wo );

   ////Create the infinite grass plane (the floor)
   wo = WO::New( grass, Vector( 1, 1, 1 ), MESH_SHADING_TYPE::mstFLAT );
   wo->setPosition( Vector( 0, 0, 0 ) );
   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   ModelMeshSkin& grassSkin = wo->getModel()->getModelDataShared()->getModelMeshes().at( 0 )->getSkins().at( 0 );
   grassSkin.getMultiTextureSet().at( 0 )->setTextureRepeats( 5.0f );
   grassSkin.setAmbient( aftrColor4f( 0.4f, 0.4f, 0.4f, 1.0f ) ); //Color of object when it is not in any light
   grassSkin.setDiffuse( aftrColor4f( 1.0f, 1.0f, 1.0f, 1.0f ) ); //Diffuse color components (ie, matte shading color of this object)
   grassSkin.setSpecular( aftrColor4f( 0.4f, 0.4f, 0.4f, 1.0f ) ); //Specular color component (ie, how "shiney" it is)
   grassSkin.setSpecularCoefficient( 10 ); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
   wo->setLabel( "Grass" );
   worldLst->push_back( wo );
   
   createOptimizationWayPoints();
}


void GLViewOptimization::createOptimizationWayPoints()
{
   // Create a waypoint with a radius of 3, a frequency of 5 seconds, activated by GLView's camera, and is visible.
   WayPointParametersBase params(this);
   params.frequency = 5000;
   params.useCamera = true;
   params.visible = true;
   WOWayPointSpherical* wayPt = WOWP1::New( params, 3 );
   wayPt->setPosition( Vector( 50, 0, 3 ) );
   worldLst->push_back( wayPt );
}

bool GLViewOptimization::objectInFrustum(WO* wo, AftrGeometryFrustum* frustum) {
	Mat4 matrix = wo->getDisplayMatrix();
	Vector boundingBox = wo->getModel()->getBoundingBox().getlxlylz() / 2.0f;
	Vector position = wo->getPosition();

	for (int plane = 0; plane < 6; plane++) {
		Vector planeNormal = frustum->getPlaneNormal(plane);
		float planeCoef = frustum->getPlaneCoef(plane);
		bool insideFrustum = false;

		for (int x = -1; x <= 1; x += 2) {
			for (int y = -1; y <= 1; y += 2) {
				for (int z = -1; z <= 1; z += 2) {
					Vector v = matrix * (Vector(x, y, z) * boundingBox) + position;
					if (planeNormal.dotProduct(v) < planeCoef) {
						insideFrustum = true;
					}
				}
			}
		}
		if (!insideFrustum)
			return false;
	}
	return true;
}

bool GLViewOptimization::objectVisible(WO* wo) {
	AftrGeometryFrustum* stationaryFrustumGeometry = new AftrGeometryFrustum(1.0f, 60.0f, 1.0f, 60.0f,
		stationaryFrustum->getLookDirection(),
		stationaryFrustum->getNormalDirection(),
		stationaryFrustum->getPosition());
	AftrGeometryFrustum* cameraFrustumGeometry = new AftrGeometryFrustum(1.0f, 60.0f, 1.0f, 60.0f,
		cameraFrustum->getLookDirection(),
		cameraFrustum->getNormalDirection(),
		cameraFrustum->getPosition());
	return objectInFrustum(wo, stationaryFrustumGeometry) || objectInFrustum(wo, cameraFrustumGeometry);
}
