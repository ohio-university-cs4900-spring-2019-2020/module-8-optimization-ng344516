#pragma once

#include "GLView.h"
#include "AftrGeometryFrustum.h"


namespace Aftr
{
   class Camera;

/**
   \class GLViewOptimization
   \author Scott Nykl 
   \brief A child of an abstract GLView. This class is the top-most manager of the module.

   Read \see GLView for important constructor and init information.

   \see GLView

    \{
*/

class GLViewOptimization : public GLView
{
public:
   static GLViewOptimization* New( const std::vector< std::string >& outArgs );
   virtual ~GLViewOptimization();
   virtual void updateWorld(); ///< Called once per frame
   virtual void loadMap(); ///< Called once at startup to build this module's scene
   virtual void createOptimizationWayPoints();
   virtual void onResizeWindow( GLsizei width, GLsizei height );
   virtual void onMouseDown( const SDL_MouseButtonEvent& e );
   virtual void onMouseUp( const SDL_MouseButtonEvent& e );
   virtual void onMouseMove( const SDL_MouseMotionEvent& e );
   virtual void onKeyDown( const SDL_KeyboardEvent& key );
   virtual void onKeyUp( const SDL_KeyboardEvent& key );

   bool objectInFrustum(WO* wo, AftrGeometryFrustum* frustum);
   bool objectVisible(WO* wo);

protected:
   GLViewOptimization( const std::vector< std::string >& args );
   virtual void onCreate();
   WO* stationaryFrustum;
   WO* cameraFrustum;
   std::map<WO*, int> objects;
   float moveSpeed = 0.5f;
};

/** \} */

} //namespace Aftr
