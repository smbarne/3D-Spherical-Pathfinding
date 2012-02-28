// main.h
// A spherical pathfinding and 3D graphics prototype
// Author: Stephen Barnes
// Created: 2011

#ifndef HEADER_MAIN_INCLUDED
#define HEADER_MAIN_INCLUDED
#if (MSC_VER > 1000)
	#pragma once
#endif

#include <osg/ShapeDrawable>
#include <osg/Billboard>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/StateSetManipulator>
#include <osgText/Text>
#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osg/Vec2>
#include <osg/Vec3>
#include <OpenThreads/Mutex>

#include "search.h"
#include "stdafx.h"

OpenThreads::Mutex mUpdateTextLock;
class SearchThread;

class SolarSystem
{
public:
	class SpaceShip
	{
	public:
		/** 
       * @param ship .
       * @param planetMap - The 2d navmesh data that is used to feed _search.
	   * @param speed - The animation speed of the spaceship along _spAnimationCallback.
       */
		SpaceShip(osg::Group* ship, std::string planetMap, float speed);
		~SpaceShip() {};

		// Get Methods
		osg::Vec3d GetPosition();
		Search* GetSearch() { return _spSearch; };
		osg::MatrixTransform* GetSpaceShipTrans() { return _spSpaceShipTrans; };
		osg::Group* GetShipGroup();

		// Utility Methods
		void ResetAnimationTime() { _dAnimationTime = 0.0; };
		/// Return true if the spaceship is done animating along its movement path
		bool FinishedAnimationPath();

		/// Every time the spaceship starts on a new search route, create an animation path
		/// and set it as the spaceship's primary movement animation.
		void CreateAndSetAnimationPath(std::vector<osg::Vec3f> points);

	private:
		// each spaceship contains a search class it can run in a separate thread
		mutable Search* _spSearch;

		// A callback to update _spSpaceShipTrans.
		osg::AnimationPathCallback* _spAnimationCallback;

		// Data
		osg::AnimationPath* _spPath;
		osg::Group*	_spSpaceShip;
		osg::MatrixTransform* _spSpaceShipTrans;
		std::vector<osg::Vec3f> _vPathPoints;
		float  _fSpeed;
		double _dAnimationTime;
	};

	SolarSystem();
	~SolarSystem() {};

	// Creation Methods
	/// Create the scene's skybox
    osg::Geode* createSpace( const std::string& name);

	/// Create the primary solar system light as well as the scene shadowing light
    osg::Group* createSolarLight();

	/// Create the primary solar system secondary, accent light
	osg::Group* createSecondaryLight();

	// Initialization Methods
	osg::Group* setupPlanet(osg::Node* planet, std::string name);
	osg::Group* setupPlanetHalo(std::string name, osg::Vec4 color);
	osg::Group* setupShip(osg::Node* ship, std::string planetMap, float scaleMod=1.0f);
	osg::Group* setupPathLineStrip(osg::Vec4f color, float width);
	
	// Utility Methods
	void printParameters();

	/// Adjust the solar system's primary ship speed to keep it in line with solar system wide changes
	void rotateSpeedCorrection();

	/// Update the primary animation path visualization to show where and how the spaceship is traveling
	void updatePathLineStrip(std::vector<osg::Vec3f> points);

	// Allows for uniform scaling of the solarsystem at runtime
	void RorbitCorrection();
	void radiusCorrection();

	// Get Methods
	double GetPlanetScale() { return _scalePlanet; };
	double GetShipScale() { return _scaleShip; };
	double GetSpaceScale() { return _scaleSpace; };
	double GetShipOrbit() { return _RorbitShip; };
	double GetShipSpeed() { return _speedShip; };
	SpaceShip* GetSpaceShip() { return _spSpaceShip; };
	osg::Light* GetShadowingLight() { return _spShadowingLight; };
	osg::Program* GetPlanetShader() { return _spPlanetShaderProgram.get(); };

protected:
	// Scaling Parameters
	double _scaleSpace;
    double _scalePlanet;
    double _scaleShip;
	double _speedShip;
    double _speedFactor;
    double _RorbitShip;
    double _RorbitFactor;
    double _radiusFactor;
	
	// Geometry and Light objects
	SpaceShip* _spSpaceShip;
	osg::Geometry* _spLinesGeom;
	osg::Light* _spShadowingLight;

	// Custom Shader Program References
	osg::ref_ptr<osg::Program> _spPlanetShaderProgram;
	osg::ref_ptr<osg::Shader>  _spPlanetFragShader;
	osg::ref_ptr<osg::Shader>  _spPlanetVertShader;
};


/// A mouse override event handler.
/** The pickhandler overrides some of the default OSG Viewer mouse behaviors and allows
 *  for click and pan behavior as well as point selection.
 */
class PickHandler : public osgGA::GUIEventHandler
{
public: 
	 
	/** 
       * @param updateText - a reference to the screen's 2D HUD text info element
	   * @param solarSystem - a reference to the primary solarSystem object
	   * @param searchThread
       */
	PickHandler(osgText::Text* updateText, SolarSystem *solar, SearchThread* searchThread) :
	  _updateText(updateText), _solarSystem(solar), _mouseDrag(false), _searchThread(searchThread) {};
	~PickHandler() {}
    
	/// Handle mouse clicks within the viewer and override specific osgViewer mouse behavior.
	bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);

	/// Select a point to navigate to on the planet
	/// Because the planet is mountainous and might not give the user the UV coordinates 
	/// that it feels like it should - a perfect sphere is hidden behind the planet object.
	/// An intersection test done through the planet object and into the perfect sphere
	/// to get the coordinates the user feels more natural selecting.
	virtual void pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);

	void setLabel(const std::string& name);
    
protected:
	// The PickHandler needs to have easy access to two key areas:
	SearchThread*				 _searchThread;
	SolarSystem*				 _solarSystem;

	// Data
	osg::ref_ptr<osgText::Text>  _updateText;
	bool						 _mouseDrag;
};

/// A spherical pathfinding search thread.
class SearchThread : public OpenThreads::Thread 
{ 
public: 

	/** 
    * @param updateText - a reference to the screen's 2D HUD text info element
    * @param solarSystem - a reference to the primary solarSystem object
	* @param vStartUV - The ship's starting location in u,v coordinate space
    */
	SearchThread(osgText::Text* updateText, SolarSystem *solarSystem, osg::Vec2f vStartUV);
	virtual ~SearchThread() {};
	virtual void run();
	
	// Utility Methods
	/// Put another search request on the back of the queue
	void QueueRun(osg::Vec2f vFinishUV);

	/// Update the HUD text with information from the search
	void SetLabel(const std::string& name);

	/// Clear any queued up travel paths that the spaceship hasn't been able to travel yet.
	void ClearQueuedPaths();

protected: 
	SolarSystem* _solarSystem;
	osg::ref_ptr<osgText::Text>  _updateText;
	std::vector<osg::Vec2f> _vUVPositions;
	std::vector<std::vector<osg::Vec3f> > _vvPaths;

	OpenThreads::Mutex _mUpdatePositionsLock;
	OpenThreads::Mutex _mUpdatePathLock;
}; 


#endif //HEADER_MAIN_INCLUDED