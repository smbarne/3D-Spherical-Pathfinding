// main.cpp
// A spherical pathfinding and 3D graphics prototype
// Based originally on OSGPlanets
//		http://www.openscenegraph.org/projects/osg/browser/OpenSceneGraph/trunk/examples/osgplanets/osgplanets.cpp
// Author: Stephen Barnes
// Created: 2011

// Contains:
// - Program initialization and main loop
// - Scene setup
// - Utility methods such as:
//		- Coordinate conversion methods
//		- Simple geometry creation methods
// - Search management handler
// - Mouse and event handler
// - UI/HUD creation and updates
// - PlayerPawn (spaceship) init, creation, and handling

#include "main.h"

#include <osgUtil/Optimizer>
#include <osgDB/ReadFile>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Viewer>

#include <osgGA/TrackballManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/NodeTrackerManipulator>

#include <osg/Material>
#include <osg/Geode>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Projection>
#include <osg/Camera>
#include <osg/io_utils>
#include <osg/Notify>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/Material>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/LightModel>
#include <osg/Billboard>
#include <osg/LineWidth>
#include <osg/TexEnv>
#include <osg/TexEnvCombine>
#include <osg/ClearNode>

#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

#include <osgFX/Cartoon>
#include <osgFX/Scribe>

#include <osg/Program>
#include <osg/Shader>
#include <osg/Uniform>

#include <osgShadow/ShadowedScene>
#include <osgShadow/LightSpacePerspectiveShadowMap>

#include <sstream>
#include <iostream>
#include <OpenThreads/Thread>

#define M_PI 3.14159265
#define UV_OFFSET_DEG -65.0f
#define UV_OFFSET_RAD -1.13446f
#define ELEVATION_RATIO 1.2

static osg::Vec3 defaultPos( 0.0f, 0.0f, 0.0f );
static osg::Vec3 centerScope(0.0f, 0.0f, 0.0f);
static int ReceivesShadowTraversalMask = 0x1;
static int CastsShadowTraversalMask = 0x2;
static int NM_PLANET_MASK = 0x10;
static int NM_HUD_MASK	  = 0x20;


/// Convert XYZ Cartesian coordinates to UV coordinate space
static osg::Vec2f convertXYZtoUV(osg::Vec3f vXYZNormal)
{
	float nx = vXYZNormal.x();
	float ny = vXYZNormal.z();
	float nz = vXYZNormal.y();
	float u=0.0f, v=0.0f;

	u = asinf(nx / (sqrtf( (nx*nx)+(nz*nz) )))/ M_PI / 2 + 0.25f;
	if (nz < 0)
		u = 1-u;
					
	v = asinf(ny)/M_PI + 0.5f;

	// flip u & v
	u = 1.0f-u;
	v = 1.0f-v;

	return osg::Vec2f(u,v);
}

/// This converts UV coordinates with radius to XYZ Cartesian coordinates
/** The longitude angle is given by Theta = 2*pi*u.
*   The latitude angle is given by Phi = pi*(v-0.5)
*
*	The sphere can be calculated by:
*	x = r*sin(theta)cos(phi)
*	y = r*sin(phi) 
*	z = r*cos(theta)cos(phi) */
static osg::Vec3f convertUVtoXYZ(osg::Vec2f vUV, float radius)
{
	float x=0.0f, y=0.0f, z=0.0f;
	float theta = (vUV.x()-1.0f) * 2 * M_PI;  // lat
	float phi = (vUV.y()-1.5f) * M_PI; // long
	
	y = sinf(theta)*cosf(phi);
	z = sinf(phi);
	x = cosf(theta)*cosf(phi);

	return (osg::Vec3f(x,y,z)*radius);
}

/// Convert the provided search type into a string for display
static std::string getSearchTypeString(Search* search)
{
	std::ostringstream os;
	switch (search->GetSearchType())
	{
		case SEARCH_ASTAR:
		default:
			os << "A*" << std::endl;
			break;
		case SEARCH_BEAM:
			os << "Memory Limited A*" << std::endl;
			break;
		case SEARCH_LOCAL:
			os << "Local Beam" << std::endl;
			break;
	};

	return os.str();
}

/// Load the shader data files
static void LoadShaderSource( osg::Shader* shader, const std::string& fileName )
{
    std::string fqFileName = osgDB::findDataFile(fileName);
    if( fqFileName.length() != 0 )
    {
        shader->loadShaderSourceFromFile( fqFileName.c_str() );
    }
    else
    {
        osg::notify(osg::WARN) << "File \"" << fileName << "\" not found." << std::endl;
    }
}

// Create a circular gradient image of color 'centerColor'
static osg::Image* createBillboardImage(const osg::Vec4& centerColor, unsigned int size, float power)
{
    osg::Vec4 backgroundColour = centerColor;
    backgroundColour[3] = 0.0f;
    
    osg::Image* image = new osg::Image;
    image->allocateImage(size,size,1,
                         GL_RGBA,GL_UNSIGNED_BYTE);
	
    float mid = (float(size)-1)*0.5f;
    float div = 2.0f/float(size);
    for(unsigned int r=0;r<size;++r)
    {
        unsigned char* ptr = image->data(0,r,0);
        for(unsigned int c=0;c<size;++c)
        {
            float dx = (float(c) - mid)*div;
            float dy = (float(r) - mid)*div;
            float r = powf(1.0f-sqrtf(dx*dx+dy*dy),power);
            if (r<0.0f) r=0.0f;
            osg::Vec4 color = centerColor*r+backgroundColour*(1.0f-r);
            *ptr++ = (unsigned char)((color[0])*255.0f);
            *ptr++ = (unsigned char)((color[1])*255.0f);
            *ptr++ = (unsigned char)((color[2])*255.0f);
            *ptr++ = (unsigned char)((color[3])*255.0f);
        }
    }
    return image;
}

/// Create a quad at specified position.
static osg::Drawable* createSquare(const osg::Vec3& corner,const osg::Vec3& width,const osg::Vec3& height, osg::Image* image)
{
    // set up the Geometry.
    osg::Geometry* geom = new osg::Geometry;
	
    osg::Vec3Array* coords = new osg::Vec3Array(4);
    (*coords)[0] = corner;
    (*coords)[1] = corner+width;
    (*coords)[2] = corner+width+height;
    (*coords)[3] = corner+height;
	
    geom->setVertexArray(coords);
	
    osg::Vec3Array* norms = new osg::Vec3Array(1);
    (*norms)[0] = width^height;
    (*norms)[0].normalize();
    
    geom->setNormalArray(norms);
    geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
	
    osg::Vec2Array* tcoords = new osg::Vec2Array(4);
    (*tcoords)[0].set(0.0f,0.0f);
    (*tcoords)[1].set(1.0f,0.0f);
    (*tcoords)[2].set(1.0f,1.0f);
    (*tcoords)[3].set(0.0f,1.0f);
    geom->setTexCoordArray(0,tcoords);
    
    osg::Vec4Array* colours = new osg::Vec4Array(1);
    (*colours)[0].set(1.0f,1.0f,1.0f,1.0f);
    geom->setColorArray(colours);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
	
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));
    
    if (image)
    {
        osg::StateSet* stateset = new osg::StateSet;
        osg::Texture2D* texture = new osg::Texture2D;
        texture->setImage(image);
        stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
        stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
        stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        geom->setStateSet(stateset);
    }
    
    return geom;
}

/// Create, initialize, and return a Heads Up Display projection layer 
static osg::Node* createHUD(osgText::Text* updateText)
{
    osg::Camera* hudCamera = new osg::Camera;
    hudCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    hudCamera->setProjectionMatrixAsOrtho2D(0,1280,0,1024);
    hudCamera->setViewMatrix(osg::Matrix::identity());
    hudCamera->setRenderOrder(osg::Camera::POST_RENDER);
    hudCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
    
    std::string timesFont("fonts/times.ttf");
    
    // turn lighting off for the text and disable depth test to ensure its always ontop.
    osg::Vec3 position(25.0f,175.0f,0.0f);
    osg::Vec3 delta(0.0f,60.0f,0.0f);
    
    {
        osg::Geode* geode = new osg::Geode();
        osg::StateSet* stateset = geode->getOrCreateStateSet();
        stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
        stateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
        geode->setName("HelpText");
        hudCamera->addChild(geode);
        
        osgText::Text* text = new  osgText::Text;
        geode->addDrawable( text );

		std::ostringstream os;
		os << "Press: 'h' for help"			<< std::endl;
		os << "       's' to switch search" << std::endl;
		os << "           algorithms"		<< std::endl;
		os << "       'c' to clear queued"  << std::endl;
		os << "           paths";
        
        text->setFont(timesFont);
        text->setText(os.str());
		text->setColor(osg::Vec4(0.8f,0.8f,0.8f,1.0f));
        text->setPosition(osg::Vec3(900.0f, 160.0f, 0.0f));
		text->setCharacterSize(35.0f);
        
        position += delta;
    }    
	
    { 
		// this displays what has been selected
        osg::Geode* geode = new osg::Geode();
        osg::StateSet* stateset = geode->getOrCreateStateSet();
        stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
        stateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
        geode->setName("InfoText");
        geode->addDrawable( updateText );
        hudCamera->addChild(geode);
        
        updateText->setCharacterSize(35.0f);
        updateText->setFont(timesFont);
        updateText->setColor(osg::Vec4(0.8f,0.8f,0.8f,1.0f));
        updateText->setText("Search Type: A*");
        updateText->setPosition(position);
        updateText->setDataVariance(osg::Object::DYNAMIC);
        
        position += delta;
    }    
    return hudCamera;
}

/// Create and initialize a new SolarSystem object
SolarSystem::SolarSystem()
{
    _scaleSpace    = 15.0;
    _scalePlanet    = 2.0;
    _scaleShip      = 0.020;

	_speedShip = 10.0;
    _RorbitShip    = 1.0;	
    		
    _speedFactor = 1.0;
	_RorbitFactor   = 1.0;
	_radiusFactor   = 1.0;

	_spSpaceShip = NULL;
	_spLinesGeom = NULL;
	_spShadowingLight = NULL;

	// Initialize shaders
	_spPlanetShaderProgram = new osg::Program();
    _spPlanetShaderProgram->setName( "Planet Shader" );
    _spPlanetVertShader = new osg::Shader( osg::Shader::VERTEX );
    _spPlanetFragShader = new osg::Shader( osg::Shader::FRAGMENT );
    _spPlanetShaderProgram->addShader( _spPlanetVertShader.get() );
    _spPlanetShaderProgram->addShader( _spPlanetFragShader.get() );
	LoadShaderSource(_spPlanetVertShader.get(), "../data/shaders/planet_vertex.glsl");
	LoadShaderSource(_spPlanetFragShader.get(), "../data/shaders/planet_frag.glsl");
}

/// Adjust the solar system's primary ship speed to keep it in line with solar system wide changes
void SolarSystem::rotateSpeedCorrection()
{
    _speedShip *= _speedFactor;
}

/// Adjust the solar system's primary ship orbit to keep it in line with solar system wide changes
void SolarSystem::RorbitCorrection()
{
    _RorbitShip *= _RorbitFactor;
}
 
/// Scale all appropriate solar system objects to keep them in line with solar system wide changes
void SolarSystem::radiusCorrection()
{
    _scaleSpace    *= _radiusFactor;
    _scalePlanet    *= _radiusFactor;
    _scaleShip      *= _radiusFactor;
}

/// Create the scene's skybox
osg::Geode* SolarSystem::createSpace( const std::string& name )
{
	osg::Box *spaceBox = new osg::Box( osg::Vec3(0.0, 0.0, 0.0 ), _scaleSpace );
	osg::ShapeDrawable *sSpaceBox = new osg::ShapeDrawable( spaceBox );
	sSpaceBox->setColor( osg::Vec4(1.0f,1.0f,1.0f,1.0f) );
	
	osg::ref_ptr<osg::Material> material = new osg::Material(); 
	material->setEmission(osg::Material::FRONT_AND_BACK,osg::Vec4f(1.0f,1.0f,1.0f,1.0f));
	osg::ref_ptr<osg::PolygonMode> polymode = new osg::PolygonMode();
	polymode->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::FILL);

	// Override state to force lighting on (so it can receive shadowing from the planet and spaceship) as
	// well as culling off.
	osg::ref_ptr<osg::StateSet> stateset = sSpaceBox->getOrCreateStateSet();
	stateset->setAttributeAndModes(material.get(),osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
	stateset->setMode(GL_LIGHTING,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
	stateset->setMode(GL_CULL_FACE, osg::StateAttribute::OFF); 
	
    osg::Geode* geodeSpace = new osg::Geode();
    geodeSpace->setName( name );	
    geodeSpace->addDrawable( sSpaceBox );
	
    return( geodeSpace );
}

/// Create the primary solar system light as well as the scene shadowing light
osg::Group* SolarSystem::createSolarLight()
{
	osg::LightSource* solarLightSource = new osg::LightSource;	
    osg::Light* solarLight = solarLightSource->getLight();
	_spShadowingLight = solarLight;
    solarLight->setPosition( osg::Vec4( 5.0f, 5.0f, 5.0f, 1.0f ) );
    solarLight->setAmbient( osg::Vec4( 0.15f, 0.15f, 0.15f, 1.0f ) );
	solarLight->setDirection(osg::Vec3( -1.0f, -1.0f, -1.0f) );
	solarLight->setDiffuse( osg::Vec4( 0.75f, 0.75f, 1.0f, 1.0f ) );

	solarLight->setLightNum(0);

    solarLightSource->setLight( solarLight );
    solarLightSource->setLocalStateSetModes( osg::StateAttribute::ON );
    solarLightSource->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	solarLightSource->getOrCreateStateSet()->setMode(GL_LIGHT0, osg::StateAttribute::ON);

    return solarLightSource;
}

/// Create the primary solar system secondary, accent light
osg::Group* SolarSystem::createSecondaryLight()
{
	osg::LightSource* secondaryLightSource = new osg::LightSource;
	osg::Light* secondaryLight = secondaryLightSource->getLight();
    secondaryLight->setPosition( osg::Vec4( -5.0f, -5.0f, -5.0f, 1.0f ) );
	secondaryLight->setDirection(osg::Vec3( 1.0f, 1.0f, 1.0f) );
	secondaryLight->setAmbient( osg::Vec4( 0.05f, 0.05f, 0.05f, 1.0f ) );
	secondaryLight->setDiffuse( osg::Vec4( 1.0f, 0.25f, 0.25f, 1.0f ) );
	secondaryLight->setLightNum(1);

    secondaryLightSource->setLight( secondaryLight );
    secondaryLightSource->setLocalStateSetModes( osg::StateAttribute::ON );
	secondaryLightSource->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	secondaryLightSource->getOrCreateStateSet()->setMode(GL_LIGHT1, osg::StateAttribute::ON);
	
	return secondaryLightSource;
}

/// Initialize the primary planet
osg::Group* SolarSystem::setupPlanet(osg::Node* planet, std::string name)
{
	if (planet)
	{
		planet->setName(name);
		planet->setNodeMask(NM_PLANET_MASK|(CastsShadowTraversalMask));

		// Scale!
		osg::MatrixTransform *scaleTrans = new osg::MatrixTransform;
        scaleTrans->setDataVariance( osg::Object::STATIC );
        scaleTrans->setMatrix( osg::Matrix::scale(_scalePlanet, _scalePlanet, _scalePlanet) );
        scaleTrans->addChild( planet );
				
		return scaleTrans->asGroup();
	}
	return NULL;
}

/// Create the primary planet's halo effect
osg::Group* SolarSystem::setupPlanetHalo(std::string name, osg::Vec4 color)
{
	osg::Billboard* planetBillboard = new osg::Billboard();
	planetBillboard->setMode(osg::Billboard::POINT_ROT_EYE);
	planetBillboard->addDrawable( createSquare(osg::Vec3(-0.75f,0.0f,-0.75f), 
								osg::Vec3(1.5f,0.0f,0.0f),
								osg::Vec3(0.0f,0.0f,1.5f),
								createBillboardImage(color, 64, 1.0)),
								osg::Vec3(0.0f,0.0f,0.0f));
	planetBillboard->setName(name + "Halo");

	// Scale!
	osg::MatrixTransform *scaleTrans = new osg::MatrixTransform;
    scaleTrans->setDataVariance( osg::Object::STATIC );
    scaleTrans->setMatrix( osg::Matrix::scale(_scalePlanet, _scalePlanet, _scalePlanet) );
	scaleTrans->addChild( planetBillboard );
				
	return scaleTrans->asGroup();
}

/// Initialize the primary player pawn
osg::Group* SolarSystem::setupShip(osg::Node* ship, std::string planetMap, float scaleMod)
{
	if (ship)
	{
		ship->setName("Ship");
		// Rotate!		
		osg::Matrix mat = osg::Matrix::rotate(osg::DegreesToRadians(90.0f), osg::Vec3f(1.0f,0.0f,0.0f)) *
						  osg::Matrix::rotate(osg::DegreesToRadians(180.0f), osg::Vec3f(0.0f,0.0f,1.0f)) *
						  osg::Matrix::rotate(osg::DegreesToRadians(180.0f), osg::Vec3f(0.0f,1.0f,0.0f)) ;
		
		// Scale!
		osg::MatrixTransform *scaleTrans = new osg::MatrixTransform;
        scaleTrans->setDataVariance( osg::Object::STATIC );
        scaleTrans->setMatrix( osg::Matrix::scale(_scaleShip*scaleMod, _scaleShip*scaleMod, _scaleShip*scaleMod) * mat);
        scaleTrans->addChild( ship );		
		
		osg::Group* shipGroup = new osg::Group();
		shipGroup->addChild(scaleTrans);
		
		osg::ref_ptr<osgFX::Cartoon> c = new osgFX::Cartoon();
		c->setOutlineColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
		c->setOutlineLineWidth(1);
		c->addChild(shipGroup);
		c->setLightNumber(0);
		
		if (_spSpaceShip == NULL)
		{
			// Non-cartoon-shaded option
			//_spSpaceShip = new SpaceShip(shipGroup, planetMap, _speedShip);
			//shipGroup = _spSpaceShip->GetShipGroup();
			
			_spSpaceShip = new SpaceShip(c->asGroup(), planetMap, _speedShip);
			return _spSpaceShip->GetShipGroup();
		}
		
		return shipGroup;
	}
	return NULL;
}

/// Creates, initializes, and returns a 3D line object ready to accept path data
osg::Group* SolarSystem::setupPathLineStrip(osg::Vec4 color, float width)
{
    osg::Geode* geode = new osg::Geode();
	_spLinesGeom = new osg::Geometry();

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(2);
    osg::Vec3Array::iterator vitr = vertices->begin();
	vertices->push_back(osg::Vec3f(-1.0f, 0.0f, 0.0f));
	vertices->push_back(osg::Vec3f(1.0f, 0.0f, 0.0f));
    
	_spLinesGeom->setVertexArray(vertices.get());
    
	// set the colors as before, plus using the above
	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(color);
	_spLinesGeom->setColorArray(colors);
	_spLinesGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
	
	osg::Vec3Array* normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
	_spLinesGeom->setNormalArray(normals);
	_spLinesGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

	_spLinesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,2));
	
	// add the points geometry to the geode.
	geode->addDrawable(_spLinesGeom);
	
	osg::LineWidth* lineWidth = new osg::LineWidth();
	lineWidth->setWidth(width); 
	geode->getOrCreateStateSet()->setAttribute(lineWidth, osg::StateAttribute::ON);
	
	osg::Group *linegroup = new osg::Group();
	linegroup->addChild(geode);
	return linegroup;
}

/// Update the primary animation path visualization to show where and how the spaceship is traveling
void SolarSystem::updatePathLineStrip(std::vector<osg::Vec3f> points)
{
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	for (std::vector<osg::Vec3f>::size_type i=0; i < points.size(); i++)
		vertices->push_back(points.at(i));
	
	// pass the created vertex array to the points geometry object.
	_spLinesGeom->setVertexArray(vertices.get());
	_spLinesGeom->setPrimitiveSet(0, new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,vertices->size()));
}

/// Create and initialize the player's primary pawn - the solar system's spaceship object.
SolarSystem::SpaceShip::SpaceShip(osg::Group* ship, std::string planetMap, float speed)
{
	_spSpaceShip = ship;
	_fSpeed = speed;
	_spSpaceShipTrans = new osg::MatrixTransform;
	_spSpaceShipTrans->addChild(_spSpaceShip);
	_vPathPoints.push_back(convertUVtoXYZ(osg::Vec2f(0.53f, 0.37f), ELEVATION_RATIO));
	_vPathPoints.push_back(convertUVtoXYZ(osg::Vec2f(0.53f, 0.36f), ELEVATION_RATIO));

	_spSearch = new Search(planetMap);

	_spSpaceShipTrans->setThreadSafeReferenceCounting(true);
	_spSpaceShipTrans->setThreadSafeRefUnref(true);
	
	_spAnimationCallback = new osg::AnimationPathCallback();
	_spSpaceShipTrans->setUpdateCallback(_spAnimationCallback);
	_spAnimationCallback->setThreadSafeReferenceCounting(true);
	_spAnimationCallback->setThreadSafeRefUnref(true);
	
	_spAnimationCallback->setTimeMultiplier(speed);
	_spAnimationCallback->setTimeOffset(0.0f);

	CreateAndSetAnimationPath(_vPathPoints);
}

/// Every time the spaceship starts on a new search route, create an animation path
/// and set it as the spaceship's primary movement animation.
void SolarSystem::SpaceShip::CreateAndSetAnimationPath(std::vector<osg::Vec3f> points)
{
    // set up the animation path 
    osg::AnimationPath* animationPath = new osg::AnimationPath;
	animationPath->setLoopMode(osg::AnimationPath::NO_LOOPING);
	int numSamples = (int)points.size();
	if (numSamples <= 0)
		numSamples = 1;
        
    double time=0.0f;
    double time_delta = (double)points.size()/(double)numSamples;
    for (std::vector<osg::Vec2f>::size_type i = 0; i<points.size(); i++)
    {
		osg::Quat rot;
		osg::Matrix MatrixTowardsDirection = osg::Matrix::identity();

		if (i > 0)
		{
			MatrixTowardsDirection.makeLookAt(points.at(i), points.at(i-1), points.at(i));		
			rot.set(osg::Matrix::inverse(MatrixTowardsDirection));
			animationPath->insert(time,osg::AnimationPath::ControlPoint(points.at(i),rot));
		}
		else
			animationPath->insert(time,osg::AnimationPath::ControlPoint(points.at(i)));
		
        time += time_delta;
    }

	_dAnimationTime = time;

	// Use this to create a custom callback to start/stop our friendly particle
	_spAnimationCallback->reset();
	_spAnimationCallback->setAnimationPath(animationPath);
}

/// Return true if the spaceship is done animating along its movement path
bool SolarSystem::SpaceShip::FinishedAnimationPath()
{
	double time = _spAnimationCallback->getAnimationTime();
	return (time > _dAnimationTime);
}

/// Get the spaceship's position in world coordinates
osg::Vec3d SolarSystem::SpaceShip::GetPosition()
{
	osg::Matrix mat = _spSpaceShipTrans->getMatrix();
	mat.rotate(UV_OFFSET_RAD, osg::Vec3f(0.0f, 0.0f, 1.0f));
	return osg::Matrix((mat * osg::Matrix::scale(0.8666f, 0.8666f, 0.8666f))).getTrans();
}

/// Get the primary spaceship model group
osg::Group* SolarSystem::SpaceShip::GetShipGroup()
{ 
	return _spSpaceShipTrans->asGroup();
}

/// Create a new search thread and start it.
SearchThread::SearchThread(osgText::Text* updateText, SolarSystem *solarSystem, osg::Vec2f vStartUV) :
	  _updateText(updateText), _solarSystem(solarSystem)
{
	_vUVPositions.push_back(vStartUV);
	start();
}

/// Clear any queued up travel paths that the spaceship hasn't been able to travel yet.
void SearchThread::ClearQueuedPaths()
{
	OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mUpdatePositionsLock);
	OpenThreads::ScopedLock<OpenThreads::Mutex> lock2(_mUpdatePathLock);

	if ( _vUVPositions.size() > (unsigned)1)
	{
		osg::Vec2f temp = _vUVPositions.at(1);
		_vUVPositions.clear();
		_vUVPositions.push_back(temp);
	}
	
	_vvPaths.clear();
}

/// Put another search request on the back of the queue
void SearchThread::QueueRun(osg::Vec2f vFinishUV)
{
	OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mUpdatePositionsLock);
	_vUVPositions.push_back(vFinishUV);
}

/// Run a search request from the queue
void SearchThread::run()
{
	SolarSystem::SpaceShip* ship = _solarSystem->GetSpaceShip();
	if (ship)
	{
		Search* search = ship->GetSearch();
		if (search)
		{
			while(1)
			{
				bool bSuccessfulRun = false;
				bool bWaterStartPoint = false;
				if ((_vUVPositions.size() > (unsigned)1))
				{
					osg::Vec2f vFinishUV = _vUVPositions.at(1);
					osg::Vec2f vStartUV = _vUVPositions.at(0);
					std::vector<osg::Vec2f> vPathList = ship->GetSearch()->Run(vStartUV.x(), vStartUV.y(), vFinishUV.x(), vFinishUV.y());
			
					std::string gdlist="";
					std::ostringstream os;
					os << "Search Type: " << getSearchTypeString(search);
							
					if (vPathList.size() > 1)
					{
						// Convert the UV points to XYZ points & add an offset for height
						std::vector<osg::Vec3f> vXYZPoints;
						for (std::vector<Vector2D>::size_type i=0; i < vPathList.size(); i++)
							vXYZPoints.push_back(convertUVtoXYZ(vPathList.at(i), ELEVATION_RATIO));

						_vvPaths.push_back(vXYZPoints);

						if (ship->GetSearch()->GetSuccessfulRun())
							os << "Complete!";
						else
							os << "I couldn't find the way! This is how far I got..." << std::endl;
				
						os << "Expanded Nodes: " << search->GetExpandedNodeCount() << std::endl;
						bSuccessfulRun = true;
					}
					else
					{
						bWaterStartPoint = true;
						os << "Sorry, this spaceship is defective and cannot fly over water." << std::endl;
					}
			
					gdlist += os.str();
					SetLabel(gdlist);
				}
				else if ( (ship->FinishedAnimationPath()) && (_vvPaths.size() > (unsigned)0))
				{
					OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mUpdatePathLock);
					if (_vvPaths.size() > (unsigned)0)
					{
						std::vector<osg::Vec3f> vXYZPoints = _vvPaths.at(0);
						ship->CreateAndSetAnimationPath(vXYZPoints);
						_solarSystem->updatePathLineStrip(vXYZPoints);
						_vvPaths.erase(_vvPaths.begin());
					}
				}
				else
					OpenThreads::Thread::microSleep(500);

				if (bSuccessfulRun && _vUVPositions.size() > (unsigned)1 && !bWaterStartPoint)
				{
					OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mUpdatePositionsLock);
					_vUVPositions.erase(_vUVPositions.begin());
				}
				else if ( _vUVPositions.size() > (unsigned)1 && bWaterStartPoint )
				{
					OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mUpdatePositionsLock);					
					_vUVPositions.erase(_vUVPositions.end()-1);
				}
				
			}
		}
	}
}

/// Update the HUD text with information from the search
void SearchThread::SetLabel(const std::string& name)
{
	OpenThreads::ScopedLock<OpenThreads::Mutex> lock(mUpdateTextLock);
    if (_updateText.get())
		_updateText->setText(name);
}

/// Update the HUD text with information from the PickHandler
void PickHandler::setLabel(const std::string& name)
{
	OpenThreads::ScopedLock<OpenThreads::Mutex> lock(mUpdateTextLock);
    if (_updateText.get())
		_updateText->setText(name);
}

/// Handle mouse clicks within the viewer and override specific osgViewer mouse behavior.
bool PickHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    switch(ea.getEventType())
    {
		case osgGA::GUIEventAdapter::DRAG:
		{
			_mouseDrag = true;
			return false;
		}
		case osgGA::GUIEventAdapter::PUSH:
		{
			_mouseDrag = false;
			return false;
		}
		case osgGA::GUIEventAdapter::RELEASE:
		{
			if (!_mouseDrag && (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON))
            {
				osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
				if (view)
					pick(view,ea);
			}
			else
				_mouseDrag = false;
            return false;
        }    
        case(osgGA::GUIEventAdapter::KEYDOWN):
        {
			// Clear queued up paths
            if (ea.getKey()=='c' || ea.getKey() == 'C')
            {        
				_searchThread->ClearQueuedPaths();
				SolarSystem::SpaceShip* ship = _solarSystem->GetSpaceShip();
				if (ship)
					ship->ResetAnimationTime();
            }
			// Switch search type
			else if (ea.getKey() == 's' || ea.getKey() == 'S')
			{
				SolarSystem::SpaceShip* ship = _solarSystem->GetSpaceShip();
				if (ship)
				{
					Search* search = ship->GetSearch();
					if (search)
					{
						search->ToggleSearchType();
						setLabel("Search Type: " + getSearchTypeString(search));
					}
				}			
			}
			// Help text
			else if (ea.getKey() == 'h' || ea.getKey() == 'H')
			{
			    std::string gdlist="";
				std::ostringstream os;
				
				os << "Hello!" << std::endl;
				os << "This is the portfolio project of Stephen Barnes." << std::endl;
				os << "This simple program mixes several search algorithms with" << std::endl;
				os << "a spherical environment.  Left click and drag to rotate the" << std::endl;
				os << "planet, and right click and drag to zoom in and out.  Simply" << std::endl;
				os << "left click anywhere on the planet to plot a route to that position!" << std::endl;
				
				gdlist += os.str();
				setLabel(gdlist);
			}
            return false;
        }    
        default:
            return false;
    }
}

/// Select a point to navigate to on the planet
/// Because the planet is mountainous and might not give the user the UV coordinates 
/// that it feels like it should - a perfect sphere is hidden behind the planet object.
/// An intersection test done through the planet object and into the perfect sphere
/// to get the coordinates the user feels more natural selecting.
void PickHandler::pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
    osgUtil::LineSegmentIntersector::Intersections intersections;
	
    std::string gdlist="";
    float x = ea.getX();
    float y = ea.getY();
    if (view->computeIntersections(x,y,intersections))
    {
		for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
            hitr != intersections.end();
            ++hitr)
        {
			std::ostringstream os;
			// Check to see if the object is our Perfect Sphere object under the Planet
			if (!hitr->nodePath.empty() && !(hitr->nodePath.back()->getName().empty()))
			{
				if (hitr->nodePath.back()->getName().compare("PerfectSphere") == 0)
				{
					SolarSystem::SpaceShip* spaceShip = _solarSystem->GetSpaceShip();
					if (spaceShip)
					{
						// Create the search request
						osg::Vec3 vShipPos = spaceShip->GetPosition();
						osg::Vec3f vNormal = hitr->getLocalIntersectNormal();
						osg::Vec2f vShipUV = convertXYZtoUV(vShipPos);
						osg::Vec2f vFinishUV = convertXYZtoUV(vNormal);

						if (_searchThread != NULL)
							_searchThread->QueueRun(vFinishUV);
		
						// Update the HUD
						Search* search = spaceShip->GetSearch();
						if (search)
							os << "Search Type: " << getSearchTypeString(search);
						
						os << "Computing route...";
						gdlist += os.str();
						}

					setLabel(gdlist);
					return;
				}
					
			}
        }
    }
}

int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);
	
	// osgViewer
	osgViewer::Viewer viewer;
	
    // Create the root scenegraph node and divide the key parts of the scene under it
    osg::Group* root = new osg::Group();
	
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 50;
    traits->y = 50;
    traits->width = 800;
    traits->height = 600;
    traits->windowDecoration = true;
    traits->doubleBuffer = true;
    traits->sharedContext = 0;

    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	
	// Setup the main camera
    osg::Camera* camera = new osg::Camera();
    camera->setRenderOrder(osg::Camera::POST_RENDER);
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);
    camera->setReferenceFrame(osg::Transform::RELATIVE_RF);
    camera->setViewMatrix(osg::Matrix::translate(0, 0, 0));
	camera->setViewport(new osg::Viewport(0,0, 800, 600));
	camera->setGraphicsContext(gc.get());
	camera->setViewport(new osg::Viewport(0,0, traits->width, traits->height));
	GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
    camera->setDrawBuffer(buffer);
    camera->setReadBuffer(buffer);

	viewer.setThreadingModel(osgViewer::ViewerBase::AutomaticSelection);

	// Setup Scene Shadowing
	// Note about OSG Shadowing Methods:
	// OSG Shadowing methods are still in their infancy.  LISPSM shadowing is my preference
	// for performance and appearance
	osg::ref_ptr<osgShadow::ShadowedScene> shadowedScene = new osgShadow::ShadowedScene;
    shadowedScene->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
    shadowedScene->setCastsShadowTraversalMask(CastsShadowTraversalMask);

	// Light Space Shadow Mapping Method (LISPSM)
	float minLightMargin = 20.f;
	float maxFarPlane = 0;
	unsigned int texSize = 2048;
	unsigned int baseTexUnit = 2;
	unsigned int shadowTexUnit = 3;
	 
	osg::ref_ptr<osgShadow::MinimalShadowMap> sm = new osgShadow::LightSpacePerspectiveShadowMapVB;
	shadowedScene->setShadowTechnique( sm.get() );	
	sm->setMinLightMargin( minLightMargin );
	sm->setMaxFarPlane( maxFarPlane );
	sm->setTextureSize( osg::Vec2s( texSize, texSize ) );
	sm->setShadowTextureCoordIndex( shadowTexUnit );
	sm->setShadowTextureUnit( shadowTexUnit );
	sm->setBaseTextureCoordIndex( baseTexUnit );
	sm->setBaseTextureUnit( baseTexUnit );
	
	// Add the shadowed scene node directly under the root. 
	// Add anything that needs to be shadowed under the shadowedScene node
	root->addChild(shadowedScene.get());

	// Initialize the solar system
	SolarSystem *solarSystem = new SolarSystem;
	solarSystem->rotateSpeedCorrection();
	solarSystem->RorbitCorrection();
	solarSystem->radiusCorrection();

	std::cout << "Loading environment..." << std::endl;

	osg::Group* solarLight = solarSystem->createSolarLight();
	osg::Group* secondaryLight = solarSystem->createSecondaryLight();
	shadowedScene->addChild(solarLight);
	shadowedScene->addChild(secondaryLight);
	sm->setLight(solarSystem->GetShadowingLight());
	
	// Create and attach the HUD to the scenegraph
    osg::ref_ptr<osgText::Text> updateText = new osgText::Text;
	updateText->setThreadSafeReferenceCounting(true);
	updateText->setThreadSafeRefUnref(true);
    root->addChild(createHUD(updateText.get()));
		
	osg::ClearNode* clearNode = new osg::ClearNode;
	clearNode->setClearColor(osg::Vec4(0.0f,0.0f,0.0f,1.0f));
	root->addChild(clearNode);
	
	// The small skybox that accepts shadowing to show a cool shadow below the planet
	// didn't look as good as intended.  Therefore it isn't added to the scenegraph
	// at this moment.
	osg::Node* space = solarSystem->createSpace( "Space" );
	space->setNodeMask(~(CastsShadowTraversalMask));
	//shadowedScene->addChild( space );
	
	// Setup the main Planet
	std::string sPlanetName = "MainPlanet";
	osg::ref_ptr<osg::Group> spMainPlanet = solarSystem->setupPlanet(osgDB::readNodeFile("../data/planet3_highres.osg"), sPlanetName);
	if (spMainPlanet.get())
	{
		shadowedScene->addChild(spMainPlanet.get());
		osg::StateSet* ss = spMainPlanet.get()->getOrCreateStateSet();
		ss->setAttributeAndModes(solarSystem->GetPlanetShader(), osg::StateAttribute::ON);
		ss->addUniform( new osg::Uniform("baseTexture", 0) );
		ss->addUniform( new osg::Uniform("shadowTexture", 3) );
	}

	// We don't want the halo to be shadowed, so add it directly to the root group
	// OSG shadowing doesn't respect the receive shadow NODEMASK, so even if you set it off
	// objects will be shadowed if they are in the shadowed scene group
	osg::ref_ptr<osg::Group> spMainPlanetHalo = solarSystem->setupPlanetHalo(sPlanetName, osg::Vec4(0.1f, 0.3f, 0.8f, 1.0f));
	if (spMainPlanetHalo.get())
		root->addChild( spMainPlanetHalo.get() );
		
	// Setup the spaceship
	osg::ref_ptr<osg::Group> spSpaceShip = solarSystem->setupShip(osgDB::readNodeFile("../data/spaceship.osg"), "../data/textfiles/planet3b.txt");
	if (spSpaceShip.get())
	{
		osg::MatrixTransform *spaceShipAdjust = new osg::MatrixTransform;
		spaceShipAdjust->setMatrix(osg::Matrix::rotate(UV_OFFSET_RAD, osg::Vec3f(0.0f, 0.0f, 1.0f)));
		spaceShipAdjust->addChild(spSpaceShip.get());
		shadowedScene->addChild(spaceShipAdjust);
	}
	
	// The UV coordinates for the main planet texture are offiset about 60 degrees, so tweak our line path showing the ship's animation.
	osg::ref_ptr<osg::Group> spPathLineStrip = solarSystem->setupPathLineStrip(osg::Vec4f(1.0f, 0.2f, 0.2f, 1.0f), 4.0f);
	osg::MatrixTransform *lineAdjustTrans= new osg::MatrixTransform;
	lineAdjustTrans->setMatrix(osg::Matrix::rotate(osg::Quat(UV_OFFSET_RAD, osg::Vec3f(0.0f, 0.0f, 1.0f))));
	lineAdjustTrans->addChild(spPathLineStrip.get());

	// The navigation path line looks so-so when shadowed, so add it above the shadowedScene node
	root->addChild(lineAdjustTrans); 

	// Setup the perfect sphere that is going to help us get correct UV coordinates for our searches
	osg::Sphere* perfectSphere = new osg::Sphere( osg::Vec3( 0.0, 0.0, 0.0 ), solarSystem->GetPlanetScale()*0.49f );
	osg::ShapeDrawable *sPerfectSphere = new osg::ShapeDrawable( perfectSphere );
	osg::Geode* geodeSphere = new osg::Geode();
    geodeSphere->setName( "PerfectSphere" );
    geodeSphere->addDrawable( sPerfectSphere );

	// The UV coordinates for the main planet texture are offset about 60 degrees, so tweak our perfect sphere for that
	osg::MatrixTransform *perfectSphereAdjust = new osg::MatrixTransform;
	perfectSphereAdjust->setMatrix(osg::Matrix::rotate(UV_OFFSET_RAD, osg::Vec3f(0.0f, 0.0f, 1.0f)));
	perfectSphereAdjust->addChild(geodeSphere);
	root->addChild(perfectSphereAdjust);
	
	// run optimization over the scene graph
	osgUtil::Optimizer optimzer;
	optimzer.optimize( root );
	
    // add all the camera manipulators
	osgGA::NodeTrackerManipulator* tm = new osgGA::NodeTrackerManipulator;
	osgGA::NodeTrackerManipulator::TrackerMode trackerMode = osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION;
	osgGA::NodeTrackerManipulator::RotationMode rotationMode = osgGA::NodeTrackerManipulator::TRACKBALL;
	tm->setTrackerMode( trackerMode );
	tm->setRotationMode( rotationMode );
	if (spMainPlanet.get())
		tm->setTrackNode( spMainPlanet.get() );
	else if (spSpaceShip)
		tm->setTrackNode( spSpaceShip.get() );
	else
		tm->setTrackNode( space );
	
	osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;
	keyswitchManipulator->addMatrixManipulator( '1', "Locked Trackball", tm );
    keyswitchManipulator->addMatrixManipulator( '2', "Trackball", new osgGA::TrackballManipulator() );	
	
    unsigned int num = keyswitchManipulator->getNumMatrixManipulators();
    keyswitchManipulator->selectMatrixManipulator(num);

	// Spawn the main pathfinding thread with our ship's starting value
	SearchThread* searchThread = new SearchThread(updateText.get(), solarSystem, osg::Vec2f(0.53f, 0.36f)); 
	
    viewer.setCameraManipulator( keyswitchManipulator.get() );
	// add this slave camera to the viewer, with a shift left of the projection matrix
    viewer.addSlave(camera, osg::Matrixd::translate(0.0,0.0,2.0), osg::Matrixd());
	
    // add the handler for doing the picking
    viewer.addEventHandler(new PickHandler(updateText.get(), solarSystem, searchThread));
	
    // set the scene to render
    viewer.setSceneData(root);

	viewer.addEventHandler(new osgViewer::ThreadingHandler); 
	viewer.setThreadingModel(osgViewer::ViewerBase::DrawThreadPerContext);
    
    return viewer.run();
}