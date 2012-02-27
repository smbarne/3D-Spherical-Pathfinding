About this project:

Screenshots, source, installer, and additional information can be found at: 
http://www.artofengineering.net/Engineering/#3DSphericalPathfindingDemo_26

This project is a demo by Stephen Barnes, a recent project that combine small bits of AI, graphics, gameplay, and UI. Simply put, this program allows the user to click on any spot on the center planet and have the user's spaceship to navigate, over land, to that point.

Some technical highlights:

 - Custom shaders to integrate Light Space Perspective Shadowing with multiple lights.
 - A cartoon shader and outline on the user's ship to help it stand out more.
 - A custum built planet model using:
    - fracPlanet (http://www.bottlenose.demon.co.uk/share/fracplanet/), an opensource fractal planet
      generator.  This produced the heightmap and initial texture.
    - Adobe Photoshop.  Photoshop was used to modify the fracPlanet texture to give it more detail and color.
    - Blender, an opensource 3D modeling program.  This used the heightmap and texture from fracPlanet to morph a sphere and assign UV coordinates.  Finally, the model was exported into .osg format
      using the latest OSG exporter for Blender.   
 - Multi-threaded pathfinding keeps the graphics from slowing down.  Pathfinding algorithms include:
    - A* search
    - Memory limited A* search (max nodes = 100)
    - Local beam search (branching factor = 100)
 - When clicking on the planet to select the next route, the 3D XYZ coordinate is converted into 2D UV coordinates.  A ~250x50 navmesh generated from the planet's spheremap is used to plan routes, then the 2D UV coordinates are translated back into 3D XYZ coordinates and used to create an animation path for the spaceship.
 - Cross platform compatability with Windows and Mac OSX.  This installer is provided for Windows, however, the source may be compiled on OSX to work with Mac currently.

Compilation on Windows:

Required/Suggested:
 - Visual Studio 2010

OpenSceneGraph (OSG) 2.8.5 libraries, dlls, and other files are included in the source tree for the Windows version.  Therefore, a seperate install of the OSG developer tools is not needed to compile.  Simply open:

demo/VisualStudio/demo.sln

Select Release, and compile.  The executable should be created in:

demo/bin

The demo executable needs the dlls and osgPlugin dlls in the bin directory to run properly.



Compilation on OSX:

Required/Suggested:
 - Xcode 4
 - OpenSceneGraph (OSG) 2.8.0

First, download and install OSG 2.8.0 libraries from (http://www.cuboslocos.com/tutorials/OSG-GettingStarted).

Next, simply open:

demo/demo.xcodeproj

Select demo | My Mac 32-bit and compile.  The executable should be created in:

demo/bin

The demo executable needs the dlls and osgPlugin dlls in the bin directory to run properly.
