------------------------------------
Copyright 2013 - Christian Schüller 2013, schuellc@inf.ethz.ch
Interactive Geometry Lab - ETH Zurich
LIM may be freely copied, modified, and redistributed under the
copyright notices stated in the file license.
------------------------------------

------------------------------------
LIM - Locally Injective Mappings
------------------------------------
Is a small c++ library to compute local injective mappings of triangle and tet meshes minimizing an arbitrary deformation energy subject to some linear positional constraints.

----------------------
c++ LIM library (src/)
----------------------
The easiest way is to use the functions provided in src/LIMSolverInterface.h. Function ComputeLIM() takes all the arguments needed and returns the resulting mesh. If you want to visualize the intermediate iterations use the functions InitLIM() and ComputeLIM_Step(). In order to have full control over the solver have a look at viewer/src/plugins/DeformLocallyInjective.cpp.
If you want to add your own deformation energy check the implementation of the existing energies in src/ which all inherit from the class LIMSolver2D or LIMSolver3D.

Dependencies:
Eigen3 : http://eigen.tuxfamily.org/
libigl : http://igl.ethz.ch/projects/libigl/

----------------------
demo viewer (viewer/)
----------------------
This is a simple demo viewer providing interactive control over triangle and tet meshes using the LIM library.

Binary: viewer/msvc/Release/msvc.exe
VS2010 project : viewer/msvc

Meshes:
Some test meshes are provided in viewer/data. Note that in 3d only closed triangle surfaces can be loaded and will be tetrahedralized by tetgen to a tet mesh.

Controls:
Select vertices - Ctrl + click and move left mouse
Translate mode - Ctrl + t
Rotate mode - Ctrl + r
Deselect mode - Ctrl + d

Dependencies:
Eigen3 : http://eigen.tuxfamily.org/
libigl : http://igl.ethz.ch/projects/libigl/
freeglut : http://freeglut.sourceforge.net/
glew : http://glew.sourceforge.net/
tetgen : http://wias-berlin.de/software/tetgen/

----------------------
MATLAB interface (matlab/)
----------------------
To use the library with matlab you can use

*.mexw64 for windows
*.mexmaci64 for mac

Have a look at matlab/example.m how to use the provided functions. Also check src/LIMSolverInterface.h for more details about the implementation and parameters.
