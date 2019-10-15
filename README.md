# Football keepie uppie animation 

Animation of a footballer who is doing keepie uppie. 
Created a simple animation project using C++ and OpenGL.

###### Some examples:

![Screenshot from 2019-05-31 17-06-44](https://user-images.githubusercontent.com/30556894/66858797-56a9ba80-ef8a-11e9-8e8e-c2183787b366.png)

![Screenshot from 2019-05-31 17-05-22](https://user-images.githubusercontent.com/30556894/66858783-50b3d980-ef8a-11e9-8fa7-084a7001b8cb.png)


How to compile this code:

Let us suppose you have a command line opened in the directory 01_minimal_window/

== Using CMake (in command line)

$ cd cmake
$ mkdir build
$ cd build
$ cmake ..
$ make
$ cd ../..
$ cmake/build/pgm

(note: the build directory is temporary and can be removed safely)


== Using CMake (with QtCreator)

$ qtcreator cmake/CMakeLists.txt &

Then follow the configuration steps from the GUI.

By default, a temporary directory build-cmake-Desktop-Default is created (in the parent directory of CMakeLists.txt file), as well as a file CMakeLists.txt.user (same directory than CMakeLists.txt file). Both can be removed safely.


= On Windows system

- Use CMakeLists.txt with Visual Studio
- Precompiled version of GLFW3 is provided (precompiled/glfw3_win) with win32
- You need to copy data/ and shaders/ directories in the executable directory
