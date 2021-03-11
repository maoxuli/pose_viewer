#! /usr/bin/env bash 

# compile 
echo "Compiling"
g++ -c -Wall -I/usr/include/eigen3 `pkg-config --cflags opencv` src/MadgwichAHRS.cpp src/gl_cuboid.cpp src/main.cpp
if [ $? -ne 0 ]; then
  echo "Compile error!"
  exit
fi

# link 
echo "Linking"
g++ MadgwichAHRS.o gl_cuboid.o main.o -lm -lGL -lGLU -lglut -lpthread `pkg-config --libs opencv` -o pose_viewer 
if [ $? -ne 0 ]; then
  echo "Link error!"
  exit
fi

# clean
rm *.o
echo "Build done!"
