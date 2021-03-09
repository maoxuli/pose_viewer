#! /usr/bin/env bash 

# compile 
echo "Compiling"
g++ -c -Wall -I/usr/include/eigen3 src/eskf.cpp src/pose_ekf.cpp src/pose_viewer.cpp src/main.cpp
if [ $? -ne 0 ]; then
  echo "Compile error!"
  exit
fi

# link 
echo "Linking"
g++ eskf.o pose_ekf.o pose_viewer.o main.o -lm -lGL -lGLU -lglut -lpthread -o pose_viewer 
if [ $? -ne 0 ]; then
  echo "Link error!"
  exit
fi

# clean
rm *.o
echo "Build done!"
