#ifndef __POSE_VIEWER_H
#define __POSE_VIEWER_H

#include <thread> 
#include <mutex> 

# include <GL/glut.h>
# include <GL/freeglut.h>

#include <Eigen/Eigen>
using namespace Eigen;

class pose_viewer
{
public:
    pose_viewer(int* argc, char** argv); 
    ~pose_viewer();

    void update_position(double x, double y, double z); 
    void update_orientation(double roll, double pitch, double yaw); 
    void update_orientation(double qw, double qx, double qy, double qz); 

    void update_display(); 

private: 
    static pose_viewer* instance;
    static void display_callback() 
    {
        assert(instance);
        instance->display(); 
    }
    static void reshape_callback(int w, int h)
    {
        assert(instance); 
        instance->reshape(w, h); 
    }

    void display(); 
    void reshape(int w, int h);

    // thread for glut 
    void gl_thread(); 
    std::thread _gl_thread; 
    bool _gl_inited; 
    std::mutex _gl_mutex; 

private: 
    GLfloat pos[3] = { 0.0, 0.0, 0.0 };
    GLfloat q[4] = {0.0, 0.0, 0.0, 0.0}; 
    GLfloat theta[3] = { 0.0, 0.0, 0.0 };
};

#endif