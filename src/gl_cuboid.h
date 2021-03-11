#ifndef CUBOID_H__
#define CUBOID_H__

#include <vector>
#include <string> 
#include <thread> 
#include <mutex> 
#include <cassert>

class GlCuboid 
{
public: 
    enum CameraViews {Right, Left, Back, Front, Top, Bottom};

    GlCuboid(int* argc, char** argv); 
    ~GlCuboid(); 

    void update_position(float x, float y, float z);
    void update_rotation(float qw, float qx, float qy, float qz);

    void update_display(); 

private: 
    std::vector<std::string> imageFiles;
    std::vector<uint> textures;

    std::vector<float> halfDimensions;
    std::vector<float> transformationMatrix;

    CameraViews CameraView; 
    float CameraDistance;

    void LoadTextureFromImage();
    void display(); 
    void reshape(int w, int h);

    // thread for glut 
    void gl_thread(); 
    std::thread _gl_thread; 
    std::mutex _gl_mutex; 

    static GlCuboid* instance;
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
}; 

#endif 
