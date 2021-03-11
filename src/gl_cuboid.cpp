#include "gl_cuboid.h"

#include <GL/glut.h>
#include <GL/freeglut.h>

#include <opencv2/opencv.hpp>

#define _USE_MATH_DEFINES
#include <math.h>
#include <functional>

GlCuboid* GlCuboid::instance = NULL;

GlCuboid::GlCuboid(int* argc, char** argv)
{
    GlCuboid::instance = this; 
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(640, 480);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("Cuboid");
    glutReshapeFunc(GlCuboid::reshape_callback); 
    glutDisplayFunc(GlCuboid::display_callback); 
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

    imageFiles.resize(6); 
    imageFiles[0] = "Right.png"; 
    imageFiles[1] = "Left.png"; 
    imageFiles[2] = "Back.png"; 
    imageFiles[3] = "Front.png"; 
    imageFiles[4] = "Top.png"; 
    imageFiles[5] = "Bottom.png";
    LoadTextureFromImage();

    halfDimensions.resize(3); 
    halfDimensions[0] = 3; 
    halfDimensions[1] = 2; 
    halfDimensions[2] = 1; 

    transformationMatrix = std::vector<float>({1.0, 0.0, 0.0, 0.0,
                                               0.0, 1.0, 0.0, 0.0,
                                               0.0, 0.0, 1.0, 0.0,
                                               0.0, 0.0, 0.0, 1.0});

    CameraView = CameraViews::Front; 
    CameraDistance = 50.0; 

    _gl_thread = std::thread(std::bind(&GlCuboid::gl_thread, this)); 
}

GlCuboid::~GlCuboid()
{
    glutLeaveMainLoop(); 
    _gl_thread.join();  
}

void GlCuboid::LoadTextureFromImage()
{
    int numOfPic = imageFiles.size();
    textures.resize(numOfPic);
    glGenTextures(numOfPic, textures.data());
    for (int i = 0; i < numOfPic; i++)
    {
        cv::Mat image = cv::imread(std::string("images/") + imageFiles[i]); 
        cv::flip(image, image, 0); 
        glBindTexture(GL_TEXTURE_2D, textures[i]);
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        gluBuild2DMipmaps(GL_TEXTURE_2D, image.channels(), image.cols, image.rows, GL_BGR, GL_UNSIGNED_BYTE, image.data);
    }
}

void GlCuboid::gl_thread() 
{
    glutMainLoop();
}

void GlCuboid::reshape(int w, int h)
{
    glViewport(0, 0, w, h);
    glPushMatrix();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(10, (float)w / (float)h, 1.0, 250);
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void GlCuboid::display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glPolygonMode(GL_FRONT, GL_FILL);

    std::unique_lock<std::mutex> lock(_gl_mutex); 

    // Set camera view and distance
    glTranslatef(0, 0, -1.0 * CameraDistance);
    switch (CameraView)
    {
        case CameraViews::Right:
            glRotatef(-90, 0, 1, 0);
            glRotatef(-90, 1, 0, 0);
            break;
        case CameraViews::Left:
            glRotatef(90, 0, 1, 0);
            glRotatef(-90, 1, 0, 0);
            break;
        case CameraViews::Back:
            glRotatef(90, 1, 0, 0);
            glRotatef(180, 0, 1, 0);
            break;
        case CameraViews::Front:
            glRotatef(-90, 1, 0, 0);
            break;
        case CameraViews::Top:
            break;
        case CameraViews::Bottom:
            glRotatef(180, 1, 0, 0);
            break;
    }

    glPushMatrix();
    glMultMatrixf(transformationMatrix.data());

    lock.unlock(); 

    // +'ve x face
    glBindTexture(GL_TEXTURE_2D, textures[0]);
    glBegin(GL_QUADS);
    glNormal3f(1, 0, 0); glTexCoord2f(0, 0); glVertex3f(halfDimensions[0], -halfDimensions[1], -halfDimensions[2]);
    glNormal3f(1, 0, 0); glTexCoord2f(0, 1); glVertex3f(halfDimensions[0], -halfDimensions[1], halfDimensions[2]);
    glNormal3f(1, 0, 0); glTexCoord2f(1, 1); glVertex3f(halfDimensions[0], halfDimensions[1], halfDimensions[2]);
    glNormal3f(1, 0, 0); glTexCoord2f(1, 0); glVertex3f(halfDimensions[0], halfDimensions[1], -halfDimensions[2]);
    glEnd();

    // -'ve x face
    glBindTexture(GL_TEXTURE_2D, textures[1]);
    glBegin(GL_QUADS);
    glNormal3f(-1, 0, 0); glTexCoord2f(1, 0); glVertex3f(-halfDimensions[0], -halfDimensions[1], -halfDimensions[2]);
    glNormal3f(-1, 0, 0); glTexCoord2f(1, 1); glVertex3f(-halfDimensions[0], -halfDimensions[1], halfDimensions[2]);
    glNormal3f(-1, 0, 0); glTexCoord2f(0, 1); glVertex3f(-halfDimensions[0], halfDimensions[1], halfDimensions[2]);
    glNormal3f(-1, 0, 0); glTexCoord2f(0, 0); glVertex3f(-halfDimensions[0], halfDimensions[1], -halfDimensions[2]);
    glEnd();

    // +'ve y face
    glBindTexture(GL_TEXTURE_2D, textures[2]);
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0); glTexCoord2f(1, 0); glVertex3f(-halfDimensions[0], halfDimensions[1], -halfDimensions[2]);
    glNormal3f(0, 1, 0); glTexCoord2f(1, 1); glVertex3f(-halfDimensions[0], halfDimensions[1], halfDimensions[2]);
    glNormal3f(0, 1, 0); glTexCoord2f(0, 1); glVertex3f(halfDimensions[0], halfDimensions[1], halfDimensions[2]);
    glNormal3f(0, 1, 0); glTexCoord2f(0, 0); glVertex3f(halfDimensions[0], halfDimensions[1], -halfDimensions[2]);
    glEnd();

    // -'ve y face
    glBindTexture(GL_TEXTURE_2D, textures[3]);
    glBegin(GL_QUADS);
    glNormal3f(0, -1, 0); glTexCoord2f(0, 0); glVertex3f(-halfDimensions[0], -halfDimensions[1], -halfDimensions[2]);
    glNormal3f(0, -1, 0); glTexCoord2f(0, 1); glVertex3f(-halfDimensions[0], -halfDimensions[1], halfDimensions[2]);
    glNormal3f(0, -1, 0); glTexCoord2f(1, 1); glVertex3f(halfDimensions[0], -halfDimensions[1], halfDimensions[2]);
    glNormal3f(0, -1, 0); glTexCoord2f(1, 0); glVertex3f(halfDimensions[0], -halfDimensions[1], -halfDimensions[2]);
    glEnd();

    // +'ve z face
    glBindTexture(GL_TEXTURE_2D, textures[4]);
    glBegin(GL_QUADS);
    glNormal3f(0, 0, 1); glTexCoord2f(0, 0); glVertex3f(-halfDimensions[0], -halfDimensions[1], halfDimensions[2]);
    glNormal3f(0, 0, 1); glTexCoord2f(1, 0); glVertex3f(halfDimensions[0], -halfDimensions[1], halfDimensions[2]);
    glNormal3f(0, 0, 1); glTexCoord2f(1, 1); glVertex3f(halfDimensions[0], halfDimensions[1], halfDimensions[2]);
    glNormal3f(0, 0, 1); glTexCoord2f(0, 1); glVertex3f(-halfDimensions[0], halfDimensions[1], halfDimensions[2]);
    glEnd();

    // -'ve z face
    glBindTexture(GL_TEXTURE_2D, textures[5]);
    glBegin(GL_QUADS);
    glNormal3f(0, 0, -1); glTexCoord2f(0, 1); glVertex3f(-halfDimensions[0], -halfDimensions[1], -halfDimensions[2]);
    glNormal3f(0, 0, -1); glTexCoord2f(1, 1); glVertex3f(halfDimensions[0], -halfDimensions[1], -halfDimensions[2]);
    glNormal3f(0, 0, -1); glTexCoord2f(1, 0); glVertex3f(halfDimensions[0], halfDimensions[1], -halfDimensions[2]);
    glNormal3f(0, 0, -1); glTexCoord2f(0, 0); glVertex3f(-halfDimensions[0], halfDimensions[1], -halfDimensions[2]);
    glEnd();

    glPopMatrix();
    glFlush();
    glutSwapBuffers();
}

void GlCuboid::update_display() 
{
    glutPostRedisplay();
}

void GlCuboid::update_position(float x, float y, float z)
{
    std::unique_lock<std::mutex> lock(_gl_mutex); 
    transformationMatrix[12] = x;
    transformationMatrix[13] = y;
    transformationMatrix[14] = z;
    lock.unlock();

    update_display(); 
}

void GlCuboid::update_rotation(float qw, float qx, float qy, float qz)
{
    float sqw = qw*qw;
    float sqx = qx*qx;
    float sqy = qy*qy;
    float sqz = qz*qz;
    float invs = 1.0f / (sqx + sqy + sqz + sqw);

	float mat[] = {1.0, 0.0, 0.0, 
                   0.0, 1.0, 0.0, 
                   0.0, 0.0, 1.0}; 

	mat[0] = ( sqx - sqy - sqz + sqw) * invs;
    mat[4] = (-sqx + sqy - sqz + sqw) * invs;
    mat[8] = (-sqx - sqy + sqz + sqw) * invs;
    
    float tmp1 = qx*qy;
    float tmp2 = qz*qw;
    mat[3] = 2.0 * (tmp1 + tmp2) * invs;
    mat[1] = 2.0 * (tmp1 - tmp2) * invs;
    
    tmp1 = qx*qz;
    tmp2 = qy*qw;
    mat[6] = 2.0 * (tmp1 - tmp2) * invs;
    mat[2] = 2.0 * (tmp1 + tmp2) * invs;

    tmp1 = qy*qz;
    tmp2 = qx*qw;
    mat[7] = 2.0 * (tmp1 + tmp2) * invs;
    mat[5] = 2.0 * (tmp1 - tmp2) * invs; 

    std::unique_lock<std::mutex> lock(_gl_mutex); 
    transformationMatrix[0] = mat[0]; transformationMatrix[4] = mat[1]; transformationMatrix[8] = mat[2];
    transformationMatrix[1] = mat[3]; transformationMatrix[5] = mat[4]; transformationMatrix[9] = mat[5];
    transformationMatrix[2] = mat[6]; transformationMatrix[6] = mat[7]; transformationMatrix[10] = mat[8];
    lock.unlock();

    update_display(); 
}
