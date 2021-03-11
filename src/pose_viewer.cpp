#include "pose_viewer.h"

#define _USE_MATH_DEFINES
#include <math.h>

static const GLfloat vertices[][3] = {
  -1.0, -1.0, -1.0,
   1.0, -1.0, -1.0,
  -1.0,  1.0, -1.0,
   1.0,  1.0, -1.0,
  -1.0, -1.0,  1.0,
   1.0, -1.0,  1.0,
  -1.0,  1.0,  1.0,
   1.0,  1.0,  1.0,
};

static const GLint surfaces[][4] = {
  0, 2, 3, 1,
  0, 4, 6, 2,
  0, 1, 5, 4,
  4, 5, 7, 6,
  1, 3, 7, 5,
  2, 6, 7, 3,
};

pose_viewer* pose_viewer::instance = NULL; 

pose_viewer::pose_viewer(int* argc, char** argv)
{
  pose_viewer::instance = this; 
  glutInit(argc, argv);

  std::unique_lock<std::mutex> lock(_gl_mutex); 
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(500, 500);
  glutInitWindowPosition(0, 0);
  glutCreateWindow("Pose Viewer");
  glutReshapeFunc(pose_viewer::reshape_callback); 
  glutDisplayFunc(pose_viewer::display_callback); 
  glEnable(GL_DEPTH_TEST);
  _gl_inited = true; 
  lock.unlock(); 
  printf("gl main loop...\n"); 
  
  printf("Start gl thread\n"); 
  _gl_inited = false; 
  _gl_thread = std::thread(std::bind(&pose_viewer::gl_thread, this));
}

pose_viewer::~pose_viewer()
{
  printf("Stop gl thread\n"); 
  glutLeaveMainLoop(); 
  _gl_thread.join();
}

void pose_viewer::update_position(double x, double y, double z)
{
  std::unique_lock<std::mutex> lock(_gl_mutex); 
  pos[0] = x; 
  pos[1] = y; 
  pos[2] = z; 
  printf("position: %f,%f,%f\n", pos[0], pos[1], pos[2]); 
  lock.unlock(); 
  update_display(); 
}

void pose_viewer::update_orientation(double roll, double pitch, double yaw)
{
  std::unique_lock<std::mutex> lock(_gl_mutex); 
  theta[0] = roll; 
  theta[1] = pitch; 
  theta[2] = yaw; 
  printf("orientation: %f,%f,%f\n", theta[0], theta[1], theta[2]); 
  lock.unlock(); 
  update_display(); 
}

void pose_viewer::update_orientation(double qw, double qx, double qy, double qz)
{
  std::unique_lock<std::mutex> lock(_gl_mutex); 
  q[0] = qw; 
  q[1] = qx; 
  q[2] = qy;
  q[3] = qz;  
  printf("quaternion: %f,%f,%f,%f\n", q[0], q[1], q[2], q[3]); 
  lock.unlock(); 
  update_display(); 
}

void pose_viewer::update_display() 
{
  std::unique_lock<std::mutex> lock(_gl_mutex); 
  if (_gl_inited) {
    glutPostRedisplay();
  }
}

void pose_viewer::display()
{
  std::unique_lock<std::mutex> lock(_gl_mutex); 
  if (_gl_inited) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    
    glTranslatef(pos[0], pos[1], pos[2]);
    // glRotatef(theta[0]+45, 1.0, 0.0, 0.0);
    // glRotatef(theta[1]+45, 0.0, 1.0, 0.0);
    // glRotatef(theta[2]+45, 0.0, 0.0, 1.0);
    double norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm == 0) {
      return;
    }
    else {
      q[0] = q[0] / norm;
      q[1] = q[1] / norm;
      q[2] = q[2] / norm;
      q[3] = q[3] / norm;
    }
    double theta = acos(q[0]) * 2;
    norm = sqrt(q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm != 0) {
      glRotatef(theta*180/M_PI, q[1]/norm, q[2]/norm, q[3]/norm);
    }
    else {
      glRotatef(theta*180/M_PI, q[1], q[2], q[3]);
    }


    glFrontFace(GL_CCW);
    // glCullFace(GL_BACK);
    // glEnable(GL_CULL_FACE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glBegin(GL_QUADS);
    for(int i = 0; i < 6; ++i) 
      for(int j = 0; j < 4; ++j) 
        glVertex3fv(vertices[surfaces[i][j]]);
    glEnd();

    glFlush();
    glutSwapBuffers();
  }
}

void pose_viewer::reshape(int w, int h)
{
  std::unique_lock<std::mutex> lock(_gl_mutex); 
  if (_gl_inited) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    if ( w <= h )
    {
      glOrtho ( 
        -2.0, 2.0, 
        -2.0 * ( GLfloat ) h / ( GLfloat ) w, 2.0 * ( GLfloat ) h / ( GLfloat ) w, 
        -10.0, 10.0 );
    }
    else
    {
      glOrtho ( 
        -2.0 * ( GLfloat ) h / ( GLfloat ) w, 2.0 * ( GLfloat ) h / ( GLfloat ) w,  
        -2.0, 2.0, 
        -10.0, 10.0 );
    }

    glMatrixMode ( GL_MODELVIEW );
  }
}

void pose_viewer::gl_thread() 
{
  printf("gl thread in\n"); 
  glutMainLoop();
  printf("gl thread out\n"); 
}
