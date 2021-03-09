#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <unistd.h>
using namespace std;

#include <chrono>
using namespace std::chrono;

#include <Eigen/Eigen>
using namespace Eigen;

#include "pose_ekf.h"
#include "pose_viewer.h"

struct IMU_LINE 
{
  Vector3d acc;
  Vector3d gyr;
  Vector3d mag; 
  double qw; 
  double qx;
  double qy;
  double qz; 
  double roll; 
  double pitch; 
  double yaw; 

  std::string to_string() 
  {
    std::ostringstream oss; 
    oss << "acc: (" << acc(0) << ", " << acc(1) << ", " << acc(2) << ") " 
        << "gyr: (" << gyr(0) << ", " << gyr(1) << ", " << gyr(2) << ") "
        << "mag: (" << mag(0) << ", " << mag(1) << ", " << mag(2) << ") "
        << std::endl; 
  }
}; 

std::vector<IMU_LINE> imu_data; 
size_t current_line = 0; 

float findValue(const std::string& s, const std::string& key)
{
    size_t start, end; 
    std::string s_value; 
    start = s.find(key); 
    if (start != std::string::npos) {
      start += key.length(); 
      end = s.find(" ", start);
      if (end != std::string::npos) {
        s_value = s.substr(start, end-start); 
      }
      else {
        s_value = s.substr(start); 
      }
    }

    if (s_value.empty()) {
      return 0; 
    }
    
    // std::cout << key << ": " << s_value << std::endl; 
    return std::stof(s_value); 
}

void loadIMU(const std::string& fname)
{
  std::ifstream f(fname); 
  if (!f.is_open()) {
    std::cout << "failed open " << fname << std::endl; 
    return; 
  }
  std::string line;
  while (std::getline(f, line))
  {
    IMU_LINE imu;
    imu.acc(0) = findValue(line, "Acc X:");
    imu.acc(1) = findValue(line, "Acc Y:");
    imu.acc(2) = findValue(line, "Acc Z:");
    imu.gyr(0) = findValue(line, "Gyr X:");
    imu.gyr(1) = findValue(line, "Gyr Y:");
    imu.gyr(2) = findValue(line, "Gyr Z:");
    imu.mag(0) = findValue(line, "Mag X:");
    imu.mag(1) = findValue(line, "Mag Y:");
    imu.mag(2) = findValue(line, "Mag Z:");
    imu.qw = findValue(line, "q0:");
    imu.qx = findValue(line, "q1:");
    imu.qy = findValue(line, "q2:");
    imu.qz = findValue(line, "q3:");
    imu.roll = findValue(line, "Roll:");
    imu.pitch = findValue(line, "Pitch:");
    imu.yaw = findValue(line, "Yaw:");
    imu_data.push_back(imu); 
  }
}

// Packet number,
// Gyroscope X (deg/s),Gyroscope Y (deg/s),Gyroscope Z (deg/s), 
// Accelerometer X (g),Accelerometer Y (g),Accelerometer Z (g),
// Magnetometer X (G),Magnetometer Y (G),Magnetometer Z (G)
void loadCSV(const std::string& fname)
{
  std::ifstream f(fname); 
  if (!f.is_open()) {
    std::cout << "failed open " << fname << std::endl; 
    return; 
  }
  std::string line;
  while (std::getline(f, line))
  {
    // std::cout << line << std::endl; 
    IMU_LINE imu;
    std::stringstream linestream(line);
    std::string value; 

    // index 
    std::getline(linestream, value, ','); 

    // gyr 
    std::getline(linestream, value, ','); 
    imu.gyr(0) = std::stod(value); 
    std::getline(linestream, value, ','); 
    imu.gyr(1) = std::stod(value);  
    std::getline(linestream, value, ','); 
    imu.gyr(2) = std::stod(value); 

    // acc
    std::getline(linestream, value, ','); 
    imu.acc(0) = std::stod(value); 
    std::getline(linestream, value, ','); 
    imu.acc(1) = std::stod(value);  
    std::getline(linestream, value, ','); 
    imu.acc(2) = std::stod(value); 

    // mag 
    std::getline(linestream, value, ','); 
    imu.mag(0) = std::stod(value); 
    std::getline(linestream, value, ','); 
    imu.mag(1) = std::stod(value);  
    std::getline(linestream, value, ','); 
    imu.mag(2) = std::stod(value); 

    imu.qw = 0;
    imu.qx = 0;
    imu.qy = 0;
    imu.qz = 0;
    imu.roll = 0;
    imu.pitch = 0;
    imu.yaw = 0;

    // printf("%s", imu.to_string().c_str()); 
    imu_data.push_back(imu); 
  }
}

int main(int argc, char *argv[])
{
  // loadIMU("imu.txt"); 
  loadCSV("test.csv"); 

  pose_viewer viewer(&argc, argv); 
  pose_ekf pose;

  double time = 0; 
  double dt = 0.1; 
  while (true) {
    IMU_LINE& imu = imu_data[current_line]; 

    if (pose.is_init_done == false) {
      Vector3d pos = Vector3d::Zero();
      Vector3d vel = Vector3d::Zero();
      pose.pose_init(pos, vel);
    }

    if (pose.is_atti_init_done == false) {
      pose.atti_init(imu.acc, imu.mag);
    } 
    else {
      pose.predict(imu.gyr, imu.acc, dt);
      pose.update_magnetic(imu.mag);
    }

    pose.set_timestatmp(time); 

    Vector3d pos = pose.pos;
    Quaterniond q = pose.q;

    // viewer.update_position(pos(0), pos(1), pos(2));
    viewer.update_position(0, 0, 0); 

    viewer.update_orientation(q.w(), q.x(), q.y(), q.z());
    // viewer.update_orientation(imu.qw, imu.qx, imu.qy, imu.qz); 
    // viewer.update_orientation(imu.roll, imu.pitch, imu.yaw); 

    current_line += 1;
    if (current_line >= imu_data.size()) {
      break; 
    } 

    sleep(dt); 
    time += dt; 
  }

  return 0;
}
