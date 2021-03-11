#ifndef MADGWICH_AHRS_H
#define MADGWICH_AHRS_H

#include <Eigen/Eigen>
using namespace Eigen;

class MadgwichAHRS 
{
public:     
    MadgwichAHRS(float sample_period, float beta = 1.0); 

    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz); 
    void update(float gx, float gy, float gz, float ax, float ay, float az); 

    const Quaterniond quaternion() const { return Q; }

private: 
    float SamplePeriod; 
    float Beta; 
    Quaterniond Q; 
}; 

#endif 