#ifndef __ASMC_H__
#define __ASMC_H__

#include <cmath>
#define PI 3.14159265359;

typedef enum DOFControllerType_E
{
    LINEAR_DOF = 0,
    ANGULAR_DOF = 1,
} DOFControllerType_E;

class ASMC
{
    public:
        float sample_time_s;

        double set_point;
        double error;
        double prev_error;
        double dot_error;
        double prev_dot_error;

        double manipulation;

        double lambda;
        double sigma;           //Sliding surface

        double K1;
        double K2;
        double Kmin;
        double Kalpha;
        double miu;

        DOFControllerType_E controller_type;

        // Constructor
        ASMC(double, const double, const double, const double, const double, const DOFControllerType_E);

        // Destructor
        ~ASMC();

        void Manipulation(double);
};

#endif __ASMC_H__