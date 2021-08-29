#include "asmc.hpp"

ASMC::ASMC(double _sample_time_s, const double _K2, const double _Kalpha, const double _Kmin, const double _miu, const DOFControllerType_E _type)
{
    sample_time_s = _sample_time_s;
    K1 = 0;
    K2 = _K2;
    Kalpha = _Kalpha;
    Kmin = _Kmin;
    miu = miu;
    controller_type = _type;
    error = 0.0;
    dot_error = 0.0;

}

ASMC::~ASMC(){}

void ASMC::Manipulation(double _current)
{
    prev_error = error;
    prev_dot_error = dot_error;
    error = set_point - _current;
    dot_error = error/sample_time_s;

    if (controller_type == ANGULAR_DOF)
    {
        if (abs(error) > PI)
        {
            error = (error / abs(error)) * (abs(error) - 2 * PI);
        }
        if (abs(dot_error) > PI)
        {
            dot_error = (dot_error / abs(dot_error)) * (abs(dot_error) - 2 * PI);
        }
    }

    sigma = dot_error + lambda*error;

    double sign = (abs(sigma) - miu) / (abs(sigma) - miu);
    double dot_K1 = K1>Kmin ?  Kalpha*sign:Kmin;
    K1 += dot_K1*sample_time_s;

    manipulation = K1*sign + K2*sigma;
}