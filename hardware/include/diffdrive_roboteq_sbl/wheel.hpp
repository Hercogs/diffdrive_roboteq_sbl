#ifndef DIFFDRIVE_ROBOTEQ_WHEEL_HPP
#define DIFFDRIVE_ROBOTEQ_WHEEL_HPP

#include <string>
#include <cmath>

class Wheel{
    public:
        std::string name = "";
        int enc = 0;
        double cmd = 0.0;
        double pos = 0.0;
        double vel = 0.0;
        double rads_per_count = 0;
        int max_rpm {};

        Wheel() = default;

        void setup(const std::string &wheel_name, int counts_per_rev)
        {
            name = wheel_name;
            rads_per_count = 2 * M_PI / counts_per_rev;
        }

        double calc_enc_angle()
        {
            return enc * rads_per_count;
        }

};

#endif // DIFFDRIVE_ROBOTEQ_WHEEL_HPP