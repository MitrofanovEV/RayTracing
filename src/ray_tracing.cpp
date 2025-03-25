#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <algorithm>

#define EPS 1e-5
#define MAX_ITER 1000000

struct Layer {
    double depth; // depth of the layer
    double speed; // sound speed on current depth
};

struct RayCoordinates {
    double dist; // travel distance
    double depth; // travel depth
    double angle; // angle of ray
    double time; // travel time
};


/// @brief class for interpolating sound speed
class SoundSpeedInterpolator {
private:
    std::vector<double> depths;
    std::vector<double> speeds;

public:

    SoundSpeedInterpolator(const std::vector<double>& depthArray, const std::vector<double>& speedArray) {
        depths = depthArray;
        speeds = speedArray;
    }

    /// @brief Method to get the speed of sound at a given depth using linear interpolation
    double get_speed(double depth) const {

        if (depth > depths.back()) return speeds.back();

        if (depth < depths.front()) return speeds.front();

        auto it = std::lower_bound(depths.begin(), depths.end(), depth);

        if (it != depths.end() && *it == depth) {
            return speeds[it - depths.begin()];
        }

        size_t upperIndex = it - depths.begin();
        size_t lowerIndex = upperIndex - 1;

        double depth0 = depths[lowerIndex];
        double depth1 = depths[upperIndex];
        double speed0 = speeds[lowerIndex];
        double speed1 = speeds[upperIndex];

        double speed = speed0 + (speed1 - speed0) * (depth - depth0) / (depth1 - depth0);

        return speed;
    }

    /// @brief Max depth getter
    double max_depth() const {
        return depths.back();
    }

};

/// @brief Convert degrees to radians
double to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

/// @brief Get normal oriented angle from horizontal oriented
double horizontal_to_normal(double degrees) {
    double sign = (degrees > 0) ? 1. : -1.;
    return sign * (90. - fabs(degrees));
}

/// @brief Ray tracing function
std::vector<RayCoordinates> trace_ray(const SoundSpeedInterpolator &speed_interpolator, const RayCoordinates &initial_position, double dz, double total_dist) {

    RayCoordinates current_position = initial_position;
    std::vector<RayCoordinates> trajectory;

    if (fabs(fabs(initial_position.angle) - M_PI_2) < EPS) {
        trajectory.push_back(current_position);
        current_position.dist = total_dist;
        trajectory.push_back(current_position);
        return trajectory;
    }

    while (total_dist > current_position.dist) {

        trajectory.push_back(current_position);
        double in_speed = speed_interpolator.get_speed(current_position.depth);
        double sign = (current_position.angle > 0) ? 1. : -1.;

        current_position.depth += sign * dz;
        current_position.dist += fabs(dz * tan(current_position.angle));
        current_position.time += fabs(dz / cos(current_position.angle) / in_speed);

        double out_speed = speed_interpolator.get_speed(current_position.depth);
        double sina = in_speed / out_speed * sin(current_position.angle);

        if (
            fabs(sina) > 1 || 
            fabs(current_position.depth) < EPS || 
            fabs(current_position.depth - speed_interpolator.max_depth()) < EPS
        ) 
            current_position.angle *= -1;
        else 
            current_position.angle = asin(sina);

        if (trajectory.size() > MAX_ITER) {
            trajectory.clear();
            return trajectory;
        }
    }
    return trajectory;
}


int main() {

    std::vector<double> depths = {
        0.0, 4.4, 13.4, 16.1, 21.1, 35.3, 38.9, 41.4,
        56.0, 65.1, 72.5, 90.8, 103.4, 115.0, 199.8, 200.0
    };

    std::vector<double> speeds = {
        1482.0, 1481.4, 1478.9, 1475.3, 1474.0, 1472.9, 1471.6, 1471.4,
        1469.2, 1469.0, 1469.5, 1469.8, 1469.6, 1471.0, 1474.6, 1474.6
    };  

    std::vector<double> ray_angles = {
        5, 10 , -10, -5
    };

    auto interpolator = SoundSpeedInterpolator(depths, speeds);
    double source_depth = 10.;

    for (const auto& ray_angle : ray_angles) {

        RayCoordinates initial_position{0, source_depth, to_radians(horizontal_to_normal(ray_angle)), 0};
        std::vector<RayCoordinates> trajectory;
        trajectory = trace_ray(interpolator, initial_position, 0.001, 2000.);
        
        std::string filename = "rays/ray_" + std::to_string(ray_angle) + ".txt";
        
        std::ofstream outFile(filename);
        for (const auto& ray : trajectory) {
            outFile << ray.dist << " " << ray.depth<< " " << ray.time << "\n";
        }

        outFile.close();
    }
    return 0;
}