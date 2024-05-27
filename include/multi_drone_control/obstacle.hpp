#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#include <Eigen/Dense>
#include <Eigen/Core>

namespace obstacle {
    typedef struct {
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        double radius;
        bool is_long;
    } obstacle_t;
} // namespace obstacle

#endif // OBSTACLE_H_
