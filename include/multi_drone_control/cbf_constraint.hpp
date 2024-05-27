#ifndef CBF_CONSTRAINT_H_
#define CBF_CONSTRAINT_H_
#include <memory>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <functional>

#include "multi_drone_control/obstacle.hpp"

namespace cbf_constraint {
    typedef struct {
        double alpha;
    } hyperparams_t;

    typedef struct {
        std::shared_ptr<Eigen::Vector3d> pos;
        std::shared_ptr<Eigen::Vector3d> vel;
        std::shared_ptr<obstacle::obstacle_t> obs;
        double time;
    } state_t;

    class CBF_Constraint {
    public:
        std::function<double(state_t, hyperparams_t)> lhs_ax;
        std::function<double(state_t, hyperparams_t)> lhs_ay;
        std::function<double(state_t, hyperparams_t)> lhs_az;

        std::function<double(state_t)> rhs;
    };

} // namespace cbf_constraint

#endif // CBF_CONSTRAINT_H_
