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
        double max_vel_xy;
        double max_vel_z;
        double max_vel;
    } hyperparams_t;

    typedef struct {
        std::shared_ptr<Eigen::Vector3d> pos;
        std::shared_ptr<Eigen::Vector3d> vel;
        std::shared_ptr<obstacle::obstacle_t> obs;
        double time;
    } state_t;

    typedef std::function<double(state_t, hyperparams_t)> barrier_coeff_fn;

    class CBF_Constraint {
    public:
        barrier_coeff_fn _lhs_ax;
        barrier_coeff_fn _lhs_ay;
        barrier_coeff_fn _lhs_az;

        barrier_coeff_fn _rhs;

        CBF_Constraint(barrier_coeff_fn lhs_ax,
                       barrier_coeff_fn lhs_ay,
                       barrier_coeff_fn lhs_az,
                       barrier_coeff_fn rhs) {
            this->_lhs_ax = lhs_ax;
            this->_lhs_ay = lhs_ay;
            this->_lhs_az = lhs_az;
            this->_rhs = rhs;
        }

        bool add_to_qp() {

        }
    };

} // namespace cbf_constraint

#endif // CBF_CONSTRAINT_H_
