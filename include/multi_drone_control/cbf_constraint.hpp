#ifndef CBF_CONSTRAINT_H_
#define CBF_CONSTRAINT_H_
#include <memory>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <functional>

#include "multi_drone_control/obstacle.hpp"

#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;


typedef CGAL::Quadratic_program<ET> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;

#define AX_IDX 0
#define AY_IDX 1
#define AZ_IDX 2

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

        void add_to_qp(std::shared_ptr<Program> qp,
                       std::shared_ptr<state_t> state,
                       std::shared_ptr<hyperparams_t> hyperparams,
                       int idx,
                       double delta=0.0) {
            qp->set_a(AX_IDX, idx, this->_lhs_ax(*state, *hyperparams));
            qp->set_a(AY_IDX, idx, this->_lhs_ay(*state, *hyperparams));
            qp->set_a(AZ_IDX, idx, this->_lhs_az(*state, *hyperparams));

            qp->set_b(idx, this->_rhs(*state, *hyperparams)+delta);
        }

        double get_lhs_ax(std::shared_ptr<state_t> state,
                          std::shared_ptr<hyperparams_t> hyperparams) {
            return this->_lhs_ax(*state, *hyperparams);
        }

        double get_lhs_ay(std::shared_ptr<state_t> state,
                          std::shared_ptr<hyperparams_t> hyperparams) {
            return this->_lhs_ay(*state, *hyperparams);
        }

        double get_lhs_az(std::shared_ptr<state_t> state,
                          std::shared_ptr<hyperparams_t> hyperparams) {
            return this->_lhs_az(*state, *hyperparams);
        }


        double get_rhs(std::shared_ptr<state_t> state,
                       std::shared_ptr<hyperparams_t> hyperparams) {
            return this->_rhs(*state, *hyperparams);
        }
    };

} // namespace cbf_constraint

#endif // CBF_CONSTRAINT_H_
