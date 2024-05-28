#ifndef VEL_BARRIERS_H_
#define VEL_BARRIERS_H_

#include "multi_drone_control/cbf_constraint.hpp"

namespace vel_barriers {
    namespace vel_min_barrier_fn {
        // LHS is -1 for the velocity component required to be constrained

        double rhs(double alpha,
                   double v_max,
                   double v) {
            double rhs_val = alpha*(v_max + v);

            return rhs_val;
        }
    }

    namespace vel_max_barrier_fn {
        // LHS is +1 for the velocity component we are trying to constrain

        double rhs(double alpha,
                   double v_max,
                   double v) {
            double rhs_val = alpha*(v_max - v);

            return rhs_val;
        }
    }


    namespace vx_min_bf {
        double lhs_ax(cbf_constraint::state_t state,
                      cbf_constraint::hyperparams_t hps) {
            return -1;
        }

        double lhs_ay(cbf_constraint::state_t state,
                      cbf_constraint::hyperparams_t hps) {
            return 0;
        }

        double lhs_az(cbf_constraint::state_t state,
                      cbf_constraint::hyperparams_t hps) {
            return 0;
        }

        double rhs(cbf_constraint::state_t state,
                   cbf_constraint::hyperparams_t hps) {
            return vel_min_barrier_fn::rhs(hps.alpha, hps.max_vel,
                                           state.vel->x());
        }
    } // namespace vx_min_bf


    namespace vy_min_bf {
        double lhs_ax(cbf_constraint::state_t state,
                      cbf_constraint::hyperparams_t hps) {
            return 0;
        }

        double lhs_ay(cbf_constraint::state_t state,
                      cbf_constraint::hyperparams_t hps) {
            return -1;
        }

        double lhs_az(cbf_constraint::state_t state,
                      cbf_constraint::hyperparams_t hps) {
            return 0;
        }

        double rhs(cbf_constraint::state_t state,
                   cbf_constraint::hyperparams_t hps) {
            return vel_min_barrier_fn::rhs(hps.alpha, hps.max_vel, state.vel->y());
        }
    }


    namespace vz_min_bf {
        double lhs_ax(cbf_constraint::state_t state,
                      cbf_constraint::hyperparams_t hps) {
            return 0;
        }

        double lhs_ay(cbf_constraint::state_t state,
                      cbf_constraint::hyperparams_t hps) {
            return 0;
        }

        double lhs_az(cbf_constraint::state_t state,
                      cbf_constraint::hyperparams_t hps) {
            return -1;
        }

        double rhs(cbf_constraint::state_t state,
                   cbf_constraint::hyperparams_t hps) {
            return vel_min_barrier_fn::rhs(hps.alpha, hps.max_vel_z, state.vel->z());
        }
    }

    namespace vx_max_bf {
        double lhs_ax(cbf_constraint::state_t state,
                      cbf_constraint::hyperparams_t hps) {
            return 1;
        }

        double lhs_ay(cbf_constraint::state_t state,
                      cbf_constraint::hyperparams_t hps) {
            return 0;
        }

        double lhs_az(cbf_constraint::state_t state,
                      cbf_constraint::hyperparams_t hps) {
            return 0;
        }

        double rhs(cbf_constraint::state_t state,
                   cbf_constraint::hyperparams_t hps) {
            return vel_max_barrier_fn::rhs(hps.alpha, hps.max_vel, state.vel->x());
        }

    }

    namespace vy_max_bf {
        double lhs_ax(cbf_constraint::state_t state,
                      cbf_constraint::hyperparams_t hps) {
            return 0;
        }

        double lhs_ay(cbf_constraint::state_t state,
                      cbf_constraint::hyperparams_t hps) {
            return 1;
        }

        double lhs_az(cbf_constraint::state_t state,
                      cbf_constraint::hyperparams_t hps) {
            return 0;
        }

        double rhs(cbf_constraint::state_t state,
                   cbf_constraint::hyperparams_t hps) {
            return vel_max_barrier_fn::rhs(hps.alpha, hps.max_vel, state.vel->y());
        }
    }


    namespace vz_max_bf {
        double lhs_ax(cbf_constraint::state_t state,
                      cbf_constraint::hyperparams_t hps) {
            return 0;
        }

        double lhs_ay(cbf_constraint::state_t state,
                      cbf_constraint::hyperparams_t hps) {
            return 0;
        }

        double lhs_az(cbf_constraint::state_t state,
                      cbf_constraint::hyperparams_t hps) {
            return 1;
        }

        double rhs(cbf_constraint::state_t state,
                   cbf_constraint::hyperparams_t hps) {
            return vel_max_barrier_fn::rhs(hps.alpha, hps.max_vel_z, state.vel->z());
        }
    }
} // namespace vel_barriers

#endif // VEL_BARRIERS_H_
