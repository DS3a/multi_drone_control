#include "cbf_reqs.hpp"
#include <math.h>

extern "C" {

    double rhs(state_t* state, hyperparams_t* hyperparams) {
        double alpha = hyperparams->alpha;
        double p_x = state->pos_x;
        double p_y = state->pos_y;
        double p_z = state->pos_z;
        double v_x = state->vel_x;
        double v_y = state->vel_y;
        double v_z = state->vel_z;
        double t = state->time;

        double c_x = state->obstacle_pos_x;
        double c_y = state->obstacle_pos_y;
        double c_z = state->obstacle_pos_z;
        double cdot_x = state->obstacle_vel_x;
        double cdot_y = state->obstacle_vel_y;
        double cdot_z = state->obstacle_vel_z;

        double r = state->radius;

        double rhs_val = alpha*((c_x - p_x)*(cdot_x - v_x) + (c_y - p_y)*(cdot_y - v_y) + sqrt(-pow(r, 2) + sqrt(pow(c_x - p_x, 2) + pow(c_y - p_y, 2)))*sqrt(pow(cdot_x - v_x, 2) + pow(cdot_y - v_y, 2))) + v_x*(-cdot_x + v_x + (1.0/2.0)*(-c_x + p_x)*sqrt(pow(cdot_x - v_x, 2) + pow(cdot_y - v_y, 2))/(sqrt(-pow(r, 2) + sqrt(pow(c_x - p_x, 2) + pow(c_y - p_y, 2)))*sqrt(pow(c_x - p_x, 2) + pow(c_y - p_y, 2)))) + v_y*(-cdot_y + v_y + (1.0/2.0)*(-c_y + p_y)*sqrt(pow(cdot_x - v_x, 2) + pow(cdot_y - v_y, 2))/(sqrt(-pow(r, 2) + sqrt(pow(c_x - p_x, 2) + pow(c_y - p_y, 2)))*sqrt(pow(c_x - p_x, 2) + pow(c_y - p_y, 2))))
;
        return rhs_val;
    }

    double lhs_ax(state_t* state, hyperparams_t* hyperparams) {
        double alpha = hyperparams->alpha;
        double p_x = state->pos_x;
        double p_y = state->pos_y;
        double p_z = state->pos_z;
        double v_x = state->vel_x;
        double v_y = state->vel_y;
        double v_z = state->vel_z;
        double t = state->time;

        double c_x = state->obstacle_pos_x;
        double c_y = state->obstacle_pos_y;
        double c_z = state->obstacle_pos_z;
        double cdot_x = state->obstacle_vel_x;
        double cdot_y = state->obstacle_vel_y;
        double cdot_z = state->obstacle_vel_z;

        double m_T = state->mass;
        double r = state->radius;
        double lhs_ax_val = -(-c_x + p_x + (-cdot_x + v_x)*sqrt(-pow(r, 2) + sqrt(pow(c_x - p_x, 2) + pow(c_y - p_y, 2)))/sqrt(pow(cdot_x - v_x, 2) + pow(cdot_y - v_y, 2)))/m_T
;

        return lhs_ax_val;
    }

    double lhs_ay(state_t *state, hyperparams_t *hyperparams) {
      double alpha = hyperparams->alpha;
      double p_x = state->pos_x;
      double p_y = state->pos_y;
      double p_z = state->pos_z;
      double v_x = state->vel_x;
      double v_y = state->vel_y;
      double v_z = state->vel_z;
      double t = state->time;

      double c_x = state->obstacle_pos_x;
      double c_y = state->obstacle_pos_y;
      double c_z = state->obstacle_pos_z;
      double cdot_x = state->obstacle_vel_x;
      double cdot_y = state->obstacle_vel_y;
      double cdot_z = state->obstacle_vel_z;

      double m_T = state->mass;
      double r = state->radius;

      double lhs_ay_val = -(-c_y + p_y + (-cdot_y + v_y)*sqrt(-pow(r, 2) + sqrt(pow(c_x - p_x, 2) + pow(c_y - p_y, 2)))/sqrt(pow(cdot_x - v_x, 2) + pow(cdot_y - v_y, 2)))/m_T
;
      return lhs_ay_val;
    }

    double lhs_az(state_t* state, hyperparams_t* hyperparams) {
        return 0.0;
    }

}
