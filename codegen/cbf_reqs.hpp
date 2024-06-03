#ifndef CBF_REQS_H_
#define CBF_REQS_H_

typedef struct {
    double mass;

    double pos_x;
    double pos_y;
    double pos_z;
    double vel_x;
    double vel_y;
    double vel_z;
    double time;

    double obstacle_pos_x;
    double obstacle_pos_y;
    double obstacle_pos_z;
    double obstacle_vel_x;
    double obstacle_vel_y;
    double obstacle_vel_z; double radius; int is_long;
} state_t;

typedef struct {
    double alpha;
    double max_vel;
    double max_vel_z;
} hyperparams_t;

extern "C" {
    state_t* new_state() {
        return new state_t();
    }

    state_t* update_pos(state_t* state,
                        double pos_x,
                        double pos_y,
                        double pos_z) {
        state->pos_x = pos_x;
        state->pos_y = pos_y;
        state->pos_z = pos_z;

        return state;
    }

    state_t* update_vel(state_t* state,
                        double vel_x,
                        double vel_y,
                        double vel_z) {
        state->vel_x = vel_x;
        state->vel_y = vel_y;
        state->vel_z = vel_z;

        return state;
    }

    state_t* update_time(state_t* state,
                         double time) {
        state->time = time;
        return state;
    }

    state_t* update_obstacle_pos(state_t* state,
                                 double pos_x,
                                 double pos_y,
                                 double pos_z) {
        state->obstacle_pos_x = pos_x;
        state->obstacle_pos_y = pos_y;
        state->obstacle_pos_z = pos_z;

        return state;
    }

    state_t* update_obstacle_vel(state_t* state,
                                 double vel_x,
                                 double vel_y,
                                 double vel_z) {
        state->obstacle_vel_x = vel_x;
        state->obstacle_vel_y = vel_y;
        state->obstacle_vel_z = vel_z;

        return state;
    }

    state_t* update_obstacle_radius(state_t* state, double radius) {
        state->radius = radius;
        return state;
    }

    state_t* update_is_obstacle_long(state_t* state, int is_long) {
        state->is_long = is_long;
        return state;
    }

    hyperparams_t* new_hyperparams() {
        return new hyperparams_t();
    }

    hyperparams_t* update_alpha(hyperparams_t* hyperparams, double alpha) {
        hyperparams->alpha = alpha;
        return hyperparams;
    }

    hyperparams_t* update_max_vel(hyperparams_t* hyperparams, double max_vel) {
        hyperparams->max_vel = max_vel;
        return hyperparams;
    }

    hyperparams_t* update_max_vel_z(hyperparams_t* hyperparams, double max_vel_z) {
        hyperparams->max_vel_z = max_vel_z;
        return hyperparams;
    }
}

#endif // CBF_REQS_H_
