#include <memory>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <osqp++.h>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;


#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "multi_drone_control/attitude_thrust_cmd.h"


#include "multi_drone_control/obstacle.hpp"
#include "multi_drone_control/cbf_constraint.hpp"
#include "multi_drone_control/hocbf_barrier_fn.hpp"
#include "multi_drone_control/vel_barriers.hpp"


typedef CGAL::Quadratic_program<ET> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;


#define UAV_MASS 0.825

const double kInfinity = std::numeric_limits<double>::infinity();


std::shared_ptr<Eigen::Vector3d> uav_pos;
std::shared_ptr<Eigen::Vector3d> uav_vel;
std::shared_ptr<cbf_constraint::state_t> uav_state;
std::shared_ptr<cbf_constraint::hyperparams_t> cbf_hyperparams;

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    uav_pos->x() = msg->pose.pose.position.x;
    uav_pos->y() = msg->pose.pose.position.y;
    uav_pos->z() = msg->pose.pose.position.z;

    uav_vel->x() = msg->twist.twist.linear.x;
    uav_vel->y() = msg->twist.twist.linear.y;
    uav_vel->z() = msg->twist.twist.linear.z;
}


// TODO create a vector of obstacles

int main(int argc, char **argv) {
    uav_pos = std::make_shared<Eigen::Vector3d>(Eigen::Vector3d(0, 0, 0));
    uav_vel = std::make_shared<Eigen::Vector3d>(Eigen::Vector3d(0, 0, 0));

    cbf_hyperparams = std::make_shared<cbf_constraint::hyperparams_t>();
    cbf_hyperparams->alpha = 0.9;
    cbf_hyperparams->max_vel = 0.8;
    cbf_hyperparams->max_vel_z = 5;
    uav_state = std::make_shared<cbf_constraint::state_t>();
    uav_state->pos = uav_pos;
    uav_state->vel = uav_vel;
    uav_state->time = 0;

    ros::init(argc, argv, "single_drone_hocbf_node");
    std::string ns = ros::this_node::getNamespace();

    // ROS Node handles and publishers/subscribers
    ros::NodeHandle nh;
    ros::Publisher drone_cmd_pub =
        nh.advertise<multi_drone_control::attitude_thrust_cmd>(ns+"/drone_cmd", 1);
    ros::Subscriber obstacle_sub;  // TODO subscribe to the obstacles topic
    ros::Subscriber odometry_sub = nh.subscribe(ns+"/ground_truth/odometry", 1, odom_callback);


    ros::Rate cbf_cmd_publisher_rate(50);

    cbf_constraint::CBF_Constraint stl_hocbf(
        stl_hocbf::lhs_ax,
        stl_hocbf::lhs_ay,
        stl_hocbf::lhs_az,
        stl_hocbf::rhs);

    cbf_constraint::CBF_Constraint vx_min_cbf(
        vel_barriers::vx_min_bf::lhs_ax,
        vel_barriers::vx_min_bf::lhs_ay,
        vel_barriers::vx_min_bf::lhs_az,
        vel_barriers::vx_min_bf::rhs);

    cbf_constraint::CBF_Constraint vy_min_cbf(
        vel_barriers::vy_min_bf::lhs_ax,
        vel_barriers::vy_min_bf::lhs_ay,
        vel_barriers::vy_min_bf::lhs_az,
        vel_barriers::vy_min_bf::rhs);

    cbf_constraint::CBF_Constraint vz_min_cbf(
        vel_barriers::vz_min_bf::lhs_ax,
        vel_barriers::vz_min_bf::lhs_ay,
        vel_barriers::vz_min_bf::lhs_az,
        vel_barriers::vz_min_bf::rhs);

    cbf_constraint::CBF_Constraint vx_max_cbf(
        vel_barriers::vx_max_bf::lhs_ax,
        vel_barriers::vx_max_bf::lhs_ay,
        vel_barriers::vx_max_bf::lhs_az,
        vel_barriers::vx_max_bf::rhs);

    cbf_constraint::CBF_Constraint vy_max_cbf(
        vel_barriers::vy_max_bf::lhs_ax,
        vel_barriers::vy_max_bf::lhs_ay,
        vel_barriers::vy_max_bf::lhs_az,
        vel_barriers::vy_max_bf::rhs);

    cbf_constraint::CBF_Constraint vz_max_cbf(
        vel_barriers::vz_max_bf::lhs_ax,
        vel_barriers::vz_max_bf::lhs_ay,
        vel_barriers::vz_max_bf::lhs_az,
        vel_barriers::vz_max_bf::rhs);


    std::vector<cbf_constraint::CBF_Constraint> cbfs = {
        stl_hocbf,
        vx_min_cbf,
        vy_min_cbf,
        vz_min_cbf,
        vx_max_cbf,
        vy_max_cbf,
        vz_max_cbf
    };

    Eigen::Matrix3d I;
    I << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;
    // std::cout << I << std::endl;

    bool taken_off = false;
    Eigen::SparseMatrix<double> objective_matrix(3, 3);
    const Eigen::Triplet<double> kTripletsP[] = {
        {0, 0, 2.0}, {1, 1, 2.0}, {2, 2, 2.0}};
    objective_matrix.setFromTriplets(std::begin(kTripletsP),
                                     std::end(kTripletsP));

    Eigen::SparseMatrix<double> constraint_matrix(cbfs.size(), 3);
    std::vector<double> lower_bounds(cbfs.size(), -kInfinity);
    std::vector<double> upper_bounds(cbfs.size(), kInfinity);

    auto start = std::chrono::high_resolution_clock::now();

    while (ros::ok()) {
        ros::spinOnce();
        if (!taken_off) {
            multi_drone_control::attitude_thrust_cmd msg;
            msg.thrust.data = 7.3;
            drone_cmd_pub.publish(msg);

            if (uav_pos->z() >= 1.0) {
                taken_off = true;
                start = std::chrono::high_resolution_clock::now();
            }
            continue;
        }

       std::shared_ptr<Program> qp = std::make_shared<Program>(CGAL::SMALLER, true, -0.5, true, 0.5);

        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start);
        uav_state->time = double(duration.count()) / 1000000.0;
        qp->set_d(AX_IDX, AX_IDX, 2);
        qp->set_d(AY_IDX, AY_IDX, 2);
        qp->set_d(AZ_IDX, AZ_IDX, 2);

         std::vector<Eigen::Triplet<double>> kTripletsA;
        cbf_hyperparams->alpha = 0.99;
        // stl_hocbf.add_to_qp(qp, uav_state, cbf_hyperparams, 0, 1.0);
        kTripletsA.push_back({0, AX_IDX, cbfs[0].get_lhs_ax(uav_state, cbf_hyperparams)});
        kTripletsA.push_back({0, AY_IDX, cbfs[0].get_lhs_ay(uav_state, cbf_hyperparams)});
        kTripletsA.push_back({0, AZ_IDX, cbfs[0].get_lhs_az(uav_state, cbf_hyperparams)});
        upper_bounds[0] = cbfs[0].get_rhs(uav_state, cbf_hyperparams) - 5.0;
        // cbf_hyperparams->alpha = 0.99;
        // for (int i=1; i<cbfs.size(); i++) {
        //     kTripletsA.push_back({i, AX_IDX, cbfs[i].get_lhs_ax(uav_state, cbf_hyperparams)});
        //     kTripletsA.push_back({i, AY_IDX, cbfs[i].get_lhs_ay(uav_state, cbf_hyperparams)});
        //     kTripletsA.push_back({i, AZ_IDX, cbfs[i].get_lhs_az(uav_state, cbf_hyperparams)});
        //     upper_bounds[i] = cbfs[i].get_rhs(uav_state, cbf_hyperparams) + 3;
        // }

        constraint_matrix.setFromTriplets(std::begin(kTripletsA), std::end(kTripletsA));

        // vx_min_cbf.add_to_qp(qp, uav_state, cbf_hyperparams, 1, 0.5);
        // vy_min_cbf.add_to_qp(qp, uav_state, cbf_hyperparams, 2, 0.5);
        // vz_min_cbf.add_to_qp(qp, uav_state, cbf_hyperparams, 3, 0.5);
        // vx_max_cbf.add_to_qp(qp, uav_state, cbf_hyperparams, 4, 0.5);
        // vy_max_cbf.add_to_qp(qp, uav_state, cbf_hyperparams, 5, 0.5);
        // vz_max_cbf.add_to_qp(qp, uav_state, cbf_hyperparams, 6, 0.5);

        // uav_state->time = 10;
        // Solution s = CGAL::solve_quadratic_program(*qp, ET());
        osqp::OsqpInstance instance;
        instance.objective_matrix = objective_matrix;
        instance.objective_vector.resize(3);
        instance.objective_vector << 0.0, 0.0, 0.0;
        instance.constraint_matrix = constraint_matrix;
        instance.lower_bounds.resize(cbfs.size());
        instance.upper_bounds.resize(cbfs.size());
        for (int i=0; i<cbfs.size(); i++) {
            instance.lower_bounds[i] = -kInfinity;
            instance.upper_bounds[i] = upper_bounds[i];
        }

        osqp::OsqpSolver solver;
        osqp::OsqpSettings settings;

        auto status = solver.Init(instance, settings);

        osqp::OsqpExitCode exit_code = solver.Solve();
        double optimal_objective = solver.objective_value();
        Eigen::VectorXd optimal_solution = solver.primal_solution();

        Eigen::Vector3d acc = optimal_solution;
        // int i = 0;
        // for (auto it = s.variable_values_begin(); it != s.variable_values_end();
        //      ++it, i++) {
        //     double soln = CGAL::to_double(*it);
        //     acc[i] = soln;
        //     // printf("the %dth value is %f\n\n", i, soln);
        // }
        // std::cout << "the solution is " << acc << std::endl;

        const double max_acc = 0.3;
        if (acc.z() > 5)
            acc.z() = 5;
        if (acc.z() < 0)
            acc.z() = 0;

        if (acc.x() > max_acc)
            acc.x() = max_acc;
        if (acc.x() < -max_acc)
            acc.x() = -max_acc;

        if (acc.y() > max_acc)
            acc.y() = max_acc;
        if (acc.y() < -max_acc)
            acc.y() = -max_acc;


        Eigen::Vector3d vector_thrust = acc * UAV_MASS;
        double thrust = vector_thrust.norm();
        if (thrust > 7.3)
            thrust = 7.3;
        // if (thrust != 0.0)
        //     vector_thrust /= thrust;
        vector_thrust.normalize();
        Eigen::Vector3d u_Z(0, 0, 1.0);

        Eigen::Vector3d v = u_Z.cross(vector_thrust);
        double c = u_Z.dot(vector_thrust);
        Eigen::Matrix3d vx;
        vx << 0, -v(2), v(1),
              v(2), 0, -v(0),
              -v(1), v(0), 0;

        Eigen::Matrix3d rot = I + vx + (vx*vx/(1.0 + c));
        rot.normalize();
        // Eigen::Vector3d euler_ypr = rot.eulerAngles(2, 1, 0);

        Eigen::Vector3d euler_ypr;
        euler_ypr(0) = std::atan2(rot(2, 1), rot(2, 2));  // roll
        euler_ypr(1) = std::atan2(-rot(2, 0), std::sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2)));  // pitch
        euler_ypr(2) = std::atan2(rot(1, 0), rot(0, 0));  // yaw

        std::cout << "the command at time " << uav_state->time << " is ";
        std::cout << thrust << " " << euler_ypr << std::endl;
        // printf("the solution is %f %f %f", s[AX_IDX], s[AY_IDX], s[AZ_IDX]);
        // printf("the values of the rhs is %f, the lhs_ax is %f\n", stl_rhs, stl_lhs_ax);
        multi_drone_control::attitude_thrust_cmd msg;
        msg.thrust.data = thrust;
        msg.roll.data = euler_ypr.x();
        msg.pitch.data = euler_ypr.y();
        msg.yaw.data = euler_ypr.z();
        drone_cmd_pub.publish(msg);
        cbf_cmd_publisher_rate.sleep();
    }

    return 0;
}
