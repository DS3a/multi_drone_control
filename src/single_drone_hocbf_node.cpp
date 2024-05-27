#include <memory>
#include <Eigen/Dense>
#include <Eigen/Core>
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
    uav_pos = std::make_shared<Eigen::Vector3d>(Eigen::Vector3d(1, 2, 3));
    uav_vel = std::make_shared<Eigen::Vector3d>(Eigen::Vector3d(0, 1, 0));

    cbf_hyperparams = std::make_shared<cbf_constraint::hyperparams_t>();
    cbf_hyperparams->alpha = 0.9;
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
    
    while (ros::ok()) {
        ros::spinOnce();

        /*
        ** TODO
        ** start putting cbf constraints in a vector
         */

        double stl_rhs = stl_hocbf::rhs(*uav_state, *cbf_hyperparams);
        double stl_lhs_ax = stl_hocbf::lhs_ax(*uav_state, *cbf_hyperparams);
        double stl_lhs_ay = stl_hocbf::lhs_ay(*uav_state, *cbf_hyperparams);
        double stl_lhs_az = stl_hocbf::lhs_az(*uav_state, *cbf_hyperparams);

        printf("the values of the rhs is %f, the lhs_ax is %f\n", stl_rhs, stl_lhs_ax);
        multi_drone_control::attitude_thrust_cmd msg;
        drone_cmd_pub.publish(msg);
        cbf_cmd_publisher_rate.sleep();
    }

    
    return 0;
}
