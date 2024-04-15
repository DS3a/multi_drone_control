#include "ros/ros.h"
#include "mav_msgs/Actuators.h"
#include "multi_drone_control/torque_thrust_cmd.h"
#include <memory.h>
#include <math.h>
#include <stdio.h>


//// PARAMETERS

#define ARM_LENGTH 0.17
// arm length in meters

#define SIM_SLOWDOWN_FACTOR 10

#define ROTOR_RADIUS 0.1
// radius of rotor in m

#define MOTOR_CONSTANT 8.54858e-06
// motor constant in kg m/s^2

#define THRUST_TO_ANG_VEL(rotor_thrust) ((1/(ROTOR_RADIUS*SIM_SLOWDOWN_FACTOR))*sqrt(rotor_thrust/MOTOR_CONSTANT))


std::shared_ptr<float> cmd_thrust;
std::shared_ptr<float> cmd_torque_x;
std::shared_ptr<float> cmd_torque_y;

void cmd_callback(const multi_drone_control::torque_thrust_cmd::ConstPtr& msg) {
    *cmd_thrust = msg->thrust.data;
    *cmd_torque_x = msg->torque_x.data;
    *cmd_torque_y = msg->torque_y.data;
}


int main(int argc, char **argv) {
    cmd_thrust = std::make_shared<float>(0.0);
    cmd_torque_x = std::make_shared<float>(0.0);
    cmd_torque_y = std::make_shared<float>(0.0);


    ros::init(argc, argv, "torque_thrust_controller_node");
    ros::NodeHandle n;
    std::string ns = ros::this_node::getNamespace();

    ros::Publisher motor_cmd_publisher =
        n.advertise<mav_msgs::Actuators>(ns+"/gazebo/command/motor_speed", 1);

    ros::Subscriber sub = n.subscribe(ns+"/drone_cmd", 1, cmd_callback);

    ros::Rate cmd_publisher_rate(200);

    float motor_front_ang_vel=0, motor_back_ang_vel=0,
        motor_left_ang_vel=0, motor_right_ang_vel=0;

    float motor_front_thrust=0, motor_back_thrust=0,
        motor_left_thrust=0, motor_right_thrust;

    while (ros::ok()) {
        ros::spinOnce();
        motor_front_thrust = *cmd_thrust / 4;
        motor_back_thrust = *cmd_thrust / 4;
        motor_left_thrust = *cmd_thrust / 4;
        motor_right_thrust = *cmd_thrust / 4;

        motor_front_thrust -= (*cmd_torque_y) / (2 * ARM_LENGTH);
        motor_back_thrust += (*cmd_torque_y) / (2 * ARM_LENGTH);

        motor_left_thrust += (*cmd_torque_x) / (2 * ARM_LENGTH);
        motor_right_thrust -= (*cmd_torque_x) / (2 * ARM_LENGTH);

        // printf("motor right thrust %f\n", motor_right_thrust);
        motor_front_ang_vel = THRUST_TO_ANG_VEL(motor_front_thrust);
        motor_back_ang_vel = THRUST_TO_ANG_VEL(motor_back_thrust);
        motor_right_ang_vel = THRUST_TO_ANG_VEL(motor_right_thrust);
        motor_left_ang_vel = THRUST_TO_ANG_VEL(motor_left_thrust);

        mav_msgs::Actuators actuator_msg;
        // actuator_msg.angular_velocities.clear();
        actuator_msg.angular_velocities.push_back(motor_front_ang_vel);
        actuator_msg.angular_velocities.push_back(motor_left_ang_vel);
        actuator_msg.angular_velocities.push_back(motor_back_ang_vel);
        actuator_msg.angular_velocities.push_back(motor_right_ang_vel);

        motor_cmd_publisher.publish(actuator_msg);

        cmd_publisher_rate.sleep();
    }
    return 0;
}
