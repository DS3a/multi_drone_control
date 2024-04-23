#include "ros/ros.h"
#include "mav_msgs/Actuators.h"
#include "multi_drone_control/torque_thrust_cmd.h"
#include "multi_drone_control/rate_thrust_cmd.h"
#include "multi_drone_control/attitude_thrust_cmd.h"
#include "sensor_msgs/Imu.h"
#include <memory.h>
#include <math.h>
#include <stdio.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "roll_rate_Controller_ert_rtw/roll_rate_Controller.h"
#include "pitch_rate_Controller_ert_rtw/pitch_rate_Controller.h"
#include "yaw_rate_Controller_ert_rtw/yaw_rate_Controller.h"
#include "roll_pid_ert_rtw/roll_pid.h"
#include "pitch_pid_ert_rtw/pitch_pid.h"
#include "yaw_pid_ert_rtw/yaw_pid.h"


//// PARAMETERS

#define ARM_LENGTH 0.17
// arm length in meters

#define SIM_SLOWDOWN_FACTOR 10

#define ROTOR_RADIUS 0.1
// radius of rotor in m


#define MOTOR_CONSTANT 8.54858e-06
// motor constant in kg m/s^2


#define MOMENT_CONSTANT 0.016

#define THRUST_TO_ANG_VEL(rotor_thrust) ((1/(ROTOR_RADIUS*SIM_SLOWDOWN_FACTOR))*sqrt(rotor_thrust/MOTOR_CONSTANT))


#define MIN_ANG_VEL 1
#define CHECK_MIN_ANG_VEL(value) ((value > MIN_ANG_VEL) ? value : MIN_ANG_VEL)



std::shared_ptr<float> cmd_thrust;
std::shared_ptr<float> cmd_rate_x;
std::shared_ptr<float> cmd_rate_y;
std::shared_ptr<float> cmd_rate_z;
std::shared_ptr<float> cmd_torque_x;
std::shared_ptr<float> cmd_torque_y;
std::shared_ptr<float> cmd_torque_z;

std::shared_ptr<float> cmd_yaw;
std::shared_ptr<float> cmd_pitch;
std::shared_ptr<float> cmd_roll;

std::shared_ptr<float> roll_rate;
std::shared_ptr<float> pitch_rate;
std::shared_ptr<float> yaw_rate;

std::shared_ptr<float> eul_yaw;
std::shared_ptr<float> eul_pitch;
std::shared_ptr<float> eul_roll;

void cmd_callback(const multi_drone_control::attitude_thrust_cmd::ConstPtr& msg) {
    *cmd_thrust = msg->thrust.data;
    *cmd_yaw = msg->yaw.data;
    *cmd_pitch = msg->pitch.data;
    *cmd_roll = msg->roll.data;
    // *cmd_rate_x = msg->roll_rate.data;
    // *cmd_rate_y = msg->pitch_rate.data;
    // *cmd_rate_z = msg->yaw_rate.data;

    // *cmd_torque_x = msg->torque_x.data;
    // *cmd_torque_y = msg->torque_y.data;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    *roll_rate = msg->angular_velocity.x;
    *pitch_rate = msg->angular_velocity.y;
    *yaw_rate = msg->angular_velocity.z;

    Eigen::Quaterniond q;
    q.w() = msg->orientation.w;
    q.x() = msg->orientation.x;
    q.y() = msg->orientation.y;
    q.z() = msg->orientation.z;

    auto eul = q.toRotationMatrix().eulerAngles(2, 1, 0);
    eul_yaw = eul[0];
    eul_pitch = eul[1];
    eul_roll = eul[2];
}


int main(int argc, char **argv) {
    cmd_thrust = std::make_shared<float>(0.0);
    cmd_torque_x = std::make_shared<float>(0.0);
    cmd_torque_y = std::make_shared<float>(0.0);
    cmd_torque_z = std::make_shared<float>(0.0);
    cmd_rate_x = std::make_shared<float>(0.0);
    cmd_rate_y = std::make_shared<float>(0.0);
    cmd_rate_z = std::make_shared<float>(0.0);

    roll_rate = std::make_shared<float>(0.0);
    pitch_rate = std::make_shared<float>(0.0);
    yaw_rate = std::make_shared<float>(0.0);

    roll_rate_Controller_initialize();
    pitch_rate_Controller_initialize();
    yaw_rate_Controller_initialize();
    roll_pid_initialize();
    pitch_pid_initialize();
    yaw_pid_initialize();

    ros::init(argc, argv, "attitude_thrust_controller_node");
    ros::NodeHandle n;
    std::string ns = ros::this_node::getNamespace();

    ros::Publisher motor_cmd_publisher =
        n.advertise<mav_msgs::Actuators>(ns+"/command/motor_speed", 1);

    ros::Subscriber sub = n.subscribe(ns+"/drone_cmd", 1, cmd_callback);
    ros::Subscriber imu_sub = n.subscribe(ns+"/ground_truth/imu", 1, imu_callback);

    ros::Rate cmd_publisher_rate(100);

    float motor_front_ang_vel=0, motor_back_ang_vel=0,
        motor_left_ang_vel=0, motor_right_ang_vel=0;

    float motor_front_thrust=0, motor_back_thrust=0,
        motor_left_thrust=0, motor_right_thrust;

    while (ros::ok()) {
        ros::spinOnce();

        roll_rate_Controller_U.u = (*cmd_rate_x) - (*roll_rate);
        roll_rate_Controller_step();
        if (std::isnan(roll_rate_Controller_Y.y)) {
            *cmd_torque_x = 0;
        } else {
            *cmd_torque_x = roll_rate_Controller_Y.y;
        }
        printf("the roll command is %f\n", *cmd_torque_x);

        pitch_rate_Controller_U.u = (*cmd_rate_y) - (*pitch_rate);
        pitch_rate_Controller_step();

        if (std::isnan(pitch_rate_Controller_Y.y)) {
            *cmd_torque_y = 0;
        }
        else {
          *cmd_torque_y = pitch_rate_Controller_Y.y;
        }
        printf("the pitch command is %f\n", *cmd_torque_y);

        yaw_rate_Controller_U.u = (*cmd_rate_z) - (*yaw_rate);
        yaw_rate_Controller_step();
        if (std::isnan(yaw_rate_Controller_Y.y)) {
            *cmd_torque_z = 0;
        }
        else {
          *cmd_torque_z = yaw_rate_Controller_Y.y;
        }
        printf("the yaw command is %f\n", *cmd_torque_z);


        motor_front_thrust = *cmd_thrust / 4;
        motor_back_thrust = *cmd_thrust / 4;
        motor_left_thrust = *cmd_thrust / 4;
        motor_right_thrust = *cmd_thrust / 4;

        motor_front_thrust -= (*cmd_torque_y) / (2 * ARM_LENGTH);
        motor_back_thrust += (*cmd_torque_y) / (2 * ARM_LENGTH);

        motor_left_thrust += (*cmd_torque_x) / (2 * ARM_LENGTH);
        motor_right_thrust -= (*cmd_torque_x) / (2 * ARM_LENGTH);


        motor_front_thrust += (*cmd_torque_z) / (MOMENT_CONSTANT*4);
        motor_back_thrust += (*cmd_torque_z) / (MOMENT_CONSTANT*4);

        motor_left_thrust -= (*cmd_torque_z) / (MOMENT_CONSTANT*4);
        motor_right_thrust -= (*cmd_torque_z) / (MOMENT_CONSTANT*4);



        // printf("motor right thrust %f\n", motor_right_thrust);
        motor_front_ang_vel = THRUST_TO_ANG_VEL(motor_front_thrust);
        motor_back_ang_vel = THRUST_TO_ANG_VEL(motor_back_thrust);
        motor_right_ang_vel = THRUST_TO_ANG_VEL(motor_right_thrust);
        motor_left_ang_vel = THRUST_TO_ANG_VEL(motor_left_thrust);

        // if (std::isnan(motor_front_ang_vel)) motor_front_ang_vel = 0;
        // if (std::isnan(motor_back_ang_vel)) motor_front_ang_vel = 0;
        // if (std::isnan(motor_left_ang_vel)) motor_front_ang_vel = 0;
        // if (std::isnan(motor_right_ang_vel)) motor_front_ang_vel = 0;


        mav_msgs::Actuators actuator_msg;
        // actuator_msg.angular_velocities.clear();
        actuator_msg.angular_velocities.push_back(CHECK_MIN_ANG_VEL(motor_front_ang_vel));
        actuator_msg.angular_velocities.push_back(CHECK_MIN_ANG_VEL(motor_left_ang_vel));
        actuator_msg.angular_velocities.push_back(CHECK_MIN_ANG_VEL(motor_back_ang_vel));
        actuator_msg.angular_velocities.push_back(CHECK_MIN_ANG_VEL(motor_right_ang_vel));

        motor_cmd_publisher.publish(actuator_msg);

        cmd_publisher_rate.sleep();
    }
    return 0;
}
