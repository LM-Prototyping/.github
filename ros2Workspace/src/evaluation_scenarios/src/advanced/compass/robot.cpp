#include <memory>
#include <vector>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "robot.hpp"
#include "../../default_robot_node.hpp"
#include "../../roboter_parameter.hpp"

using namespace std;
using std::placeholders::_1;

/*
    Evaluate Light Sensor

    Robot should drive straight until Black ground is reached

 */

Robot::Robot(): DefaultRobotNode("compass_sensor_node", ROBOT_NAME) {
    this->sub_compass_sensor = this->create_subscription<std_msgs::msg::Float32>(
        ((string) ROBOT_NAME) + "/" + COMPASS_SENSOR_PORT + "/compass",
        10,
        bind(&Robot::compass_sensor_callback, this, _1) 
    );
}

void Robot::compass_sensor_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    this->compass_sensor_data = msg->data;
}

void Robot::step_callback(const std_msgs::msg::Empty::SharedPtr) {
    if ( !this->should_send ) {
        this->should_send = !this->should_send;
        return;
    }

    State current_state = this->states[this->state_counter];

    switch ( current_state ) {
        case FORWARD:   {
            if ( this->step_counter < 25 ) {
                this->forwards();
                this->step_counter++;
            } else {
                this->step_counter = 0;
                this->state_counter++;   
            }
            break;
        }
        case ROTATE_90: {
            this->handle_rotation(M_PI / 2);
            break;
        }
        case ROTATE_180: {
            this->handle_rotation(M_PI);
            break;
        }
        case ROTATE_270: {
            this->handle_rotation(M_PI + M_PI / 2);
            break;
        }
        case STOP: {
            this->stop();
            this->publish_finished();
            break;
        }
    }

    this->should_send = !this->should_send;
}

void Robot::handle_rotation(float rotation_angle) {
    if ( !this->is_turning ) {
        this->is_turning = true;
        this->desired_angle = fmod(this->compass_sensor_data + rotation_angle, (float) (2 * M_PI));
        this->left();
    } else {
        if ( abs(this->desired_angle - this->compass_sensor_data) < 1.0 / 36.0 * M_PI ) {
            this->state_counter++;
            this->is_turning = false;
        }
    }
}

void Robot::forwards() {
    this->publish_velocity(this->pub_right_wheel_velocity, VEL_LIMIT / 2);
    this->publish_velocity(this->pub_left_wheel_velocity, VEL_LIMIT / 2);
}

void Robot::stop() {
    this->publish_velocity(this->pub_right_wheel_velocity, 0);
    this->publish_velocity(this->pub_left_wheel_velocity, 0);
}

void Robot::left() {
    this->publish_velocity(this->pub_right_wheel_velocity, -VEL_LIMIT / 20);
    this->publish_velocity(this->pub_left_wheel_velocity, VEL_LIMIT / 20);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Robot>());    
    rclcpp::shutdown();

    return 0;
}