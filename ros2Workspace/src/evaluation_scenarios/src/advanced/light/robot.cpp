#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

#include "robot.hpp"
#include "../../default_robot_node.hpp"
#include "../../roboter_parameter.hpp"

using namespace std;
using std::placeholders::_1;

/*
    Evaluate Light Sensor

    Robot should drive straight until Black ground is reached

 */

Robot::Robot(): DefaultRobotNode("light_sensor_node", ROBOT_NAME) {
    this->sub_light_sensor = this->create_subscription<std_msgs::msg::Int16>(
        ((string) ROBOT_NAME) + "/" + LIGHT_SENSOR_PORT + "/light",
        10,
        bind(&Robot::light_sensor_callback, this, _1) 
    );
}

void Robot::light_sensor_callback(const std_msgs::msg::Int16::SharedPtr msg) {
    this->light_sensor_data = msg->data;
}

void Robot::step_callback(const std_msgs::msg::Empty::SharedPtr) {
    if ( !this->should_send ) {
        this->should_send = !this->should_send;
        return;
    }

    if ( (this->light_sensor_data == -1 || this->light_sensor_data <= 2500) && !this->has_finished ) {
        this->forwards();
    } else {
        this->has_finished = true;
        this->stop();
        this->publish_finished();
    }

    this->should_send = !this->should_send;
}

void Robot::forwards() {
    this->publish_velocity(this->pub_right_wheel_velocity, VEL_LIMIT / 2);
    this->publish_velocity(this->pub_left_wheel_velocity, VEL_LIMIT / 2);
}

void Robot::stop() {
    this->publish_velocity(this->pub_right_wheel_velocity, 0);
    this->publish_velocity(this->pub_left_wheel_velocity, 0);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Robot>());    
    rclcpp::shutdown();

    return 0;
}