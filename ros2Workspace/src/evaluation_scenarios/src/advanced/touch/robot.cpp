#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "robot.hpp"
#include "../../default_robot_node.hpp"
#include "../../roboter_parameter.hpp"

using namespace std;
using std::placeholders::_1;

/*
    Evaluate Light Sensor

    Robot should drive straight until Black ground is reached
    After that, robot should drive backwards turn for 100 steps
 
 */

Robot::Robot(): DefaultRobotNode("touch_sensor_node", ROBOT_NAME) {
    this->sub_touch_sensor = this->create_subscription<std_msgs::msg::Bool>(
        ((string) ROBOT_NAME) + "/" + TOUCH_SENSOR_PORT + "/touch",
        10,
        bind(&Robot::touch_sensor_callback, this, _1) 
    );
}

void Robot::touch_sensor_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    this->is_touched = msg->data;
}

void Robot::step_callback(const std_msgs::msg::Empty::SharedPtr) {
    if ( !this->should_send ) {
        this->should_send = !this->should_send;
        return;
    }
    
    if ( !this->is_touched ) {
        this->forwards();
    } else {
        this->stop();
        this->publish_finished();
    }

    this->should_send = !this->should_send;
}

void Robot::forwards() {
    this->publish_velocity(this->pub_right_wheel_velocity, VEL_LIMIT / 2);
    this->publish_velocity(this->pub_left_wheel_velocity, VEL_LIMIT / 2);
}

void Robot::backturn() {
    this->publish_velocity(this->pub_right_wheel_velocity, -VEL_LIMIT / 2);
    this->publish_velocity(this->pub_left_wheel_velocity, VEL_LIMIT / 3);
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