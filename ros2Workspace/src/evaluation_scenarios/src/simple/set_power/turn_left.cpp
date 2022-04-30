#include <memory>
#include <vector>
#include <math.h>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"

#include "set_power_node.hpp"
#include "../constants.hpp"
#include "../../default_robot_node.hpp"
#include "../../roboter_parameter.hpp"

using namespace std;
using std::placeholders::_1;

/*
    Evaluate Set Power with left Turn
 */

SetPowerNode::SetPowerNode(): DefaultRobotNode("set_power_turn_left_node", ROBOT_NAME) {}

void SetPowerNode::step_callback(const std_msgs::msg::Empty::SharedPtr) {
    if ( this->counter > MAX_STEPS) {
        this->publish_finished();
    }

    if ( this->should_send ) {
        if ( this->counter > MAX_STEPS ) {
            this->stop();
        } else {
            this->left();
        }
    }

    this->should_send = !this->should_send;

    this->counter++;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SetPowerNode>());
    rclcpp::shutdown();

    return 0;
}