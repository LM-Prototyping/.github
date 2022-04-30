#include <memory>
#include <vector>
#include <math.h>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"

#include "set_position_node.hpp"
#include "../constants.hpp"
#include "../../roboter_parameter.hpp"
#include "../../default_robot_node.hpp"

using namespace std;
using std::placeholders::_1;

/*
    Evaluate Set Position with left Turn

    4pi forwards
    2pi left
    4pi forwards
    2pi left
    4pi forwards
 */

SetPositionNode::SetPositionNode(): DefaultRobotNode("set_position_turn_left_node", ROBOT_NAME) {
    this->init_publishers();
}

void SetPositionNode::step_callback(const std_msgs::msg::Empty::SharedPtr) {
    if ( this->counter >= 5 ) {
        return this->publish_finished();
    } 

    if ( this->should_send ) {
        this->handle_current_direction();
    }

    bool has_finished = false;

    if ( 
        this->direction == FORWARD 
        && this->current_position_left >= 6 * M_PI
        && this->current_position_right >= 6 * M_PI 
    ) {
        this->direction = LEFT;
        has_finished = true;
    } else if ( this->direction == LEFT && this->current_position_right >= 4 * M_PI) {
        this->direction = FORWARD;
        has_finished = true;
    }

    if ( has_finished ) {
        this->offset_position();
        
        this->current_position_right = 0;
        this->current_position_left = 0;

        this->counter++;
    }

    this->should_send = !this->should_send;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SetPositionNode>());
    rclcpp::shutdown();

    return 0;
}