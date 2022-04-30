#ifndef SET_POWER_NODE_HPP
#define SET_POWER_NODE_HPP

#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "../../default_robot_node.hpp"
#include "../../roboter_parameter.hpp"

#include <string>

#define MAX_STEPS 1000

class SetPowerNode: public DefaultRobotNode {
    private:
        int counter = 0;

        int outer_wheel_power = 100 / 2;
        int inner_wheel_power = 100 / 3;
        
        void stop();

    protected:
        void forwards() override;
        void left() override;
        void right() override;

        void step_callback(const std_msgs::msg::Empty::SharedPtr _) override;

    public:
        SetPowerNode();

};

#endif