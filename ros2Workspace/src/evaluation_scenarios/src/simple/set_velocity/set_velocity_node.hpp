#ifndef SET_VELOCITY_NODE_HPP
#define SET_VELOCITY_NODE_HPP

#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "../../default_robot_node.hpp"
#include "../../roboter_parameter.hpp"

#include <string>

#define MAX_STEPS 1000

class SetVelocityNode: public DefaultRobotNode {
    private:
        int counter = 0;

        float outer_wheel_velocity = VEL_LIMIT / 2;
        float inner_wheel_velocity = VEL_LIMIT / 3;
        
        void stop();

    protected:
        void forwards() override;
        void left() override;
        void right() override;

        void step_callback(const std_msgs::msg::Empty::SharedPtr _) override;

    public:
        SetVelocityNode();

};

#endif