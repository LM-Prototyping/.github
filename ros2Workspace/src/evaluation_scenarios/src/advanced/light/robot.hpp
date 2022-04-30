#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

#include "../../default_robot_node.hpp"

#define ROBOT_NAME "Eval_LightSensor"
#define LIGHT_SENSOR_PORT "PORT_2"

class Robot: public DefaultRobotNode {
    private:
        int light_sensor_data = -1;
        bool has_finished = false;

        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_light_sensor;

        void light_sensor_callback(const std_msgs::msg::Int16::SharedPtr light_sensor_data);

    protected:
        void forwards() override; 
        void stop() override;

        void step_callback(const std_msgs::msg::Empty::SharedPtr) override;

    public:
        Robot();
};

#endif