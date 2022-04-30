#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "../../default_robot_node.hpp"

#define ROBOT_NAME "Eval_TouchSensor"
#define TOUCH_SENSOR_PORT "PORT_2"

class Robot: public DefaultRobotNode {
    private:
        bool is_touched = false;
        int counter = 0;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_touch_sensor;

        void touch_sensor_callback(const std_msgs::msg::Bool::SharedPtr touch_sensor_data);

    protected:
        void forwards() override;
        void stop() override;
        void backturn();

        void step_callback(const std_msgs::msg::Empty::SharedPtr) override;

    public:
        Robot();
};

#endif