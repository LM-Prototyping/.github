#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "../../default_robot_node.hpp"

#define ROBOT_NAME "Eval_NoSensors"
#define COMPASS_SENSOR_PORT "PORT_1"

enum State {
    FORWARD,
    ROTATE_90,
    ROTATE_180,
    ROTATE_270,
    STOP
};


class Robot: public DefaultRobotNode {
    private:
        float compass_sensor_data = -1;

        State states[8] = { 
            FORWARD, 
            ROTATE_90, 
            FORWARD, 
            ROTATE_180,
            FORWARD,
            ROTATE_270,
            FORWARD,
            STOP
        };
        int state_counter = 0;
        int step_counter = 0;

        bool is_turning = false;
        float desired_angle = 0.0;

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_compass_sensor;

        void compass_sensor_callback(const std_msgs::msg::Float32::SharedPtr compass_sensor_data);
        bool handle_desired_angle_reached();

        void handle_rotation(float rotation_angle);

    protected:
        void forwards() override; 
        void stop() override;
        void left() override;

        void step_callback(const std_msgs::msg::Empty::SharedPtr) override;

    public:
        Robot();
};

#endif