#pragma once
#ifndef LOGGING_UTILS
#define LOGGING_UTILS

//* set non-debug mode *//
#ifndef NDEBUG
 #define NDEBUG 
#endif

#include <stdio.h>      /* printf */
#include <stdlib.h>     /* getenv */

//* log with fprintf *//
#define LOGPF(format, ...) fprintf(stderr ,"[%s:%d] " format "\n", __FILE__, __LINE__, ##__VA_ARGS__)

/* 1 for ROS, 2 for ROS2 */
#define ROS_LOG_STYLE 2

#if ROS_LOG_STYLE == 2
    #include <rclcpp/logging.hpp>
    #include <rclcpp/rclcpp.hpp>
    //* this should be a rclcpp::Node derived class *//
    #define RLOGD(...) RCLCPP_DEBUG(this->get_logger(), ##__VA_ARGS__)
    #define RLOGI(...) RCLCPP_INFO(this->get_logger(), ##__VA_ARGS__)
    #define RLOGW(...) RCLCPP_WARN(this->get_logger(), ##__VA_ARGS__)
    #define RLOGE(...) RCLCPP_ERROR(this->get_logger(), ##__VA_ARGS__)
    #define RLOGF(...) RCLCPP_FATAL(this->get_logger(), ##__VA_ARGS__)

    void _run_logger_test_(){
        class Ros2LoggerTest : public rclcpp::Node
        {
        public:
            Ros2LoggerTest() : Node("Ros2LoggerTest")
            {
                RLOGD("this is debug");
                RLOGI("this is info");
                RLOGW("this is warn");
                RLOGE("this is error");
                RLOGF("this is fatal");
            }
        };

        rclcpp::Node::SharedPtr _node = std::make_shared<Ros2LoggerTest>();
    }
#elif ROS_LOG_STYLE == 1
    #include <ros/ros.h>
    #define RLOGD(...) ROS_DEBUG(__VA_ARGS__)
    #define RLOGI(...) ROS_INFO(__VA_ARGS__)
    #define RLOGW(...) ROS_WARN(__VA_ARGS__)
    #define RLOGE(...) ROS_ERROR(__VA_ARGS__)
    #define RLOGF(...) ROS_FATAL(__VA_ARGS__)

    void _run_logger_test_(){
        RLOGD("this is debug");
        RLOGI("this is info");
        RLOGW("this is warn");
        RLOGE("this is error");
        RLOGF("this is fatal");
    }
#endif

static inline void _print_ros_env_(){
    LOGPF("\n*** ROS_VERSION: %s ***\n", getenv("ROS_VERSION"));
    LOGPF("\n*** ROS_DISTRO: %s ***\n", getenv("ROS_DISTRO"));
    LOGPF("\n*** ROS_PYTHON_VERSION: %s ***\n", getenv("ROS_PYTHON_VERSION"));
}

#endif