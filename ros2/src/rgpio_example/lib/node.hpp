#pragma once

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rgpio/input.hpp>
#include <rgpio/output.hpp>
#include <rutil/fmt.hpp>
#include <rutil/toml.hpp>
#include <string>

#define GPIO_CONFIG_FILE "/home/ros/rocketGPIO/config.toml"

namespace rgpio_example {
    using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class Node : public rclcpp_lifecycle::LifecycleNode {
    public:
        Node();
        ~Node();

    private:
        std::unique_ptr<rgpio::Input> sense;
        std::unique_ptr<rgpio::Output> pyro;

        LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &);
        LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &);
        LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
        LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
        LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

        void readConfig(toml::table toml);

        void deleteAllPointers();
    };
} // namespace rgpio_example
