#include "pcie_bridge_node.h"

int main(int argc, char **argv)
{
    for(int i = 0; i < argc; i++){
        LOGPF("argv[%d] = %s\n", i, argv[i]);
    }

    _print_ros_env_();
    rclcpp::init(argc, argv);
    // _run_logger_test_();
    rclcpp::executors::MultiThreadedExecutor executor;

    auto g_pcie_bridge_node = std::make_shared<PcieBridgeNode>();

    executor.add_node(g_pcie_bridge_node);

    std::string config_file_path = argv[1];
    RCLCPP_INFO(g_pcie_bridge_node->get_logger(), "config_file_path: %s\n", argv[1]);

    g_pcie_bridge_node -> init(config_file_path);
    // g_pcie_bridge_node -> run_test_loop();

    executor.spin();
    rclcpp::shutdown();
    g_pcie_bridge_node -> destroy();

    return 0;
}
