#include "rclcpp/rclcpp.hpp"
#include "meamr_drive_model/meamr_base.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MeamrBase>();

    if (node->Init() != 0)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize MeamrBase.");
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
