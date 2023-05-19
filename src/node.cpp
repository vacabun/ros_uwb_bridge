#include "rclcpp/rclcpp.hpp"

#include "node.hpp"

int main(int argc, const char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SerialPortReader>());
	rclcpp::shutdown();

	return 0;
}


