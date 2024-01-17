#ifndef _BRIDGE_HPP_
#define _BRIDGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "jsoncpp/json/json.h"
#include "uwb_interfaces/msg/uwb_data.hpp"
#include "uwb_interfaces/msg/uwb_distance.hpp"
#include "uwb_interfaces/srv/uwb_measure.hpp"
#ifdef __cplusplus
extern "C"
{
#endif

#include <termios.h>
#include <fcntl.h>

#ifdef __cplusplus
}
#endif

#define SERIAL_BUF_SIZE (1024)

class UWBRosBridge : public rclcpp::Node
{
public:
    UWBRosBridge();

private:
    void config();
    int open_serial(const char *__file, unsigned short int __baud);
    int json_parse(std::string jsonStr);
    void timer_callback();
    void handle_service(const std::shared_ptr<uwb_interfaces::srv::UWBMeasure::Request> request,
                        std::shared_ptr<uwb_interfaces::srv::UWBMeasure::Response> response);
    int serial_fd = 0;

    bool response_flag = false;
    int src_addr;
    int dst_addr;
    int distance;

    std::string serial_file_path;
    int baud;
    std::string labelName;
    unsigned short int baud_flag;

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<uint8_t> jsonBuf;
    rclcpp::Publisher<uwb_interfaces::msg::UWBData>::SharedPtr msgPublisher_;
    rclcpp::Service<uwb_interfaces::srv::UWBMeasure>::SharedPtr service_;
};
#endif
