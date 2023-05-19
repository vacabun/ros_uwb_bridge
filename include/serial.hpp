#ifndef _SERIAL_PORT_HPP_
#define _SERIAL_PORT_HPP_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "jsoncpp/json/json.h"
#include "uwb_interfaces/msg/uwb_data.hpp"
#include "uwb_interfaces/msg/uwb_distance.hpp"
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

class SerialPortReader : public rclcpp::Node
{
public:
    SerialPortReader();

private:
    int open_serial(const char *__file, unsigned short int __baud);
    void timer_callback();
    int fd = 0;

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<uint8_t> jsonBuf;
    rclcpp::Publisher<uwb_interfaces::msg::UWBData>::SharedPtr msgPublisher_;
};

#endif
