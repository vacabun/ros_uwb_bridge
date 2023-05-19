#include "serial.hpp"

SerialPortReader::SerialPortReader() : Node("serial_port_reader")
{

    this->declare_parameter("serial_file_path", "/dev/ttyACM1");
    this->declare_parameter("baud", 115200);
    this->declare_parameter("label_name", "tag_0");
    std::string serial_file_path =
        this->get_parameter("serial_file_path").get_parameter_value().get<std::string>();
    int baud =
        this->get_parameter("baud").get_parameter_value().get<int>();
    std::string labelName =
        this->get_parameter("label_name").get_parameter_value().get<std::string>();

    RCLCPP_INFO(this->get_logger(), "serial_file_path: %s", serial_file_path.c_str());
    RCLCPP_INFO(this->get_logger(), "baud: %d", baud);
    RCLCPP_INFO(this->get_logger(), "label_name: %s", labelName.c_str());

    unsigned short int baud_flag;

    if (baud == 115200)
    {
        baud_flag = B115200;
    }
    else if (baud == 1000000)
    {
        baud_flag = B1000000;
    }
    else
    {
        baud_flag = B115200;
        RCLCPP_INFO(this->get_logger(), "baud rate not supported, defaulting to 115200.");
    }

    fd = open_serial(serial_file_path.c_str(), baud_flag);

    std::string publishTopic = "/uwbData/" + labelName;
    msgPublisher_ = this->create_publisher<uwb_interfaces::msg::UWBData>(publishTopic, 10);
    RCLCPP_INFO(this->get_logger(), "publish topic : %s", publishTopic.c_str());

    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(20ms, std::bind(&SerialPortReader::timer_callback, this));
}

int SerialPortReader::open_serial(const char *__file, unsigned short int __baud)
{
    struct termios options;

    int fd = open(__file, O_RDWR | O_NOCTTY);
    tcgetattr(fd, &options);

    if (fd == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "unable to open serial port");
        exit(0);
    }

    if (fcntl(fd, F_SETFL, 0) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "fcntl failed");
    }

    else
        fcntl(fd, F_SETFL, 0);

    if (isatty(STDIN_FILENO) == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "standard input is not a terminal device");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "serial device init success!");
    }

    memset(&options, 0, sizeof(options));

    options.c_cflag = __baud | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

void SerialPortReader::timer_callback()
{
    uint8_t serialBuf[SERIAL_BUF_SIZE] = {0};
    int n = read(fd, serialBuf, sizeof(serialBuf));

    if (n > 0)
    {
        for (int i = 0; i < n; i++)
        {
            uint8_t c = serialBuf[i];
            if (c != '\n')
            {
                jsonBuf.push_back(c);
            }
            else if (c != '\r')
            {
                std::string jsonStr = std::string(jsonBuf.begin(), jsonBuf.end());
                RCLCPP_INFO(this->get_logger(), "%s", jsonStr.c_str());
                jsonBuf.clear();

                Json::Reader reader;
                Json::Value root;
                try
                {
                    if (reader.parse(jsonStr, root))
                    {
                        uwb_interfaces::msg::UWBData msg;

                        if (!root["addr"].isNull())
                        {
                            std::string addrStr = root["addr"].asString();
                            // std::cout << addrStr << std::endl;
                        }

                        if (!root["data"].isNull())
                        {
                            for (auto element : root["data"])
                            {
                                uwb_interfaces::msg::UWBDistance distance;

                                if (!element["addr"].isNull())
                                {
                                    std::string addrStr = element["addr"].asString();
                                    distance.id = addrStr;
                                    // std::cout << addrStr << std::endl;
                                }
                                if (!element["distance"].isNull())
                                {
                                    std::string distanceStr = element["distance"].asString();
                                    distance.distance = atoi(distanceStr.c_str());
                                    // std::cout << distanceStr << std::endl;
                                }
                                msg.distances.push_back(distance);
                            }
                            msgPublisher_->publish(msg);
                        }
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), e.what());
                }
            }
        }
    }
}