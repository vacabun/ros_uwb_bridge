#include "rclcpp/rclcpp.hpp"
#include "bridge.hpp"

int main(int argc, const char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UWBRosBridge>());
    rclcpp::shutdown();

    return 0;
}

UWBRosBridge::UWBRosBridge() : Node("uwb_ros_bridge")
{
    config();

    serial_fd = open_serial(serial_file_path.c_str(), baud_flag);

    std::string publishTopic = "/uwbData/" + labelName;
    msgPublisher_ = this->create_publisher<uwb_interfaces::msg::UWBData>(publishTopic, 10);
    RCLCPP_INFO(this->get_logger(), "publish topic : %s", publishTopic.c_str());

    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(1ms, std::bind(&UWBRosBridge::timer_callback, this));

    service_ = this->create_service<uwb_interfaces::srv::UWBMeasure>(
        "uwb_control", std::bind(&UWBRosBridge::handle_service, this, std::placeholders::_1, std::placeholders::_2));
}

void UWBRosBridge::config()
{
    this->declare_parameter("serial_file_path", "/dev/ttyACM0");
    this->declare_parameter("baud", 115200);
    this->declare_parameter("label_name", "tag_0");

    serial_file_path = this->get_parameter("serial_file_path").get_parameter_value().get<std::string>();
    baud = this->get_parameter("baud").get_parameter_value().get<int>();
    labelName = this->get_parameter("label_name").get_parameter_value().get<std::string>();

    RCLCPP_INFO(this->get_logger(), "serial_file_path: %s", serial_file_path.c_str());
    RCLCPP_INFO(this->get_logger(), "baud: %d", baud);
    RCLCPP_INFO(this->get_logger(), "label_name: %s", labelName.c_str());

    switch (baud)
    {
    case 115200:
        baud_flag = B115200;
        break;
    case 1000000:
        baud_flag = B1000000;
        break;
    default:
        baud_flag = B115200;
        RCLCPP_INFO(this->get_logger(), "baud rate not supported, defaulting to 115200.");
        break;
    }
}
int UWBRosBridge::open_serial(const char *__file, unsigned short int __baud)
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

void UWBRosBridge::timer_callback()
{
    uint8_t serialBuf[SERIAL_BUF_SIZE] = {0};
    int n = read(serial_fd, serialBuf, sizeof(serialBuf));

    for (int i = 0; i < n; i++)
    {
        uint8_t c = serialBuf[i];
        if (c != '\n')
        {
            jsonBuf.push_back(c);
        }
        else if (c == '\n' && jsonBuf.size() > 0 && jsonBuf[0] == '{')
        {
            std::string jsonStr = std::string(jsonBuf.begin(), jsonBuf.end());
            RCLCPP_INFO(this->get_logger(), "%s", jsonStr.c_str());
            jsonBuf.clear();
            json_parse(jsonStr);
        }
        else
        {
            jsonBuf.clear();
            RCLCPP_ERROR(this->get_logger(), "invalid json string");
        }
    }
}

int UWBRosBridge::json_parse(std::string jsonStr)
{
    Json::Reader reader;
    Json::Value root;
    try
    {
        if (reader.parse(jsonStr, root))
        {
            if (!root["type"].isNull())
            {
                std::string type = root["type"].asString();
                if (type == "measure")
                {
                    if (!root["src"].isNull())
                    {
                        src_addr = atoi(root["src"].asString().c_str());
                    }
                    if (!root["dst"].isNull())
                    {
                        dst_addr = atoi(root["dst"].asString().c_str());
                    }
                    if (!root["distance"].isNull())
                    {
                        distance = atoi(root["distance"].asString().c_str());
                    }
                    RCLCPP_INFO(this->get_logger(), "src: %d, dst: %d, distance: %d", src_addr, dst_addr, distance);

                    response_flag = true;
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "unknown type");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "can not find type");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "json parse error");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "json parse error");
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
    return 0;
}

void UWBRosBridge::handle_service(
    const std::shared_ptr<uwb_interfaces::srv::UWBMeasure::Request> request,
    std::shared_ptr<uwb_interfaces::srv::UWBMeasure::Response> response)
{
    int cmd = request->cmd;
    RCLCPP_INFO(this->get_logger(), "Received cmd: %ld", request->cmd);
    switch (cmd)
    {
    case 0:
    {
        RCLCPP_INFO(this->get_logger(), "cmd: 0");
        break;
    }
    case 1:
    {
        RCLCPP_INFO(this->get_logger(), "cmd: 1");

        response_flag = false;

        char data[100];
        sprintf(data, "{type: %d, arg1: %ld}\r\n", cmd, request->dest);

        write(serial_fd, data, strlen(data));

        // while (!response_flag)
        // {
        //     usleep(1000);
        // }

        
        RCLCPP_INFO(this->get_logger(), "src: %d, dst: %d, distance: %d", src_addr, dst_addr, distance);

        response->distance = distance;
        break;
    }
    }
}
