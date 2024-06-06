#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_port.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>

#include <string>
#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

using namespace std::chrono_literals;

struct SensorData
{
    float fX;
    float fY;
    float fZ;
    float mX;
    float mY;
    float mZ;
    float temperature;
};

class HexFT : public rclcpp::Node
{
public:
    HexFT() : Node("FTsensor_code")
    {

        const std::array<std::string,2> &portStr = {"/dev/ttyACM0", "/dev/ttyACM1"};

        // Trying to open ACM0
        m_serialPort1 = open(portStr[0].c_str(), O_RDONLY);
        m_serialPort2 = open(portStr[1].c_str(), O_RDONLY);

        if (m_serialPort1 < 0)
        {
            std::cout << "Error " << errno << " from open: " << strerror(errno) << std::endl;
            std::cout << "This means that the sensor could not be found at " << portStr[0] << std::endl;
            flag = 2;
            close(m_serialPort1);
            // exit(-1);
        }
        else if (m_serialPort2 < 0)
        {
            std::cout << "Error " << errno << " from open: " << strerror(errno) << std::endl;
            std::cout << "This means that the sensor could not be found at " << portStr[1] << std::endl;
            flag = 1;
            close(m_serialPort2);
        }
        else
        {
            exit(-1);
        }
        publisher_1 = this->create_publisher<std_msgs::msg::Float64MultiArray>("FTsensor", 10);
        timer_1 = this->create_wall_timer(10ms, std::bind(&HexFT::ReadSensorData, this));
    }

    ~HexFT()
    {
        if (flag == 1)
        {
            close(m_serialPort1);
        }
        else if (flag == 2)
        {
            close(m_serialPort2);
        }
    }

    void ReadSensorData()
    {
        SensorData data;
        auto value = std_msgs::msg::Float64MultiArray();

        if (flag == 1)
        {
            read(m_serialPort1, reinterpret_cast<uint8_t *>(&data), sizeof(data));
            value.data.push_back(data.fX);
            value.data.push_back(data.fY);
            value.data.push_back(data.fZ);
            publisher_1->publish(value);
        }
        else if (flag == 2)
        {
            read(m_serialPort2, reinterpret_cast<uint8_t *>(&data), sizeof(data));
            value.data.push_back(data.fX);
            value.data.push_back(data.fY);
            value.data.push_back(data.fZ);
            publisher_1->publish(value);
        }
        else
        {
            std::cout << "ERROR\n";
        }

        // value.data.push_back(data.fX);
        // value.data.push_back(data.fY);
        // value.data.push_back(data.fZ);
        // value.data.push_back(1.34837);
        // publisher_1->publish(value);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_1;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_1;
    int m_serialPort1, m_serialPort2, flag;
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    auto ftsensor = std::make_shared<HexFT>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(ftsensor);
    executor.spin();

    rclcpp::shutdown();
    ftsensor->~HexFT();

    return 0;
}