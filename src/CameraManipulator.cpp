#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <opencv2/opencv.hpp>

class CameraManipulator : public rclcpp::Node
{
public:
    CameraManipulator() : Node("CameraManipulator")
    {
        
        cap.open(0);

        if (!cap.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&CameraManipulator::timer_callback, this));
    }

private:
    void timer_callback()
    {

        cv::Mat frame;

        cap >> frame;

        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Empty frame captured.");
            return;
        }

        cv::imshow("Camera Stream", frame);
        cv::waitKey(1);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraManipulator>());
    rclcpp::shutdown();
    return 0;
}
