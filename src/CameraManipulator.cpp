#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <opencv2/opencv.hpp>


class CameraProcessor
{
    private:

    cv::VideoCapture cap;
    cv::Mat frame;

    public:

    // Constructor
    CameraProcessor(int cameraIndex = 0) : cap(cameraIndex){}

    // Methods
    bool captureFrame()
    {
        return cap.read(frame);
    }

    cv::Mat getFrame() const 
    {
        return frame;
    }

    cv::Mat toGrayscale(const cv::Mat& frame) 
    {
        cv::Mat gray;
        cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        return gray;
    }

    cv::Mat applyGaussianBlur(const cv::Mat& frame, int kernelSize = 5)
    {
        cv::Mat blurred;
        cv::GaussianBlur(frame, blurred, cv::Size(kernelSize, kernelSize),0);
        return blurred;
    }

    cv::Mat detectEdges(const cv::Mat& frame, double threshold1 = 100, double threshold2 = 200) {
        cv::Mat edges;
        cv::Canny(frame, edges, threshold1, threshold2);
        return edges;
    }

};


class CameraManipulator : public rclcpp::Node
{
    public:
        CameraManipulator() : Node("CameraManipulator"), cameraProcessor(0)
        {
            timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),  // Częstotliwość odświeżania
            std::bind(&CameraManipulator::processImage, this));
        }

    private:

        void processImage()
        {
            if (cameraProcessor.captureFrame()) {

                cv::Mat frame = cameraProcessor.getFrame();

                cv::Mat grayFrame = cameraProcessor.toGrayscale(frame);
                cv::Mat blurredFrame = cameraProcessor.applyGaussianBlur(frame, 11);
                cv::Mat edgesFrame = cameraProcessor.detectEdges(frame,50,100);

                // Resize all images to the same size if necessary
                cv::resize(grayFrame, grayFrame, frame.size());
                cv::resize(blurredFrame, blurredFrame, frame.size());
                cv::resize(edgesFrame, edgesFrame, frame.size());

                // Convert grayscale and edges to 3-channel BGR for concatenation
                cv::Mat grayFrameBGR, edgesFrameBGR;
                cv::cvtColor(grayFrame, grayFrameBGR, cv::COLOR_GRAY2BGR);
                cv::cvtColor(edgesFrame, edgesFrameBGR, cv::COLOR_GRAY2BGR);

                // Create a 2x2 grid of images
                cv::Mat topRow, bottomRow, finalImage;

                // Concatenate images horizontally
                cv::hconcat(frame, grayFrameBGR, topRow);          // Original and Grayscale
                cv::hconcat(blurredFrame, edgesFrameBGR, bottomRow); // Blurred and Edges

                // Concatenate the two rows vertically
                cv::vconcat(topRow, bottomRow, finalImage); // Combine rows

                // Display the combined image
                cv::imshow("2x2 Image Grid", finalImage);
                cv::waitKey(1); // Adjust if needed
        
            }
        }

        CameraProcessor cameraProcessor;
        rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraManipulator>());
    rclcpp::shutdown();
    return 0;
}
