#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

class built_in_camera{
    public:
        built_in_camera(){
            cv::VideoCapture cam(cv::CAP_ANY);
            if (!cam.isOpened()) {
                std::cout << "could not access webcam!" << std::endl;
            }

            double frame_width = cam.get(cv::CAP_PROP_FRAME_WIDTH);
            double frame_height = cam.get(cv::CAP_PROP_FRAME_HEIGHT);

            std::cout << "Frame-Size: " << frame_width << "x" << frame_height << std::endl;
            cv::namedWindow("original webcam video feed", cv::WINDOW_AUTOSIZE);
            cv::namedWindow("filtered image", cv::WINDOW_AUTOSIZE);

            cv::Mat frame;
            cv::Mat frame_blurred;
            cv::Mat frame_dilated;
            cv::Mat frame_sobel;

            cv::Size filter_size;

            filter_size.width = 10;
            filter_size.height = 10;

            cv::Mat kern = cv::getStructuringElement(cv::MORPH_RECT, filter_size);

            while (true) {
                bool next_frame_readable = cam.read(frame);

                cv::blur(frame, frame_blurred, filter_size);
                cv::dilate(frame, frame_dilated, kern);
                cv::Sobel(frame, frame_sobel, -1/CV_16S/CV_32F/CV_64F, 2, 2);
                //cv::normalize(frame_sobel, frame_sobel, 0, cv::NORM_MINMAX);


                if (!next_frame_readable) {
                    std::cout << "current frame nor readable!" << std::endl;
                }

                cv::imshow("original webcam video feed", frame);
                cv::imshow("filtered image", frame_sobel);

                if(cv::waitKey(5) == 27) {
                    break;
                }

            }
        }

        ~built_in_camera() {
            cv::destroyWindow("webcam OpenCV test");
            cv::destroyWindow("filtered image");
        }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_opencv_camera");
    built_in_camera my_capture;
    ROS_INFO("Cam Tested");
}
