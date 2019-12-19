

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo/photo.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/ps_eye_camera/image_raw", 1,
                                   &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/ps_eye_camera/filtered/image_raw", 1);

       // cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
    }

    ~ImageConverter()
    {
     //   cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat gray_image_b, gray_image_g, gray_image_r, filtered_image;
        cv::Mat bgr[3], img; //destination array

        split(cv_ptr->image, bgr); //split source

        cv::threshold(bgr[0], gray_image_b, 250, 20, cv::THRESH_TOZERO_INV);
        cv::threshold(bgr[1], gray_image_g, 250, 20, cv::THRESH_TOZERO_INV);
        cv::threshold(bgr[2], gray_image_r, 250, 50, cv::THRESH_TOZERO_INV);

        bgr[0] = gray_image_b;
        bgr[1] = gray_image_g;
        bgr[2] = gray_image_r;

        std::vector<cv::Mat> channelse = {bgr[0], bgr[1], bgr[2]};
        cv::merge(channelse, filtered_image);
        filtered_image.convertTo(filtered_image, -1, 1.5, 0); 
/*
        cv::cvtColor(filtered_image, img, CV_BGR2Lab);
        split(img, bgr); //split source
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(1);
        cv::Mat dst;
        clahe->apply(bgr[0], dst);

        dst.copyTo(bgr[0]);
        std::vector<cv::Mat> channels = {bgr[0], bgr[1], bgr[2]};
        cv::merge(channels, filtered_image);
        cv::cvtColor(filtered_image, filtered_image, CV_Lab2BGR);
*/
        // cv::inpaint(cv_ptr->image, binarized_image, filtered_image, 3, cv::INPAINT_NS);

        //  cv::imshow(OPENCV_WINDOW, filtered_image);
    // cv::waitKey(1);
        cv_ptr->image = filtered_image;

        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}