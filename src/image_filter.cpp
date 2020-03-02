

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo/photo.hpp>
#define SHOW_MARKER_

#ifdef SHOW_MARKER
#include <apriltag_ros/AprilTagDetectionArray.h>

static const std::string OPENCV_WINDOW = "Image window";
typedef apriltag_ros::AprilTagDetectionArray AprilTagPose;
#endif
class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
#ifdef SHOW_MARKER
    ros::Subscriber sub_UAVmark;
    int x, y;
    double max_dimm = 0.1;
    double scale_x;
    double scale_y;
    double width;
    double height;
    double fl_x;
    double fl_y;
    double fl_a;
#endif

public:
    ImageConverter()
        : it_(nh_)
    {

        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/ps_eye_camera/image_raw", 1,
                                   &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/ps_eye_camera/filtered/image_raw", 1);
#ifdef SHOW_MARKER
        sub_UAVmark = nh_.subscribe("/tag_detections", 1, &ImageConverter::Marker_Handler, this);
        width = 640.0;
        height = 480.0;
        fl_x = 0.004 / 1.534653976; // (2.0 * tan(37.5));    //2*tan(Thetax/2) 75/2
        fl_y = 0.003 / 1.069022272; // (2.0 * tan(28.125)); //56.25/2

        fl_a = (fl_x + fl_y) / 2.0;

        scale_x = width / (2.0 * max_dimm);
        scale_y = height / (2.0 * max_dimm);
#endif
        // cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
    }

    ~ImageConverter()
    {
        //   cv::destroyWindow(OPENCV_WINDOW);
    }
#ifdef SHOW_MARKER
    void Marker_Handler(const AprilTagPose &apriltag_marker_detections)
    {
        int state = apriltag_marker_detections.detections.size();
        x = 0;
        y = 0;
        if (state > 0)
        {
            double x_d = (apriltag_marker_detections.detections[0].pose.pose.pose.position.x * fl_a) /
                         (apriltag_marker_detections.detections[0].pose.pose.pose.position.z + fl_a);
            double y_d = (apriltag_marker_detections.detections[0].pose.pose.pose.position.y * fl_a) /
                         (apriltag_marker_detections.detections[0].pose.pose.pose.position.z + fl_a);
            std::cout << x_d << ", " << y_d << std::endl;

            double norm_x = (x_d * 200000.0 + (width / 2.0)) / width;
            double norm_y = (y_d * 200000.0 + (height / 2.0)) / height;
            std::cout << norm_x << ", " << norm_y << std::endl;

            x = round((norm_x) * 640.0);
            y = round((norm_y) * 480.0);
            // std::cout << x << ", " << y << std::endl;
        }
    }
#endif
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
        //ROS_INFO("Camera Size %d x %d",cv_ptr->image.rows,cv_ptr->image.cols);

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

        cv_ptr->image = filtered_image;
        image_pub_.publish(cv_ptr->toImageMsg());
#ifdef SHOW_MARKER
        cv::circle(filtered_image, cv::Point(x, y), 5, cv::Scalar(200, 0, 0), -1, 8);
        cv::imshow(OPENCV_WINDOW, filtered_image);
        cv::waitKey(1);
#endif
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}