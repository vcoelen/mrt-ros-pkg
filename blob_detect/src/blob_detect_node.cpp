#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window 2";
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
        image_sub_ = it_.subscribe("/image_raw", 1,
        &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW);
        cv::namedWindow(OPENCV_WINDOW2);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
        cv::destroyWindow(OPENCV_WINDOW2);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        int j=3;
        cv::Mat im_blurred;
        blur(cv_ptr->image, im_blurred, cv::Size(j, j), cv::Point(-1,-1));

        //convert to hsv
        cv::Mat im_HSV;
        //cvtColor(cv_ptr->image, im_HSV, CV_BGR2HSV);
        cvtColor(im_blurred, im_HSV, CV_BGR2HSV);

        cv::Mat im_Gray;
        cv::Scalar colorMin(1, 100, 100);
        cv::Scalar colorMax(15, 255, 255);
        cv::inRange(im_HSV, colorMin, colorMax, im_Gray);

        //bitwise_not(im_Gray, im_Gray);

        //int operation = morph_operator + 2;
		int morph_elem = 2;
		int morph_size = 3;
		int morph_operator = cv::MORPH_CLOSE;
		cv::Mat element = getStructuringElement(morph_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
        morphologyEx(im_Gray, im_Gray, morph_operator, element);

        morph_operator = cv::MORPH_OPEN;
        element = getStructuringElement(morph_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
        morphologyEx(im_Gray, im_Gray, morph_operator, element);

        j=7;
        //blur(im_Gray, im_Gray, cv::Size(j, j), cv::Point(-1,-1));
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours( im_Gray,
                          contours,
                          hierarchy,
                          CV_RETR_TREE,
                          CV_CHAIN_APPROX_SIMPLE,
                          cv::Point(0, 0) );

        std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
        std::vector<cv::Rect> boundRect;
        for( int i = 0; i < contours.size(); i++ )
        {
            cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true );

            if (std::fabs(cv::contourArea(cv::Mat(contours_poly[i]))) > 600)
		    {
                boundRect.push_back(cv::boundingRect( cv::Mat(contours_poly[i]) ));
            }
        }
        // Debug purposes: draw bonding rects
        //cv::Mat tmp = cv::Mat::zeros( red_image.size(), CV_8UC3 );
        for( int i = 0; i< boundRect.size(); i++ )
          rectangle(cv_ptr->image, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0, 255, 0), 2, 8, 0 );
        //cv::imwrite("out2.png", tmp);


        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::imshow(OPENCV_WINDOW2, im_Gray);
        cv::waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
