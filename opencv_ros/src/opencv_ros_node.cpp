#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;

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
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/iiwa/camera1/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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

    // Draw an example circle on the video stream
    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;
    // Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 200;
    // Filter by Area.
    params.filterByArea = false;
    //params.minArea = 1;
    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.8;
    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.9;
    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.9;

    // Set up detector with params
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    
    // Detect blobs.
    std::vector<KeyPoint> keypoints;
    detector->detect(cv_ptr->image, keypoints);
    
    // Draw detected blobs as red circles.
    Mat im_with_keypoints;
    drawKeypoints(cv_ptr->image, keypoints, im_with_keypoints, Scalar(255,0,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Stampa informazioni di debug
	  ROS_INFO("Number of keypoints detected: %d", keypoints.size());
	  //ROS_INFO("Area del disco: %f", params.minArea);
	  //ROS_INFO("Image size: %dx%d", cv_ptr->image.cols, cv_ptr->image.rows);
	  //ROS_INFO("Thresholds: %d, %d", params.minThreshold, params.maxThreshold);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, im_with_keypoints);
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