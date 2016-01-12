#include <ros/ros.h>

#include <opencv2/opencv.hpp> // opencv image processing libraries

// to facilitate the processing of the images in ROS
#include <image_transport/image_transport.h> // for publishing and subscribing to images in ROS
#include <cv_bridge/cv_bridge.h> // to convert between ROS and OpenCV Image formats
#include <sensor_msgs/image_encodings.h> 

//for publishing rois
#include "std_msgs/String.h" 
#include <sstream>

static const std::string OPENCV_WINDOW = "People detection window";

class HogPeopleDetection
{
public: 
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber im_sub_;
  image_transport::Publisher im_pub_;
  ros::Publisher rois_pub_;

  cv::HOGDescriptor hog_;

  //Default constructor
  HogPeopleDetection()
    :  it_(nh_)
  {
    // Get ROS params from launch file
    std::string image_topic;
    if (!nh_.getParam("image_topic", image_topic))
      ROS_ERROR("Could not get image_topic");   
  
    // Load the hog descriptor
    hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());    

    // Subscrive to input video feed, publish output video feed and rois
    im_sub_ = it_.subscribe(image_topic, 1, &HogPeopleDetection::imageCallback, this);
    im_pub_ = it_.advertise("/image", 1);
    rois_pub_ = nh_.advertise<std_msgs::String>("rois", 1000);

    cv::namedWindow(OPENCV_WINDOW);
  }
  
  
  ~HogPeopleDetection()
  {
    cv::destroyWindow(OPENCV_WINDOW);  
  }
  
  
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    // Make image processable in ROS
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
    cv::Mat im_bgr = cv_ptr->image;
    
    //Preprocess the OpenCV image
    cv::Mat im_gray;
    cv::cvtColor( im_bgr, im_gray, CV_BGR2GRAY );
    cv::equalizeHist( im_gray, im_gray );
    
    // HOG pedestrian detector
    std::vector<cv::Rect> detected_pedestrian;
    hog_.detectMultiScale(im_gray, detected_pedestrian, 0.0, cv::Size(8, 8), cv::Size(0,0), 1.05, 4); 
    
    //Prepare rois messages
    std_msgs::String rois_msg;
    std::stringstream ss;
    ss <<  detected_pedestrian.size() << " ";   

    // Draw detections from HOG to the screen and put rois to String
    for(unsigned i = 0; i < detected_pedestrian.size(); i++) 
    {
      cv::rectangle(im_bgr, detected_pedestrian[i], cv::Scalar(255));
      ss << detected_pedestrian[i].x << " " << detected_pedestrian[i].y << " " << detected_pedestrian[i].height << " " << detected_pedestrian[i].width;      
    }

    //Send ROS image messages
    sensor_msgs::ImagePtr image_msg_ptr;
    image_msg_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", im_bgr).toImageMsg();
    im_pub_.publish(image_msg_ptr);
    
    //Send rois
    rois_msg.data = ss.str();
    ROS_INFO("%s", rois_msg.data.c_str());
    rois_pub_.publish(rois_msg);
   
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, im_bgr);
    cv::waitKey(3);
  }
};


int main (int argc, char** argv)
{
  ros::init (argc, argv, "people_detector");
  HogPeopleDetection hhpd;
  ros::spin ();
  return 0;
}
