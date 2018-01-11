
#include "realsense_msgs/realsense_msgs.h"
#include "common_include.h"

using namespace std;
using namespace cv;

sensor_msgs::Image rgb_image;
sensor_msgs::Image depth_image;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_rgb;
  image_transport::Subscriber image_sub_depth;
  
public:

  ImageConverter()
    : it_(nh_)
  {
    image_sub_rgb = it_.subscribe("/realsense_sr300/rgb", 1, &ImageConverter::imageCb_rgb, this);
    image_sub_depth = it_.subscribe("/realsense_sr300/depth", 1, &ImageConverter::imageCb_depth, this);

  }

  ~ImageConverter()
  {
  }

  void imageCb_rgb(const sensor_msgs::ImageConstPtr& msg)
  {
     rgb_image = *msg;
  }
  void imageCb_depth(const sensor_msgs::ImageConstPtr& msg)
  {  
     depth_image = *msg;
  }

};

bool get_image(realsense_msgs::realsense_msgs::Request &req,realsense_msgs::realsense_msgs::Response &res)
{
    if (req.start) 
    {
         res.rgb_image = rgb_image;
         res.depth_image = depth_image;
    }
    ROS_INFO("success");
    return true;
}


int main(int argc, char** argv)
{    
  ros::init(argc, argv, "get_image");
  ImageConverter ic; 
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("get_image", get_image);

  ros::Rate loop_rate(200);
  while (ros::ok())
  {

     ros::spinOnce();
    loop_rate.sleep();


  }
  ros::spin();
  return 0;
}

