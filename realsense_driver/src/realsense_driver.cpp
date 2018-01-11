//ros
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>

//opencv
#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

//pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace rs;
using namespace cv;

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;

// Window size and frame rate
int const INPUT_WIDTH      = 640;
int const INPUT_HEIGHT     = 480;
int const FRAMERATE        = 60;

context     _rs_ctx;
device&     _rs_camera = *_rs_ctx.get_device( 0 );
intrinsics  _depth_intrin;
intrinsics  _color_intrin;
bool         _loop = true;
// Initialize the application state. Upon success will return the static app_state vars address
bool initialize_streaming( )
{
       bool success = false;
       if( _rs_ctx.get_device_count( ) > 0 )
       {
             _rs_camera.enable_stream( rs::stream::color, INPUT_WIDTH, INPUT_HEIGHT, rs::format::rgb8, FRAMERATE );
             _rs_camera.enable_stream( rs::stream::depth, INPUT_WIDTH, INPUT_HEIGHT, rs::format::z16, FRAMERATE );
             _rs_camera.start( );

             success = true;
       }
       return success;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "realsense_driver");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub;
  image_transport::Publisher image_pub_depth;
  image_pub = it.advertise("/realsense_sr300/rgb", 1);
  image_pub_depth = it.advertise("/realsense_sr300/depth", 1);
  ros::Publisher cloud_pub = nh.advertise<PointCloud> ("/realsense_sr300/cloud", 1);
  PointCloud::Ptr cloud_scene (new PointCloud);
  //pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
  //viewer.addCoordinateSystem(0.1);
  rs::log_to_console( rs::log_severity::warn );
  if( !initialize_streaming( ) )
  {
       std::cout << "Unable to locate a camera" << std::endl;
       rs::log_to_console( rs::log_severity::fatal );
       return EXIT_FAILURE;
  }
  _depth_intrin = _rs_camera.get_stream_intrinsics( rs::stream::depth_aligned_to_color );
  _color_intrin = _rs_camera.get_stream_intrinsics( rs::stream::color );
  rs::intrinsics depth_intrin = _rs_camera.get_stream_intrinsics(rs::stream::depth_aligned_to_color);
  rs::intrinsics color_intrin = _rs_camera.get_stream_intrinsics(rs::stream::color);
  // Loop until someone left clicks on either of the images in either window.
  /*tf::TransformBroadcaster br;
  tf::Transform transform_b;
  transform_b.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform_b.setRotation( tf::Quaternion(0, 0, 0, 1) );
  br.sendTransform(tf::StampedTransform(transform_b, ros::Time::now(), "camera_link", "camera_rgb_optical_frame"));//initiate tf*/
  ros::Rate loop_rate(1000);
  while (ros::ok())
  {
    if(_rs_camera.is_streaming()){_rs_camera.wait_for_frames();}
    // Create depth image
    Mat depth16( _depth_intrin.height,
                              _depth_intrin.width,
                              CV_16U,
                              (uchar *)_rs_camera.get_frame_data( rs::stream::depth_aligned_to_color ) );
    // Create color image
    Mat rgb( _color_intrin.height,
                        _color_intrin.width,
                        CV_8UC3,
                        (uchar *)_rs_camera.get_frame_data( rs::stream::color ) );
    float scale = _rs_camera.get_depth_scale();
    //cout<<"scale: "<<scale<<endl;
    Mat depth16_out;
    depth16.copyTo(depth16_out);
    cloud_scene->clear();
    cloud_scene->header.frame_id = "camera_rgb_optical_frame";
    //cloud_scene->height = 1;
    
    for (int r=0;r<INPUT_HEIGHT;r++)
    {
      for (int c=0;c<INPUT_WIDTH;c++)
      {
        pcl::PointXYZRGBA p;
        uint16_t depth_value = depth16.at<short>(r,c);
        float depth_in_meters = depth_value * scale;
        depth16_out.at<short>(r,c) = (short)(depth_in_meters*1000);
        rs::float2 depth_pixel = {(float)c, (float)r};
        rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
        double scene_z = depth_point.z;
        double scene_x = depth_point.x;
        double scene_y = depth_point.y;
        p.x = scene_x;
        p.y = scene_y;
        p.z = scene_z;
        p.r = rgb.ptr<uchar>(r)[c*3];
        p.g = rgb.ptr<uchar>(r)[c*3+1];
        p.b = rgb.ptr<uchar>(r)[c*3+2];
        cloud_scene->points.push_back(p);
      }
    }
    //imwrite("depth_image.png",depth16_out);
    // < 800
    //imshow( WINDOW_DEPTH, depth8u );
    //cvWaitKey(1);
    cv::cvtColor( rgb, rgb, cv::COLOR_BGR2RGB );
    //cv::imwrite("rgb_image.png",rgb);
    
    //cvWaitKey(1);
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "camera_rgb_optical_frame";

    cv_bridge::CvImage cv;
    cv.header = header;
    cv.image = rgb;
    cv.encoding = sensor_msgs::image_encodings::BGR8;

    cv_bridge::CvImage cv_depth;
    cv_depth.header = header;
    cv_depth.image = depth16_out;
    cv_depth.encoding = sensor_msgs::image_encodings::MONO16;

    image_pub.publish(cv.toImageMsg());
    image_pub_depth.publish(cv_depth.toImageMsg());
    //br.sendTransform(tf::StampedTransform(transform_b, ros::Time::now(), "camera_link", "camera_rgb_optical_frame"));
    //cloud_scene->header.stamp = ros::Time::now().toSec();
    cloud_pub.publish(cloud_scene);
    
    /*viewer.addPointCloud(cloud_scene,"cloud_scene");
    viewer.spinOnce();
    viewer.removeAllPointClouds();*/
    ros::spinOnce();
    loop_rate.sleep();
  }
  _rs_camera.stop( );
  ros::spin();
  return 0;
}

