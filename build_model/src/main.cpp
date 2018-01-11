#include "common_include.h"
#include "config.h"
#include "camera.h"
#include "qrplane.h"

using namespace std;
using namespace cv;
using namespace buildmodel;

#define numberOfQRCodeOnSide 6

int main(int argc,char *argv[])
{
    Mat rgb_image;
    Mat rgb_tmp;
    Mat depth_image;
    Mat depth_tmp;
    //config file of camera
    std::string config_filename = "/home/mzm/new_sr300_build_model/src/build_model/config/default2.yaml";
    buildmodel::Camera::Ptr camera1 (new buildmodel::Camera);
    camera1->_camera_config_filename = config_filename;
    pcl::visualization::PCLVisualizer viewer1 ("cube_points_cloud_from_camera");

    //ros node
    ros::init(argc, argv, "build_model_node");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<realsense_msgs::realsense_msgs>("get_image");
    realsense_msgs::realsense_msgs srv;
    srv.request.start = true;
    sensor_msgs::Image msg_rgb;
    sensor_msgs::Image msg_depth;
    ros::Rate loop_rate(200);

    while(ros::ok())
    {
        if (client.call(srv))
        {
            try
            {
              msg_rgb = srv.response.rgb_image;
              msg_depth = srv.response.depth_image;
              cout<<"running!"<<endl;
              rgb_image = cv_bridge::toCvCopy(msg_rgb)->image;
              depth_image = cv_bridge::toCvCopy(msg_depth)->image;
              normalize(depth_image,depth_tmp,255,0,NORM_MINMAX);
              depth_tmp.convertTo(depth_tmp, CV_8UC1, 1.0);
            }
            catch (cv_bridge::Exception& e)
            {
              ROS_ERROR("cv_bridge exception: %s", e.what());
              return 1;
            }
            depth_image.convertTo(depth_image,CV_32F);
        }

        if( !rgb_image.data )
        {
            printf( " No image data \n " );
            return -1;
        }

        rgb_image.copyTo(rgb_tmp);
        Mat result_depth_image = Mat::zeros(depth_image.size(),CV_32FC1);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cube_points_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        buildmodel::QRPlane::Ptr plane1(new buildmodel::QRPlane(numberOfQRCodeOnSide,
                                                                  camera1->_camera_intrinsic_matrix,
                                                                  camera1->_distParam));

        plane1->_rgb_image = rgb_image;
        plane1->_depth_image = depth_image;
        plane1->detectQRCode();
        if(plane1->whetherDectectQRCode())
        {
            plane1->GetTransfromCameraToWorld();
            plane1->DrawAxis(rgb_tmp);
            plane1->DrawCube(rgb_tmp);
            plane1->CutCubeInDepthImageAndPointsCloud(result_depth_image,cube_points_cloud);
        }

//        imshow("rgb",rgb_image);
//        imshow("depth",depth_tmp);
        imshow("result",rgb_tmp);
        imshow("depth_result",result_depth_image);
        waitKey(1);

        viewer1.addPointCloud(cube_points_cloud,"cube_points_cloud");
        viewer1.addCoordinateSystem(0.3);
        viewer1.spinOnce();
        viewer1.removeAllPointClouds();
        viewer1.removeCoordinateSystem();

    }
}
