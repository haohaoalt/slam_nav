/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include"../../../include/System.h"

#include "opencv2/core/eigen.hpp"
//#include "/home/xin/turtlebot3_ws/src/ORB_SLAM3_PRO/Examples_old/ROS/ORB_SLAM3_MAX/include/skimap_ros/SkiMapServiceClient.hpp"

using namespace std;
using namespace octomap;
ros::Publisher CamPose_Pub;
ros::Publisher Camodom_Pub;
ros::Publisher odom_pub;

geometry_msgs::PoseStamped Cam_Pose;
geometry_msgs::PoseWithCovarianceStamped Cam_odom;

cv::Mat Camera_Pose;
tf::Transform orb_slam;
tf::TransformBroadcaster * orb_slam_broadcaster;
std::vector<float> Pose_quat(4);
std::vector<float> Pose_trans(3);

ros::Time current_time, last_time;
double lastx=0,lasty=0,lastth=0;
unsigned int a =0,b=0;
octomap::ColorOcTree tree( 0.05 );
//skimap_ros::SkimapServiceClient* skimap_service_client;
typedef octomap::ColorOcTree::leaf_iterator it_t;
class ImageGrabber
{
public:
    ros::NodeHandle nh;
    ros::Publisher  pub_rgb,pub_depth,pub_tcw,pub_camerapath,pub_odom;
    size_t mcounter=0;
    nav_msgs::Path  camerapath;

    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM),nh("~")
    {
        //创建ROS的发布节点
        pub_rgb= nh.advertise<sensor_msgs::Image> ("RGB/Image", 100);
        pub_depth= nh.advertise<sensor_msgs::Image> ("Depth/Image", 100);
        pub_tcw= nh.advertise<geometry_msgs::PoseStamped> ("CameraPose", 100);
        pub_odom= nh.advertise<nav_msgs::Odometry> ("Odometry", 100);
        pub_camerapath= nh.advertise<nav_msgs::Path> ("Path", 100);
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 100);
    //message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_rect_raw", 100);
    //skimap_service_client = new skimap_ros::SkimapServiceClient(&nh, "/skimap_map_service/integration_service");
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));


    CamPose_Pub = nh.advertise<geometry_msgs::PoseStamped>("/Camera_Pose",100);
    Camodom_Pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/Camera_Odom", 100);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    SLAM.SaveTrajectoryTUM("FrameTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    // Save camera trajectory
    ros::shutdown();

    return 0;
}

//以上代码是将ROS图像消息转换为OpenCV中的cv::Mat格式。其中，cv_bridge库提供了ROS图像消息和OpenCV图像之间的转换。
//
//具体来说，首先通过cv_bridge::toCvShare()函数将RGB图像消息msgRGB转换为
// cv_bridge::CvImageConstPtr类型的指针cv_ptrRGB，CvImageConstPtr中包含了OpenCV的cv::Mat格式的图像数据，
// 以及一些额外的图像信息。如果转换失败，则会抛出cv_bridge::Exception异常。
//
//接着，将深度图像消息msgD转换为cv_ptrD。这里的步骤与RGB图像的转换类似。最终，通过cv_ptrRGB和cv_ptrD可以获取到RGB图像和深度图像的OpenCV格式的数据，以便后续处理。
void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    ///skimap
    //typedef skimap_ros::SkimapServiceClient::ColorPoint CPoint;
    //std::vector<CPoint> points;

//Fill vector with your points
    geometry_msgs::Pose sensor_pose;

    bool  isKeyFrame =false;
    Sophus::SE3f sophus_Tcw;
    ///稀疏重建做好用所有帧
    ///sophus_Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    /// 只要关键帧
    sophus_Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec(),isKeyFrame);
    cv::Mat Tcw = cv::Mat(4, 4, CV_32F);
    cv::eigen2cv(sophus_Tcw.matrix(), Tcw);
    ///
    orb_slam_broadcaster = new tf::TransformBroadcaster;
    //  Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    if (!Tcw.empty())
    {
        cv::Mat Twc =Tcw.inv();
        cv::Mat RWC= Twc.rowRange(0,3).colRange(0,3);
        cv::Mat tWC=  Twc.rowRange(0,3).col(3);
        cv::Mat twc(3,1,CV_32F);
        twc = tWC;

        Eigen::Matrix<double,3,3> eigMat ;
        eigMat <<RWC.at<float>(0,0),RWC.at<float>(0,1),RWC.at<float>(0,2),
                RWC.at<float>(1,0),RWC.at<float>(1,1),RWC.at<float>(1,2),
                RWC.at<float>(2,0),RWC.at<float>(2,1),RWC.at<float>(2,2);
        Eigen::Quaterniond q(eigMat);

//        geometry_msgs::PoseStamped tcw_msg;
//        tcw_msg.pose.position.x=tWC.at<float>(0);
//        tcw_msg.pose.position.y=tWC.at<float>(1);
//        tcw_msg.pose.position.z=tWC.at<float>(2);
//
//        tcw_msg.pose.orientation.x=q.x();
//        tcw_msg.pose.orientation.y=q.y();
//        tcw_msg.pose.orientation.z=q.z();
//        tcw_msg.pose.orientation.w=q.w();

        Pose_quat[0] = q.x(); Pose_quat[1] = q.y();
        Pose_quat[2] = q.z(); Pose_quat[3] = q.w();

        Pose_trans[0] = twc.at<float>(0);
        Pose_trans[1] = twc.at<float>(1);
        //Pose_trans[1] = 0.5;
        Pose_trans[2] = twc.at<float>(2);

        sensor_pose.position.x = twc.at<float>(0);
        sensor_pose.position.y = twc.at<float>(1);
        sensor_pose.position.z = twc.at<float>(2);
        sensor_pose.orientation.x = q.x();
        sensor_pose.orientation.y = q.y();
        sensor_pose.orientation.z = q.z();
        sensor_pose.orientation.w = q.w();

//Fill sensor_pose with your camera pose
        //skimap_service_client->integratePoints(points, sensor_pose);
//        这行代码是将ORB_SLAM2计算出的相机位姿转换为ROS的tf变换，
//        并将其设置为ORB_SLAM2的原点。在ROS中，通常使用右手坐标系，
//        其中X轴向前，Y轴向左，Z轴向上，而ORB_SLAM2使用的是左手坐标系，其中X轴向右，Y轴向下，Z轴向前。
//
//        因此，为了将ORB_SLAM2的坐标系与ROS的坐标系对齐，
//        需要将ORB_SLAM2计算出的相机位姿中的X轴和Y轴的方向取反，并将Z轴和Y轴的顺序交换。
//        由于ORB_SLAM2使用的是左手坐标系，因此将Y轴的正方向取反后，需要将其乘以-1。
//
//        因此，这行代码将ORB_SLAM2计算出的相机位姿的平移分量[0, 1, 2]，分别乘以-1、1、1，
//        并按照ROS坐标系的顺序重新排列，然后将其作为tf变换的平移分量，从而将ORB_SLAM2的坐标系与ROS的坐标系对齐。

        orb_slam.setOrigin(tf::Vector3(Pose_trans[2], -Pose_trans[0], -Pose_trans[1]));
        orb_slam.setRotation(tf::Quaternion(q.z(), -q.x(), -q.y(), q.w()));
        orb_slam_broadcaster->sendTransform(tf::StampedTransform(orb_slam, ros::Time::now(), "/map", "/ORB_SLAM3"));

        Cam_Pose.header.stamp =ros::Time::now();
        //Cam_Pose.header.seq = msgRGB->header.seq;
        Cam_Pose.header.frame_id = "/map";
        tf::pointTFToMsg(orb_slam.getOrigin(), Cam_Pose.pose.position);
        tf::quaternionTFToMsg(orb_slam.getRotation(), Cam_Pose.pose.orientation);

        Cam_odom.header.stamp = ros::Time::now();
        Cam_odom.header.frame_id = "/map";
//        这段代码是将ORB-SLAM中的相机位姿信息转换为ROS中的geometry_msgs/PoseWithCovarianceStamped消息类型，以便发布到ROS系统中。
//
//        具体来说，orb_slam.getOrigin()和orb_slam.getRotation()分别获取ORB-SLAM中相机的位置和方向，将其转换为ROS中的geometry_msgs/Point和geometry_msgs/Quaternion类型的消息。
//
//        然后，这些消息被设置到Cam_odom.pose.pose.position和Cam_odom.pose.pose.orientation中，分别表示相机的位置和方向。
//
//        最后，通过设置Cam_odom.pose.covariance来提供相机位姿估计的协方差矩阵，这个矩阵可以表示位姿的不确定性和精度。在这个例子中，协方差矩阵是一个6x6的对角矩阵，其中每个元素表示相应位姿变量的方差。
//
        tf::pointTFToMsg(orb_slam.getOrigin(), Cam_odom.pose.pose.position);
        tf::quaternionTFToMsg(orb_slam.getRotation(), Cam_odom.pose.pose.orientation);
        Cam_odom.pose.covariance = {0.01, 0, 0, 0, 0, 0,
                                    0, 0.01, 0, 0, 0, 0,
                                    0, 0, 0.01, 0, 0, 0,
                                    0, 0, 0, 0.01, 0, 0,
                                    0, 0, 0, 0, 0.01, 0,
                                    0, 0, 0, 0, 0, 0.01};
        CamPose_Pub.publish(Cam_Pose);
        Camodom_Pub.publish(Cam_odom);

        // odometry information
        nav_msgs::Odometry odom;
        odom.header.stamp =ros::Time::now();
        odom.pose.pose.position = Cam_odom.pose.pose.position;
        odom.pose.pose.orientation = Cam_odom.pose.pose.orientation;
        // Set the velocity
        odom.child_frame_id = "/ORB_SLAM3";
        current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        double vx = (Cam_odom.pose.pose.position.x - lastx)/dt;
        double vy = (Cam_odom.pose.pose.position.y - lasty)/dt;
        double vth = (Cam_odom.pose.pose.orientation.z - lastth)/dt;

        double _x = Cam_odom.pose.pose.position.x;
        double _y = Cam_odom.pose.pose.position.y;
        double _z = Cam_odom.pose.pose.position.z;

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        // Publish the message
        odom_pub.publish(odom);
        last_time = current_time;
        lastx = Cam_odom.pose.pose.position.x;
        lasty = Cam_odom.pose.pose.position.y;
        lastth = Cam_odom.pose.pose.orientation.z;
//        odom_msg.pose.pose.position.x=tWC.at<float>(0);
//        odom_msg.pose.pose.position.y=tWC.at<float>(1);
//        odom_msg.pose.pose.position.z=tWC.at<float>(2);
//
//        odom_msg.pose.pose.orientation.x=q.x();
//        odom_msg.pose.pose.orientation.y=q.y();
//        odom_msg.pose.pose.orientation.z=q.z();
//        odom_msg.pose.pose.orientation.w=q.w();
//
//        odom_msg.header=header;
//        odom_msg.child_frame_id="base_link";

////////
        geometry_msgs::PoseStamped tcw_msg;
        tcw_msg.pose.position.x=tWC.at<float>(0);
        tcw_msg.pose.position.y=tWC.at<float>(1);
        tcw_msg.pose.position.z=tWC.at<float>(2);

        tcw_msg.pose.orientation.x=q.x();
        tcw_msg.pose.orientation.y=q.y();
        tcw_msg.pose.orientation.z=q.z();
        tcw_msg.pose.orientation.w=q.w();

        std_msgs::Header header ;
        header.stamp =msgRGB->header.stamp;
        header.seq = msgRGB->header.seq;
        header.frame_id="/map";

        tcw_msg.header=header;

        // odometry information
        nav_msgs::Odometry odom_msg;
        odom_msg.pose.pose.position.x=tWC.at<float>(0);
        odom_msg.pose.pose.position.y=tWC.at<float>(1);
        odom_msg.pose.pose.position.z=tWC.at<float>(2);

        odom_msg.pose.pose.orientation.x=q.x();
        odom_msg.pose.pose.orientation.y=q.y();
        odom_msg.pose.pose.orientation.z=q.z();
        odom_msg.pose.pose.orientation.w=q.w();

        odom_msg.header=header;
        odom_msg.child_frame_id="/map";

        camerapath.header =header;
        camerapath.poses.push_back(Cam_Pose);
        //pub_odom.publish(odom_msg);
        pub_camerapath.publish(camerapath);  //相机轨迹
////////
        if( isKeyFrame)
        {
            pub_tcw.publish(Cam_Pose);	                      //Tcw位姿信息
            pub_rgb.publish(msgRGB);
            pub_depth.publish(msgD);
        }

    }
    else
    {
        cout<<"Twc is empty ..."<<endl;
    }

}




