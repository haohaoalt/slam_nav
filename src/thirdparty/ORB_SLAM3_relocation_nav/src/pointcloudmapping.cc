#include "pointcloudmapping.h"
#include "PointCloude.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_broadcaster.h>
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
namespace ORB_SLAM3 {
    pcl::PointCloud<pcl::PointXYZRGBA> pcl_filter;
    pcl::PointCloud<pcl::PointXYZRGBA> pcl_local_filter;
    ros::Publisher pclPoint_pub;
    ros::Publisher pclPoint_local_pub;
    ros::Publisher octomap_pub;
    sensor_msgs::PointCloud2 pcl_point;
    sensor_msgs::PointCloud2 pcl_local_point;
    pcl::PointCloud<pcl::PointXYZRGBA> pcl_cloud_local_kf;
    pcl::PointCloud<pcl::PointXYZRGBA> pcl_cloud_kf;
    PointCloudMapping::PointCloudMapping(double resolution)
    {
        mResolution = resolution;
        mCx = 0;
        mCy = 0;
        mFx = 0;
        mFy = 0;
        mbShutdown = false;
        mbFinish = false;

        voxel.setLeafSize( resolution, resolution, resolution);
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);

        mPointCloud = boost::make_shared<PointCloud>();
        viewerThread = std::make_shared<std::thread>(&PointCloudMapping::NormalshowPointCloud, this);  // make_unique是c++14的
    }

    PointCloudMapping::~PointCloudMapping()
    {
        viewerThread->join();
    }

    void PointCloudMapping::requestFinish()
    {
        {
            unique_lock<mutex> locker(mKeyFrameMtx);
            mbShutdown = true;
            cout << "稠密点云图构建完成--结束程序" <<endl;
        }
        mKeyFrameUpdatedCond.notify_one();
    }

    bool PointCloudMapping::isFinished()
    {
        return mbFinish;
    }

    void PointCloudMapping::insertKeyFrame(KeyFrame* kf, const cv::Mat& color, const cv::Mat& depth,vector<KeyFrame*> vpKFs)
    {
        unique_lock<mutex> locker(mKeyFrameMtx);
        cout << GREEN <<"receive a keyframe, id = "<<kf->mnId<<" 第"<<kf->mnId<<"个"<<endl;
        mvKeyFrames.push(kf);
        keyframes.push_back( kf );
        currentvpKFs = vpKFs;
        cv::Mat colorImg_ ,depth_;
        mvColorImgs.push( color.clone() );
        mvDepthImgs.push( depth.clone() );
        mKeyFrameUpdatedCond.notify_one();
    }

    void PointCloudMapping::NormalshowPointCloud()
    {
        //pcl::visualization::CloudViewer viewer("Dense pointcloud viewer");
        PointCloude pointcloude;
        ros::NodeHandle n;
        pclPoint_pub = n.advertise<sensor_msgs::PointCloud2>("/ORB_SLAM3/Point_Clouds",1000000);
        pclPoint_local_pub = n.advertise<sensor_msgs::PointCloud2>("/ORB_SLAM3/Point_local_Clouds",1000000);
        ros::Rate r(5);
        while(true) {
            KeyFrame* kf;
            cv::Mat colorImg, depthImg;
            {
                std::unique_lock<std::mutex> locker(mKeyFrameMtx);
                while(mvKeyFrames.empty() && !mbShutdown){
                    mKeyFrameUpdatedCond.wait(locker);

                }
                {
                    unique_lock<mutex> lck( keyframeMutex );
                }
                if(lastKeyframeSize  == LoopKfId)
                    updatecloud();
                if (!(mvDepthImgs.size() == mvColorImgs.size() && mvKeyFrames.size() == mvColorImgs.size())) {
                    std::cout << RED << "这是不应该出现的情况！" << std::endl;
                    continue;
                }

                if (mbShutdown && mvColorImgs.empty() && mvDepthImgs.empty() && mvKeyFrames.empty()) {
                    break;
                }
                kf = mvKeyFrames.front();
                colorImg = mvColorImgs.front();
                depthImg = mvDepthImgs.front();
                mvKeyFrames.pop();
                mvColorImgs.pop();
                mvDepthImgs.pop();
            }

            if (mCx==0 || mCy==0 || mFx==0 || mFy==0) {
                mCx = kf->cx;
                mCy = kf->cy;
                mFx = kf->fx;
                mFy = kf->fy;
            }

            {
                std::unique_lock<std::mutex> locker(mPointCloudMtx);
                cv::Mat mTcw_Mat = kf->GetPoseMat();
                pointcloude.pcE=generatePointCloud(kf,colorImg, depthImg, mTcw_Mat);
                pointcloude.pcID = kf->mnId;
                pointcloude.T = ORB_SLAM3::Converter::toSE3Quat(mTcw_Mat);
                /// 以下是计算角度的代码
//                Eigen::Vector3f currentPoint = kf->GetTranslationRos();
//                // 原点在这里假设为坐标系的原点 (0, 0, 0)
//                Eigen::Vector3f origin(0.0, 0.0, 0.0);
//
//                // 计算当前点与原点连成的向量
//                Eigen::Vector3f vectorToOrigin = currentPoint - origin;
//
//                // 计算向量与水平面的夹角
//                double angleToHorizontal = std::atan2(vectorToOrigin(2), std::sqrt(vectorToOrigin(0) * vectorToOrigin(0) + vectorToOrigin(1) * vectorToOrigin(1)));
//
//                // 将弧度转换为度数
//                double angleDegrees = angleToHorizontal * 180.0 / M_PI;
//
//                // 输出结果
//                std::cout << "Angle to horizontal plane: " << angleDegrees << " degrees" << std::endl;
                ///
                pointcloud.push_back(pointcloude);
                //viewer.showCloud(mPointCloud);
                if(pointcloude.pcE->empty())
                    continue;
                pcl_cloud_local_kf = *pointcloude.pcE;
                pcl_cloud_kf = *mPointCloud;
                Cloud_transform(pcl_cloud_local_kf,pcl_local_filter);
                Cloud_transform(pcl_cloud_kf,pcl_filter);
                pcl::toROSMsg(pcl_local_filter, pcl_local_point);
                pcl::toROSMsg(pcl_filter, pcl_point);
                pcl_local_point.header.frame_id = "/pointCloud_local";
                pcl_point.header.frame_id = "/pointCloud";
                pclPoint_local_pub.publish(pcl_local_point);
                pclPoint_pub.publish(pcl_point);
                std::cout << YELLOW << "show point cloud, size=" << mPointCloud->points.size() << std::endl;
                lastKeyframeSize++;
            }
        }
        {
            if(!mPointCloud->empty())
            {
                // 存储点云
                string save_path = "./VSLAMRGBD.pcd";
                pcl::io::savePCDFile(save_path, *mPointCloud);
                cout << GREEN << "save pcd files to :  " << save_path << endl;
            }

        }
        mbFinish = true;
    }


    pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame *kf,const cv::Mat& imRGB, const cv::Mat& imD, const cv::Mat& pose)
    {
        cout << "generatePointCloud" << endl;
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        PointCloud::Ptr current(new PointCloud);
        PointCloud::Ptr loop_points(new PointCloud);
        for(size_t v = 0; v < imRGB.rows ; v+=3){
            for(size_t u = 0; u < imRGB.cols ; u+=3){
                cv::Point2i pt(u,v);
                bool isDynamic = false;
                float d = imD.ptr<float>(v)[u];
                if(d < 0.1 || d>15)
                    continue;
                PointT p;
                p.z = d;
                p.x = ( u - mCx) * p.z / mFx;
                p.y = ( v - mCy) * p.z / mFy;
                p.r = imRGB.ptr<uchar>(v)[u*3];
                p.g = imRGB.ptr<uchar>(v)[u*3+1];
                p.b = imRGB.ptr<uchar>(v)[u*3+2];
                current->points.push_back(p);
                loop_points->points.push_back(p);
            }
        }
        Eigen::Isometry3d T = Converter::toSE3Quat( pose );
        PointCloud::Ptr tmp(new PointCloud);
        // tmp为转换到世界坐标系下的点云
        pcl::transformPointCloud(*current, *tmp, T.inverse().matrix());

        // depth filter and statistical removal，离群点剔除
        statistical_filter.setInputCloud(tmp);
        statistical_filter.filter(*current);
        (*mPointCloud) += *current;

        pcl::transformPointCloud(*mPointCloud, *tmp, T.inverse().matrix());
        // 加入新的点云后，对整个点云进行体素滤波
        voxel.setInputCloud(mPointCloud);
        voxel.filter(*tmp);
        mPointCloud->swap(*tmp);
        mPointCloud->is_dense = false;
        return loop_points;
    }

    void PointCloudMapping::updatecloud()
    {
        {
            std::unique_lock<std::mutex> locker(mPointCloudMtx);
            chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
            cout <<GREEN<< "#--------------------------------------------#" << endl;
            cout<<GREEN<<"暂停稠密点云构建，检测到回环,正在更新点云ing"<<endl;
            Five_pointed_star();
            PointCloud::Ptr tmp1(new PointCloud);
            for (int i=0;i<currentvpKFs.size();i++)
            {
                for (int j=0;j<pointcloud.size();j++)
                {
                    if(pointcloud[j].pcID==currentvpKFs[i]->mnId)
                    {
                        Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat(currentvpKFs[i]->GetPoseMat());
                        PointCloud::Ptr cloud(new PointCloud);
                        pcl::transformPointCloud( *pointcloud[j].pcE, *cloud, T.inverse().matrix());
                        *tmp1 +=*cloud;
                    }
                }
            }
            chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
            chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
            cout <<GREEN<< "#--------------------------------------------#" << endl;
            cout << endl;
            cout <<GREEN<<"      检测到       " << loopcount+1<< "     次回环 "<<endl;
            cout << endl;
            cout <<GREEN<<"      调整了       " << currentvpKFs.size() << "    关键帧位姿  "<<endl;
            cout << endl;
            cout <<GREEN<<"      共计调整了   " << mPointCloud->points.size() << "   个点云位姿  "<<endl;
            cout << endl;
            cout <<GREEN<<"      耗时         " << time_used.count()*1000 << "   ms  "<<endl;
            cout << endl;
            cout <<GREEN<< "#--------------------------------------------#" << endl;
            PointCloud::Ptr tmp2(new PointCloud());
            voxel.setInputCloud( tmp1 );
            voxel.filter( *tmp2 );

            if(mPointCloud->points.size() < 10000)
            {
                cout <<GREEN << "没有检测到回环！"  << endl;
            }
            else
            {
                mPointCloud->points.clear();
                mPointCloud->swap( *tmp2 );
                Five_pointed_star();
                cout << GREEN<<"地图调整完毕，恢复稠密点云构建！"<<endl;
                cout <<GREEN<< "#--------------------------------------------#" << endl;
                loopcount++;
                string save_path = "./VSLAMRGBD_Loop.pcd";
                pcl::io::savePCDFile(save_path, *mPointCloud);
                cout << "updatecloud() save pcd files to :  " << save_path << endl;
            }
        }
    }

    void PointCloudMapping::getGlobalCloudMap(PointCloud::Ptr &outputMap)
    {
        std::unique_lock<std::mutex> locker(mPointCloudMtx);
        outputMap = mPointCloud;
    }
    void PointCloudMapping::Cloud_transform(pcl::PointCloud<pcl::PointXYZRGBA>& source, pcl::PointCloud<pcl::PointXYZRGBA>& out)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered;
        Eigen::Matrix4f m;

        m<< 0,0,1,0,
            -1,0,0,0,
            0,-1,0,0;
        Eigen::Affine3f transform(m);
        pcl::transformPointCloud (source, out, transform);
    }
    void PointCloudMapping::Cloud_transform1(pcl::PointCloud<pcl::PointXYZRGBA>& source, pcl::PointCloud<pcl::PointXYZRGBA>& out, double angleDegrees)
    {
        // 基础变换矩阵
        Eigen::Matrix4f m;
        m << 0, 0, 1, 0,
                -1, 0, 0, 0,
                0, -1, 0, 0,
                0, 0, 0, 1;

        // 将点云调整到水平方向
        Eigen::Matrix4f combinedTransformation = Eigen::Matrix4f::Identity();
        combinedTransformation.block<3, 3>(0, 0) = m.block<3, 3>(0, 0);

        pcl::transformPointCloud(source, out, combinedTransformation);
    }


    void PointCloudMapping::Five_pointed_star()
    {
        int i,j;
        for(i=1;i<=6;i++)  //处理上层顶角
        {
            for(j=0;j<19-i;j++)  //输出空格
                printf(" ");
            for(j=0;j<2*i-1;j++)  //输出*号
                printf("*");
            printf("\n");
        }
        for(i=6;i>=3;i--)  //处理中层
        {
            for(j=0;j<6-i;j++)
                printf("   ");
            for(j=0;j<6*i;j++)
                printf("*");
            printf("\n");
        }
        for(i=1;i<=6;i++)  //处理下层
        {   for(int j=0 ; j<12-i; j++ )
                printf(" ");
            for(int j=0 ; j<12+2*i ; j++) {
                if(i<=2)
                    printf("*");
                else {
                    if(j>=14-2*i && j<=4*i-3)
                        printf(" ");
                    else
                        printf("*");
                }
            }
            printf("\n");
        }
    }
}
