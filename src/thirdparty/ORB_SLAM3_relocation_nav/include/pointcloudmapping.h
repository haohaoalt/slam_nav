#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <boost/make_shared.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>  // Eigen核心部分
#include <Eigen/Geometry> // 提供了各种旋转和平移的表示
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/projection_matrix.h>
#include <pcl_conversions/pcl_conversions.h>
#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H
#include "KeyFrame.h"
#include "Converter.h"
#include "PointCloude.h"

namespace ORB_SLAM3 {
    class Converter;
    class KeyFrame;
    class PointCloude;
    class Viewer; /// 为什么要加这个类
    class PointCloudMapping {
    public:
        typedef Eigen::Matrix<double, 6, 1> Vector6d;
        typedef pcl::PointXYZRGBA PointT; // A point structure representing Euclidean xyz coordinates, and the RGB color.
        typedef pcl::PointCloud<PointT> PointCloud;
        int loopcount = 0;
        vector<KeyFrame*> currentvpKFs;
        void updatecloud();
        PointCloudMapping(double resolution=0.01);
        ~PointCloudMapping();
        void insertKeyFrame(KeyFrame* kf, const cv::Mat& color, const cv::Mat& depth, vector<KeyFrame*> vpKFs); // 传入的深度图像的深度值单位已经是m
        void requestFinish();
        bool isFinished();
        void getGlobalCloudMap(PointCloud::Ptr &outputMap);
        void Cloud_transform(pcl::PointCloud<pcl::PointXYZRGBA>& source, pcl::PointCloud<pcl::PointXYZRGBA>& out);
        void Cloud_transform1(pcl::PointCloud<pcl::PointXYZRGBA>& source, pcl::PointCloud<pcl::PointXYZRGBA>& out, double angleDegrees);
        void Five_pointed_star();

        bool mbFinish = false;
        int LoopKfId = -1;

    private:
        void NormalshowPointCloud();
        PointCloud::Ptr generatePointCloud(KeyFrame* kf,const cv::Mat& imRGB, const cv::Mat& imD, const cv::Mat& pose);
        double mCx, mCy, mFx, mFy, mResolution;

        std::shared_ptr<std::thread>  viewerThread;

        std::mutex mKeyFrameMtx;
        std::condition_variable mKeyFrameUpdatedCond;
        std::queue<KeyFrame*> mvKeyFrames;
        std::queue<cv::Mat> mvColorImgs, mvDepthImgs;
        bool mbShutdown;
        std::mutex mPointCloudMtx;
        PointCloud::Ptr mPointCloud;

        // filter
        pcl::VoxelGrid<PointT> voxel;
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
        vector<PointCloude>     pointcloud;
        // data to generate point clouds
        vector<KeyFrame*>       keyframes;
        vector<cv::Mat>         colorImgs;
        vector<cv::Mat>         depthImgs;
        vector<cv::Mat>         colorImgks;
        vector<cv::Mat>         depthImgks;
        vector<int>             ids;
        mutex                   keyframeMutex;
        uint16_t                lastKeyframeSize =0;
    };

}
#endif
