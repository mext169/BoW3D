#include <ros/ros.h>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>
#include <pcl/filters/extract_indices.h>

#include <sstream>
#include <iomanip>
#include "LinK3D_Extractor.h"
#include "BoW3D.h"
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace BoW3D;

BoW3D::LinK3D_Extractor* pLinK3dExtractor; 
BoW3D::BoW3D* pBoW3D;


/*KITTI's 00, 02, 05, 06, 07, 08 have loops*/
void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *current_cloud);
    Frame *pCurrentFrame = new Frame(pLinK3dExtractor, current_cloud);

    if (pCurrentFrame->mnId < 2) {
        pBoW3D->update(pCurrentFrame);
    } else {
        int loopFrameId = -1;
        Eigen::Matrix3d loopRelR;
        Eigen::Vector3d loopRelt;

        clock_t start, end;
        double time;
        start = clock();

        pBoW3D->retrieve(pCurrentFrame, loopFrameId, loopRelR, loopRelt);

        end = clock();
        time = ((double) (end - start)) / CLOCKS_PER_SEC;

        pBoW3D->update(pCurrentFrame);

        if (loopFrameId == -1) {
            cout << "-------------------------" << endl;
            cout << "Detection Time: " << time << "s" << endl;
            cout << "Frame" << pCurrentFrame->mnId << " Has No Loop..." << endl;
        } else {
            cout << "--------------------------------------" << endl;
            cout << "Detection Time: " << time << "s" << endl;
            cout << "Frame" << pCurrentFrame->mnId << " Has Loop Frame" << loopFrameId << endl;

            cout << "Loop Relative R: " << endl;
            cout << loopRelR << endl;

            cout << "Loop Relative t: " << endl;
            cout << "   " << loopRelt.x() << " " << loopRelt.y() << " " << loopRelt.z() << endl;
        }
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "BoW3D_topic");
    ros::NodeHandle nh("~");

    std::string topic_name;
    nh.param<std::string>("topic_name", topic_name, "/livox/lidar");

    // Parameters of LinK3D
    int nScans; // Number of LiDAR scan lines
    nh.param<int>("nScans", nScans, 64);
    float scanPeriod;
    nh.param<float>("scanPeriod", scanPeriod, 0.1);
    float minimumRange;
    nh.param<float>("minimumRange", minimumRange, 0.1);
    float distanceTh;
    nh.param<float>("distanceTh", distanceTh, 0.4);
    int matchTh;
    nh.param<int>("matchTh", matchTh, 6);

    //Parameters of BoW3D
    float thr;
    nh.param<float>("thr", thr, 3.5);
    int thf = 5;
    nh.param<int>("thf", thf, 5);
    int num_add_retrieve_features;
    nh.param<int>("num_add_retrieve_features", num_add_retrieve_features, 5);

    std::cout << " --- topic_name: " << topic_name << std::endl;
    std::cout << " --- nScans: " << nScans << std::endl;
    std::cout << " --- scanPeriod: " << scanPeriod << std::endl;
    std::cout << " --- minimumRange: " << minimumRange << std::endl;
    std::cout << " --- distanceTh: " << distanceTh << std::endl;
    std::cout << " --- matchTh: " << matchTh << std::endl;
    std::cout << " --- thr: " << thr << std::endl;
    std::cout << " --- thf: " << thf << std::endl;
    std::cout << " --- num_add_retrieve_features: " << num_add_retrieve_features << std::endl;

    pLinK3dExtractor = new BoW3D::LinK3D_Extractor(nScans, scanPeriod, minimumRange, distanceTh, matchTh); 
    pBoW3D = new BoW3D::BoW3D(pLinK3dExtractor, thr, thf, num_add_retrieve_features);

    ros::Subscriber pc_sub = nh.subscribe("/" + topic_name, 1, PointCloudCallback);

    ros::spin();
    return 0;
}