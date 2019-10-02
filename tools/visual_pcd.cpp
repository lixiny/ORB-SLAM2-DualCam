//pcl
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <Eigen/StdVector>


#include <memory>
#include <opencv2/core/core.hpp>
#include <map>
#include <unordered_map>
#include<string>
#include<thread>
#include <sys/types.h>
#include <dirent.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>


using namespace std;
using namespace cv;
typedef unsigned char uchar;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

int main() {

    string pcdfile = "/home/sirius/Documents/NRB_SLAM/Hall_test/RGBD/pointcloud.pcd";
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::PCDReader reader;
    reader.read<pcl::PointXYZRGBA>(pcdfile, *cloud);


    pcl::visualization::PCLVisualizer viewer("find_sphere");
    viewer.setBackgroundColor(1.0f,1.0f,1.0f);
    viewer.addCoordinateSystem(0.1);
    viewer.addPointCloud(cloud, "visual_pcd");

    while (!viewer.wasStopped())
    { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }
    return 0;

}
