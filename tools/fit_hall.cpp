//pcl
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/filters/voxel_grid.h>//滤波
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/crop_box.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h> //ICP(iterative closest point)配准

#include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

//STD C++ INCLUDES
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <sstream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <memory>
#include <dirent.h>

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;


void extractPlaneFeatureFromCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_source,
                                  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_destin,
                                  pcl::ModelCoefficients::Ptr& coefficient);

int main(int argc, char** argv) 
{
    if(argc!=2)
    {
        cerr<<"args error"<<endl;
        cerr<<"usage fit_hall  *.pcd"<<endl;
        return -1;
    }

    pcl::visualization::PCLVisualizer viewer ( "3D viewer" );
    viewer.setBackgroundColor ( 1.0,1.0,1.0 );
    string filenames = string(argv[1]);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cell(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr onlywall (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plain1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plain2 (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::ModelCoefficients::Ptr coefficient1(new pcl::ModelCoefficients());
    pcl::ModelCoefficients::Ptr coefficient2(new pcl::ModelCoefficients());

    pcl::PCDReader reader;
    reader.read<pcl::PointXYZRGBA>(filenames, *cloud);

//    extractPlaneFeatureFromCloud(cloud, cell, coefficient1);
//    std::cerr << "Model coefficients: ax + by + cz + d = 0" << coefficient1->values[0] << " "
//              << coefficient1->values[1] << " "
//              << coefficient1->values[2] << " "
//              << coefficient1->values[3] << std::endl;


    extractPlaneFeatureFromCloud(cloud, cloud_plain1, coefficient1);
    std::cerr << "Model coefficients: ax + by + cz + d = 0" << coefficient1->values[0] << " "
              << coefficient1->values[1] << " "
              << coefficient1->values[2] << " "
              << coefficient1->values[3] << std::endl;

    extractPlaneFeatureFromCloud(cloud, cloud_plain2, coefficient2);

    std::cerr << "Model coefficients: ax + by + cz + d = 0   " << coefficient2->values[0] << " "
              << coefficient2->values[1] << " "
              << coefficient2->values[2] << " "
              << coefficient2->values[3] << std::endl;

    string filenameDistance = "WallDistance.txt";
    ofstream f;
    f.open(filenameDistance.c_str());
    f << fixed;

    for(int i = 0; i < (int)cloud_plain2->points.size(); i++) {
        float x = cloud_plain2->points[i].x;
        float y = cloud_plain2->points[i].y;
        float z = cloud_plain2->points[i].z;
        float a = (float)coefficient1->values[0];
        float b = (float)coefficient1->values[1];
        float c = (float)coefficient1->values[2];
        float d = (float)coefficient1->values[3];
        float dist = abs(a * x + b * y + c * z + d )/pow((a * a + b * b + c * c),0.5);
        cout << dist << endl;
        f << setprecision(6) << dist << endl;
    }
    f.close();
    cout << endl << "Distance saved!" << endl;



    *onlywall = *cloud_plain2 + *cloud_plain1;

    string filenameWallPoint = "WallInliers.txt";
    f.open(filenameWallPoint.c_str());
    f << fixed;
    for(auto p: onlywall->points) {
        f << setprecision(6) << p.x <<" "<<p.y <<" "<<p.z <<endl;
    }
    cout << endl << "MapPoint Inliers saved!" << endl;
    f.close();

    viewer.removeAllPointClouds();
    viewer.addPointCloud ( onlywall, "wall");
    viewer.addCoordinateSystem ( 1.0 );
    viewer.spin();


}


void extractPlaneFeatureFromCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_source,
                                  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_destin,
                                  pcl::ModelCoefficients::Ptr& coefficient)
{
    // creat the filtering object
    // creat the inliers indices object
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    // create the extract object
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(3000);
    seg.setDistanceThreshold(0.05);

    seg.setInputCloud(cloud_source);
    seg.segment(*inliers, *coefficient);

    extract.setInputCloud(cloud_source);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_destin);

    extract.setNegative(true);
    extract.filter(*cloud_source);

}
