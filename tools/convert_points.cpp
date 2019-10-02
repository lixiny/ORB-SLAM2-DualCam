#include<fstream>
#include<string>
#include<vector>
#include<iostream>
#include <stdio.h>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <Eigen/StdVector>

#include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
using namespace std;

class Point{
public:
    Point(){}
    Point(float _x, float _y, float _z):x(_x),y(_y),z(_z){
        float distsquare = (x * x + y * y + z * z);
        float dists = pow( distsquare, 0.5);
        d = dists;
    }

    float x = 0;
    float y = 0;
    float z = 0;
    float d = 0;
};



int main(int argc ,char** argv)

{

    if(argc!=2)
    {
        cerr<<"args error"<<endl;
        cerr<<"usage convert_points mappoint.txt"<<endl;
        return -1;
    }

    vector<Point> points;
    ifstream input_file;
    string s ;
    string filename  = string(argv[1]);
    input_file.open(argv[1], ifstream::in);
    PointCloud::Ptr cloud ( new PointCloud );
    float dmax = 0;
    float dmin = 1000;
    while(!input_file.eof())
    {
        getline(input_file, s);
        //  cout<<s.c_str()<<endl;
        float x , y , z;
        sscanf(s.c_str(),"%f %f %f",&x,&y,&z);
        Point p(x, y, z);
        if(p.d> 40.f) continue;
        if(p.d > dmax) dmax = p.d;
        if(p.d < dmin) dmin = p.d;
        points.push_back(p);

    }

    float range = dmax - dmin;
    cv::Mat grayImg(1, points.size(), CV_8UC1);
    for(size_t i=0, iend=points.size(); i<iend;i++)
    {
        float dist = points[i].d;
        float ratio = (dmax - dist)/range;
        float gray = 255 * ratio;
        if (gray < 0) gray = 0;
        if (gray > 255) gray = 255;
        grayImg.at<uchar>(0,i) = gray;
        cout << gray << endl;
    }
    cout << "max , min" << dmax << " " << dmin << endl;
    cv::Mat colorImg;
    cv::applyColorMap(grayImg, colorImg, cv::COLORMAP_RAINBOW);
    // cout << colorImg.type() << endl;

    for(size_t i=0, iend=points.size(); i<iend;i++)
    {
        uchar r = colorImg.at<uchar>(0, i * 3);
        uchar g = colorImg.at<uchar>(0, i * 3 + 1);
        uchar b = colorImg.at<uchar>(0, i * 3 + 2);
        PointT p;
        p.x = points[i].x;
        p.y = points[i].y;
        p.z = points[i].z;
        p.r = r;
        p.g = g;
        p.b = b;
        cloud->points.push_back(p);

    }



        cloud->height = 1;
        cloud->width = cloud->points.size();
        cout<<"point cloud size = "<<cloud->points.size()<<endl;
        cloud->is_dense = false;


        //*cloud+=*Traj;
        pcl::io::savePCDFile( filename + ".pcd", *cloud );
        // 清除数据并退出
        pcl::visualization::PCLVisualizer viewer("vis");
        viewer.setBackgroundColor(1.0f,1.0f,1.0f);
        viewer.addCoordinateSystem(5);
        viewer.addPointCloud(cloud, "visual_pcd");

        while (!viewer.wasStopped())
        { // Display the visualiser until 'q' key is pressed
            viewer.spinOnce();
        }
        return 0;
        cloud->points.clear();
        cout<<"Point cloud saved."<<endl;
    return 0;
}
