#include <iostream>
#include <vector>
#include <map>
#include <math.h>
#include <queue>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/node_handle.h>
#include <pcl_ros/point_cloud.h>
// #include "std_msgs/String.h"
// #include "mrs_msgs/Reference.h"
// #include <nav_msgs/Odometry.h>
// #include <sstream>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

// #include "angle_criterion.cpp"
// #include "halfdisc_criterion.cpp"
// #include "shape_criterion.cpp"
// #include "boundary_coherence.cpp"

#include "../include/angle_criterion.h"
#include "../include/halfdisc_criterion.h"
#include "../include/shape_criterion.h"
#include "../include/boundary_coherence.h"
// #include "boundary_coherence.cpp"

#include <mrs_lib/param_loader.h>

// #include "auxiliary_functions.h"

// typedef std::vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> > VectorType; 
// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudN;
#include <omp.h>

int main(int argc, char* argv[]){
	const std::string node_name("pc_detection");
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
    mrs_lib::ParamLoader pl(nh, node_name);
	unsigned int queue_length = 1000;
	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", queue_length);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", queue_length);
    ros::Publisher marker_pub2 = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", queue_length);
    ros::Publisher marker_pub3 = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", queue_length);
	
    int K;
    double epsilon;
    std::string name;
    double trashold;
    double k_angle;
    double k_halfdisc;
    double k_shape;
    std::string abs_path;
    double weight_boundary;
    pl.loadParam(node_name + "/K", K);
    pl.loadParam(node_name + "/epsilon", epsilon);
    pl.loadParam(node_name + "/abs_path", abs_path);
    pl.loadParam(node_name + "/file_name", name);
    pl.loadParam(node_name + "/trashold", trashold);
    pl.loadParam(node_name + "/k_angle", k_angle);
    pl.loadParam(node_name + "/k_halfdisc", k_halfdisc);
    pl.loadParam(node_name + "/k_shape", k_shape);
    pl.loadParam(node_name + "/weight_boundary", weight_boundary);

    int q;
    pl.loadParam(node_name + "/q", q);

    
    const std::string pcl_file_name(abs_path + name);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	
	if (pcl::io::loadPCDFile (pcl_file_name, *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file\n");
		return (-1);
	}
	
	std::cout << "Loaded " << cloud->width * cloud->height << " data points with the following fields "	<< std::endl;
	unsigned int coef = 1;
	for (auto& point: *cloud){
        point.x = coef*point.x;
        point.y = coef*point.y;
        point.z = coef*point.z;
        
        point.r = 0;
        point.g = 0;
        point.b = 255;
	}

// 	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
// // pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
//     kdtree.setInputCloud (cloud);
    
// //     pcl::PointXYZRGB searchPoint = (*cloud)[9999];

// //     int K = 100;
// // 
// //     std::vector<int> pointIdxKNNSearch(K);
// //     std::vector<float> pointKNNSquaredDistance(K);
// // 
// //     std::cout << "K nearest neighbor search at (" << searchPoint.x 
// //             << " " << searchPoint.y 
// //             << " " << searchPoint.z
// //             << ") with K=" << K << std::endl;
// // 
// //     if ( kdtree.nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
// //     {
// //         for (std::size_t i = 0; i < pointIdxKNNSearch.size (); ++i){
// //             std::cout << "    "  <<   (*cloud)[ pointIdxKNNSearch[i] ].x 
// //                 << " " << (*cloud)[ pointIdxKNNSearch[i] ].y 
// //                 << " " << (*cloud)[ pointIdxKNNSearch[i] ].z 
// //                 << " (squared distance: " << pointKNNSquaredDistance[i] << ")" << std::endl;
// //             (*cloud)[pointIdxKNNSearch[i]].r = 0;
// //             (*cloud)[pointIdxKNNSearch[i]].g = 0;
// //             (*cloud)[pointIdxKNNSearch[i]].b = 255;
// //         }
// //     }
//     // long int K = 20;
//     // float epsilon = 0.1;
    // std::map<unsigned long int, std::vector< int>> graph;
    // long int neighbors_num = 0;
//     std::cout << "before kd-tree\n";
//     auto start = std::chrono::high_resolution_clock::now();
//     for (unsigned long int i = 0; i < (*cloud).size(); i++){
//         std::vector< int> pointIdxRadiusSearch;
//         std::vector<float> pointRadiusSquaredDistance;
//         kdtree.radiusSearch((*cloud)[i], epsilon, pointIdxRadiusSearch, pointRadiusSquaredDistance);
// //         for(int i = 0; i < pointIdxRadiusSearch.size(); i++){
// //             nn_points.push_back((*cloud)[i]);
// //         }
// //         graph[i] = pointIdxRadiusSearch;
// //         neighbors_num += pointIdxRadiusSearch.size();
//         graph[i].insert(graph[i].end(), pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
        
//         // std::vector<int> pointIdxKNNSearch(K);
//         // std::vector<float> pointKNNSquaredDistance(K);
//         // kdtree.nearestKSearch ((*cloud)[i], K, pointIdxKNNSearch, pointKNNSquaredDistance);
        
//         // graph[i].insert(graph[i].end(), pointIdxKNNSearch.begin(), pointIdxKNNSearch.end());
//         std::vector<int> pointIdxKNNSearch(K);
//         std::vector<float> pointKNNSquaredDistance(K);
//         kdtree.nearestKSearch ((*cloud)[i], K, pointIdxKNNSearch, pointKNNSquaredDistance);
//         graph[i].insert(graph[i].end(), pointIdxKNNSearch.begin(), pointIdxKNNSearch.end());
        
//         for(int j = 0; j < pointIdxRadiusSearch.size(); j++){
//             graph[pointIdxRadiusSearch[j]].push_back(i);
//         }
// //         if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) < (0.6 * avg) ){
// //             searchPoint.r = 255;
// //             searchPoint.g = 0;
// //             searchPoint.b = 0;
// //         }
// 	}
// 	auto end = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double, std::milli> fp_ms = end - start;
//     for (unsigned long int i = 0; i < (*cloud).size(); i++){
//         std::sort(graph[i].begin(), graph[i].end());
//         graph[i].erase(unique(graph[i].begin(), graph[i].end()), graph[i].end());
//         neighbors_num += graph[i].size();
//     }
//     std::cout << "neighbours num " << neighbors_num << std::endl;
//     neighbors_num = 0;
//     std::cout << "slept for " << fp_ms.count() << " milliseconds." << std::endl;
    
// 	int s1 = graph[4].size();
//     int s2 = graph[4444].size();
//     int s3 = graph[44444].size();
//     int s4 = graph[444444].size();
//     graph.clear();
    
//     start = std::chrono::high_resolution_clock::now();
//     for (unsigned long int i = 0; i < (*cloud).size(); i++){
//         std::vector< int> pointIdxRadiusSearch;
//         std::vector<float> pointRadiusSquaredDistance;
//         kdtree.radiusSearch((*cloud)[i], epsilon, pointIdxRadiusSearch, pointRadiusSquaredDistance);
// //         for(int i = 0; i < pointIdxRadiusSearch.size(); i++){
// //             nn_points.push_back((*cloud)[i]);
// //         }
// //         graph[i] = pointIdxRadiusSearch;
//         graph[i].insert(graph[i].end(), pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
        
        
        
//         std::vector<int> pointIdxKNNSearch(K);
//         std::vector<float> pointKNNSquaredDistance(K);
//         kdtree.nearestKSearch ((*cloud)[i], K, pointIdxKNNSearch, pointKNNSquaredDistance);
//         graph[i].insert(graph[i].end(), pointIdxKNNSearch.begin(), pointIdxKNNSearch.end());
        
//         for(int j = 0; j < pointIdxRadiusSearch.size(); j++){
//             graph[pointIdxRadiusSearch[j]].push_back(i);
//         }

//         for(int j = 0; j < pointIdxKNNSearch.size(); j++){
//             graph[pointIdxKNNSearch[j]].push_back(i);
//         }

//         // for(int j = 0; j < graph[i].size(); j++){
//         //     graph[graph[i][j]].push_back(i);
//         // }
// //         if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) < (0.6 * avg) ){
// //             searchPoint.r = 255;
// //             searchPoint.g = 0;
// //             searchPoint.b = 0;
// //         }
// 	}
//     // std::cout << "before erase " << std::endl;
// 	for (unsigned long int i = 0; i < (*cloud).size(); i++){
//         std::sort(graph[i].begin(), graph[i].end());
//         graph[i].erase(unique(graph[i].begin(), graph[i].end()), graph[i].end());
//         neighbors_num += graph[i].size();
//     }
//     std::cout << "neighbours num " << neighbors_num << std::endl;
// 	end = std::chrono::high_resolution_clock::now();
//     fp_ms = end - start;
//     std::cout << "slept for " << fp_ms.count() << " milliseconds." << std::endl;
    
// //     Toto (bez paralelizace) běželo cca 33 min (2.01086e+06 ms)
// //    
// //     for (unsigned long int i = 0; i < (*cloud).size(); i++){
// //         std::vector< int> neighbors = graph[i];
// //         for(unsigned int j = 0; j < neighbors.size(); j++){
// //             if(std::find(graph[j].begin(), graph[j].end(), i) == graph[j].end()){
// //                 graph[j].push_back(i);
// //             }
// //         }
// //     }

//     std::cout << "after kd-tree\n";
    
// //     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//     pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
// //     pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> ne;
//     ne.setInputCloud (cloud);
//     // Create an empty kdtree representation, and pass it to the normal estimation object.
//     // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
// //     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//     pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
// //     pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal> ());
//     ne.setSearchMethod (tree);
//     // Output datasets
//     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//     // Use all neighbors in a sphere of radius 3cm
//     ne.setKSearch(20);
//     // Compute the features
//     ne.compute (*cloud_normals);
//     std::cout << "after normal estimation\n";
    
// //     std::map<unsigned long int, float> angle_prob;
// //     for(int i = 0; i < 1; i++){
// //         double d = (*cloud)[i].x;
// //         double e = (*cloud)[i].y;
// //         double f = (*cloud)[i].z;
// //         double a = cloud_normals->points[i].normal_x;
// //         double b = cloud_normals->points[i].normal_y;
// //         double c = cloud_normals->points[i].normal_z;
// //         std::vector<float> angles;
// //         for(int j = 0; j < graph[i].size(); j++){
// //             double x = (*cloud)[graph[q_i][i]].x;
// //             double y = (*cloud)[graph[q_i][i]].y;
// //             double z = (*cloud)[graph[q_i][i]].z;
// //             double t = (a*d - a*x + b*e - b*y + c*f - c*z)/(a*a + b*b + c*c);
// //             x = x + a*t;
// //             y = y + b*t;
// //             z = z + c*t;
// // //             angles.push_back(atan2(((*cloud)[graph[i][j]]).y, ((*cloud)[graph[i][j]]).x));
// //             double phi = arccos();
// //             std::cout << "angle: " << angles.back() << std::endl;
// //         }
        
// //         geometry_msgs::Point p;
// //         p.x = x + a*t;
// //         p.y = y + b*t;
// //         p.z = z + c*t;
// //         marker.points.push_back(p);
// //     }


// //      ========== ANGLE CRITERION ===========
// //     for each point (starts with small amount)
// //         compute normal vector (point + normal)
// //         for each neighbor
// //             compute projection
// //             compute angle
// //         sort by angle
// //         compute gaps
// //         take biggest gap
// //         compute boundary probability
        

// //     ros::Publisher pc_pub = nh.advertise<PointCloudN>("normal_pointcloud", queue_length);
    
//     // NORMAL VECTORS
//     visualization_msgs::MarkerArray markerArray;
//     for(unsigned long int i = 205000; i < 208500; i++){
//             visualization_msgs::Marker marker;
//             marker.header.frame_id = "base_frame";
//             marker.header.stamp = ros::Time::now();
//             marker.action = visualization_msgs::Marker::ADD;
//             marker.id = i;
//             marker.type = visualization_msgs::Marker::ARROW;
//             marker.scale.x = 0.003;
//             marker.scale.y = 0.005;
// //             marker.scale.z = 0.2;
//             marker.color.g = 1.0f;
//             marker.color.a = 1.0;
// //             marker.pose.position.x = (*cloud)[i].x;
// //             marker.pose.position.y = (*cloud)[i].y;
// //             marker.pose.position.z = (*cloud)[i].z;
//             geometry_msgs::Point p0;
//             p0.x = (*cloud)[i].x;
//             p0.y = (*cloud)[i].y;
//             p0.z = (*cloud)[i].z;
//             geometry_msgs::Point p1;
//             p1.x = (*cloud)[i].x + 0.05*cloud_normals->points[i].normal_x;
//             p1.y = (*cloud)[i].y + 0.05*cloud_normals->points[i].normal_y;
//             p1.z = (*cloud)[i].z + 0.05*cloud_normals->points[i].normal_z;
//             marker.points.push_back(p0);
//             marker.points.push_back(p1);
//             markerArray.markers.push_back(marker);
//     }
// //     int q_i = 60;
// //     (*cloud)[q_i].r = 255;
// //     (*cloud)[q_i].g = 0;
// //     (*cloud)[q_i].b = 0;
//     visualization_msgs::MarkerArray markerArray2;
//     visualization_msgs::MarkerArray markerArray3;

// //     for(int i = q_i; i < q_i + 30; i++){
// //         std::cout << "graph " << i << " size " << graph[i].size() << std::endl;
// //     }
//     // for(int j = 0; j < 1000; j++){
//         int j = 208003;
//         double new_x = 0;
//         double new_y = 0;
//         double new_z = 0;
//         for(int i = 0; i < graph[j].size(); i++){
//             (*cloud)[graph[j][i]].r = 0;
//             (*cloud)[graph[j][i]].g = 255;
//             (*cloud)[graph[j][i]].b = 255;
            
//             visualization_msgs::Marker marker;
//                 marker.header.frame_id = "base_frame";
//                 marker.header.stamp = ros::Time::now();
//                 marker.action = visualization_msgs::Marker::ADD;
//                 marker.id = i;
//                 marker.type = visualization_msgs::Marker::POINTS;
//                 marker.scale.x = 0.01;
//                 marker.scale.y = 0.01;
//                 marker.scale.z = 0.01;
//                 marker.color.r = 1.0f;
//                 marker.color.g = 1.0f;
//                 marker.color.a = 1.0;
//                 double d = (*cloud)[j].x;
//                 double e = (*cloud)[j].y;
//                 double f = (*cloud)[j].z;
//                 double a = cloud_normals->points[j].normal_x;
//                 double b = cloud_normals->points[j].normal_y;
//                 double c = cloud_normals->points[j].normal_z;
//                 double x = (*cloud)[graph[j][i]].x;
//                 double y = (*cloud)[graph[j][i]].y;
//                 double z = (*cloud)[graph[j][i]].z;
//                 double t = (a*d - a*x + b*e - b*y + c*f - c*z)/(a*a + b*b + c*c);
//                 geometry_msgs::Point p;
//                 p.x = x + a*t;
//                 p.y = y + b*t;
//                 p.z = z + c*t;
//                 marker.points.push_back(p);
//                 markerArray2.markers.push_back(marker);


//                 visualization_msgs::Marker marker2;
//                 marker2.header.frame_id = "base_frame";
//                 marker2.header.stamp = ros::Time::now();
//                 marker2.action = visualization_msgs::Marker::ADD;
//                 marker2.id = i + 200;
//                 marker2.type = visualization_msgs::Marker::POINTS;
//                 // marker2.scale.x = 2;
//                 // marker2.scale.y = 2;
//                 // marker2.scale.z = 2;
//                 marker2.scale.x = 0.01;
//                 marker2.scale.y = 0.01;
//                 marker2.scale.z = 0.01;
//                 marker2.color.g = 1.0f;
//                 marker2.color.a = 1.0;
//                 double phi = acos(c/sqrt(a*a + b*b + c*c));
//                 double cosphi = cos(M_PI - phi);
//                 double c1 = 0*c - 1*b;
//                 double c2 = 1*a - 0*c;
//                 double c3 = 0*b - 0*a;
//                 double norm_c = sqrt(c1*c1 + c2*c2 + c3*c3);
//                 double axx = c1/norm_c;
//                 double axy = c2/norm_c;
//                 double axz = c3/norm_c;
//                 c = cosphi;
//                 double s = sqrt(1-c*c);
//                 double C = 1-c;
                
//                 geometry_msgs::Point p2;
//                 p2.x = (axx*axx*C+c)*p.x + (axx*axy*C-axz*s)*p.y + (axx*axz*C+axy*s)*p.z;
//                 p2.y = (axy*axx*C+axz*s)*p.x + (axy*axy*C+c)*p.y + (axy*axz*C-axx*s)*p.z;
//                 p2.z = (axz*axx*C-axy*s)*p.x + (axz*axy*C+axx*s)*p.y + (axz*axz*C+c)*p.z;

//                 // p2.x = p2.x - (*cloud)[q_i].x;
//                 // p2.y = p2.y - (*cloud)[q_i].y;
//                 if((*cloud)[graph[j][i]].x == (*cloud)[j].x){
//                     // std::cout << " == " << std::endl;
//                     marker2.color.r = 1.0f;
//                     marker2.color.g = 0.0f;
//                     new_x = p2.x;
//                     new_y = p2.y;
//                     new_z = p2.z;
//                 }
//                 // std::cout << "point " << i << " (" << p2.x << ", " << p2.y << ", " << p2.z << ")" << std::endl;
//                 // double angle = atan2(y, x);
//                 // marker2.color.r = abs(0.5*angle/M_PI) * 1.0f;
//                 // std::cout << "angle " << angle << " color " << marker2.color.r << std::endl;
//                 marker2.points.push_back(p2);
//                 markerArray3.markers.push_back(marker2);
//         }
//         for(int i = 0; i < markerArray3.markers.size(); i++){
//             markerArray3.markers[i].points[0].x -= new_x;
//             markerArray3.markers[i].points[0].y -= new_y;
//             markerArray3.markers[i].points[0].z -= new_z;
//             double angle = atan2(markerArray3.markers[i].points[0].y, markerArray3.markers[i].points[0].x);
//             markerArray3.markers[i].color.r = 0.5f;
//             markerArray3.markers[i].color.g = abs(angle/M_PI) * 1.0f;
//             markerArray3.markers[i].color.b = abs(angle/M_PI) * 1.0f;
//             markerArray3.markers[i].color.a = 1.0f;
//             if((*cloud)[graph[j][i]].x == (*cloud)[j].x){
//                     std::cout << " == " << std::endl;
//                     markerArray3.markers[i].color.r = 1.0f;
//                     markerArray3.markers[i].color.g = 1.0f;
//                 }
//             // if(i == idx1 || i == idx2){
//             //     markerArray3.markers[i].color.r = 1.0f;
//             //     markerArray3.markers[i].color.g = 1.0f;
//             //     markerArray3.markers[i].color.b = 1.0f;
//             //     markerArray3.markers[i].color.a = 1.0f;
//             // }
//             // std::cout << "angle " << angle << " color " << markerArray3.markers[i].color.b << std::endl;
//         }
    // }
//     (*cloud)[q_i].r = 255;
//     (*cloud)[q_i].g = 0;
//     (*cloud)[q_i].b = 0;
   
//     for(int i = 0; i < markerArray3.markers.size(); i++){
        
//         markerArray3.markers[i].points[0].x -= new_x;
//         markerArray3.markers[i].points[0].y -= new_y;
//         markerArray3.markers[i].points[0].z -= new_z;
//         double angle = atan2(markerArray3.markers[i].points[0].y, markerArray3.markers[i].points[0].x);
//         markerArray3.markers[i].color.r = 0.5f;
//         markerArray3.markers[i].color.g = abs(0.5*angle/M_PI) * 1.0f;
//         markerArray3.markers[i].color.b = abs(0.5*angle/M_PI) * 1.0f;
//         markerArray3.markers[i].color.a = 1.0f;
//         if((*cloud)[graph[q_i][i]].x == (*cloud)[q_i].x){
//                 std::cout << " == " << std::endl;
//                 markerArray3.markers[i].color.r = 1.0f;
//                 markerArray3.markers[i].color.g = 1.0f;
//             }
//         // if(i == idx1 || i == idx2){
//         //     markerArray3.markers[i].color.r = 1.0f;
//         //     markerArray3.markers[i].color.g = 1.0f;
//         //     markerArray3.markers[i].color.b = 1.0f;
//         //     markerArray3.markers[i].color.a = 1.0f;
//         // }
//         std::cout << "angle " << angle << " color " << markerArray3.markers[i].color.b << std::endl;
//     }

//      std::vector<double> angles;
//     std::vector<double> angles_normal;
//     for(int i = 0; i < markerArray3.markers.size(); i++){
//         double angle = atan2(markerArray3.markers[i].points[0].y, markerArray3.markers[i].points[0].x);
//         angles.push_back(angle);
//         angles_normal.push_back(angle);
//     }
//     std::sort(angles.begin(), angles.end());
//     double max_angle = 0;
//     int idx1 = -1;
//     int idx2 = -1;
//     for(int i = 0; i < markerArray3.markers.size() - 1; i++){
//         double angl = abs(angles[i+1] - angles[i]);
//         if(angl > max_angle){
//             // idx2 = i+1;
//             // idx1 = i;
//             idx2 = std::find(angles_normal.begin(), angles_normal.end(), angles[i+1]) - angles_normal.begin();
//             idx1 = std::find(angles_normal.begin(), angles_normal.end(), angles[i]) - angles_normal.begin();
//             max_angle = angl;
//         }
//     }
//     double angl = abs((M_PI - angles[markerArray3.markers.size() - 1]) + angles[0]);
//         if(angl > max_angle){
//             idx2 = std::find(angles_normal.begin(), angles_normal.end(), angles[markerArray3.markers.size() - 1]) - angles_normal.begin();
//             idx1 = std::find(angles_normal.begin(), angles_normal.end(), angles[0]) - angles_normal.begin();
//             max_angle = angl;
//         }
//         std::cout << "max angle: " << max_angle << std::endl;
    
//     markerArray3.markers[idx1].color.r = 1.0f;
//     markerArray3.markers[idx1].color.g = 1.0f;
//     markerArray3.markers[idx1].color.b = 1.0f;
//     markerArray3.markers[idx1].color.a = 1.0f;

//     markerArray3.markers[idx2].color.r = 1.0f;
//     markerArray3.markers[idx2].color.g = 1.0f;
//     markerArray3.markers[idx2].color.b = 1.0f;
//     markerArray3.markers[idx2].color.a = 1.0f;

    
    // std::vector<double> angle_probabilities = angle_criterion(cloud, K, epsilon);
    // std::vector<double> halfdisc_probabilities = halfdisc_criterion(cloud, K, epsilon);
    // std::vector<double> shape_probabilities = shape_criterion(cloud, K, epsilon);

    pcl::PointCloud<pcl::Normal>::Ptr normals = get_normal_vectors(cloud, K, epsilon);
    std::map<unsigned long int, std::vector<int>> neighbours = get_neighbours(cloud, K, epsilon);
    std::map<unsigned long int, std::vector<std::pair<int, double>>> neighbours_distances = get_neighbours_distances(cloud, K, epsilon);
    std::vector<double> average_distances = compute_average_distances(neighbours_distances);

    std::vector<double> angle_probabilities = angle_criterion(cloud, K, epsilon, normals, neighbours);
    std::vector<double> halfdisc_probabilities = halfdisc_criterion(cloud, K, epsilon, normals, neighbours_distances, average_distances);
    std::vector<double> shape_probabilities = shape_criterion(cloud, K, epsilon, normals, neighbours, neighbours_distances, average_distances);

    // std::vector<double> probabilities = angle_criterion(cloud, K, epsilon);
    // std::vector<double> probabilities = halfdisc_criterion(cloud, K, epsilon);
    // std::vector<double> probabilities = shape_criterion(cloud, K, epsilon);
    
    
    std::vector<double> probabilities;
    for(unsigned long int i = 0; i < (*cloud).size(); i++){
        probabilities.push_back(k_halfdisc * halfdisc_probabilities[i] + k_angle * angle_probabilities[i] + k_shape * shape_probabilities[i]);
    }

    std::vector<char> boundaries = get_boundary_points(cloud, K, epsilon, k_angle, k_halfdisc, k_shape, trashold);

    for(unsigned long int i = 0; i < (*cloud).size(); i++){
        // if(probabilities[i] > 1){
        //     std::cerr << "ERROR: " << i << " with prob " << probabilities[i] << std::endl;
        // }
        // else if(probabilities[i] > 0.7){
        //     (*cloud)[i].r = 255;
        //     (*cloud)[i].g = 255;
        //     (*cloud)[i].b = 255;
        // }
        // else if(probabilities[i] > 0.6){
        //     (*cloud)[i].r = 255;
        //     (*cloud)[i].g = 255;
        //     (*cloud)[i].b = 0;
        // }
        // else if(probabilities[i] > trashold){
        //     (*cloud)[i].r = 255;
        //     (*cloud)[i].g = 255;
        //     (*cloud)[i].b = 255;
        //     // (*cloud)[i].g = 0;
        //     // (*cloud)[i].b = 0;
        // }
        // else if(probabilities[i] > 0.4){
        //     (*cloud)[i].r = 0;
        //     (*cloud)[i].g = 255;
        //     (*cloud)[i].b = 0;
        // }
        // else{
        //     (*cloud)[i].r = 0;
        //     (*cloud)[i].g = 0;
        //     (*cloud)[i].b = 255;
        // }
        // (*cloud)[i].r = 0;
        // (*cloud)[i].g = 0;
        // (*cloud)[i].b = 255;
        if(boundaries[i] == 1){
            (*cloud)[i].r = 0;
            (*cloud)[i].g = 0;
            (*cloud)[i].b = 0;
            
            // (*cloud)[i].r = 255;
            // (*cloud)[i].g = 255;
            // (*cloud)[i].b = 255;
        }
    }
    

    // std::vector<std::vector<int>> edges = get_neighbours(cloud, K, epsilon);    // neighbours
    std::cout << "edges" << std::endl;
    std::map<unsigned long int, std::vector<int>> edges = get_neighbours(cloud, K, epsilon);    // neighbours
    std::vector<int> graph_points;  // boundary points
    for(unsigned long int i = 0; i < (*cloud).size(); i++){
        // double prob = k_halfdisc * halfdisc_probabilities[i] + k_angle * angle_probabilities[i] + k_shape * shape_probabilities[i];
        // if(prob > trashold){
        //     graph_points.push_back(i);
        // }
        if(boundaries[i] == 1){
            graph_points.push_back(i);
        }
    }

    // for(int i = 0; i < graph_points.size(); i++){
    //     // for(int j = 0; j < components_points[i].size(); j++){
    //         (*cloud)[graph_points[i]].r = 255;
    //         (*cloud)[graph_points[i]].g = 255;
    //         (*cloud)[graph_points[i]].b = 0;
    //     // }
    // }

    // std::map<unsigned long int, std::vector<std::pair<int, double>>> neighbours_distances = get_neighbours_distances(cloud, K, epsilon);
    // std::vector<double> average_distances = compute_average_distances(neighbours_distances);
    std::cout << "weights" << std::endl;
    std::map<int, std::vector<double>> weights;    // weights in the format of neighbours
    for(unsigned long int l = 0; l < graph_points.size(); l++){
        unsigned long int i = graph_points[l];
        // (*cloud)[i].r = 255;
        // (*cloud)[i].g = 0;
        // (*cloud)[i].b = 0;
        // std::vector<double> weights;
        // std::vector<double> w;
        for(int j = 0; j < edges[i].size(); j++){
            unsigned long int neighbour = edges[i][j];
            if(std::find(graph_points.begin(), graph_points.end(), neighbour) != graph_points.end()){
                double w_prob = 2 - probabilities[i] - probabilities[neighbour];
                // std::cout << "w_prob: " << w_prob;
                double w_density = (2 * vect_norm((*cloud)[i], (*cloud)[neighbour])) / (average_distances[i] + average_distances[neighbour]);
                // std::cout << " w_density: " << w_density << std::endl;
                double weight = w_prob + w_density;
                // weights.push_back(weight);
                // weights[neighbour].insert(weight);
                // weights.insert({neighbour, weight});
                // if(weight < 1.8 && w_prob < 0.9){
                //     weights[i].push_back(weight);
                // }
                weights[i].push_back(weight);

                // std::cout << "( " << weight << " = " << w_prob << " + " << w_density << " )";
                // if(w_prob < 1.1 && w_density < 1){
                //     (*cloud)[i].r = 255;
                //     (*cloud)[i].g = 255;
                //     (*cloud)[i].b = 0;
                // }
            }
        }
        // std::cout << std::endl;
        // std::cout << std::endl;
        // weights[i].push_back(weights);
    }

    // edges to whole graph, meaning for all points which has boundary = 1
    std::vector<Edge> graph;
    std::cout << "graph" << std::endl;
    for(int i = 0; i < graph_points.size(); i++){
        auto edge_from = graph_points[i];
        int edge_num = 0;
        for(int j = 0; j < edges[edge_from].size(); j++){
            auto edge_to = edges[edge_from][j];
            if(std::find(graph_points.begin(), graph_points.end(), edge_to) != graph_points.end()){
                if(std::find_if(graph.begin(), graph.end(), [edge_from, edge_to](auto edge){ return ( (edge_from == edge.from && edge_to == edge.to) || (edge_from == edge.to && edge_to == edge.from) ); }) == graph.end()){
                    int l = edge_from; //graph_points[i];
                    Edge edge;
                    edge.from = l;
                    edge.to = edges[l][j];
                    if(edge.from == edge.to){
                        continue;
                    }
                    // edge.weight = weights[l][j];
                    edge.weight = weights[l][edge_num];
                    if(weights[l][edge_num] < weight_boundary){
                        graph.push_back(edge);
                    }
                    edge_num++;
                    // graph.push_back(edge);
                }
            }
        }
    }

    for(int i = 0; i < graph_points.size(); i++){
        auto point = graph_points[i];
        if(std::find_if(graph.begin(), graph.end(), [point](auto edge){ return ( (point == edge.from || point == edge.to) ); }) == graph.end()){
            graph_points.erase(graph_points.begin() + i);
            auto it = std::find(graph_points.begin(), graph_points.end(), point) - graph_points.begin();
            std::cout << "i: " << i << ", it: " << it << std::endl;
            i--;
        }
    }
    std::cout << "graph points: " << graph_points.size() << std::endl;

    // for(int i = 0; i < graph.size(); i++){
    //     // for(int j = 0; j < components_points[i].size(); j++){
    //         (*cloud)[graph[i].from].r = 255;
    //         (*cloud)[graph[i].from].g = 0;
    //         (*cloud)[graph[i].from].b = 255;

    //         (*cloud)[graph[i].to].r = 255;
    //         (*cloud)[graph[i].to].g = 0;
    //         (*cloud)[graph[i].to].b = 255;
    //     // }
    // }

    std::cout << "sortion" << std::endl;
    std::sort(graph.begin(), graph.end(), [](auto a, auto b){return a.weight < b.weight;});
    Edge min_edge;
    for(Edge edge : graph){
        if(edge.weight < min_edge.weight){
            std::cerr << "ERROR: edges are not sorted!" << std::endl;
        }
        min_edge = edge;
    }
    // list of vectors (for each edge in graph) / ints (for each point in graph)
    std::vector<std::vector<Edge>> components_edges;
    std::vector<std::vector<int>> components_points;
    std::vector<double> components_max_cycle_len;
    std::vector<Edge> components_max_cycle_edge;
    std::cout << "points components" << std::endl;
  	for(int i = 0; i < graph_points.size(); i++){
      	std::vector<int> vc;
        vc.push_back(graph_points[i]);
        components_points.push_back(vc);
	}
    std::cout << "edges components" << std::endl;
    for(int i = 0; i < graph.size(); i++){
        std::vector<Edge> vec;
        components_edges.push_back(vec);
    }
    for(int i = 0; i < graph_points.size(); i++){
        components_max_cycle_len.push_back(0.0);
	}
    for(int i = 0; i < graph_points.size(); i++){
      	Edge e;
        e.from = -1;
        // e.print();
        components_max_cycle_edge.push_back(e);
	}
    std::cout << "MSG" << std::endl;
    std::cout << "Total edges to process: " << components_edges.size() << std::endl;
    std::cout << "Total points to process: " << components_points.size() << std::endl;

    visualization_msgs::MarkerArray markerArray;

    int i = 0;
    int cont = 0;
    int iff = 0;
    int elsee = 0;
    for(Edge edge : graph){
        i++;
        if(i % 1000 == 0){
            std::cout << "i: " << i << " / " << components_edges.size() << " ? " << graph.size() << std::endl;
        }
        // std::cout << "in" << std::endl;
        int idx_from = get_component_idx(components_points, edge.from);
        // std::cout << "after idx from" << std::endl;
        int idx_to = get_component_idx(components_points, edge.to);
        if(idx_to == -1 || idx_from == -1){
            cont++;
            continue;
        }
        
        // std::cout << "from " << edge.from << " at " << idx_from << " to " << edge.to << " at " << idx_to << std::endl;
        // std::cout << "points length " << components_points.size() << " edges length " << components_edges.size() << std::endl;
        // for(auto v : components_edges){
        // 	for(auto e : v){
        //     	e.print();
        //     }
        //     std::cout << "------------" << std::endl;
        // }
        // std::cout << "============" << std::endl;
        if(idx_from != idx_to){
        //     std::cout << "if" << std::endl;
        //     std::cout << "from: ";
        //             components_max_cycle_edge[idx_from].print();
        // std::cout << get_component_idx(components_points, components_max_cycle_edge[idx_from].from) << std::endl;
        // std::cout << get_component_idx(components_points, components_max_cycle_edge[idx_from].to) << std::endl;
        // std::cout << "to: ";
        //             components_max_cycle_edge[idx_to].print();
        // std::cout << get_component_idx(components_points, components_max_cycle_edge[idx_to].from) << std::endl;
        // std::cout << get_component_idx(components_points, components_max_cycle_edge[idx_to].to) << std::endl;
        //     iff++;
        //     std::cout << "current cycle edge: ";
        //             components_max_cycle_edge[idx_from].print();
        //     std::cout << "idx from " << idx_from << ", idx to " << idx_to << std::endl;
        //     std::cout << get_component_idx(components_points, components_max_cycle_edge[idx_from].from) << std::endl;
        //     std::cout << get_component_idx(components_points, components_max_cycle_edge[idx_to].from) << std::endl;
            // if(components_max_cycle_edge[idx_from].from != -1){
            //         std::cout << "current cycle edge: ";
            //         components_max_cycle_edge[idx_from].print();
            //         std::cout << "current cycle length: " << components_max_cycle_len[idx_from] << std::endl;

            //         std::cout << "new cycle edge: ";
            //         edge.print();
            //         // std::cout << "new cycle length: " << length << std::endl;
            //         std::cout << "1" << std::endl;
            //         Edge xtc = components_max_cycle_edge[idx_from];
            //         std::cout << "2" << std::endl;
            //         auto max_it = std::find_if(components_edges[idx_from].begin(), components_edges[idx_from].end(), [xtc](Edge ee){ return (xtc.from == ee.from && xtc.to == ee.to) || (xtc.to == ee.from && xtc.from == ee.to); }) - components_edges[idx_from].begin();
            //         std::cout << "3" << std::endl;
            //         if(components_edges[idx_from].size() > 0){
                    
                    
            //         std::cout << "components_edges[idx_from].size() " << components_edges[idx_from].size() << ", max_it " << max_it << std::endl;
            //         // if(max_it == components_edges[idx_from].size()){
            //         //     std::cout << "\t-----" << std::endl;
            //         //     for(Edge eedgg : components_edges[idx_from]){
            //         //         eedgg.print();
            //         //     } 
            //         //     std::cout << "\t-----" << std::endl;
            //         // }
            //         // if(max_it != components_edges[idx_from].size()){
                    
            //         components_edges[idx_from].erase(components_edges[idx_from].begin() + max_it);
            // }}
            // if(components_max_cycle_edge[idx_to].from != -1){
            //         std::cout << "4" << std::endl;
            //         Edge xuc = components_max_cycle_edge[idx_to];
            //         xuc.print();
            //         // std::cout << "2" << std::endl;
            //         if(components_edges[idx_to].size() > 0){
            //             std::cout << "5" << std::endl;
            //         auto max_it = std::find_if(components_edges[idx_to].begin(), components_edges[idx_to].end(), [xuc](Edge ee){ return (xuc.from == ee.from && xuc.to == ee.to) || (xuc.to == ee.from && xuc.from == ee.to); }) - components_edges[idx_to].begin();
            //         std::cout << "6" << std::endl;
            //         // std::cout << "3" << std::endl;
            //         std::cout << "components_edges[idx_to].size() " << components_edges[idx_to].size() << ", max_it " << max_it << std::endl;
            //         // if(max_it == components_edges[idx_from].size()){
            //         //     std::cout << "\t-----" << std::endl;
            //         //     for(Edge eedgg : components_edges[idx_from]){
            //         //         eedgg.print();
            //         //     }
            //         //     std::cout << "\t-----" << std::endl;
            //         // }
            //         if(max_it != components_edges[idx_to].size()){
            //             std::cout << "6.5" << std::endl;
            //         components_edges[idx_to].erase(components_edges[idx_to].begin() + max_it);

            //         }
            //         }
            //         std::cout << "7" << std::endl;

            //         components_max_cycle_len[idx_from] = 0;
            //         components_max_cycle_len[idx_to] = 0;
            //         Edge e;
            //         e.from = -1;
            //         // e.print();
            //         components_max_cycle_edge[idx_from] = e;
            //         components_max_cycle_edge[idx_to] = e;
            //         std::cout << "8" << std::endl;
            //     }
            // std::cout << "if" << std::endl;
            // auto v = components_points[idx_to];
        	components_points[idx_from].insert(components_points[idx_from].end(), components_points[idx_to].begin(), components_points[idx_to].end());
            // components_points[idx_from].insert(components_points[idx_from].end(), v.begin(), v.end());
            // std::cout << "before erase" << std::endl;
            // std::cout << "components_points.size() " << components_points.size() << std::endl;
            components_points.erase(components_points.begin() + idx_to);
            // std::cout << "components_points.size() " << components_points.size() << std::endl;
            // components_points[idx_to].clear();
            
            // std::cout << "between" << std::endl;
            // if(true){//!components_edges[idx_from].empty()){
            	components_edges[idx_from].push_back(edge);
                // Edge reverse_edge;
                // reverse_edge.from = edge.to;
                // reverse_edge.to = edge.from;
                // reverse_edge.weight = edge.weight;
                // components_edges[idx_from].push_back(reverse_edge);
                // auto w = components_edges[idx_to];
            	components_edges[idx_from].insert(components_edges[idx_from].end(), components_edges[idx_to].begin(), components_edges[idx_to].end());
                // components_edges[idx_from].insert(components_edges[idx_from].end(), w.begin(), w.end());
                // std::cout << "components_edges.size() " << components_edges.size() << std::endl;
            	components_edges.erase(components_edges.begin() + idx_to);
                // std::cout << "components_edges.size() " << components_edges.size() << std::endl;

                // std::cout << "components_max_cycle_edge.size() " << components_max_cycle_edge.size() << std::endl;
                components_max_cycle_edge.erase(components_max_cycle_edge.begin() + idx_to);
                components_max_cycle_edge[idx_from].from = -1;
                // std::cout << "components_max_cycle_edge.size() " << components_max_cycle_edge.size() << std::endl;
                // std::cout << "components_max_cycle_len.size() " << components_max_cycle_len.size() << std::endl;
                components_max_cycle_len.erase(components_max_cycle_len.begin() + idx_to);
                components_max_cycle_len[idx_from] = 0;
                // std::cout << "components_max_cycle_len.size() " << components_max_cycle_len.size() << std::endl;
                // components_edges[idx_to].clear();

    //             visualization_msgs::Marker marker;
    //             marker.header.frame_id = "base_frame";
    //             marker.header.stamp = ros::Time::now();
    //             marker.action = visualization_msgs::Marker::ADD;
    //             marker.id = edge.from*10000 + edge.to*3;
    //             marker.type = visualization_msgs::Marker::ARROW;
    //             marker.scale.x = 0.003;
    //             marker.scale.y = 0.005;
    // //             marker.scale.z = 0.2;
    //             marker.color.g = 1.0f;
    //             marker.color.a = 1.0;
    // //             marker.pose.position.x = (*cloud)[i].x;
    // //             marker.pose.position.y = (*cloud)[i].y;
    // //             marker.pose.position.z = (*cloud)[i].z;
    //             geometry_msgs::Point p0;
    //             p0.x = (*cloud)[edge.from].x;
    //             p0.y = (*cloud)[edge.from].y;
    //             p0.z = (*cloud)[edge.from].z;
    //             geometry_msgs::Point p1;
    //             p1.x = (*cloud)[edge.to].x;
    //             p1.y = (*cloud)[edge.to].y;
    //             p1.z = (*cloud)[edge.to].z;
    //             marker.points.push_back(p0);
    //             marker.points.push_back(p1);
    //             markerArray.markers.push_back(marker);

            // }
            // else{
            // 	components_edges[idx_from].push_back(edge);
            // }
        }
        else{
            elsee++;
            // continue;
            // TODO connecting points in same component
        //     std::cout << "else " << idx_from << std::endl;
        //     std::cout << "from: ";
        //             components_max_cycle_edge[idx_from].print();
        // std::cout << get_component_idx(components_points, components_max_cycle_edge[idx_from].from) << std::endl;
        // std::cout << get_component_idx(components_points, components_max_cycle_edge[idx_from].to) << std::endl;
        // std::cout << "to: ";
        //             components_max_cycle_edge[idx_to].print();
        // std::cout << get_component_idx(components_points, components_max_cycle_edge[idx_to].from) << std::endl;
        // std::cout << get_component_idx(components_points, components_max_cycle_edge[idx_to].to) << std::endl;
            // auto curr_len = components_max_cycle_len[idx_from];
            // auto curr_edg = components_max_cycle_edge[idx_from];
            if(components_max_cycle_edge[idx_from].from != -1){
                    // std::cout << "current cycle edge: ";
                    // components_max_cycle_edge[idx_from].print();
                    // std::cout << "current cycle length: " << components_max_cycle_len[idx_from] << std::endl;

                    // std::cout << "new cycle edge: ";
                    // edge.print();
                    // std::cout << "new cycle length: " << length << std::endl;
                    // std::cout << "1" << std::endl;
                    Edge xtc = components_max_cycle_edge[idx_from];
                    // std::cout << "2" << std::endl;
                    auto max_it = std::find_if(components_edges[idx_from].begin(), components_edges[idx_from].end(), [xtc](Edge ee){ return ((xtc.from == ee.from && xtc.to == ee.to) || (xtc.to == ee.from && xtc.from == ee.to)); }) - components_edges[idx_from].begin();
                    // std::cout << "3" << std::endl;
                    // std::cout << "components_edges[idx_from].size() " << components_edges[idx_from].size() << ", max_it " << max_it << std::endl;
                    // if(max_it == components_edges[idx_from].size()){
                    //     std::cout << "\t-----" << std::endl;
                    //     for(Edge eedgg : components_edges[idx_from]){
                    //         eedgg.print();
                    //     }
                    //     std::cout << "\t-----" << std::endl;
                    // }
                    // if(max_it != components_edges[idx_from].size()){
                    components_edges[idx_from].erase(components_edges[idx_from].begin() + max_it);
                    // }
                    // std::cout << "4" << std::endl;
                }
                // std::cout << "ok 1" << std::endl;
                // components_max_cycle_len[idx_from] = length;
                // components_max_cycle_edge[idx_from].from = edge.from;
                // components_max_cycle_edge[idx_from].to = edge.to;
                // components_max_cycle_edge[idx_from].weight = edge.weight;

            double d = 0.0;
            // auto component_num = components_edges[idx_from];
            // for(int i = 0; i < component_num.size(); i++){
            //     int from = component_num[i].from;
            //     int to = component_num[i].to;
            //     int idx = std::find_if(neighbours_distances[from].begin(), neighbours_distances[from].end(), [to](std::pair<int, double> a) { return a.first == to;}) - neighbours_distances[from].begin();
            //     d += neighbours_distances[from][idx].second;
            // }

            for(int i = 0; i < components_edges[idx_from].size(); i++){
                int from = components_edges[idx_from][i].from;
                int to = components_edges[idx_from][i].to;
                // components_edges[idx_from][i].print();
                int idx = std::find_if(neighbours_distances[from].begin(), neighbours_distances[from].end(), [to](std::pair<int, double> a) { return a.first == to;}) - neighbours_distances[from].begin();
                // std::cout << "idx " << idx << std::endl;
                d += neighbours_distances[from][idx].second;
                // std::cout << "d " << d << std::endl;
            }
            d /= components_edges[idx_from].size();
            double e = (2 * M_PI * epsilon) / d;
            // double e = components_points[idx_from].size() / 7;
            // std::cout << "ok 2" << std::endl;
            // double e = 20.0;
            // std::cout << "e: " << e << ", d: " << d << std::endl;
            // std::cout << "cond: " << components_edges[idx_from].size() + components_edges[idx_to].size() << std::endl;
            // if(components_edges[idx_from].size() + components_edges[idx_to].size() > e){
            // int l = get_cycle_length(components_edges[idx_from], edge.from, edge.to);

            // std::cout << "--- start ---" << std::endl;
            int start = edge.from;
            int end = edge.to;
            // std::vector<Edge> component_edges = components_edges[idx_from];
            std::vector<Edge> visited_edges;
            std::queue<int> q;
            // std::cout << "--- before q push ---" << std::endl;
            q.push(start);
            // std::cout << "--- before while ---" << std::endl;
        //     while(!q.empty()){
        //         // std::cout << "--- in while ---" << std::endl;
        //         int point = q.front();
        //         // std::cout << "--- after front ---" << std::endl;
        //         if(point == end){
        //             // std::cout << "--- end ---" << std::endl;
        //             break;
        //         }
        //         q.pop();
        //         // std::cout << "--- after pop ---" << std::endl;
        //         for(int j = 0; j < component_edges.size(); j++){
        //             // std::cout << "--- in for ---" << std::endl;
        //             Edge edge = component_edges[j];
        //             // std::cout << "--- edges[j] ---" << std::endl;
        //             // if(std::find_if(component_edges.begin(), component_edges.end(), [](auto e){ return (e.from == point || e.to == point); }) != component_edges.end()){
        //             if(std::find_if(visited_edges.begin(), visited_edges.end(), [edge](auto e){ return ((edge.from == e.from && edge.to == e.to) || (edge.from == e.to && edge.to == e.from)); }) != visited_edges.end()){
        //                 continue;
        //             }
        //             if(edge.from == point || edge.to == point){
        //                 // std::cout << "--- if ---" << std::endl;
        // //            	q.push(node);
        //                 Edge e;
        //                 e.from = point;
        //                 e.to = edge.from == point ? edge.to : edge.from;
        //                 q.push(e.to);
        //                 // std::cout << "--- after queue push ---" << std::endl;
        //                 visited_edges.push_back(e);
        //                 // std::cout << "--- after visited push ---" << std::endl;
        //             }
        //         }
        //     }
            int cou = 0;
            while(!q.empty() && cou < 100){
                cou++;
                // std::cout << "--- in while ---" << std::endl;
                int point = q.front();
                // std::cout << "--- after front ---" << std::endl;
                if(point == end){
                    // std::cout << "--- end ---" << std::endl;
                    break;
                }
                q.pop();
                // std::cout << "--- after pop ---" << std::endl;
                for(int j = 0; j < components_edges[idx_from].size(); j++){
                    // std::cout << "--- in for ---" << std::endl;
                    Edge comp_edge = components_edges[idx_from][j];
                    // std::cout << "--- edges[j] ---" << std::endl;
                    // if(std::find_if(component_edges.begin(), component_edges.end(), [](auto e){ return (e.from == point || e.to == point); }) != component_edges.end()){
                    if(std::find_if(visited_edges.begin(), visited_edges.end(), [comp_edge](auto e){ return ((comp_edge.from == e.from && comp_edge.to == e.to) || (comp_edge.from == e.to && comp_edge.to == e.from)); }) != visited_edges.end()){
                        continue;
                    }
                    if(comp_edge.from == point || comp_edge.to == point){
                        // std::cout << "--- if ---" << std::endl;
        //            	q.push(node);
                        Edge e;
                        e.from = point;
                        e.to = comp_edge.from == point ? comp_edge.to : comp_edge.from;
                        q.push(e.to);
                        // std::cout << "--- after queue push ---" << std::endl;
                        visited_edges.push_back(e);
                        // std::cout << "--- after visited push ---" << std::endl;
                    }
                }
            }
            // std::cout << "ok 3" << std::endl;
            int length = 0;
            int j = 0;
            int s = end;
            bool f = true;
            // std::cout << "--- before while ---" << std::endl;
            // std::cout << "visited edges len: " << visited_edges.size() << std::endl;
            while(f && j < 1000){
                // std::cout << "j: " << j << std::endl;
                for(Edge e : visited_edges){
                    // std::cout << "--- in for ---" << std::endl;
                    if(e.to == s){
                        // std::cout << "--- len++ and break ---" << std::endl;
                        length++;
                        s = e.from;
                        break;
                    }
                    if(s == start){
                        // std::cout << "--- end and break ---" << std::endl;
                        f = false;
                        break;
                    }
                }
                
                j++;
            }
            // std::cout << "ok 4" << std::endl;
            // std::cout << "length: " << length << std::endl;
            // std::cout << "e: " << e << ", length: " << length << std::endl;
            if(length > e && length > components_max_cycle_len[idx_from]){
                std::cout << "in IF" << std::endl;
                std::cout << "processed edge: ";
                edge.print();
                std::cout << "processed length: " << length << std::endl;
                std::cout << "current cycle edge: ";
                components_max_cycle_edge[idx_from].print();
                std::cout << "current cycle length: " << components_max_cycle_len[idx_from] << std::endl;

                // std::cout << "new cycle edge: ";
                // edge.print();
                // std::cout << "new cycle length: " << length << std::endl;

                // if(components_max_cycle_edge[idx_from].from == -1){
                //     std::cout << "IS -1" << std::endl;
                // }
                // else{
                //     std::cout << "IS NOT -1" << std::endl;
                // }
                // if(components_max_cycle_edge[idx_from].from != -1){
                //     std::cout << "current cycle edge: ";
                //     components_max_cycle_edge[idx_from].print();
                //     std::cout << "current cycle length: " << components_max_cycle_len[idx_from] << std::endl;

                //     std::cout << "new cycle edge: ";
                //     edge.print();
                //     std::cout << "new cycle length: " << length << std::endl;
                //     std::cout << "1" << std::endl;
                //     Edge xtc = components_max_cycle_edge[idx_from];
                //     std::cout << "2" << std::endl;
                //     auto max_it = std::find_if(components_edges[idx_from].begin(), components_edges[idx_from].end(), [xtc](Edge ee){ return (xtc.from == ee.from && xtc.to == ee.to) || (xtc.to == ee.from && xtc.from == ee.to); }) - components_edges[idx_from].begin();
                //     std::cout << "3" << std::endl;
                //     std::cout << "components_edges[idx_from].size() " << components_edges[idx_from].size() << ", max_it " << max_it << std::endl;
                //     if(max_it == components_edges[idx_from].size()){
                //         std::cout << "\t-----" << std::endl;
                //         for(Edge eedgg : components_edges[idx_from]){
                //             eedgg.print();
                //         }
                //         std::cout << "\t-----" << std::endl;
                //     }
                //     if(max_it != components_edges[idx_from].size()){
                //     components_edges[idx_from].erase(components_edges[idx_from].begin() + max_it);
                //     }
                //     std::cout << "4" << std::endl;
                // }
                // // std::cout << "ok 1" << std::endl;
                std::cout << "after change" << std::endl;
                components_max_cycle_len[idx_from] = length;
                components_max_cycle_edge[idx_from].from = edge.from;
                components_max_cycle_edge[idx_from].to = edge.to;
                components_max_cycle_edge[idx_from].weight = edge.weight;
                std::cout << "processed edge: ";
                edge.print();
                std::cout << "processed length: " << length << std::endl;
                std::cout << "current cycle edge: ";
                components_max_cycle_edge[idx_from].print();
                std::cout << "current cycle length: " << components_max_cycle_len[idx_from] << std::endl;
                // for(int k = 0; k < components_edges[idx_from].size(); k++){
                //     components_edges[idx_from][k].print();
                // }
                // std::cout << "ok 2" << std::endl;
                // std::cout << "cycle size: " << components_edges[idx_from].size() << std::endl;
                // std::cout << "cond ok" << std::endl;
                // components_points[idx_from].insert(components_points[idx_from].end(), components_points[idx_to].begin(), components_points[idx_to].end());
                // std::cout << "before erase" << std::endl;
                // components_points.erase(components_points.begin() + idx_to);
                
                // std::cout << "between" << std::endl;
                // if(true){//!components_edges[idx_from].empty()){
            	components_edges[idx_from].push_back(edge);

                // auto edge = components_edges[i][j];
    //             visualization_msgs::Marker marker;
    //             marker.header.frame_id = "base_frame";
    //             marker.header.stamp = ros::Time::now();
    //             marker.action = visualization_msgs::Marker::ADD;
    //             marker.id = edge.from*10000 + edge.to*3;
    //             marker.type = visualization_msgs::Marker::ARROW;
    //             marker.scale.x = 0.003;
    //             marker.scale.y = 0.005;
    // //             marker.scale.z = 0.2;
    //             marker.color.r = 1.0f;
    //             marker.color.g = 1.0f;
    //             marker.color.b = 1.0f;
    //             marker.color.a = 1.0;
    // //             marker.pose.position.x = (*cloud)[i].x;
    // //             marker.pose.position.y = (*cloud)[i].y;
    // //             marker.pose.position.z = (*cloud)[i].z;
    //             geometry_msgs::Point p0;
    //             p0.x = (*cloud)[edge.from].x;
    //             p0.y = (*cloud)[edge.from].y;
    //             p0.z = (*cloud)[edge.from].z;
    //             geometry_msgs::Point p1;
    //             p1.x = (*cloud)[edge.to].x;
    //             p1.y = (*cloud)[edge.to].y;
    //             p1.z = (*cloud)[edge.to].z;
    //             marker.points.push_back(p0);
    //             marker.points.push_back(p1);
    //             markerArray.markers.push_back(marker);

                // std::cout << "1" << std::endl;
                // Edge reverse_edge;
                // reverse_edge.from = edge.to;
                // reverse_edge.to = edge.from;
                // reverse_edge.weight = edge.weight;
                // // std::cout << "2" << std::endl;
                // components_edges[idx_from].push_back(reverse_edge);
                // std::cout << "3" << std::endl;

            	// components_edges[idx_from].insert(components_edges[idx_from].end(), components_edges[idx_to].begin(), components_edges[idx_to].end());
            	// std::cout << "4" << std::endl;
                // components_edges.erase(components_edges.begin() + idx_to);
                // std::cout << "5" << std::endl;
            }
            else if(components_max_cycle_edge[idx_from].from != -1){
                std::cout << "in ELSE" << std::endl;
                std::cout << "processed edge: ";
                edge.print();
                std::cout << "processed length: " << length << std::endl;
                std::cout << "current cycle edge: ";
                components_max_cycle_edge[idx_from].print();
                std::cout << "current cycle length: " << components_max_cycle_len[idx_from] << std::endl;
                auto sdlfkj = components_max_cycle_edge[idx_from];
                components_edges[idx_from].push_back(sdlfkj);
                // for(int k = 0; k < components_edges[idx_from].size(); k++){
                //     components_edges[idx_from][k].print();
                // }
            }
        }
        // std::cout << "----------" << std::endl;
    }

    // for(int i = 0; i < components_points.size(); i++){
    //     for(int j = 0; j < components_points[i].size(); j++){
    //         (*cloud)[components_points[i][j]].r = 255;
    //         (*cloud)[components_points[i][j]].g = 0;
    //         (*cloud)[components_points[i][j]].b = 0;
    //     }
    // }

    srand(time(NULL));
    for(int i = 0; i < components_edges.size(); i++){
        int r = rand() % 255;
        int g = rand() % 255;
        int b = rand() % 255;
        for(int j = 0; j < components_edges[i].size(); j++){
            // (*cloud)[components_edges[i][j].from].r = 255;
            // (*cloud)[components_edges[i][j].from].g = 0;
            // (*cloud)[components_edges[i][j].from].b = 0;

            // (*cloud)[components_edges[i][j].to].r = 255;
            // (*cloud)[components_edges[i][j].to].g = 0;
            // (*cloud)[components_edges[i][j].to].b = 0;

            (*cloud)[components_edges[i][j].from].r = r;
            (*cloud)[components_edges[i][j].from].g = g;
            (*cloud)[components_edges[i][j].from].b = b;

            (*cloud)[components_edges[i][j].to].r = r;
            (*cloud)[components_edges[i][j].to].g = g;
            (*cloud)[components_edges[i][j].to].b = b;
        }
    }

    std::cout << "iff: " << iff << std::endl;
    std::cout << "elsee: " << elsee << std::endl;
    std::cout << "Points skipped: " << cont << std::endl;
    std::cout << "DONE" << std::endl;
    // std::cout << "========== COMPONENTS ==========" << std::endl;
//     visualization_msgs::MarkerArray markerArray;
//     // for(auto vec : components_edges){
//     for(int i = 0; i < components_edges.size(); i++){
//         // for(auto edge : vec){
//         for(int j = 0; j < components_edges[i].size(); j++){
//             auto edge = components_edges[i][j];
//         	// edge.print();
//             visualization_msgs::Marker marker;
//             marker.header.frame_id = "base_frame";
//             marker.header.stamp = ros::Time::now();
//             marker.action = visualization_msgs::Marker::ADD;
//             marker.id = i*10000 + j*3;
//             marker.type = visualization_msgs::Marker::ARROW;
//             marker.scale.x = 0.003;
//             marker.scale.y = 0.005;
// //             marker.scale.z = 0.2;
//             marker.color.g = 1.0f;
//             marker.color.a = 1.0;
// //             marker.pose.position.x = (*cloud)[i].x;
// //             marker.pose.position.y = (*cloud)[i].y;
// //             marker.pose.position.z = (*cloud)[i].z;
//             geometry_msgs::Point p0;
//             p0.x = (*cloud)[edge.from].x;
//             p0.y = (*cloud)[edge.from].y;
//             p0.z = (*cloud)[edge.from].z;
//             geometry_msgs::Point p1;
//             p1.x = (*cloud)[edge.to].x;
//             p1.y = (*cloud)[edge.to].y;
//             p1.z = (*cloud)[edge.to].z;
//             marker.points.push_back(p0);
//             marker.points.push_back(p1);
//             markerArray.markers.push_back(marker);
//         }
//         // std::cout << "----------" << std::endl;
//     }

    std::cout << "total points count " << components_points.size() << std::endl;
    std::cout << "edges count " << components_edges[0].size() << std::endl;
    std::cout << "total edges count " << components_edges.size() << std::endl;

//     for(auto vec : components_points){
//         // std::cout << "vec size: " << vec.size() << std::endl;
//         for(auto p : vec){
// //             edge.print();
//             // std::cout << p << std::endl;
//             (*cloud)[p].r = 255;
//             (*cloud)[p].g = 0;
//             (*cloud)[p].b = 0;
//         }
//         // std::cout << "----------" << std::endl;
//     }

    std::vector<int> visited_points;
    std::vector<std::vector<Edge>> boundary_edges;
    std::vector<int> boundary_points_start;
    std::vector<int> boundary_points_end;
    for(int i = 0; i < components_points.size(); i++){
//         std::cout << "\t==========\t" << std::endl;
        std::vector<Edge> final_edges;
        auto component_p = components_points[i];
        auto component_e = components_edges[i];
        std::vector<int> colors;
        for(int j = 0; j < component_p.size(); j++){
            colors.push_back(0);
        }
//         std::vector<Edge> visited_edges;
        std::queue<int> q;
        q.push(component_p[0]);
        colors[0] = 1;
        bool f = true;
        int n = 0;
        while(!q.empty() && f){
//             std::cout << "\t----------\t" << std::endl;
            auto new_q = q;
//             std::cout << "queue: ";
//             for(int qc = 0; qc < new_q.size()+1; qc++){
//                 std::cout << new_q.front() << ", ";
//                 new_q.pop();
//             }
//             std::cout << std::endl;
            int point = q.front();
            // if(point < 0 || point > 25000){
            //         std::cout << "ERROR: point " << point << std::endl;
            //     }
            // std::cout << "point " << point << std::endl;
            // (*cloud)[point].r = 255;
            // (*cloud)[point].g = 0;
            // (*cloud)[point].b = 255;
            auto itt = std::find(component_p.begin(), component_p.end(), point) - component_p.begin();
//             std::cout << "itt " << itt << std::endl;
//             std::cout << "colors: ";
//             for(int j = 0; j < colors.size(); j++){
//                 std::cout << colors[j] << ", ";
//             }
//             std::cout << std::endl;
            colors[itt] = 2;
//             if(point == end){
//                 break;
//             }
            q.pop();
//             for(int j = 0; j < edges[i].size(); j++){
//                 auto neigh = edges[i][j];
            for(int j = 0; j < component_p.size(); j++){
                auto neigh = component_p[j];
                // std::cout << "neigh " << neigh << std::endl;
                // if(neigh < 0 || neigh > 25000){
                //     std::cout << "ERROR: neigh " << neigh << std::endl;
                // }
                if(neigh == point){
                    continue;
                }
                // std::cout << "neigh " << neigh << std::endl;
//                 std::cout << "neigh " << neigh << std::endl;
//                 if(std::find(component_p.begin(), component_p.end(), neigh) != component_p.end()){
//                     Edge edge = component_edges[i];
//                     std::cout << "point found" << std::endl;
                    if(std::find_if(component_e.begin(), component_e.end(), [point, neigh](auto e){ return (e.from == point && e.to == neigh) || (e.to == point && e.from == neigh) ; }) != component_e.end()){
//                     if(std::find_if(component_e.begin(), component_e.end(), [point, neigh](auto e){ return (e.from == point) || (e.to == point) ; }) != component_e.end()){
//                         std::cout << "edge found: " << point << "-> " << neigh << std::endl;
//                         auto it = std::find(component_p.begin(), component_p.end(), neigh) - component_p.begin();
                        auto it = j;
//                         std::cout << " it " << it << std::endl;
//                         std::cout << " colors[it] " << colors[it] << std::endl;
//                         std::cout << "point in component " << component_p[it] << std::endl;
                        if(colors[it] == 0){
//                             std::cout << "in if" << std::endl;
            //            	q.push(node);
                            Edge e;
                            e.from = neigh;
                            e.to = point;
                            q.push(e.from);
//                             visited_edges.push_back(e);
                            colors[it] = 1;
                            final_edges.push_back(e);
//                             std::cout << "adding edge: ";
//                             e.print();
                        }
                        else if(colors[it] == 1){
                            // std::cout << "break" << std::endl;
                            // std::cout << "i " << i << ", j " << j << std::endl;
                            f = false;
                            colors[it] == 2;
                            Edge e;
                            e.from = neigh;
                            e.to = point;
                            final_edges.push_back(e);
                            // std::cout << "final edges size " << final_edges.size() << std::endl;
                            boundary_points_start.push_back(point);
                            boundary_points_end.push_back(neigh);
                            break;
                        }
                    }
//                 }
            }
            n++;
            if(n == 100000){
                break;
            }
        }
        if(!f){
            // std::cout << "i " << i << std::endl;
            boundary_edges.push_back(final_edges);
        }
        // boundary_edges.push_back(final_edges);
        
    }

    // for(int i = 0; i < boundary_edges.size(); i++){
    //     for(int j = 0; j < boundary_edges[i].size(); j++){
    //         (*cloud)[boundary_edges[i][j].from].r = 255;
    //         (*cloud)[boundary_edges[i][j].from].g = 255;
    //         (*cloud)[boundary_edges[i][j].from].b = 0;

    //         (*cloud)[boundary_edges[i][j].to].r = 255;
    //         (*cloud)[boundary_edges[i][j].to].g = 255;
    //         (*cloud)[boundary_edges[i][j].to].b = 0;
    //     }
    //     // (*cloud)[boundary_points_start[i]].r = 255;
    //     // (*cloud)[boundary_points_start[i]].g = 0;
    //     // (*cloud)[boundary_points_start[i]].b = 255;

    //     // (*cloud)[boundary_points_end[i]].r = 255;
    //     // (*cloud)[boundary_points_end[i]].g = 0;
    //     // (*cloud)[boundary_points_end[i]].b = 255;
    // }

    // for(int i = 0; i < components_points.size(); i++){
    //     auto vec_edges = components_edges[i];
    //     auto vec_points = components_points[i];
    //     if(vec_edges.size() == 0){
    //         continue;
    //     }
    //     std::vector<int> points_colors;
    //     // 0 = white, 1 = grey, 2 = black;
    //     for(int j = 0; j < vec_points.size(); j++){
    //         points_colors.push_back(0);
    //     }
    //     // std::vector<int> visited_points;
    //     std::queue<int> queue;
    //     std::vector<Edge> backwards_edges;
    //     queue.push(vec_points[0]);
    //     points_colors[0] = 1;
    //     while(!queue.empty()){
    //         // std::cout << "while start" << std::endl;
    //         int point = queue.front();
    //         queue.pop();
    //         points_colors[std::find(vec_points.begin(), vec_points.end(), point) - vec_points.begin()] = 2;
    //         // (*cloud)[point].r = 255;
    //         // (*cloud)[point].g = 0;
    //         // (*cloud)[point].b = 0;
    //         for(int j = 0; j < neighbours[point].size(); j++){
    //             // std::cout << "for each neighbour" << std::endl;
    //             auto neigh = neighbours[point][j];
    //             // std::cout << "before if" << std::endl;
    //             // auto itt = std::find(vec_points.begin(), vec_points.end(), neigh) - vec_points.begin();
    //             // if(points_colors[itt] == 1){
    //             //         std::cout << "END OF BFS - outer" << std::endl;
    //             //         boundary_points_start.push_back(point);
    //             //         std::queue<int> empty;
    //             //         std::swap( queue, empty );
    //             //         std::cout << "before break" << std::endl;
    //             //         break;
    //             //     }
    //             if(std::find_if(vec_edges.begin(), vec_edges.end(), [point, neigh](Edge edge){return ((edge.from == point && edge.to == neigh) || (edge.to == point && edge.from == neigh));}) != vec_edges.end()){
    //             // if(std::find_if(vec_edges.begin(), vec_edges.end(), [point, neigh](Edge edge){return (edge.from == neigh || edge.to == neigh);}) != vec_edges.end()){
    //             // if(true){
    //                 // neighbour point edge
    //                 // std::cout << "after if" << std::endl;
    //                 // std::cout << j << std::endl;
    //                 auto it = std::find(vec_points.begin(), vec_points.end(), neigh) - vec_points.begin();
    //                 // std::cout << "it " << it << std::endl;
    //                 // std::cout << "color " << points_colors[it] << std::endl;
    //                 if(points_colors[it] == 0){
    //                     // std::cout << "new point" << std::endl;
    //                     (*cloud)[neigh].r = 255;
    //                     (*cloud)[neigh].g = 255;
    //                     (*cloud)[neigh].b = 0;
    //                     Edge edg;
    //                     edg.from = neigh;
    //                     edg.to = point;
    //                     // weights are not set
    //                     backwards_edges.push_back(edg);
    //                     points_colors[it] = 1;
    //                     queue.push(neigh);
    //                     visited_points.push_back(neigh);
    //                 }
    //                 else if(points_colors[it] == 1){
    //                     std::cout << "END OF BFS" << std::endl;
    //                     boundary_points_start.push_back(point);
    //                     boundary_points_end.push_back(neigh);
    //                     std::queue<int> empty;
    //                     std::swap( queue, empty );
    //                     std::cout << "before break" << std::endl;
    //                     break;
    //                 }
    //             }
    //         }
    //         // std::cout << "--- 1 ---" << std::endl;
    //         // points_colors[std::find(vec_points.begin(), vec_points.end(), point) - vec_points.begin()] = 2;
    //         // std::cout << "--- 2 ---" << std::endl;
    //     }
    //     // std::cout << "--- 3 ---" << std::endl;
    //     boundary_edges.push_back(backwards_edges);
    //     // std::cout << "--- 4 ---" << std::endl;
    // }

    std::cout << " before last run " << std::endl;
    visualization_msgs::MarkerArray markerArray2;
    std::cout << " boundary_points_start len " << boundary_points_start.size() << std::endl;
    std::cout << " boundary_points_end len " << boundary_points_end.size() << std::endl;
    std::cout << " boundary_edges len " << boundary_edges.size() << std::endl;
    std::vector<std::vector<Edge>> boundary_e;
    std::vector<std::vector<int>> boundary_p;
    for(int i = 0; i < boundary_points_start.size(); i++){
        auto start = boundary_points_start[i];
        auto end = boundary_points_end[i];
//         auto it = std::find_if(boundary_edges[i].begin(), boundary_edges[i].end(), [start](auto c){return e.from == start;}) - boundary_edges[i].begin();
//         auto e = boundary_edges[i][it];
        std::cout << " ---> *_* <--- " << std::endl;
        std::cout << "boundary_edges[i] " << boundary_edges[i].size() << std::endl;
        std::cout << "starting point " << start << std::endl;
        std::cout << "ending point " << end << std::endl;
        std::vector<Edge> edgs;
        std::vector<int> pnts;
        int q = 0;
//         while(true && q < 10){
//             auto it = std::find_if(boundary_edges[i].begin(), boundary_edges[i].end(), [start](auto c){return c.from == start;}) - boundary_edges[i].begin();
//             std::cout << "it " << it << std::endl;
//             auto e = boundary_edges[i][it];
//             e.print();
//             pnts.push_back(start);
//             edgs.push_back(e);
//             start = e.to;
//             
//             if(start == end){
//                 std::cout << "break, q " << q << std::endl;
//                 break;
//             }
//             
//             q++;
//         }
        Edge edg;
        edg.from = start;
        edg.to = end;
        edgs.push_back(edg);
        edg.print();

//         visualization_msgs::Marker marker_;
//             marker_.header.frame_id = "base_frame";
//             marker_.header.stamp = ros::Time::now();
//             marker_.action = visualization_msgs::Marker::ADD;
//             marker_.id = edg.from*1001 + edg.to*3;
//             marker_.type = visualization_msgs::Marker::ARROW;
//             marker_.scale.x = 0.003;
//             marker_.scale.y = 0.005;
// //             marker.scale.z = 0.2;
//             marker_.color.r = 1.0f;
//             marker_.color.a = 1.0;
// //             marker.pose.position.x = (*cloud)[i].x;
// //             marker.pose.position.y = (*cloud)[i].y;
// //             marker.pose.position.z = (*cloud)[i].z;
//             geometry_msgs::Point p;
//             p.x = (*cloud)[edg.from].x;
//             p.y = (*cloud)[edg.from].y;
//             p.z = (*cloud)[edg.from].z;
//             geometry_msgs::Point p_;
//             p_.x = (*cloud)[edg.to].x;
//             p_.y = (*cloud)[edg.to].y;
//             p_.z = (*cloud)[edg.to].z;
//             marker_.points.push_back(p);
//             marker_.points.push_back(p_);
//             markerArray2.markers.push_back(marker_);

        // std::cout << "========---------============" << std::endl;
        // std::cout << "before while for i = " << i << std::endl;
        while(true){
            auto it_s = std::find_if(boundary_edges[i].begin(), boundary_edges[i].end(), [start](Edge c){return c.from == start;}) - boundary_edges[i].begin();
            auto it_e = std::find_if(boundary_edges[i].begin(), boundary_edges[i].end(), [end](Edge c){return c.from == end;}) - boundary_edges[i].begin();
            // std::cout << "boundary_edges[i] " << boundary_edges[i].size() << std::endl;
            // std::cout << "it_s " << it_s << std::endl;
            // std::cout << "it_e " << it_e << std::endl;
            Edge e_s = boundary_edges[i][it_s];
            // e_s.print();
            pnts.push_back(start);
            edgs.push_back(e_s);
            start = e_s.to;
            
            Edge e_e = boundary_edges[i][it_e];
            // e_e.print();
            pnts.push_back(end);
            edgs.push_back(e_e);
            end = e_e.to;

            // std::cout << "i  " << i << std::endl;
            // std::cout << "start == end  " << (start == end) << std::endl;

//             visualization_msgs::Marker marker;
//             marker.header.frame_id = "base_frame";
//             marker.header.stamp = ros::Time::now();
//             marker.action = visualization_msgs::Marker::ADD;
//             marker.id = e_s.from*100 + e_s.to*3;
//             marker.type = visualization_msgs::Marker::ARROW;
//             marker.scale.x = 0.003;
//             marker.scale.y = 0.005;
// //             marker.scale.z = 0.2;
//             marker.color.r = 1.0f;
//             marker.color.a = 1.0;
// //             marker.pose.position.x = (*cloud)[i].x;
// //             marker.pose.position.y = (*cloud)[i].y;
// //             marker.pose.position.z = (*cloud)[i].z;
//             // std::cout << " --- --- " << std::endl;
//             // std::cout << "\tfrom " << e_s.from << std::endl;
//             // std::cout << "\tto " << e_s.to << std::endl;
//             // std::cout << (*cloud)[e_s.from].x << std::endl;
//             // std::cout << (*cloud)[e_s.from].y << std::endl;
//             // std::cout << (*cloud)[e_s.from].z << std::endl;
//             // std::cout << " xxx " << std::endl;
//             // std::cout << (*cloud)[e_s.to].x << std::endl;
//             // std::cout << (*cloud)[e_s.to].y << std::endl;
//             // std::cout << (*cloud)[e_s.to].z << std::endl;
//             // std::cout << " <<< >>> " << std::endl;
//             geometry_msgs::Point p0;
//             // std::cout << " p0__ " << std::endl;
//             p0.x = (*cloud)[e_s.from].x;
//             // std::cout << " p0 " << std::endl;
//             p0.y = (*cloud)[e_s.from].y;
//             p0.z = (*cloud)[e_s.from].z;
//             // std::cout << " p0.x " << p0.x << std::endl;
//             // p0.x = 0;
//             // p0.y = 0;
//             // p0.z = 0;
//             // std::cout << " p0.x " << p0.x << std::endl;
//             // std::cout << " pppp " << std::endl;
//             geometry_msgs::Point p1;
//             p1.x = (*cloud)[e_s.to].x;
//             p1.y = (*cloud)[e_s.to].y;
//             p1.z = (*cloud)[e_s.to].z;
//             // std::cout << " p1.x " << p1.x << std::endl;
//             // p1.x = 0;
//             // p1.y = 0;
//             // p1.z = 0;
//             // std::cout << " p1.x " << p1.x << std::endl;
//             // std::cout << " a " << std::endl;
//             marker.points.push_back(p0);
//             // std::cout << " b " << std::endl;
//             marker.points.push_back(p1);
//             // std::cout << " c " << std::endl;
//             markerArray2.markers.push_back(marker);
//             // std::cout << " === === " << std::endl;

//             visualization_msgs::Marker marker2;
//             marker2.header.frame_id = "base_frame";
//             marker2.header.stamp = ros::Time::now();
//             marker2.action = visualization_msgs::Marker::ADD;
//             marker2.id = e_e.from*100 + e_e.to*3;
//             marker2.type = visualization_msgs::Marker::ARROW;
//             marker2.scale.x = 0.003;
//             marker2.scale.y = 0.005;
// //             marker.scale.z = 0.2;
//             marker2.color.r = 1.0f;
//             marker2.color.a = 1.0;
// //             marker.pose.position.x = (*cloud)[i].x;
// //             marker.pose.position.y = (*cloud)[i].y;
// //             marker.pose.position.z = (*cloud)[i].z;
//             geometry_msgs::Point p2;
//             p2.x = (*cloud)[e_e.from].x;
//             p2.y = (*cloud)[e_e.from].y;
//             p2.z = (*cloud)[e_e.from].z;
//             geometry_msgs::Point p3;
//             p3.x = (*cloud)[e_e.to].x;
//             p3.y = (*cloud)[e_e.to].y;
//             p3.z = (*cloud)[e_e.to].z;
//             marker2.points.push_back(p2);
//             marker2.points.push_back(p3);
//             markerArray2.markers.push_back(marker2);
            
            if(start == end){
                pnts.push_back(start);
//                 std::cout << "break, q " << q << std::endl;
                break;
            }
            
            q++;
            if(q > 10000){
                std::cout << "ERROR: q = 10 000" << std::endl;
                break;
            }
        }
        boundary_e.push_back(edgs);
        boundary_p.push_back(pnts);
        // break;
    }

    // std::cout << "len " << boundary_points_start.size() << std::endl;
    // std::cout << "colorisation" << std::endl;
    // // for(int i = 0; i < boundary_edges.size(); i++){
    // for(int i = 0; i < boundary_points_start.size(); i++){
    //     std::cout << "for start" << std::endl;
    //     auto start = boundary_points_start[i];
    //     auto end = boundary_points_end[i];
    //     std::cout << "after start init: " << start << std::endl;
    //     (*cloud)[start].r = 255;
    //     (*cloud)[start].g = 0;
    //     (*cloud)[start].b = 0;
    //     (*cloud)[end].r = 255;
    //     (*cloud)[end].g = 0;
    //     (*cloud)[end].b = 255;
    //     if(end == start){
    //         (*cloud)[start].r = 0;
    //         (*cloud)[start].g = 255;
    //         (*cloud)[start].b = 255;
    //     }
    //     std::cout << "before 2 start" << std::endl;
    //     std::cout << "edges size: " << boundary_edges[i].size() << std::endl;
    //     for(int j = 0; j < boundary_edges[i].size(); j++){
    //         std::cout << "for 2 start" << std::endl;
    //         std::cout << "boundary_edges[i].size() = " << boundary_edges[i].size() << std::endl;
    //         auto idx = std::find_if(boundary_edges[i].begin(), boundary_edges[i].end(), [start](Edge edge){return (edge.from == start);}) - boundary_edges[i].begin();// - 1;
    //         std::cout << "after idx assignment: " << idx << std::endl;
    //         start = boundary_edges[i][idx].to;
    //         std::cout << "after start assignment: " << start << std::endl;
    //         (*cloud)[start].r = 255;
    //         (*cloud)[start].g = 0;
    //         (*cloud)[start].b = 0;
    //         (*cloud)[end].r = 255;
    //         (*cloud)[end].g = 0;
    //         (*cloud)[end].b = 255;
    //         if(end == start){
    //             (*cloud)[start].r = 0;
    //             (*cloud)[start].g = 255;
    //             (*cloud)[start].b = 255;
    //         }
    //     }

    // }
    std::cout << "coloration" << std::endl;
    for(auto vec : boundary_p){
        std::cout << "vec size: " << vec.size() << std::endl;
        for(auto p : vec){
//             edge.print();
            // std::cout << p << std::endl;
            if(p >= (*cloud).size()){
                continue;
            }
            (*cloud)[p].r = 0;
            (*cloud)[p].g = 255;
            (*cloud)[p].b = 0;
        }
        std::cout << "----------" << std::endl;
    }

//     visualization_msgs::MarkerArray markerArray;
//     // for(auto vec : components_edges){
//     for(int i = 0; i < boundary_e.size(); i++){
//         // for(auto edge : vec){
//         for(int j = 0; j < boundary_e[i].size(); j++){
//             auto edge = boundary_e[i][j];
//         	// edge.print();
//             visualization_msgs::Marker marker;
//             marker.header.frame_id = "base_frame";
//             marker.header.stamp = ros::Time::now();
//             marker.action = visualization_msgs::Marker::ADD;
//             marker.id = i*10000 + j*3;
//             marker.type = visualization_msgs::Marker::ARROW;
//             marker.scale.x = 0.003;
//             marker.scale.y = 0.005;
// //             marker.scale.z = 0.2;
//             marker.color.g = 1.0f;
//             marker.color.a = 1.0;
// //             marker.pose.position.x = (*cloud)[i].x;
// //             marker.pose.position.y = (*cloud)[i].y;
// //             marker.pose.position.z = (*cloud)[i].z;
//             geometry_msgs::Point p0;
//             p0.x = (*cloud)[edge.from].x;
//             p0.y = (*cloud)[edge.from].y;
//             p0.z = (*cloud)[edge.from].z;
//             geometry_msgs::Point p1;
//             p1.x = (*cloud)[edge.to].x;
//             p1.y = (*cloud)[edge.to].y;
//             p1.z = (*cloud)[edge.to].z;
//             marker.points.push_back(p0);
//             marker.points.push_back(p1);
//             markerArray.markers.push_back(marker);
//         }
//         // std::cout << "----------" << std::endl;
//     }
    // for(auto w : weights){
    //     for(auto e : w){
    //         std::cout << e << std::endl;
    //     }
    // }
//     visualization_msgs::MarkerArray markerArray2;
//     for(int i = 0; i < graph.size() && i < 1000; i++){
//         // for(auto edge : vec){
//             Edge edge = graph[i];
//         	edge.print();
//             visualization_msgs::Marker marker;
//             marker.header.frame_id = "base_frame";
//             marker.header.stamp = ros::Time::now();
//             marker.action = visualization_msgs::Marker::ADD;
//             marker.id = i;
//             marker.type = visualization_msgs::Marker::ARROW;
//             marker.scale.x = 0.003;
//             marker.scale.y = 0.005;
// //             marker.scale.z = 0.2;
//             marker.color.r = 1.0f;
//             marker.color.a = 1.0;
// //             marker.pose.position.x = (*cloud)[i].x;
// //             marker.pose.position.y = (*cloud)[i].y;
// //             marker.pose.position.z = (*cloud)[i].z;
//             geometry_msgs::Point p0;
//             p0.x = (*cloud)[edge.from].x;
//             p0.y = (*cloud)[edge.from].y;
//             p0.z = (*cloud)[edge.from].z;
//             geometry_msgs::Point p1;
//             p1.x = (*cloud)[edge.to].x;
//             p1.y = (*cloud)[edge.to].y;
//             p1.z = (*cloud)[edge.to].z;
//             marker.points.push_back(p0);
//             marker.points.push_back(p1);
//             markerArray2.markers.push_back(marker);
//         // }
//         // std::cout << "----------" << std::endl;
//     }

    // std::cout << "==============================" << std::endl;
    // for(auto vec : components_points){
    // 	for(auto i : vec){
    //     	std::cout << i << std::endl;
    //     }
    //     std::cout << "----------" << std::endl;
    // }

    
    // std::map<unsigned long int, std::vector<int>> neighbours = get_neighbours(cloud, K, epsilon);
    // pcl::PointCloud<pcl::Normal>::Ptr normals = get_normal_vectors(cloud, K, epsilon);
    // std::map<unsigned long int, std::vector<std::pair<int, double>>> graph = get_neighbours_distances(cloud, K, epsilon);
    // std::vector<double> average_distances = compute_average_distances(graph);
    // std::vector<Point> mi_p = compute_mi(cloud, graph, average_distances, normals);
    // std::vector<Point> mi_p_prime = project_mis(mi_p, normals, cloud);

    // Point p = (*cloud)[q];
    // visualization_msgs::MarkerArray markerArray2;
    // visualization_msgs::MarkerArray markerArray3;

    // for(int i = 0; i < (*cloud).size() && i < 5000; i++){
    //     auto w = [] (Point p, Point q) -> double { 
    //             return 1;// / vect_norm(p, q); 
    //         };
    //     double c11 = 0;
    //     double c12 = 0;
    //     double c13 = 0;
    //     double c22 = 0;
    //     double c23 = 0;
    //     double c33 = 0;
    //     for(int j = 0; j < neighbours[i].size(); j++){
    //         Point v = (*cloud)[neighbours[i][j]];
    //         double d1 = mi_p[i].x - v.x;
    //         double d2 = mi_p[i].y - v.y;
    //         double d3 = mi_p[i].z - v.z;
    //         double wq = w((*cloud)[i], v);
    //         c11 += wq*d1*d1;
    //         c12 += wq*d1*d2;
    //         c13 += wq*d1*d3;
    //         c22 += wq*d2*d2;
    //         c23 += wq*d2*d3;
    //         c33 += wq*d3*d3;
    //     }
    //     Eigen::MatrixXd C_p (3, 3);
    //     C_p << c11, c12, c13, c12, c22, c23, c13, c23, c33;
    //     Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es;
    //     es.compute(C_p, false);
    //     // SORT EIGENVALUES!!
    //     double l1 = es.eigenvalues()[2];
    //     double l2 = es.eigenvalues()[1];
    //     double l3 = es.eigenvalues()[0];
    //     // double l1 = es.eigenvalues()[0];
    //     // double l2 = es.eigenvalues()[1];
    //     // double l3 = es.eigenvalues()[2];
    //     if(l1 < l2 || l1 < l3 || l2 < l3){
    //         std::cout << l1 << ", " << l2 << ", " << l3 << std::endl;
    //         std::cerr << "ERROR: eigenvalues are sort wrongly" << std::endl;
    //         exit(100);
    //     }
    //     // std::cout << l1 << ", " << l2 << ", " << l3 << std::endl;
    //     double alpha = l1 + l2 + l3;
    //     // std::cout << alpha << std::endl;
    //     Point A_p;
    //     A_p.x = l1 / alpha;
    //     A_p.y = l2 / alpha;
    //     A_p.z = l3 / alpha;
        
    //         visualization_msgs::Marker marker;
    //         marker.header.frame_id = "base_frame";
    //         marker.header.stamp = ros::Time::now();
    //         marker.action = visualization_msgs::Marker::ADD;
    //         marker.id = i+1000;
    //         marker.type = visualization_msgs::Marker::POINTS;
    //         marker.color.r = 1.0f;
    //         marker.color.g = 0.0f;
    //         marker.color.b = 1.0f;
    //         if(probabilities[i] > 0.7){
    //             marker.color.r = 1.0f;
    //             marker.color.g = 0.5f;
    //             marker.color.b = 0.0f;
    //         }
    //         marker.color.a = 1.0f;
    //         marker.scale.x = 0.005;
    //         marker.scale.y = 0.005;
    //         marker.scale.z = 0.005;
    //         geometry_msgs::Point p;
    //         p.x = A_p.x;
    //         p.y = A_p.y;
    //         p.z = A_p.z;
    //         marker.points.push_back(p);
    //         markerArray3.markers.push_back(marker);
        
    // }

    // for(int i = 0; i < neighbours[q].size(); i++){
    //     (*cloud)[neighbours[q][i]].r = 0;
    //     (*cloud)[neighbours[q][i]].g = 255;
    //     (*cloud)[neighbours[q][i]].b = 255;

    //     Point t = tangent_projection(normals->points[q], (*cloud)[q], (*cloud)[neighbours[q][i]]);

    //     visualization_msgs::Marker marker;
    //     marker.header.frame_id = "base_frame";
    //     marker.header.stamp = ros::Time::now();
    //     marker.action = visualization_msgs::Marker::ADD;
    //     marker.id = i;
    //     marker.type = visualization_msgs::Marker::POINTS;
    //     marker.color.r = 1.0f;
    //     marker.color.g = 1.0f;
    //     marker.color.b = 0.0f;
    //     marker.color.a = 1.0f;
    //     marker.scale.x = 0.005;
    //     marker.scale.y = 0.005;
    //     marker.scale.z = 0.005;
    //     geometry_msgs::Point p;
    //     p.x = t.x;
    //     p.y = t.y;
    //     p.z = t.z;
    //     marker.points.push_back(p);
    //     markerArray2.markers.push_back(marker);
    // }

    // visualization_msgs::Marker marker;
    // marker.header.frame_id = "base_frame";
    // marker.header.stamp = ros::Time::now();
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.id = 666;
    // marker.type = visualization_msgs::Marker::POINTS;
    // marker.color.r = 0.0f;
    // marker.color.g = 1.0f;
    // marker.color.b = 0.0f;
    // marker.color.a = 1.0f;
    // marker.scale.x = 0.005;
    // marker.scale.y = 0.005;
    // marker.scale.z = 0.005;
    // geometry_msgs::Point p;
    // p.x = mi_p[q].x;
    // p.y = mi_p[q].y;
    // p.z = mi_p[q].z;
    // marker.points.push_back(p);
    // markerArray2.markers.push_back(marker);

    // visualization_msgs::Marker marker2;
    // marker2.header.frame_id = "base_frame";
    // marker2.header.stamp = ros::Time::now();
    // marker2.action = visualization_msgs::Marker::ADD;
    // marker2.id = 6666;
    // marker2.type = visualization_msgs::Marker::POINTS;
    // marker2.color.r = 1.0f;
    // marker2.color.g = 0.0f;
    // marker2.color.b = 1.0f;
    // marker2.color.a = 1.0f;
    // marker2.scale.x = 0.005;
    // marker2.scale.y = 0.005;
    // marker2.scale.z = 0.005;

    // p.x = mi_p_prime[q].x;
    // p.y = mi_p_prime[q].y;
    // p.z = mi_p_prime[q].z;
    // marker2.points.push_back(p);
    // markerArray2.markers.push_back(marker2);

    // visualization_msgs::Marker marker3;
    // marker3.header.frame_id = "base_frame";
    // marker3.header.stamp = ros::Time::now();
    // marker3.action = visualization_msgs::Marker::ADD;
    // marker3.id = 331;
    // marker3.type = visualization_msgs::Marker::POINTS;
    // marker3.color.r = 1.0f;
    // marker3.color.g = 1.0f;
    // marker3.color.b = 1.0f;
    // marker3.color.a = 1.0f;
    // marker3.scale.x = 0.005;
    // marker3.scale.y = 0.005;
    // marker3.scale.z = 0.005;
    // p.x = 0.66;
    // p.y = 0.33;
    // p.z = 0;
    // marker3.points.push_back(p);
    // markerArray2.markers.push_back(marker3);

    // visualization_msgs::Marker marker4;
    // marker4.header.frame_id = "base_frame";
    // marker4.header.stamp = ros::Time::now();
    // marker4.action = visualization_msgs::Marker::ADD;
    // marker4.id = 332;
    // marker4.type = visualization_msgs::Marker::POINTS;
    // marker4.color.r = 1.0f;
    // marker4.color.g = 0.0f;
    // marker4.color.b = 0.0f;
    // marker4.color.a = 1.0f;
    // marker4.scale.x = 0.005;
    // marker4.scale.y = 0.005;
    // marker4.scale.z = 0.005;
    // p.x = 0.5;
    // p.y = 0.5;
    // p.z = 0;
    // marker4.points.push_back(p);
    // markerArray2.markers.push_back(marker4);

    // visualization_msgs::Marker marker5;
    // marker5.header.frame_id = "base_frame";
    // marker5.header.stamp = ros::Time::now();
    // marker5.action = visualization_msgs::Marker::ADD;
    // marker5.id = 333;
    // marker5.type = visualization_msgs::Marker::POINTS;
    // marker5.color.r = 0.0f;
    // marker5.color.g = 1.0f;
    // marker5.color.b = 0.0f;
    // marker5.color.a = 1.0f;
    // marker5.scale.x = 0.005;
    // marker5.scale.y = 0.005;
    // marker5.scale.z = 0.005;
    // p.x = 0.33;
    // p.y = 0.33;
    // p.z = 0.33;
    // marker5.points.push_back(p);
    // markerArray2.markers.push_back(marker5);

    // visualization_msgs::Marker marker6;
    // marker6.header.frame_id = "base_frame";
    // marker6.header.stamp = ros::Time::now();
    // marker6.action = visualization_msgs::Marker::ADD;
    // marker6.id = 334;
    // marker6.type = visualization_msgs::Marker::POINTS;
    // marker6.color.r = 0.0f;
    // marker6.color.g = 1.0f;
    // marker6.color.b = 1.0f;
    // marker6.color.a = 1.0f;
    // marker6.scale.x = 0.005;
    // marker6.scale.y = 0.005;
    // marker6.scale.z = 0.005;
    // p.x = 1;
    // p.y = 0;
    // p.z = 0;
    // marker6.points.push_back(p);
    // markerArray2.markers.push_back(marker6);

    // visualization_msgs::Marker marker7;
    // marker7.header.frame_id = "base_frame";
    // marker7.header.stamp = ros::Time::now();
    // marker7.action = visualization_msgs::Marker::ADD;
    // marker7.id = 335;
    // marker7.type = visualization_msgs::Marker::POINTS;
    // marker7.color.r = 1.0f;
    // marker7.color.g = 1.0f;
    // marker7.color.b = 0.0f;
    // marker7.color.a = 1.0f;
    // marker7.scale.x = 0.005;
    // marker7.scale.y = 0.005;
    // marker7.scale.z = 0.005;
    // p.x = 0.61;
    // p.y = 0.27;
    // p.z = 0.11;
    // marker7.points.push_back(p);
    // markerArray2.markers.push_back(marker7);

    // visualization_msgs::Marker marker8;
    // marker8.header.frame_id = "base_frame";
    // marker8.header.stamp = ros::Time::now();
    // marker8.action = visualization_msgs::Marker::ADD;
    // marker8.id = 336;
    // marker8.type = visualization_msgs::Marker::POINTS;
    // marker8.color.r = 0.0f;
    // marker8.color.g = 0.0f;
    // marker8.color.b = 0.0f;
    // marker8.color.a = 1.0f;
    // marker8.scale.x = 0.005;
    // marker8.scale.y = 0.005;
    // marker8.scale.z = 0.005;
    // p.x = 0.625;
    // p.y = 0.291;
    // p.z = 0.08;
    // marker8.points.push_back(p);
    // markerArray2.markers.push_back(marker8);

    // visualization_msgs::Marker marker10;
    // marker10.header.frame_id = "base_frame";
    // marker10.header.stamp = ros::Time::now();
    // marker10.action = visualization_msgs::Marker::ADD;
    // marker10.id = 338;
    // marker10.type = visualization_msgs::Marker::POINTS;
    // marker10.color.r = 0.0f;
    // marker10.color.g = 0.5f;
    // marker10.color.b = 1.0f;
    // marker10.color.a = 1.0f;
    // marker10.scale.x = 0.005;
    // marker10.scale.y = 0.005;
    // marker10.scale.z = 0.005;
    // p.x = 0.5;
    // p.y = 0.25;
    // p.z = 0.25;
    // marker10.points.push_back(p);
    // markerArray2.markers.push_back(marker10);

    // (*cloud)[q].r = 255;
    // (*cloud)[q].g = 0;
    // (*cloud)[q].b = 0;

	
    marker_pub.publish(markerArray);
    marker_pub2.publish(markerArray2);
    // marker_pub3.publish(markerArray3);

    std::cout << "===== DONE =====" << std::endl;
    ros::Rate loop_rate(4);
	while (nh.ok())
	{
		// pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
		// pub.publish (*msg);
		cloud->header.frame_id = "base_frame";
        // cloud_normals->header.frame_id = "base_frame";

		//colored_cloud->header.frame_id = "base_frame";
		pub.publish (*cloud);
//         pc_pub.publish(*cloud_normals);
        
        // marker_pub.publish(markerArray);
        // marker_pub2.publish(markerArray2);
        // marker_pub3.publish(markerArray3);

		//pub.publish (*colored_cloud);
		//pub.publish (*cloud_blob);
		//ros::spinOnce ();
		loop_rate.sleep ();
	}

	return 0;
}
