#include <iostream>
#include <vector>
#include <map>
#include <math.h>

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

#include "angle_criterion.cpp"
#include "halfdisc_criterion.cpp"
#include "shape_criterion.cpp"

#include <mrs_lib/param_loader.h>

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
    pl.loadParam(node_name + "/K", K);
    pl.loadParam(node_name + "/epsilon", epsilon);
    pl.loadParam(node_name + "/file_name", name);
    pl.loadParam(node_name + "/trashold", trashold);
    pl.loadParam(node_name + "/k_angle", k_angle);
    pl.loadParam(node_name + "/k_halfdisc", k_halfdisc);
    pl.loadParam(node_name + "/k_shape", k_shape);

	// const std::string pcl_file_name("/home/honzuna/moje/bakalarka/modely/naki_gazebo_resources-master/models/cholina/pcd/cholina.pcd");
    const std::string pcl_file_name("/home/honzuna/moje/bakalarka/modely/naki_gazebo_resources-master/models/" + name);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	
	if (pcl::io::loadPCDFile (pcl_file_name, *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file\n");
		return (-1);
	}
	
	std::cout << "Loaded " << cloud->width * cloud->height << " data points with the following fields "	<< std::endl;
// 	unsigned int coef = 1;
// 	for (auto& point: *cloud){
//         point.x = coef*point.x;
//         point.y = coef*point.y;
//         point.z = coef*point.z;
        
//         point.r = 0;
//         point.g = 0;
//         point.b = 255;
// 	}

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

    
    // std::vector<double> halfdisc_probabilities = halfdisc_criterion(cloud, K, epsilon);
    // std::vector<double> angle_probabilities = angle_criterion(cloud, K, epsilon);

    // std::vector<double> probabilities = halfdisc_criterion(cloud, K, epsilon);
    // std::vector<double> probabilities = angle_criterion(cloud, K, epsilon);
    std::vector<double> probabilities = shape_criterion(cloud, K, epsilon);
    
    
    // std::vector<double> probabilities;
    // for(unsigned long int i = 0; i < (*cloud).size(); i++){
    //     probabilities.push_back(k_halfdisc * halfdisc_probabilities[i] + k_angle * angle_probabilities[i]);
    // }

    for(unsigned long int i = 0; i < (*cloud).size(); i++){
        if(probabilities[i] > 1){
            std::cerr << "ERROR: " << i << " with prob " << probabilities[i] << std::endl;
        }
        else if(probabilities[i] > trashold){
            (*cloud)[i].r = 255;
            (*cloud)[i].g = 0;
            (*cloud)[i].b = 0;
        }
        // else if(probabilities[i] > 0.6){
        //     (*cloud)[i].r = 255;
        //     (*cloud)[i].g = 255;
        //     (*cloud)[i].b = 0;
        // }
        // else if(probabilities[i] > 0.4){
        //     (*cloud)[i].r = 0;
        //     (*cloud)[i].g = 255;
        //     (*cloud)[i].b = 0;
        // }
        else{
            (*cloud)[i].r = 0;
            (*cloud)[i].g = 0;
            (*cloud)[i].b = 255;
        }
    }

	ros::Rate loop_rate(4);
    // marker_pub.publish(markerArray);
    // marker_pub2.publish(markerArray2);
    // marker_pub3.publish(markerArray3);
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
