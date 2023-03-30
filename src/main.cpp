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
    int K_2;
    double epsilon_2;
    std::string name;
    double trashold;
    double k_angle;
    double k_halfdisc;
    double k_shape;
    std::string abs_path;
    double weight_boundary;
    pl.loadParam(node_name + "/K", K);
    pl.loadParam(node_name + "/epsilon", epsilon);
    pl.loadParam(node_name + "/K_2", K_2);
    pl.loadParam(node_name + "/epsilon_2", epsilon_2);
    pl.loadParam(node_name + "/abs_path", abs_path);
    pl.loadParam(node_name + "/file_name", name);
    pl.loadParam(node_name + "/trashold", trashold);
    pl.loadParam(node_name + "/k_angle", k_angle);
    pl.loadParam(node_name + "/k_halfdisc", k_halfdisc);
    pl.loadParam(node_name + "/k_shape", k_shape);
    pl.loadParam(node_name + "/weight_boundary", weight_boundary);

    int n_ransac;
    int k_ransac;
    double t_ransac;
    int d_ransac;
    pl.loadParam(node_name + "/n_ransac", n_ransac);
    pl.loadParam(node_name + "/k_ransac", k_ransac);
    pl.loadParam(node_name + "/t_ransac", t_ransac);
    pl.loadParam(node_name + "/d_ransac", d_ransac);


    // int q;
    // pl.loadParam(node_name + "/q", q);

    
    const std::string pcl_file_name(abs_path + name);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	
	if (pcl::io::loadPCDFile (pcl_file_name, *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file\n");
		return (-1);
	}

    cloud->header.frame_id = "base_frame";
	
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

    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::MarkerArray markerArray2;
    visualization_msgs::MarkerArray markerArray3;



    std::vector<std::vector<int>> boundaries = get_boundary_points(cloud, K, epsilon, K_2, epsilon_2, k_angle, k_halfdisc, k_shape, trashold, weight_boundary);
    std::cout << "coloration" << std::endl;
    // int ii = 0;
    std::cout << "boundary size: " << boundaries.size() << std::endl;
    for(auto vec : boundaries){
        std::cout << "vec size: " << vec.size() << std::endl;
        // std::cout << "vec size: " << vec.size() << " of " << boundary_p.size() << " and " << boundary_e.size() << ", " << boundary_points_start.size() << ", " << boundary_points_end.size() << std::endl;
        for(auto p : vec){
            // edge.print();
            // std::cout << p << std::endl;
            // if(p >= (*cloud).size()){
            //     continue;
            // }
            (*cloud)[p].r = 255;
            (*cloud)[p].g = 255;
            (*cloud)[p].b = 255;
        }
        // auto point = boundary_points_start[ii];
        // auto neigh = boundary_points_end[ii];
        // std::cout << point << std::endl;
        // std::cout << neigh << std::endl;
        // (*cloud)[point].r = 255;
        // (*cloud)[point].g = 0;
        // (*cloud)[point].b = 0;

        // (*cloud)[neigh].r = 255;
        // (*cloud)[neigh].g = 0;
        // (*cloud)[neigh].b = 0;
        // ii++;
        std::cout << "----------" << std::endl;
    }

    // auto current_boundary = boundaries[0];
    std::cout << "best plane computing" << std::endl;
    PointCloud final_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);// = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(cloud);
    final_cloud->width = cloud->width;
    final_cloud->height = cloud->height;
    pcl::copyPointCloud<Point>(*cloud, *final_cloud);
    srand(time(NULL));
    std::vector<std::vector<int>> best_planes;
    std::vector<Point> best_planes_origin;
    std::vector<pcl::Normal> best_planes_normal;
    std::cout << "before for" << std::endl;
    for(auto current_boundary : boundaries){
        std::vector<int> best_plane;
        Point best_plane_origin;
        pcl::Normal best_plane_normal;
        // int error = 100;
        int maximum_points = 0;
        std::cout << "current boundary size: " << current_boundary.size() << std::endl;

        for(int i = 0; i < k_ransac; i++){
            auto new_cloud = cloud;
            Point p;
            p.x = 0.0;
            p.y = 0.0;
            p.z = 0.0;
            p.r = 255;
            p.g = 255;
            p.b = 0;
            p.a = 255;
            std::vector<int> random_indexes;
            for(int j = 0; j < n_ransac; j++){
                auto idx = current_boundary[rand() % current_boundary.size()];
                // std::cout << "random idx: " << idx << std::endl;
                random_indexes.push_back(idx);
                // (*new_cloud)[idx].r = 255;
                // (*new_cloud)[idx].g = 0;
                // (*new_cloud)[idx].b = 0;
                p.x += (*new_cloud)[idx].x;
                p.y += (*new_cloud)[idx].y;
                p.z += (*new_cloud)[idx].z;
            }
            p.x /= 3;
            p.y /= 3;
            p.z /= 3;
            new_cloud->push_back(p);

            // pub.publish (*new_cloud);
            // sleep(2);
            // Point
            // int divide = 10;
            pcl::PointCloud<pcl::Normal>::Ptr normals = get_normal_vectors(new_cloud, K, epsilon);
            auto plane_normal = normals->points[new_cloud->size() - 1];
            std::vector<int> points_below_trashold;
            int sum_of_distances = 0;
            for(int j = 0; j < current_boundary.size(); j++){
                int point_idx = current_boundary[j];
                
                Point projection = tangent_projection(plane_normal, p, (*new_cloud)[point_idx]);
                double dist = vect_norm((*new_cloud)[point_idx], projection);
                sum_of_distances += (dist * dist);
                // std::cout << "dist: " << dist << std::endl;
                if(dist < t_ransac && std::find(random_indexes.begin(), random_indexes.end(), point_idx) == random_indexes.end()){
                    // std::cout << "below trashold" << std::endl;
                    points_below_trashold.push_back(point_idx);
                    // (*new_cloud)[point_idx].r = 0;
                    // (*new_cloud)[point_idx].g = 255;
                    // (*new_cloud)[point_idx].b = 255;
                }
            }

            // pub.publish (*new_cloud);
            // sleep(3);

            if(points_below_trashold.size() > d_ransac){
                int curr_points = points_below_trashold.size();
                if(curr_points > maximum_points){
                    std::cout << "curr maximum points fits: " << curr_points << std::endl;
                    maximum_points = curr_points;
                    best_plane = random_indexes;
                    best_plane_origin = p;
                    best_plane_normal = plane_normal;
                }
            }

            // for(int j = 0; j < current_boundary.size(); j++){
            //     (*new_cloud)[current_boundary[j]].r = 255;
            //     (*new_cloud)[current_boundary[j]].g = 255;
            //     (*new_cloud)[current_boundary[j]].b = 255;
            // }
        }

        best_planes.push_back(best_plane);
        best_planes_origin.push_back(best_plane_origin);
        best_planes_normal.push_back(best_plane_normal);
    }

    std::cout << "vizualisation" << std::endl;
    for(int c = 0; c < best_planes.size(); c++){
        auto best_plane = best_planes[c];
        for(int i = 0; i < best_plane.size(); i++){
            (*final_cloud)[best_plane[i]].r = 255;
            (*final_cloud)[best_plane[i]].g = 0;
            (*final_cloud)[best_plane[i]].b = 0;
        }
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_frame";
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = c;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.003;
        marker.scale.y = 0.005;
        marker.color.g = 1.0f;
        marker.color.a = 1.0;
        geometry_msgs::Point p0;
        p0.x = best_planes_origin[c].x;
        p0.y = best_planes_origin[c].y;
        p0.z = best_planes_origin[c].z;
        geometry_msgs::Point p1;
        p1.x = (best_planes_origin[c].x + best_planes_normal[c].normal_x);
        p1.y = (best_planes_origin[c].y + best_planes_normal[c].normal_y);
        p1.z = (best_planes_origin[c].z + best_planes_normal[c].normal_z);
        marker.points.push_back(p0);
        marker.points.push_back(p1);
        markerArray.markers.push_back(marker);

        visualization_msgs::Marker marker2;
        marker2.header.frame_id = "base_frame";
        marker2.header.stamp = ros::Time::now();
        marker2.action = visualization_msgs::Marker::ADD;
        marker2.id = c+100;
        marker2.type = visualization_msgs::Marker::POINTS;
        marker2.scale.x = 0.005;
        marker2.scale.y = 0.005;
        marker2.scale.z = 0.005;
        marker2.color.r = 1.0f;
        marker2.color.b = 1.0f;
        marker2.color.a = 1.0;
        geometry_msgs::Point p2;
        p2.x = best_planes_origin[c].x;
        p2.y = best_planes_origin[c].y;
        p2.z = best_planes_origin[c].z;
        marker2.points.push_back(p2);
        markerArray2.markers.push_back(marker2);

        for(int j = 0; j < 50; j++){
            visualization_msgs::Marker marker3;
            marker3.header.frame_id = "base_frame";
            marker3.header.stamp = ros::Time::now();
            marker3.action = visualization_msgs::Marker::ADD;
            marker3.id = c+100+2*j;
            marker3.type = visualization_msgs::Marker::POINTS;
            marker3.scale.x = 0.005;
            marker3.scale.y = 0.005;
            marker3.scale.z = 0.005;
            marker3.color.r = 1.0f;
            marker3.color.g = 1.0f;
            marker3.color.a = 1.0;
            geometry_msgs::Point p3;
            p3.x = best_planes_origin[c].x;
            p3.y = best_planes_origin[c].y;
            p3.z = best_planes_origin[c].z;
            marker3.points.push_back(p3);
            markerArray2.markers.push_back(marker3);
        }

    }
    
	
    marker_pub.publish(markerArray);
    marker_pub2.publish(markerArray2);
    // marker_pub3.publish(markerArray3);

    std::cout << " <<< ===== DONE ===== >>>" << std::endl;
    ros::Rate loop_rate(4);
	while (nh.ok())
	{
		// pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
		// pub.publish (*msg);
		// cloud->header.frame_id = "base_frame";
        // cloud_normals->header.frame_id = "base_frame";

		//colored_cloud->header.frame_id = "base_frame";

		// pub.publish (*cloud);
        // pub.publish (*new_cloud);
        pub.publish (*final_cloud);
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
