#include <iostream>
#include <vector>
#include <map>
#include <math.h>
#include <queue>
#include <fstream>

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
#include <pcl/filters/voxel_grid.h>

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

#include <mrs_msgs/Reference.h>
// #include "../../aerial_core_planning/ros_packages/mrs_msgs/srv/GetPath.srv"
// #include "mrs_msgs/GetPathSrv.srv"
#include <mrs_msgs/GetPath.h>

#include <mrs_msgs/PathSrv.h>
#include <mrs_msgs/Path.h>

#include <nav_msgs/Odometry.h>


// #include "auxiliary_functions.h"

// typedef std::vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> > VectorType; 
// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudN;
#include <omp.h>

void callback(const nav_msgs::Odometry::ConstPtr& msg, ros::Publisher pub, PointCloud cld){
    // ROS_INFO("I heard on /uav%d/acceleration", uav->getId());
    // uav->setLinearAcceleration(msg->position.x, msg->position.y, msg->position.z);
    // uav->setAngularAcceleration(msg->heading);
    Point p;
    p.x = msg->pose.pose.position.x;
    p.y = msg->pose.pose.position.y;
    p.z = msg->pose.pose.position.z;
    p.g = 255;
    p.a = 255;
    (*cld).push_back(p);
    // ROS_INFO("\tx: %f, y: %f, z: %f, heading: %f", p.x, p.y, p.z, 0.0);
    pub.publish(cld);
}

void callback_scan(const sensor_msgs::PointCloud2::ConstPtr& msg, ros::Publisher pub, PointCloud cld, std::vector<Point> viewpoints, PointCloud scans, std::vector<bool> visited){
    //(*cld).push_back(p);
    // ROS_INFO("\tx: %f, y: %f, z: %f, heading: %f", p.x, p.y, p.z, 0.0);
    // PointCloud final_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);// = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(cloud);
    // final_cloud->width = msg->width;
    // final_cloud->height = msg->height;
    // final_cloud->header.frame_id = frame_id;
    //pcl::copyPointCloud<Point>(*msg, *final_cloud);
    // final_cloud = msg;
    bool send = false;
    Point drone_pose;
    for(int i = 0; i < viewpoints.size(); i++){
        Point uav_position = cld->back();
        double dist = vect_norm(uav_position, viewpoints[i]);
        if(dist < 0.1){//} && !visited[i] && uav_position.z < 9.0){
            ROS_INFO("Near the viewpoint, scanning.");
            send = true;
            visited[i] = true;
            drone_pose = viewpoints[i];
            std::cout << "visited: ";
            for(int j = 0; j < visited.size(); j++){
                std::cout << visited[j] << " ";
            }
            std::cout << std::endl;
            break;
        }
    }
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    temp_cloud->header.frame_id = "frame";
    if(send){
        // (*scans) += (*temp_cloud);
        for(int i = 0; i < (*temp_cloud).size(); i++){
            Point p = (*temp_cloud)[i];
            //double dist = vect_norm(p, drone_pose);
            double dist = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if(dist < 3 && (p.y + drone_pose.y) < -5){
                p.x += drone_pose.x;
                p.y += drone_pose.y;
                p.z += drone_pose.z;
                p.r = 255;
                p.a = 255;
                (*scans).push_back(p);
            }
        }
        // pub.publish(*temp_cloud);
        pub.publish(*scans);
    }
}

int main(int argc, char* argv[]){
	const std::string node_name("pc_detection");
    const std::string frame_id("frame");
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
    mrs_lib::ParamLoader pl(nh, node_name);
	unsigned int queue_length = 1000;
	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", queue_length);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>("bound", queue_length);
    ros::Publisher pub3 = nh.advertise<sensor_msgs::PointCloud2>("proj", queue_length);
    ros::Publisher pub_callback = nh.advertise<sensor_msgs::PointCloud2>("uav", queue_length);
    ros::Publisher pub_callback_scan = nh.advertise<sensor_msgs::PointCloud2>("scan", queue_length);
    ros::Publisher pub_final_data = nh.advertise<sensor_msgs::PointCloud2>("final_data", queue_length);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", queue_length);
    ros::Publisher marker_pub2 = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", queue_length);
    ros::Publisher marker_pub3 = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", queue_length);
    ros::Publisher marker_path = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", queue_length);
	
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
    int minimum_loop_length;
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
    pl.loadParam(node_name + "/minimum_loop_length", minimum_loop_length);

    int n_ransac;
    int k_ransac;
    double t_ransac;
    int d_ransac;
    pl.loadParam(node_name + "/n_ransac", n_ransac);
    pl.loadParam(node_name + "/k_ransac", k_ransac);
    pl.loadParam(node_name + "/t_ransac", t_ransac);
    pl.loadParam(node_name + "/d_ransac", d_ransac);

    double uav_min_distance;
    double uav_max_distance;
    double uav_min_angle;
    double uav_min_area_width;
    int area_samples_first;
    int area_samples_second;
    int area_samples_mid;
    double uav_sample_width;
    double uav_sample_height;
    pl.loadParam(node_name + "/uav_min_distance", uav_min_distance);
    pl.loadParam(node_name + "/uav_max_distance", uav_max_distance);
    pl.loadParam(node_name + "/uav_min_angle", uav_min_angle);
    pl.loadParam(node_name + "/uav_min_area_width", uav_min_area_width);
    //pl.loadParam(node_name + "/uav_min_area_width", uav_min_area_width);
    pl.loadParam(node_name + "/area_samples_first", area_samples_first);
    pl.loadParam(node_name + "/area_samples_second", area_samples_second);
    pl.loadParam(node_name + "/area_samples_mid", area_samples_mid);
    pl.loadParam(node_name + "/uav_sample_width", uav_sample_width);
    pl.loadParam(node_name + "/uav_sample_height", uav_sample_height);

    double uav_start_x;
    double uav_start_y;
    double uav_start_z;
    double uav_start_heading;
    double uav_end_x;
    double uav_end_y;
    double uav_end_z;
    double uav_end_heading;
    pl.loadParam(node_name + "/uav_start_x", uav_start_x);
    pl.loadParam(node_name + "/uav_start_y", uav_start_y);
    pl.loadParam(node_name + "/uav_start_z", uav_start_z);
    pl.loadParam(node_name + "/uav_start_heading", uav_start_heading);
    pl.loadParam(node_name + "/uav_end_x", uav_end_x);
    pl.loadParam(node_name + "/uav_end_y", uav_end_y);
    pl.loadParam(node_name + "/uav_end_z", uav_end_z);
    pl.loadParam(node_name + "/uav_end_heading", uav_end_heading);

    bool listen;
    pl.loadParam(node_name + "/listen", listen);

    uav_min_angle *= (2 * M_PI) / 360.0 ;

    srand(time(NULL));
    
    std::cout << "uav_min_angle: " << uav_min_angle << std::endl;

    // int q;
    // pl.loadParam(node_name + "/q", q);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr areas_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr path_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    ros::Publisher pub_areas = nh.advertise<sensor_msgs::PointCloud2>("areas", queue_length);
    ros::Publisher pub_path = nh.advertise<sensor_msgs::PointCloud2>("path", queue_length);

    PointCloud boundary_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloud boundary_projections (new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloud uav_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloud scan_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    uav_cloud->header.frame_id = frame_id;
    scan_cloud->header.frame_id = frame_id;
    Point p_uav_orig;
    p_uav_orig.x = 0;
    p_uav_orig.y = 0;
    p_uav_orig.z = 0;
    uav_cloud->push_back(p_uav_orig);
    scan_cloud->push_back(p_uav_orig);
    
    const std::string pcl_file_name(abs_path + name);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
// //     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	
	// if (pcl::io::loadPCDFile (pcl_file_name, *cloud) == -1) //* load the file
	// {
	// 	PCL_ERROR ("Couldn't read file\n");
	// 	return (-1);
	// }

    // loading scan
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_scan_data (new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile ("/home/honzuna/.ros/scan.pcd", *final_scan_data) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file\n");
		return (-1);
	}
    std::cout << "Loaded " << final_scan_data->width * final_scan_data->height << " data points with the following fields "	<< std::endl;
    // for (auto& point: *final_scan_data){  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_to_send (new pcl::PointCloud<pcl::PointXYZRGB>); 
    for (int i = 0; i < (*final_scan_data).size(); i++){     
        Point point;
        point.x = (*final_scan_data)[i].x;
        point.y = (*final_scan_data)[i].y;
        point.z = (*final_scan_data)[i].z;
        point.r = 255;
        point.g = 255;
        point.b = 255;
        (*scan_to_send).push_back(point);
	}
    scan_to_send->header.frame_id = frame_id;
    ros::Publisher pub_mis_data = nh.advertise<sensor_msgs::PointCloud2>("missing_data", queue_length);
    pub_mis_data.publish(scan_to_send);



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld (new pcl::PointCloud<pcl::PointXYZRGB>);	
	if (pcl::io::loadPCDFile (pcl_file_name, *cld) == -1)
	{
		PCL_ERROR ("Couldn't read file\n");
		return (-1);
	}
    for(auto& p: *cld){
        if(p.y > -5.1){
            cloud->push_back(p);
        }
    }

    // cloud->header.frame_id = "base_frame";
    cloud->header.frame_id = frame_id;
	
	std::cout << "Loaded " << cloud->width * cloud->height << " data points with the following fields "	<< std::endl;
	unsigned int coef = 1;
	for (auto& point: *cloud){
        point.x = coef*point.x;
        point.y = coef*point.y;
        point.z = coef*point.z;
        
        point.r = 0;
        point.g = 0;
        point.b = 255;

        // point.r = 40;
        // point.g = 160;
        // point.b = 255;
	}

    

    // std::cout << "11 :)" << std::endl;
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    // std::cout << "22 :)" << std::endl;
    pcl::PCLPointCloud2::Ptr pcl_pc2 (new pcl::PCLPointCloud2 ());
    // std::cout << "33 :)" << std::endl;
    pcl_pc2->width = cloud->width;
    // std::cout << "44 :)" << std::endl;
    pcl_pc2->height = cloud->height;
    // std::cout << "55 :)" << std::endl;
    pcl::toPCLPointCloud2((*cloud), (*pcl_pc2));
    // std::cout << "22 :)" << std::endl;
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (pcl_pc2);
//   sor.setLeafSize (0.01f, 0.01f, 0.01f);
sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

    // (*cloud).clear();
    // (*cloud).width = pcl_pc2->width;
    // (*cloud).height = pcl_pc2->height;
    // pcl::fromPCLPointCloud2((*cloud_filtered), (*cloud));



    // to see pc before some crash during following computations
    pub.publish (*cloud);

    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::MarkerArray markerArray2;
    visualization_msgs::MarkerArray markerArray3;
    visualization_msgs::MarkerArray markerArrayPath;
    visualization_msgs::MarkerArray markerArrayPlanesNormals;

    /*
    std::vector<std::vector<int>> boundaries = get_boundary_points(cloud, K, epsilon, K_2, epsilon_2, k_angle, k_halfdisc, k_shape, trashold, weight_boundary, minimum_loop_length);
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

            // (*cloud)[p].r = 255;
            // (*cloud)[p].g = 0;
            // (*cloud)[p].b = 0;
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
    
    
    
    std::vector<Point> suitable_area_points;
    std::vector<int> suitable_areas_points_indices;

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

    pcl::PointCloud<pcl::Normal>::Ptr normals = get_normal_vectors(final_cloud, K, epsilon);
    // std::cout << "normal size: " << normals->size() << std::endl;
    int id = 0;
    for(auto current_boundary : boundaries){
        std::vector<int> best_plane;
        Point best_plane_origin;
        pcl::Normal best_plane_normal;
        // int error = 100;
        int maximum_points = 0;
        double minimum_distance = 1000000;
        std::cout << "current boundary size: " << current_boundary.size() << std::endl;

        for(int i = 0; i < k_ransac; i++){
            PointCloud new_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);// = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(cloud);
            new_cloud->width = cloud->width;
            new_cloud->height = cloud->height;
            // pcl::copyPointCloud<Point>(*cloud, *new_cloud);
            pcl::copyPointCloud<Point>(*final_cloud, *new_cloud);

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
                if(std::find(random_indexes.begin(), random_indexes.end(), idx) != random_indexes.end()){
                    j--;
                    continue;
                }
                // std::cout << "random idx: " << idx << std::endl;
                random_indexes.push_back(idx);
                // (*new_cloud)[idx].r = 255;
                // (*new_cloud)[idx].g = 0;
                // (*new_cloud)[idx].b = 0;
                p.x += (*new_cloud)[idx].x;
                p.y += (*new_cloud)[idx].y;
                p.z += (*new_cloud)[idx].z;
            }
            p.x /= 3.0;
            p.y /= 3.0;
            p.z /= 3.0;
            new_cloud->push_back(p);

            // pub.publish (*new_cloud);
            // sleep(2);
            // Point
            // int divide = 10;
            
            // pcl::PointCloud<pcl::Normal>::Ptr normals = get_normal_vectors(new_cloud, K, epsilon);  // get normals just for the point, not for the whole pc again
            
            // pcl::NormalEstimation<Point, pcl::Normal> ne;
            // pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());
            // pcl::PointCloud<pcl::Normal>::Ptr new_normals (new pcl::PointCloud<pcl::Normal>);
            // auto start = std::chrono::high_resolution_clock::now();

            // PointCloud norm_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);// = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(cloud);
            // norm_cloud->width = 1;
            // norm_cloud->height = 1;
            // norm_cloud->push_back(p);

            // ne.setInputCloud (norm_cloud);
            // ne.setSearchSurface(new_cloud);
            // ne.setSearchMethod (tree);
            // // ne.setKSearch(5*K);
            // ne.setKSearch(K);
            // // ne.setRadiusSearch(5*epsilon);
            // ne.compute (*new_normals);

            // normals->push_back((*new_normals)[0]);
            
            // std::cout << "normal size: " << normals->size() << std::endl;

            // auto plane_normal = normals->points[new_cloud->size() - 1];
            pcl::Normal plane_normal;
            plane_normal.normal_x = 0.0;
            plane_normal.normal_y = 0.0;
            plane_normal.normal_z = 0.0;
            for(int j = 0; j < n_ransac; j++){
                plane_normal.normal_x += normals->points[random_indexes[j]].normal_x;
                plane_normal.normal_y += normals->points[random_indexes[j]].normal_y;
                plane_normal.normal_z += normals->points[random_indexes[j]].normal_z;
            }
            plane_normal.normal_x /= 3.0;
            plane_normal.normal_y /= 3.0;
            plane_normal.normal_z /= 3.0;
            // for(int j = 0; j < current_boundary.size(); j++){
            //     int point_idx = current_boundary[j];
            //     plane_normal.normal_x += normals->points[current_boundary[j]].normal_x;
            //     plane_normal.normal_y += normals->points[current_boundary[j]].normal_y;
            //     plane_normal.normal_z += normals->points[current_boundary[j]].normal_z;
            // }
            // plane_normal.normal_x /= static_cast <double> (current_boundary.size());
            // plane_normal.normal_y /= static_cast <double> (current_boundary.size());
            // plane_normal.normal_z /= static_cast <double> (current_boundary.size());

            std::vector<int> points_below_trashold;
            double sum_of_distances = 0.0;
            for(int j = 0; j < current_boundary.size(); j++){
                int point_idx = current_boundary[j];
                
                Point projection = tangent_projection(plane_normal, p, (*new_cloud)[point_idx]);
                double dist = vect_norm((*new_cloud)[point_idx], projection);
                sum_of_distances += (dist * dist);
                // std::cout << "dist: " << dist << std::endl;
                if(dist < t_ransac){//} && std::find(random_indexes.begin(), random_indexes.end(), point_idx) == random_indexes.end()){
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
                if(curr_points >= maximum_points){
            // if(true){
            //     int curr_points = points_below_trashold.size();
                if(sum_of_distances < minimum_distance){
                    std::cout << "curr maximum points fits: " << curr_points << std::endl;
                    // Point best_fit_origin;
                    // best_fit_origin.x = 0;
                    // best_fit_origin.y = 0;
                    // best_fit_origin.z = 0;
                    // pcl::Normal best_fit_normal;
                    // best_fit_normal.normal_x = 0;
                    // best_fit_normal.normal_y = 0;
                    // best_fit_normal.normal_z = 0;
                    // for(int pi = 0; pi < points_below_trashold.size(); pi++){
                    //     best_fit_origin.x += (*new_cloud)[points_below_trashold[pi]].x;
                    //     best_fit_origin.y += (*new_cloud)[points_below_trashold[pi]].y;
                    //     best_fit_origin.z += (*new_cloud)[points_below_trashold[pi]].z;

                    //     best_fit_normal.normal_x = normals->points[points_below_trashold[pi]].normal_x;
                    //     best_fit_normal.normal_y = normals->points[points_below_trashold[pi]].normal_y;
                    //     best_fit_normal.normal_z = normals->points[points_below_trashold[pi]].normal_z;
                    // }
                    // best_fit_origin.x /= points_below_trashold.size();
                    // best_fit_origin.y /= points_below_trashold.size();
                    // best_fit_origin.z /= points_below_trashold.size();

                    // best_fit_normal.normal_x /= points_below_trashold.size();
                    // best_fit_normal.normal_y /= points_below_trashold.size();
                    // best_fit_normal.normal_z /= points_below_trashold.size();

                    minimum_distance = sum_of_distances;
                    maximum_points = curr_points;
                    best_plane = random_indexes;
                    best_plane_origin = p;
                    best_plane_normal = plane_normal;
                    // best_plane_origin = best_fit_origin;
                    // best_plane_normal = best_fit_normal;
                }
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

        // (*boundary_cloud).push_back(best_plane_origin);
        best_plane_origin.r = 255;
        best_plane_origin.b = 255;
        best_plane_origin.a = 255;
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = 100 * id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.003;
        marker.scale.y = 0.005;
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.a = 1.0;
        geometry_msgs::Point p0;
        p0.x = best_plane_origin.x;
        p0.y = best_plane_origin.y;
        p0.z = best_plane_origin.z;
        geometry_msgs::Point p1;
        p1.x = best_plane_origin.x + best_plane_normal.normal_x;
        p1.y = best_plane_origin.y + best_plane_normal.normal_y;
        p1.z = best_plane_origin.z + best_plane_normal.normal_z;
        marker.points.push_back(p0);
        marker.points.push_back(p1);
        markerArrayPlanesNormals.markers.push_back(marker);
        id++;
        // marker_path.publish(markerArrayPlanesNormals);
    }

    // std::vector<std::vector<int>> bounds;
    std::vector<std::vector<Point>> bounds;
    std::vector<std::vector<int>> bounds_indices;
    std::vector<std::vector<Point>> rectangle_vectors;

    // PointCloud boundary_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // PointCloud boundary_projections (new pcl::PointCloud<pcl::PointXYZRGB>);
    // boundary_cloud.resize
    for(int i = 0; i < boundaries.size(); i++){
        for(int j = 0; j < boundaries[i].size(); j++){
            // (*final_cloud)[boundaries[i][j]].r = 0;
            // (*final_cloud)[boundaries[i][j]].g = 0;
            // (*final_cloud)[boundaries[i][j]].b = 255;
            // boundary_cloud->push_back((*final_cloud)[boundaries[i][j]]);
        }
    }

    for(int i = 0; i < best_planes.size(); i++){
        std::vector<int> b;
        std::vector<Point> b_p;
        std::vector<Point> vects;
        // double min_x = 0;
        // double min_y = 0;
        // double max_x = 0;
        // double max_y = 0;
        double min_x = 1000.0;
        double min_y = 1000.0;
        double max_x = -1000.0;
        double max_y = -1000.0;

        double min_x_idx = 0;
        double min_y_idx = 0;
        double max_x_idx = 0;
        double max_y_idx = 0;
        for(int j = 0; j < boundaries[i].size(); j++){
            Point projection = tangent_projection(best_planes_normal[i], best_planes_origin[i], (*final_cloud)[boundaries[i][j]]);
            (*boundary_projections).push_back(projection);
            auto n1 = best_planes_normal[i];
            Point n;
            n.x = 0;
            n.y = 0;
            n.z = 1;
            double angle = acos( (n1.normal_z) / sqrt(n1.normal_x*n1.normal_x + n1.normal_y*n1.normal_y + n1.normal_z*n1.normal_z) );
            Point new_projection = rotate_point(angle, projection, n1, n);
            if(fmin(min_x, new_projection.x) < min_x){
                min_x = new_projection.x;
                min_x_idx = boundaries[i][j];
            }
            if(fmin(min_y, new_projection.y) < min_y){
                min_y = new_projection.y;
                min_y_idx = boundaries[i][j];
            }
            if(fmax(max_x, new_projection.x) > max_x){
                max_x = new_projection.x;
                max_x_idx = boundaries[i][j];
            }
            if(fmax(max_y, new_projection.y) > max_y){
                max_y = new_projection.y;
                max_y_idx = boundaries[i][j];
            }
        }
        b.push_back(min_x_idx);
        b.push_back(min_y_idx);
        b.push_back(max_x_idx);
        b.push_back(max_y_idx);
        // for(auto idx : b){
        //     (*final_cloud)[idx].r = 0;
        //     (*final_cloud)[idx].g = 255;
        //     (*final_cloud)[idx].b = 0;
        // }
        // (*final_cloud)[b[0]].r = 255;
        // (*final_cloud)[b[0]].g = 0;
        // (*final_cloud)[b[0]].b = 255;

        // (*final_cloud)[b[2]].r = 255;
        // (*final_cloud)[b[2]].g = 100;
        // (*final_cloud)[b[2]].b = 255;
        // rectangle boundary
        auto n1 = best_planes_normal[i];
        Point n;
        n.x = 0;
        n.y = 0;
        n.z = 1;

        Point p_left = tangent_projection(best_planes_normal[i], best_planes_origin[i], (*final_cloud)[min_x_idx]);
        Point p_bottom = tangent_projection(best_planes_normal[i], best_planes_origin[i], (*final_cloud)[min_y_idx]);
        Point p_right = tangent_projection(best_planes_normal[i], best_planes_origin[i], (*final_cloud)[max_x_idx]);
        Point p_top = tangent_projection(best_planes_normal[i], best_planes_origin[i], (*final_cloud)[max_y_idx]);

        Point rect_origin;
        rect_origin.x = (p_left.x + p_bottom.x + p_right.x + p_top.x) / 4.0;
        rect_origin.y = (p_left.y + p_bottom.y + p_right.y + p_top.y) / 4.0;
        rect_origin.z = (p_left.z + p_bottom.z + p_right.z + p_top.z) / 4.0;

        p_left.r = 255;
        p_left.b = 255;
        p_left.a = 255;
        p_bottom.r = 255;
        p_bottom.b = 255;
        p_bottom.a = 255;
        p_right.r = 255;
        p_right.b = 255;
        p_right.a = 255;
        p_top.r = 255;
        p_top.b = 255;
        p_top.a = 255;
        rect_origin.r = 255;
        // rect_origin.b = 255;
        rect_origin.a = 255;
        // if(i == 0)
        // (*boundary_projections).push_back(p_left);
        // // (*boundary_projections).push_back(p_bottom);
        // if(i == 0)
        // (*boundary_projections).push_back(p_right);
        // (*boundary_projections).push_back(p_top);
        // (*boundary_projections).push_back(rect_origin);

        pcl::Normal normal;
        normal.normal_x = 0;
        normal.normal_y = 0;
        normal.normal_z = 1;

        Point norm;
        norm.x = best_planes_normal[i].normal_x;
        norm.y = best_planes_normal[i].normal_y;
        norm.z = best_planes_normal[i].normal_z;

        Point zero;
        zero.x = 0;
        zero.y = 0;
        zero.z = 0;

        double angle = acos( (n1.normal_z) / sqrt(n1.normal_x*n1.normal_x + n1.normal_y*n1.normal_y + n1.normal_z*n1.normal_z) );
        Point p_left_projected = rotate_point(angle, p_left, n1, n);
        // p_left_projected = tangent_projection(normal, zero, p_left_projected);
        Point p_bottom_projected = rotate_point(angle, p_bottom, n1, n);
        Point p_right_projected = rotate_point(angle, p_right, n1, n);
        // p_right_projected = tangent_projection(normal, zero, p_right_projected);
        Point p_top_projected = rotate_point(angle, p_top, n1, n);
        Point rect_origin_projected = rotate_point(angle, rect_origin, n1, n);

        p_left_projected.r = 255;
        p_left_projected.g = 100;
        p_left_projected.a = 255;
        p_bottom_projected.r = 255;
        p_bottom_projected.g = 100;
        p_bottom_projected.a = 255;
        p_right_projected.r = 255;
        p_right_projected.g = 100;
        p_right_projected.a = 255;
        p_top_projected.r = 255;
        p_top_projected.g = 100;
        p_top_projected.a = 255;
        rect_origin_projected.r = 255;
        // rect_origin_projected.b = 255;
        rect_origin_projected.a = 255;
        // if(i == 0)
        // (*boundary_projections).push_back(p_left_projected);
        // (*boundary_projections).push_back(p_bottom_projected);
        // if(i == 0)
        // (*boundary_projections).push_back(p_right_projected);
        // (*boundary_projections).push_back(p_top_projected);
        // (*boundary_projections).push_back(rect_origin_projected);

        double dist_left_to_right = p_right_projected.x - rect_origin_projected.x + rect_origin_projected.x - p_left_projected.x;
        double dist_bottom_to_top = p_top_projected.y - rect_origin_projected.y + rect_origin_projected.y - p_bottom_projected.y;

        // double dist_left_to_right = vect_norm(p_left, p_right);
        // double dist_bottom_to_top = vect_norm(p_bottom, p_top);

        Point new_p_right;
        new_p_right.x = p_right_projected.x;
        new_p_right.y = p_left_projected.y;
        new_p_right.z = p_right_projected.z;

        Point new_p_top;
        new_p_top.x = p_bottom_projected.x;
        new_p_top.y = p_top_projected.y;
        new_p_top.z = p_top_projected.z;

        new_p_right.r = 255;
        new_p_right.g = 255;
        new_p_right.a = 255;
        // if(i == 0)
        // (*boundary_projections).push_back(new_p_right);
        new_p_top.r = 255;
        new_p_top.g = 255;
        new_p_top.a = 255;
        // (*boundary_projections).push_back(new_p_top);

        // pcl::Normal normal;
        normal.normal_x = 0;
        normal.normal_y = 0;
        normal.normal_z = 1.0;

        // Point norm;
        norm.x = best_planes_normal[i].normal_x;
        norm.y = best_planes_normal[i].normal_y;
        norm.z = best_planes_normal[i].normal_z;

        // p_right = rotate_point(M_PI-angle, new_p_right, normal, norm);
        p_top = rotate_point(-angle, new_p_top, normal, norm);
        p_top.g = 255;
        p_top.a = 255;
        // p_right = rotate_point(angle, new_p_right, normal, norm);
        // p_top = rotate_point(angle, new_p_top, normal, norm);

        // (*boundary_projections).push_back(p_left);
        // (*boundary_projections).push_back(p_bottom);
        // p_right.r = 255;
        // p_right = rotate_point(angle, new_p_right, normal, norm);
        // p_right.g = 255;
        // p_right.a = 255;
        // if(i == 0)
        // (*boundary_projections).push_back(p_right);
        // (*boundary_projections).push_back(p_top);
        // (*boundary_projections).push_back(rect_origin);

        // p_right = rotate_point(angle, new_p_right, normal, norm);
        // p_right.g = 255;
        // p_right.a = 255;
        // if(i == 0)
        // (*boundary_projections).push_back(p_right);
        p_right = rotate_point(-angle, new_p_right, normal, norm);
        p_right.g = 255;
        p_right.a = 255;
        // if(i == 0)
        // (*boundary_projections).push_back(p_right);
        // p_right = rotate_point(M_PI/2 + angle, new_p_right, normal, norm);
        // p_right.g = 255;
        // p_right.a = 255;
        // if(i == 0)
        // (*boundary_projections).push_back(p_right);
        // p_right = rotate_point(M_PI + angle, new_p_right, normal, norm);
        // p_right.g = 255;
        // p_right.a = 255;
        // if(i == 0)
        // (*boundary_projections).push_back(p_right);
        // p_right = rotate_point(M_PI/2 - angle, new_p_right, normal, norm);
        // p_right.g = 255;
        // p_right.a = 255;
        // if(i == 0)
        // (*boundary_projections).push_back(p_right);
        // p_right = rotate_point(M_PI - angle, new_p_right, normal, norm);
        // p_right.g = 255;
        // p_right.a = 255;
        // if(i == 0)
        // (*boundary_projections).push_back(p_right);
        // p_right = rotate_point((3*M_PI)/4 - angle, new_p_right, normal, norm);
        // p_right.g = 255;
        // p_right.a = 255;
        // if(i == 0)
        // (*boundary_projections).push_back(p_right);
        // p_right = rotate_point(2*M_PI - angle, new_p_right, normal, norm);
        // p_right.g = 255;
        // p_right.a = 255;
        // if(i == 0)
        // (*boundary_projections).push_back(p_right);

        Point vector_left_to_right = get_vector(p_left, p_right);

        // double s_a = vect_norm(p_right, p_top);
        // double s_b = vect_norm(p_left, p_top);
        // double s_c = vect_norm(p_right, p_left);
        // double v_c = sin( acos( ( s_c * s_c + s_a * s_a - s_b * s_b ) / ( 2 * s_a * s_c ) ) ) * s_a;
        // double x = sqrt( s_b * s_b - v_c * v_c );
        // s_a = vect_norm(p_right, p_bottom);
        // s_b = vect_norm(p_left, p_bottom);
        // v_c = sin( acos( ( s_c * s_c + s_a * s_a - s_b * s_b ) / ( 2 * s_a * s_c ) ) ) * s_a;
        // double y = sqrt( s_b * s_b - v_c * v_c );
        // double diff_y_x = y - x;

        // MAYBE PROJECTED NEW BOUNDARY POINTS
        // (*final_cloud).push_back(p_right);
        // (*final_cloud).push_back(p_top);

        // Point rectangle_origin;
        // rectangle_origin.x = p_left.x + vector_left_to_right.x * (y / dist_left_to_right);
        // rectangle_origin.y = p_left.y + vector_left_to_right.y * (y / dist_left_to_right);
        // rectangle_origin.z = p_left.z + vector_left_to_right.z * (y / dist_left_to_right);
        // Point vec_bottom_origin = get_vector(p_bottom, rectangle_origin);
        // Point vec_left_origin = get_vector(p_left, rectangle_origin);
        // // Point vec_bottom_origin = get_vector(p_bottom, rectangle_origin);
        // Point new_top;
        // new_top.x = p_top.x + vector_left_to_right.x * (diff_y_x / dist_left_to_right);
        // new_top.y = p_top.y + vector_left_to_right.y * (diff_y_x / dist_left_to_right);
        // new_top.z = p_top.z + vector_left_to_right.z * (diff_y_x / dist_left_to_right);

        Point vector_bottom_to_top = get_vector(p_bottom, p_top);

        vects.push_back(vector_left_to_right);
        vects.push_back(vector_bottom_to_top);

        b_p.push_back(p_left);
        b_p.push_back(p_bottom);
        b_p.push_back(p_right);
        b_p.push_back(p_top);

        bounds_indices.push_back(b);
        bounds.push_back(b_p);
        rectangle_vectors.push_back(vects);
    }

    std::vector<std::vector<Point>> boundary_samples;
    std::vector<Point> boundary_samples_origins;
    std::vector<int> boundary_samples_indices;
    std::vector<std::vector<Point>> boundary_samples_bounds;

    // double sample_criterion = 2 * ( sqrt( uav_max_distance*uav_max_distance - uav_min_distance*uav_min_distance ) - (uav_min_area_width / 2) );
    double sample_step_w = uav_sample_width;    //2 * ( sqrt( uav_max_distance*uav_max_distance - uav_min_distance*uav_min_distance ) - (uav_min_area_width / 2) );
    double sample_step_h = uav_sample_height;    //2 * ( sqrt( uav_max_distance*uav_max_distance - uav_min_distance*uav_min_distance ) - (uav_min_area_width / 2) );
    std::cout << "sample criterion w: " << sample_step_w << std::endl;
    std::cout << "sample criterion h: " << sample_step_h << std::endl;
    std::cout << "vizualisation" << std::endl;
    for(int c = 0; c < best_planes.size(); c++){
    // for(int c = 3; c < 4; c++){
        auto best_plane = best_planes[c];
        for(int i = 0; i < best_plane.size(); i++){
            (*final_cloud)[best_plane[i]].r = 255;
            (*final_cloud)[best_plane[i]].g = 0;
            (*final_cloud)[best_plane[i]].b = 0;
        }

        // rectangle boundary
        // Point p_left = tangent_projection(best_planes_normal[c], best_planes_origin[c], (*final_cloud)[bounds[c][0]]);
        // Point p_bottom = tangent_projection(best_planes_normal[c], best_planes_origin[c], (*final_cloud)[bounds[c][1]]);
        // Point p_right = tangent_projection(best_planes_normal[c], best_planes_origin[c], (*final_cloud)[bounds[c][2]]);
        // Point p_top = tangent_projection(best_planes_normal[c], best_planes_origin[c], (*final_cloud)[bounds[c][3]]);
        Point p_left = bounds[c][0];
        Point p_bottom = bounds[c][1];
        Point p_right = bounds[c][2];
        Point p_top = bounds[c][3];

    //     bounds;
    // std::vector<std::vector<int>> bounds_indices;
    // std::vector<std::vector<Point>> rectangle_vectors;

        double dist_left_to_right = vect_norm(p_left, p_right);
        double dist_bottom_to_top = vect_norm(p_bottom, p_top);

        Point vector_left_to_right = rectangle_vectors[c][0];
        Point vector_bottom_to_top = rectangle_vectors[c][1];

        // visualization_msgs::Marker marker3;
        // marker3.header.frame_id = "base_frame";
        // marker3.header.stamp = ros::Time::now();
        // marker3.action = visualization_msgs::Marker::ADD;
        // marker3.id = c;
        // marker3.type = visualization_msgs::Marker::POINTS;
        // marker3.scale.x = 0.01;
        // marker3.scale.y = 0.01;
        // marker3.scale.z = 0.01;
        // marker3.color.r = 0.0;
        // marker3.color.g = 255.0;
        // marker3.color.b = 255.0;
        // marker3.color.a = 1.0;

        // geometry_msgs::Point p2;
        // p2.x = p_right.x;
        // p2.y = p_right.y;
        // p2.z = p_right.z;
        // marker3.points.push_back(p2);
        // // markerArray.markers.push_back(marker3);

        // visualization_msgs::Marker marker4;
        // marker4.header.frame_id = "base_frame";
        // marker4.header.stamp = ros::Time::now();
        // marker4.action = visualization_msgs::Marker::ADD;
        // marker4.id = c+10;
        // marker4.type = visualization_msgs::Marker::POINTS;
        // marker4.scale.x = 0.01;
        // marker4.scale.y = 0.01;
        // marker4.scale.z = 0.01;
        // marker3.color.r = 0.0;
        // marker3.color.g = 255.0;
        // marker3.color.b = 255.0;
        // marker4.color.a = 1.0;

        // geometry_msgs::Point p3;
        // p3.x = p_top.x;
        // p3.y = p_top.y;
        // p3.z = p_top.z;
        // marker4.points.push_back(p3);
        // markerArray.markers.push_back(marker4);

        // Point vector_left_to_right = get_vector(p_left, p_right);

        double s_a = vect_norm(p_right, p_top);
        double s_b = vect_norm(p_left, p_top);
        double s_c = vect_norm(p_right, p_left);
        double v_c = sin( acos( ( s_c * s_c + s_a * s_a - s_b * s_b ) / ( 2 * s_a * s_c ) ) ) * s_a;
        double x = sqrt( s_b * s_b - v_c * v_c );
        s_a = vect_norm(p_right, p_bottom);
        s_b = vect_norm(p_left, p_bottom);
        v_c = sin( acos( ( s_c * s_c + s_a * s_a - s_b * s_b ) / ( 2 * s_a * s_c ) ) ) * s_a;
        double y = sqrt( s_b * s_b - v_c * v_c );
        double diff_y_x = y - x;
        
        Point rectangle_origin;
        rectangle_origin.x = p_left.x + vector_left_to_right.x * (y / dist_left_to_right);
        rectangle_origin.y = p_left.y + vector_left_to_right.y * (y / dist_left_to_right);
        rectangle_origin.z = p_left.z + vector_left_to_right.z * (y / dist_left_to_right);
        Point vec_bottom_origin = get_vector(p_bottom, rectangle_origin);

        // Point rectangle_origin;
        // rectangle_origin.x = (p_left.x + p_bottom.x + p_right.x + p_top.x) / 4.0;
        // rectangle_origin.y = (p_left.y + p_bottom.y + p_right.y + p_top.y) / 4.0;
        // rectangle_origin.z = (p_left.z + p_bottom.z + p_right.z + p_top.z) / 4.0;
        // rectangle_origin.x = (p_left.x + p_right.x) / 2.0;
        // rectangle_origin.y = (p_left.y + p_right.y) / 2.0;
        // rectangle_origin.z = (p_left.z + p_right.z) / 2.0;

        // rectangle_origin.r = 255;
        // rectangle_origin.g = 150;
        // rectangle_origin.a = 255;
        // (*final_cloud).push_back(rectangle_origin);
        
        Point vec_left_origin = get_vector(p_left, rectangle_origin);
        // Point vec_bottom_origin = get_vector(p_bottom, rectangle_origin);
        // Point new_top;
        // new_top.x = p_top.x + vector_left_to_right.x * (diff_y_x / dist_left_to_right);
        // new_top.y = p_top.y + vector_left_to_right.y * (diff_y_x / dist_left_to_right);
        // new_top.z = p_top.z + vector_left_to_right.z * (diff_y_x / dist_left_to_right);

        // Point vector_bottom_to_top = get_vector(p_bottom, new_top);

        // for(double sample_height_start = 0.0; sample_height_start < dist_bottom_to_top; sample_height_start += sample_criterion){
        for(double sample_height_start = 0.0; sample_height_start < dist_bottom_to_top; sample_height_start += sample_step_h){
            // std::cout << "sample_height_start: " << sample_height_start << " / dist_bottom_to_top: " << dist_bottom_to_top << std::endl;
            // for(double sample_width_start = 0.0; sample_width_start < dist_left_to_right; sample_width_start += sample_criterion){
            for(double sample_width_start = 0.0; sample_width_start < dist_left_to_right; sample_width_start += sample_step_w){
                // std::cout << "sample_width_start: " << sample_width_start << " / dist_left_to_right: " << dist_left_to_right << std::endl;

                int sample_max = 5;
                // int r = 100 + rand() % 155;
                // int g = 100 + rand() % 155;
                // int b = 100 + rand() % 155;
                int r = 0;
                int g = 0;
                int b = 0;
                // int r = 0;
                // int g = 0;
                // int b = 255;
                // int r = 255;
                // int g = 255;
                // int b = 0;

                Point h_bottom;
                h_bottom.x = p_bottom.x + vector_bottom_to_top.x * ( sample_height_start / dist_bottom_to_top);
                h_bottom.y = p_bottom.y + vector_bottom_to_top.y * ( sample_height_start / dist_bottom_to_top);
                h_bottom.z = p_bottom.z + vector_bottom_to_top.z * ( sample_height_start / dist_bottom_to_top);

                Point h_top;
                // double sample_size_h = (sample_height_start + sample_criterion);// > dist_bottom_to_top ? (dist_bottom_to_top - sample_height_start) : (sample_height_start + sample_criterion);
                double sample_size_h = (sample_height_start + sample_step_h);
                h_top.x = p_bottom.x + vector_bottom_to_top.x * ( sample_size_h / dist_bottom_to_top);
                h_top.y = p_bottom.y + vector_bottom_to_top.y * ( sample_size_h / dist_bottom_to_top);
                h_top.z = p_bottom.z + vector_bottom_to_top.z * ( sample_size_h / dist_bottom_to_top);

                Point h = get_vector(h_bottom, h_top);
                // std::cout << "h = (" << h.x << ", " << h.y << ", " << h.z << ")" << std::endl;

                Point w_left;
                w_left.x = p_left.x + vector_left_to_right.x * ( sample_width_start / dist_left_to_right);
                w_left.y = p_left.y + vector_left_to_right.y * ( sample_width_start / dist_left_to_right);
                w_left.z = p_left.z + vector_left_to_right.z * ( sample_width_start / dist_left_to_right);

                Point w_right;
                // double sample_size_w = (sample_width_start + sample_criterion);// > dist_left_to_right ? (dist_left_to_right - sample_width_start) : (sample_width_start + sample_criterion);
                double sample_size_w = (sample_width_start + sample_step_w);
                w_right.x = p_left.x + vector_left_to_right.x * ( sample_size_w / dist_left_to_right);
                w_right.y = p_left.y + vector_left_to_right.y * ( sample_size_w / dist_left_to_right);
                w_right.z = p_left.z + vector_left_to_right.z * ( sample_size_w / dist_left_to_right);

                Point w = get_vector(w_left, w_right);
                // std::cout << "w = (" << w.x << ", " << w.y << ", " << w.z << ")" << std::endl;

                // Point h_top_e;
                // sample_size_h = (sample_height_start + sample_criterion) > dist_bottom_to_top ? (dist_bottom_to_top - sample_height_start) : (sample_height_start + sample_criterion);
                // h_top_e.x = p_bottom.x + vector_bottom_to_top.x * ( sample_size_h / dist_bottom_to_top);
                // h_top_e.y = p_bottom.y + vector_bottom_to_top.y * ( sample_size_h / dist_bottom_to_top);
                // h_top_e.z = p_bottom.z + vector_bottom_to_top.z * ( sample_size_h / dist_bottom_to_top);

                // Point h_e = get_vector(h_bottom, h_top_e);

                // Point w_right_e;
                // sample_size_w = (sample_width_start + sample_criterion) > dist_left_to_right ? (dist_left_to_right - sample_width_start) : (sample_width_start + sample_criterion);
                // w_right_e.x = p_left.x + vector_left_to_right.x * ( sample_size_w / dist_left_to_right);
                // w_right_e.y = p_left.y + vector_left_to_right.y * ( sample_size_w / dist_left_to_right);
                // w_right_e.z = p_left.z + vector_left_to_right.z * ( sample_size_w / dist_left_to_right);

                // Point w_e = get_vector(w_left, w_right_e);

                std::vector<Point> samples;
                std::vector<Point> samples_bounds;
                samples_bounds.push_back(w_left);
                samples_bounds.push_back(h_bottom);
                samples_bounds.push_back(w_right);
                samples_bounds.push_back(h_top);

                for(int i = 0; i < sample_max; i++){
                    double m = static_cast <double> (sample_max) ;
                    double coef =  i / m;

                    // Point curr_left;
                    // curr_left.x = p_left + 
                    // p_left.r = 255;
                    // p_left.g = 255;
                    // p_left.b = 0;
                    // p_left.a = 255;

                    // // std::cout << "left_sample = (" << left_sample.x << ", " << left_sample.y << ", " << left_sample.z << ")" << std::endl;
                    // (*boundary_cloud).push_back(p_left);
                    pcl::Normal norm_up;
                    norm_up.normal_x = vector_bottom_to_top.x;
                    norm_up.normal_y = vector_bottom_to_top.y;
                    norm_up.normal_z = vector_bottom_to_top.z;

                    pcl::Normal norm_right;
                    norm_right.normal_x = vector_left_to_right.x;
                    norm_right.normal_y = vector_left_to_right.y;
                    norm_right.normal_z = vector_left_to_right.z;
                    

                    Point left_sample;
                    // left_sample.x = p_left.x + ( vector_left_to_right.x * ( sample_height_start / dist_left_to_right ) );
                    // double width_dist = sample_width_start / dist_left_to_right > dist_left_to_right ? 1.0 : sample_width_start / dist_left_to_right;
                    double width_dist = sample_width_start > dist_left_to_right ? 1.0 : sample_width_start / dist_left_to_right;
                    left_sample.x = p_left.x + ( vector_left_to_right.x * ( width_dist ) );
                    left_sample.y = p_left.y + ( vector_left_to_right.y * ( width_dist ) );
                    left_sample.z = p_left.z + ( vector_left_to_right.z * ( width_dist ) );

                    // left_sample.r = 125;
                    // left_sample.g = 125;
                    // left_sample.b = 0;
                    // left_sample.a = 255;
                    // (*boundary_cloud).push_back(left_sample);

                    // left_sample.x = left_sample.x + ( vector_bottom_to_top.x * ( ( coef ) ) ) - vec_bottom_origin.x;
                    // left_sample.x = left_sample.x + ( ( sample_height_start / sample_criterion) * h.x + h.x * ( ( coef ) ) ) - vec_bottom_origin.x;
                    // left_sample.y = left_sample.y + ( ( sample_height_start / sample_criterion) * h.y + h.y * ( ( coef ) ) ) - vec_bottom_origin.y;
                    // left_sample.z = left_sample.z + ( ( sample_height_start / sample_criterion) * h.z + h.z * ( ( coef ) ) ) - vec_bottom_origin.z;
                    left_sample.x = left_sample.x + ( ( sample_height_start / sample_step_h) * h.x + h.x * ( ( coef ) ) ) - vec_bottom_origin.x;
                    left_sample.y = left_sample.y + ( ( sample_height_start / sample_step_h) * h.y + h.y * ( ( coef ) ) ) - vec_bottom_origin.y;
                    left_sample.z = left_sample.z + ( ( sample_height_start / sample_step_h) * h.z + h.z * ( ( coef ) ) ) - vec_bottom_origin.z;
                    // left_sample.x = left_sample.x + ( ( sample_height_start / sample_criterion) * h.x + h_e.x * ( ( coef ) ) ) - vec_bottom_origin.x;
                    // left_sample.y = left_sample.y + ( ( sample_height_start / sample_criterion) * h.y + h_e.y * ( ( coef ) ) ) - vec_bottom_origin.y;
                    // left_sample.z = left_sample.z + ( ( sample_height_start / sample_criterion) * h.z + h_e.z * ( ( coef ) ) ) - vec_bottom_origin.z;

                    left_sample.r = r;
                    left_sample.g = g;
                    left_sample.b = b;
                    left_sample.a = 255;

                    // std::cout << "left_sample = (" << left_sample.x << ", " << left_sample.y << ", " << left_sample.z << ")" << std::endl;
                    // if(vect_norm(left_sample, tangent_projection(norm_up, p_bottom, left_sample)) <= dist_left_to_right){
                    if(vect_norm(left_sample, tangent_projection(norm_up, p_bottom, left_sample)) <= dist_bottom_to_top){
                        samples.push_back(left_sample);
                        (*boundary_cloud).push_back(left_sample);
                    }
                    
                    Point right_sample;
                    // right_sample.x = p_left.x + ( vector_left_to_right.x * ( ( sample_height_start + sample_criterion ) / dist_left_to_right ) );
                    // double width_dist_sample = ( sample_width_start + sample_criterion ) / dist_left_to_right > dist_left_to_right ? 1.0 : ( sample_width_start + sample_criterion ) / dist_left_to_right;
                    // double width_dist_sample = ( sample_width_start + sample_criterion ) > dist_left_to_right ? 1.0 : ( sample_width_start + sample_criterion ) / dist_left_to_right;
                    double width_dist_sample = ( sample_width_start + sample_step_w ) > dist_left_to_right ? 1.0 : ( sample_width_start + sample_step_w ) / dist_left_to_right;
                    right_sample.x = p_left.x + ( vector_left_to_right.x * ( width_dist_sample ) );
                    right_sample.y = p_left.y + ( vector_left_to_right.y * ( width_dist_sample ) );
                    right_sample.z = p_left.z + ( vector_left_to_right.z * ( width_dist_sample ) );

                    // double width_dist_sample = ( sample_height_start / sample_criterion) > dist_left_to_right ? 1.0 : ( sample_width_start + sample_criterion ) / dist_left_to_right;
                    // right_sample.x = right_sample.x + ( ( sample_height_start / sample_criterion) * h.x + h.x * ( ( coef ) ) ) - vec_bottom_origin.x;
                    // right_sample.y = right_sample.y + ( ( sample_height_start / sample_criterion) * h.y + h.y * ( ( coef ) ) ) - vec_bottom_origin.y;
                    // right_sample.z = right_sample.z + ( ( sample_height_start / sample_criterion) * h.z + h.z * ( ( coef ) ) ) - vec_bottom_origin.z;
                    right_sample.x = right_sample.x + ( ( sample_height_start / sample_step_h) * h.x + h.x * ( ( coef ) ) ) - vec_bottom_origin.x;
                    right_sample.y = right_sample.y + ( ( sample_height_start / sample_step_h) * h.y + h.y * ( ( coef ) ) ) - vec_bottom_origin.y;
                    right_sample.z = right_sample.z + ( ( sample_height_start / sample_step_h) * h.z + h.z * ( ( coef ) ) ) - vec_bottom_origin.z;
                    // right_sample.x = right_sample.x + ( ( sample_height_start / sample_criterion) * h.x + h_e.x * ( ( coef ) ) ) - vec_bottom_origin.x;
                    // right_sample.y = right_sample.y + ( ( sample_height_start / sample_criterion) * h.y + h_e.y * ( ( coef ) ) ) - vec_bottom_origin.y;
                    // right_sample.z = right_sample.z + ( ( sample_height_start / sample_criterion) * h.z + h_e.z * ( ( coef ) ) ) - vec_bottom_origin.z;

                    right_sample.r = r;
                    right_sample.g = g;
                    right_sample.b = b;
                    right_sample.a = 255;

                    // if(vect_norm(right_sample, tangent_projection(norm_up, p_bottom, right_sample)) <= dist_left_to_right){
                    if(vect_norm(right_sample, tangent_projection(norm_up, p_bottom, right_sample)) <= dist_bottom_to_top){
                        samples.push_back(right_sample);
                        (*boundary_cloud).push_back(right_sample);
                    }

                    Point bottom_sample;
                    double height_dist = sample_height_start > dist_bottom_to_top ? 1.0 : sample_height_start / dist_bottom_to_top;
                    bottom_sample.x = p_bottom.x + ( vector_bottom_to_top.x * ( height_dist ) );
                    bottom_sample.y = p_bottom.y + ( vector_bottom_to_top.y * ( height_dist ) );
                    bottom_sample.z = p_bottom.z + ( vector_bottom_to_top.z * ( height_dist ) );

                    // bottom_sample.x = bottom_sample.x + ( ( sample_width_start / sample_criterion) * w.x + w.x * ( ( coef ) ) ) - vec_left_origin.x;
                    // bottom_sample.y = bottom_sample.y + ( ( sample_width_start / sample_criterion) * w.y + w.y * ( ( coef ) ) ) - vec_left_origin.y;
                    // bottom_sample.z = bottom_sample.z + ( ( sample_width_start / sample_criterion) * w.z + w.z * ( ( coef ) ) ) - vec_left_origin.z;
                    bottom_sample.x = bottom_sample.x + ( ( sample_width_start / sample_step_w) * w.x + w.x * ( ( coef ) ) ) - vec_left_origin.x;
                    bottom_sample.y = bottom_sample.y + ( ( sample_width_start / sample_step_w) * w.y + w.y * ( ( coef ) ) ) - vec_left_origin.y;
                    bottom_sample.z = bottom_sample.z + ( ( sample_width_start / sample_step_w) * w.z + w.z * ( ( coef ) ) ) - vec_left_origin.z;
                    // bottom_sample.x = bottom_sample.x + ( ( sample_width_start / sample_criterion) * w.x + w_e.x * ( ( coef ) ) ) - vec_left_origin.x;
                    // bottom_sample.y = bottom_sample.y + ( ( sample_width_start / sample_criterion) * w.y + w_e.y * ( ( coef ) ) ) - vec_left_origin.y;
                    // bottom_sample.z = bottom_sample.z + ( ( sample_width_start / sample_criterion) * w.z + w_e.z * ( ( coef ) ) ) - vec_left_origin.z;

                    bottom_sample.r = r;
                    bottom_sample.g = g;
                    bottom_sample.b = b;
                    bottom_sample.a = 255;
                    // if(vect_norm(bottom_sample, tangent_projection(norm_right, p_left, bottom_sample)) <= dist_bottom_to_top){
                    if(vect_norm(bottom_sample, tangent_projection(norm_right, p_left, bottom_sample)) <= dist_left_to_right){
                        samples.push_back(bottom_sample);
                        (*boundary_cloud).push_back(bottom_sample);
                    }
                    
                    Point top_sample;
                    // double height_dist_sample = ( sample_height_start + sample_criterion ) > dist_bottom_to_top ? 1.0 : ( sample_height_start + sample_criterion ) / dist_bottom_to_top;
                    double height_dist_sample = ( sample_height_start + sample_step_h ) > dist_bottom_to_top ? 1.0 : ( sample_height_start + sample_step_h ) / dist_bottom_to_top;
                    top_sample.x = p_bottom.x + ( vector_bottom_to_top.x * ( height_dist_sample ) );
                    top_sample.y = p_bottom.y + ( vector_bottom_to_top.y * ( height_dist_sample ) );
                    top_sample.z = p_bottom.z + ( vector_bottom_to_top.z * ( height_dist_sample ) );

                    // top_sample.x = top_sample.x + ( ( sample_width_start / sample_criterion) * w.x + w.x * ( ( coef ) ) ) - vec_left_origin.x;
                    // top_sample.y = top_sample.y + ( ( sample_width_start / sample_criterion) * w.y + w.y * ( ( coef ) ) ) - vec_left_origin.y;
                    // top_sample.z = top_sample.z + ( ( sample_width_start / sample_criterion) * w.z + w.z * ( ( coef ) ) ) - vec_left_origin.z;
                    top_sample.x = top_sample.x + ( ( sample_width_start / sample_step_w) * w.x + w.x * ( ( coef ) ) ) - vec_left_origin.x;
                    top_sample.y = top_sample.y + ( ( sample_width_start / sample_step_w) * w.y + w.y * ( ( coef ) ) ) - vec_left_origin.y;
                    top_sample.z = top_sample.z + ( ( sample_width_start / sample_step_w) * w.z + w.z * ( ( coef ) ) ) - vec_left_origin.z;
                    // top_sample.x = top_sample.x + ( ( sample_width_start / sample_criterion) * w.x + w_e.x * ( ( coef ) ) ) - vec_left_origin.x;
                    // top_sample.y = top_sample.y + ( ( sample_width_start / sample_criterion) * w.y + w_e.y * ( ( coef ) ) ) - vec_left_origin.y;
                    // top_sample.z = top_sample.z + ( ( sample_width_start / sample_criterion) * w.z + w_e.z * ( ( coef ) ) ) - vec_left_origin.z;

                    top_sample.r = r;
                    top_sample.g = g;
                    top_sample.b = b;
                    top_sample.a = 255;
                    // if(vect_norm(top_sample, tangent_projection(norm_right, p_left, top_sample)) <= dist_bottom_to_top){
                    if(vect_norm(top_sample, tangent_projection(norm_right, p_left, top_sample)) <= dist_left_to_right){
                        samples.push_back(top_sample);
                        (*boundary_cloud).push_back(top_sample);
                    }
                    

                    // samples.push_back(left_sample);
                    // samples.push_back(right_sample);
                    // samples.push_back(bottom_sample);
                    // samples.push_back(top_sample);

                    // visualization_msgs::Marker marker3;
                    // marker3.header.frame_id = "base_frame";
                    // marker3.header.stamp = ros::Time::now();
                    // marker3.action = visualization_msgs::Marker::ADD;
                    // marker3.id = c+10000 + 1000*sample_height_start + sample_width_start*100 + i;
                    // marker3.type = visualization_msgs::Marker::POINTS;
                    // marker3.scale.x = 0.01;
                    // marker3.scale.y = 0.01;
                    // marker3.scale.z = 0.01;
                    // marker3.color.r = r / 255.0;
                    // marker3.color.g = g / 255.0;
                    // marker3.color.b = b / 255.0;
                    // marker3.color.a = 1.0;

                    // geometry_msgs::Point p2;
                    // p2.x = left_sample.x;
                    // p2.y = left_sample.y;
                    // p2.z = left_sample.z;
                    // marker3.points.push_back(p2);
                    // markerArray3.markers.push_back(marker3);

                    // visualization_msgs::Marker marker4;
                    // marker4.header.frame_id = "base_frame";
                    // marker4.header.stamp = ros::Time::now();
                    // marker4.action = visualization_msgs::Marker::ADD;
                    // marker4.id = c+10000 + 1000*sample_height_start + sample_width_start*100 + i+20;
                    // marker4.type = visualization_msgs::Marker::POINTS;
                    // marker4.scale.x = 0.01;
                    // marker4.scale.y = 0.01;
                    // marker4.scale.z = 0.01;
                    // marker4.color.r = r / 255.0;
                    // marker4.color.g = g / 255.0;
                    // marker4.color.b = b / 255.0;
                    // marker4.color.a = 1.0;

                    // geometry_msgs::Point p3;
                    // p3.x = right_sample.x;
                    // p3.y = right_sample.y;
                    // p3.z = right_sample.z;
                    // marker4.points.push_back(p3);
                    // markerArray3.markers.push_back(marker4);

                }
                // pcl::Normal nor;
                // Point l_r = get_vector(w_left, w_right);
                // Point b_tt = get_vector(h_bottom, h_top);
                // nor.normal_x = l_r.x;
                // nor.normal_y = l_r.y;
                // nor.normal_z = l_r.z;
                // Point proj = tangent_projection(nor, h_bottom, w_left);
                // proj.r = 100;
                // proj.b = 255;
                // proj.a = 255;
                // (*final_cloud).push_back(proj);
                // double dist1 = vect_norm(h_bottom, h_top) / 2.0;
                // double dist2 = vect_norm(h_bottom, proj);
                // double dist3 = vect_norm(p_left, p_right) / 2.0;
                
                // Point p11;
                // // p11.x = p_left.x + dist3 * l_r.x;
                // // p11.y = p_left.y + dist3 * l_r.y;
                // // p11.z = p_left.z + dist3 * l_r.z;
                // p11.x = p_left.x + l_r.x/2.0;
                // p11.y = p_left.y + l_r.y/2.0;
                // p11.z = p_left.z + l_r.z/2.0;
                // p11.r = 100;
                // p11.b = 100;
                // p11.a = 255;
                // Point p22;
                // p22.x = p_left.x + (dist1 - dist2) * b_tt.x;
                // p22.y = p_left.y + (dist1 - dist2) * b_tt.y;
                // p22.z = p_left.z + (dist1 - dist2) * b_tt.z;
                // p22.r = 200;
                // p22.b = 200;
                // p22.a = 255;
                // (*final_cloud).push_back(p11);
                // (*final_cloud).push_back(p22);

                Point origin;
                // origin.x = w_left.x + l_r.x / 2.0 + (dist1 - dist2) * b_tt.x;
                // origin.y = w_left.y + l_r.y / 2.0 + (dist1 - dist2) * b_tt.y;
                // origin.z = w_left.z + l_r.z / 2.0 + (dist1 - dist2) * b_tt.z;
                origin.x = 0;
                origin.y = 0;
                origin.z = 0;
                // origin.x = ((p_left.x + p_right.x)/2 + (p_bottom.x + p_top.x)/2)/2;
                // origin.y = ((p_left.y + p_right.y)/2 + (p_bottom.y + p_top.y)/2)/2;
                // origin.z = ((p_left.z + p_right.z)/2 + (p_bottom.z + p_top.z)/2)/2;
                origin.r = 100;
                origin.g = 255;
                origin.a = 255;
                for(int i = 0; i < samples.size(); i++){
                    origin.x += samples[i].x;
                    origin.y += samples[i].y;
                    origin.z += samples[i].z;
                }
                origin.x /= samples.size();
                origin.y /= samples.size();
                origin.z /= samples.size();

                // std::cout << "origin x " << origin.x << " / " << ((p_left.x + p_right.x)/2 + (p_bottom.x + p_top.x)/2)/2 << std::endl;
                // std::cout << "origin y " << origin.y << " / " << ((p_left.y + p_right.y)/2 + (p_bottom.y + p_top.y)/2)/2 << std::endl;
                // std::cout << "origin z " << origin.z << " / " << ((p_left.z + p_right.z)/2 + (p_bottom.z + p_top.z)/2)/2 << std::endl;

                // (*boundary_cloud).push_back(origin);

                boundary_samples.push_back(samples);
                boundary_samples_origins.push_back(origin);
                boundary_samples_indices.push_back(c);
                boundary_samples_bounds.push_back(samples_bounds);
                // break;
            }
            // break;
        }

        std::cout << "after while" << std::endl;
    }
    
    std::cout << "sampling " << boundary_samples.size() << std::endl;
    for(int c = 0; c < boundary_samples.size(); c++){
    // for(int c = 6; c < boundary_samples.size(); c++){
    // for(int c = 5; c < 6; c++){
    
        int boundary_idx = boundary_samples_indices[c];

        Point n;
        n.x = best_planes_normal[boundary_idx].normal_x;
        n.y = best_planes_normal[boundary_idx].normal_y;
        n.z = best_planes_normal[boundary_idx].normal_z;
        Point norm_1 = get_vector(boundary_samples_origins[c], n);

        // visualization_msgs::Marker marker;
        // marker.header.frame_id = "base_frame";
        // marker.header.stamp = ros::Time::now();
        // marker.action = visualization_msgs::Marker::ADD;
        // marker.id = 505+ 10*c -1;
        // marker.type = visualization_msgs::Marker::ARROW;
        // marker.scale.x = 0.003;
        // marker.scale.y = 0.005;
        // marker.color.g = 1.0f;
        // marker.color.a = 1.0;
        // geometry_msgs::Point p0;
        // p0.x = boundary_samples_origins[c].x;
        // p0.y = boundary_samples_origins[c].y;
        // p0.z = boundary_samples_origins[c].z;
        // // p0.x = 0;
        // // p0.y = 0;
        // // p0.z = 0;
        // geometry_msgs::Point p1;
        // p1.x = (boundary_samples_origins[c].x + best_planes_normal[boundary_idx].normal_x);
        // p1.y = (boundary_samples_origins[c].y + best_planes_normal[boundary_idx].normal_y);
        // p1.z = (boundary_samples_origins[c].z + best_planes_normal[boundary_idx].normal_z);
        // // p1.x = (best_planes_normal[c].normal_x);
        // // p1.y = (best_planes_normal[c].normal_y);
        // // p1.z = (best_planes_normal[c].normal_z);
        // marker.points.push_back(p0);
        // marker.points.push_back(p1);
        // markerArray.markers.push_back(marker);
        // // markerArray3.markers.push_back(marker);
        // marker_pub.publish(markerArray);
        // marker_pub2.publish(markerArray2);  

        // visualization_msgs::Marker marker2;
        // marker2.header.frame_id = "base_frame";
        // marker2.header.stamp = ros::Time::now();
        // marker2.action = visualization_msgs::Marker::ADD;
        // marker2.id = c+100;
        // marker2.type = visualization_msgs::Marker::POINTS;
        // marker2.scale.x = 0.005;
        // marker2.scale.y = 0.005;
        // marker2.scale.z = 0.005;
        // marker2.color.r = 1.0f;
        // marker2.color.g = 1.0f;
        // marker2.color.a = 1.0;
        // geometry_msgs::Point p2;
        // p2.x = boundary_samples_origins[c].x;
        // p2.y = boundary_samples_origins[c].y;
        // p2.z = boundary_samples_origins[c].z;
        // marker2.points.push_back(p2);
        // markerArray2.markers.push_back(marker2);

        // continue;
        int r = 100 + rand() % 155;
        int g = 100 + rand() % 155;
        int b = 100 + rand() % 155;

        double tmp_angle = uav_min_angle;
        // Point p_left = bounds[boundary_idx][0];
        // Point p_right = bounds[boundary_idx][2];
        // Point p_bottom = bounds[boundary_idx][1];
        // Point p_top = bounds[boundary_idx][3];
        // (*boundary_cloud).push_back(p_left);
        Point p_left = boundary_samples_bounds[c][0];
        Point p_right = boundary_samples_bounds[c][2];
        Point p_bottom = boundary_samples_bounds[c][1];
        Point p_top = boundary_samples_bounds[c][3];

        // Point vect_left_to_right = rectangle_vectors[c][0];
        // Point vect_bottom_to_top = rectangle_vectors[c][1];
        Point vect_left_to_right = get_vector(p_left, p_right);
        Point vect_bottom_to_top = get_vector(p_bottom, p_top);
        double dist_left_to_right = vect_norm(p_left, p_right);
        double dist_bottom_to_top = vect_norm(p_bottom, p_top);
        double dist_a = uav_max_distance * cos(tmp_angle);
        double dist_b = uav_max_distance * sin(tmp_angle);
        double dist_c = sqrt(uav_max_distance*uav_max_distance - (dist_left_to_right / 2.0)*(dist_left_to_right / 2.0));
        double dist_a_min = uav_min_distance / tan(tmp_angle);// - (dist_left_to_right / 2.0);
        double angle_line_intersection_dist = (dist_left_to_right / 2.0) * tan(tmp_angle);

        Point origin_norm_bound;
        origin_norm_bound.x = boundary_samples_origins[c].x + dist_c * n.x;
        origin_norm_bound.y = boundary_samples_origins[c].y + dist_c * n.y;
        origin_norm_bound.z = boundary_samples_origins[c].z + dist_c * n.z;
        origin_norm_bound.r = 255;
        origin_norm_bound.a = 255;
        
        Point left_bound;
        Point right_bound;
        Point bottom_bound;
        Point top_bound;
        if(dist_b > uav_min_distance){
            // Point left_bound;
            // left_bound.x = boundary_samples[c][0].x + vect_bottom_to_top.x / 2.0 + (dist_left_to_right - cos(tmp_angle)*uav_max_distance)*vect_left_to_right.x + sin(tmp_angle)*uav_max_distance*n.x;
            // left_bound.y = boundary_samples[c][0].y + vect_bottom_to_top.y / 2.0 + (dist_left_to_right - cos(tmp_angle)*uav_max_distance)*vect_left_to_right.y + sin(tmp_angle)*uav_max_distance*n.y;
            // left_bound.z = boundary_samples[c][0].z + vect_bottom_to_top.z / 2.0 + (dist_left_to_right - cos(tmp_angle)*uav_max_distance)*vect_left_to_right.z + sin(tmp_angle)*uav_max_distance*n.z;
            left_bound.x = boundary_samples_origins[c].x + ((dist_a - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.x + dist_b * n.x;
            left_bound.y = boundary_samples_origins[c].y + ((dist_a - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.y + dist_b * n.y;
            left_bound.z = boundary_samples_origins[c].z + ((dist_a - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.z + dist_b * n.z;
            // left_bound.x = boundary_samples_origins[c].x + (dist_left_to_right / (dist_a - dist_left_to_right / 2.0)) * vect_left_to_right.x + dist_b * n.x;
            // left_bound.y = boundary_samples_origins[c].y + (dist_left_to_right / (dist_a - dist_left_to_right / 2.0)) * vect_left_to_right.y + dist_b * n.y;
            // left_bound.z = boundary_samples_origins[c].z + (dist_left_to_right / (dist_a - dist_left_to_right / 2.0)) * vect_left_to_right.z + dist_b * n.z;
            left_bound.r = 255;
            left_bound.a = 255;
            // Point right_bound;
            right_bound.x = boundary_samples_origins[c].x - ((dist_a - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.x + dist_b * n.x;
            right_bound.y = boundary_samples_origins[c].y - ((dist_a - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.y + dist_b * n.y;
            right_bound.z = boundary_samples_origins[c].z - ((dist_a - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.z + dist_b * n.z;
            right_bound.r = 255;
            right_bound.a = 255;
            // Point bottom_bound;
            bottom_bound.x = boundary_samples_origins[c].x + ((dist_a - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.x + dist_b * n.x;
            bottom_bound.y = boundary_samples_origins[c].y + ((dist_a - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.y + dist_b * n.y;
            bottom_bound.z = boundary_samples_origins[c].z + ((dist_a - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.z + dist_b * n.z;
            bottom_bound.r = 255;
            bottom_bound.a = 255;
            // Point top_bound;
            top_bound.x = boundary_samples_origins[c].x - ((dist_a - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.x + dist_b * n.x;
            top_bound.y = boundary_samples_origins[c].y - ((dist_a - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.y + dist_b * n.y;
            top_bound.z = boundary_samples_origins[c].z - ((dist_a - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.z + dist_b * n.z;
            top_bound.r = 255;
            top_bound.a = 255;
        }
        else{
            // Point left_bound;
            left_bound.x = boundary_samples_origins[c].x + ((dist_a - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.x + uav_min_distance * n.x;
            left_bound.y = boundary_samples_origins[c].y + ((dist_a - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.y + uav_min_distance * n.y;
            left_bound.z = boundary_samples_origins[c].z + ((dist_a - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.z + uav_min_distance * n.z;
            left_bound.r = 255;
            left_bound.a = 255;
            // Point right_bound;
            right_bound.x = boundary_samples_origins[c].x - ((dist_a - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.x + uav_min_distance * n.x;
            right_bound.y = boundary_samples_origins[c].y - ((dist_a - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.y + uav_min_distance * n.y;
            right_bound.z = boundary_samples_origins[c].z - ((dist_a - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.z + uav_min_distance * n.z;
            right_bound.r = 255;
            right_bound.a = 255;
            // Point bottom_bound;
            bottom_bound.x = boundary_samples_origins[c].x + ((dist_a - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.x + uav_min_distance * n.x;
            bottom_bound.y = boundary_samples_origins[c].y + ((dist_a - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.y + uav_min_distance * n.y;
            bottom_bound.z = boundary_samples_origins[c].z + ((dist_a - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.z + uav_min_distance * n.z;
            bottom_bound.r = 255;
            bottom_bound.a = 255;
            // Point top_bound;
            top_bound.x = boundary_samples_origins[c].x - ((dist_a - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.x + uav_min_distance * n.x;
            top_bound.y = boundary_samples_origins[c].y - ((dist_a - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.y + uav_min_distance * n.y;
            top_bound.z = boundary_samples_origins[c].z - ((dist_a - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.z + uav_min_distance * n.z;
            top_bound.r = 255;
            top_bound.a = 255;
        }
        

        // Point left_bound_min;
        // left_bound_min.x = boundary_samples_origins[c].x + ((dist_a_min - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.x + uav_min_distance * n.x;
        // left_bound_min.y = boundary_samples_origins[c].y + ((dist_a_min - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.y + uav_min_distance * n.y;
        // left_bound_min.z = boundary_samples_origins[c].z + ((dist_a_min - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.z + uav_min_distance * n.z;
        // left_bound_min.r = 255;
        // left_bound_min.a = 255;
        // Point right_bound_min;
        // right_bound_min.x = boundary_samples_origins[c].x - ((dist_a_min - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.x + uav_min_distance * n.x;
        // right_bound_min.y = boundary_samples_origins[c].y - ((dist_a_min - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.y + uav_min_distance * n.y;
        // right_bound_min.z = boundary_samples_origins[c].z - ((dist_a_min - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.z + uav_min_distance * n.z;
        // right_bound_min.r = 255;
        // right_bound_min.a = 255;
        // Point bottom_bound_min;
        // bottom_bound_min.x = boundary_samples_origins[c].x + ((dist_a_min - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.x + uav_min_distance * n.x;
        // bottom_bound_min.y = boundary_samples_origins[c].y + ((dist_a_min - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.y + uav_min_distance * n.y;
        // bottom_bound_min.z = boundary_samples_origins[c].z + ((dist_a_min - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.z + uav_min_distance * n.z;
        // bottom_bound_min.r = 255;
        // bottom_bound_min.a = 255;
        // Point top_bound_min;
        // top_bound_min.x = boundary_samples_origins[c].x - ((dist_a_min - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.x + uav_min_distance * n.x;
        // top_bound_min.y = boundary_samples_origins[c].y - ((dist_a_min - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.y + uav_min_distance * n.y;
        // top_bound_min.z = boundary_samples_origins[c].z - ((dist_a_min - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.z + uav_min_distance * n.z;
        // top_bound_min.r = 255;
        // top_bound_min.a = 255;

        Point left_bound_min;
        Point right_bound_min;
        Point bottom_bound_min;
        Point top_bound_min;
        // std::cout << "angle_line_intersection_dist: " << angle_line_intersection_dist << ", uav_min_distance: " << uav_min_distance << std::endl;
        if(angle_line_intersection_dist < uav_min_distance){
            left_bound_min.x = boundary_samples_origins[c].x + ((dist_a_min - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.x + uav_min_distance * n.x;
            left_bound_min.y = boundary_samples_origins[c].y + ((dist_a_min - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.y + uav_min_distance * n.y;
            left_bound_min.z = boundary_samples_origins[c].z + ((dist_a_min - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.z + uav_min_distance * n.z;
            left_bound_min.r = 255;
            left_bound_min.a = 255;

            right_bound_min.x = boundary_samples_origins[c].x - ((dist_a_min - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.x + uav_min_distance * n.x;
            right_bound_min.y = boundary_samples_origins[c].y - ((dist_a_min - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.y + uav_min_distance * n.y;
            right_bound_min.z = boundary_samples_origins[c].z - ((dist_a_min - dist_left_to_right / 2.0) / dist_left_to_right) * vect_left_to_right.z + uav_min_distance * n.z;
            right_bound_min.r = 255;
            right_bound_min.a = 255;

            bottom_bound_min.x = boundary_samples_origins[c].x + ((dist_a_min - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.x + uav_min_distance * n.x;
            bottom_bound_min.y = boundary_samples_origins[c].y + ((dist_a_min - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.y + uav_min_distance * n.y;
            bottom_bound_min.z = boundary_samples_origins[c].z + ((dist_a_min - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.z + uav_min_distance * n.z;
            bottom_bound_min.r = 255;
            bottom_bound_min.a = 255;

            top_bound_min.x = boundary_samples_origins[c].x - ((dist_a_min - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.x + uav_min_distance * n.x;
            top_bound_min.y = boundary_samples_origins[c].y - ((dist_a_min - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.y + uav_min_distance * n.y;
            top_bound_min.z = boundary_samples_origins[c].z - ((dist_a_min - dist_bottom_to_top / 2.0) / dist_bottom_to_top) * vect_bottom_to_top.z + uav_min_distance * n.z;
            top_bound_min.r = 255;
            top_bound_min.a = 255;
        }
        else{
            left_bound_min.x = boundary_samples_origins[c].x + angle_line_intersection_dist * n.x;
            left_bound_min.y = boundary_samples_origins[c].y + angle_line_intersection_dist * n.y;
            left_bound_min.z = boundary_samples_origins[c].z + angle_line_intersection_dist * n.z;
            left_bound_min.r = 255;
            left_bound_min.a = 255;

            right_bound_min.x = boundary_samples_origins[c].x + angle_line_intersection_dist * n.x;
            right_bound_min.y = boundary_samples_origins[c].y + angle_line_intersection_dist * n.y;
            right_bound_min.z = boundary_samples_origins[c].z + angle_line_intersection_dist * n.z;
            right_bound_min.r = 255;
            right_bound_min.a = 255;

            bottom_bound_min.x = boundary_samples_origins[c].x + angle_line_intersection_dist * n.x;
            bottom_bound_min.y = boundary_samples_origins[c].y + angle_line_intersection_dist * n.y;
            bottom_bound_min.z = boundary_samples_origins[c].z + angle_line_intersection_dist * n.z;
            bottom_bound_min.r = 255;
            bottom_bound_min.a = 255;

            top_bound_min.x = boundary_samples_origins[c].x + angle_line_intersection_dist * n.x;
            top_bound_min.y = boundary_samples_origins[c].y + angle_line_intersection_dist * n.y;
            top_bound_min.z = boundary_samples_origins[c].z + angle_line_intersection_dist * n.z;
            top_bound_min.r = 255;
            top_bound_min.a = 255;
        }

        // (*boundary_cloud).push_back(left_bound);
        // (*boundary_cloud).push_back(right_bound);
        // (*boundary_cloud).push_back(bottom_bound);
        // (*boundary_cloud).push_back(top_bound);
        // (*boundary_cloud).push_back(origin_norm_bound);
        // (*boundary_cloud).push_back(left_bound_min);
        // (*boundary_cloud).push_back(right_bound_min);
        // (*boundary_cloud).push_back(bottom_bound_min);
        // (*boundary_cloud).push_back(top_bound_min);
        std::vector<Point> bound_points_min;
        std::vector<Point> bound_points_middle;
        std::vector<Point> bound_points;
        bound_points_middle.push_back(left_bound);
        bound_points_middle.push_back(right_bound);
        bound_points_middle.push_back(bottom_bound);
        bound_points_middle.push_back(top_bound);
        bound_points.push_back(origin_norm_bound);
        bound_points_min.push_back(left_bound_min);
        bound_points_min.push_back(right_bound_min);
        bound_points_min.push_back(bottom_bound_min);
        bound_points_min.push_back(top_bound_min);

        for(int i = 0; i < area_samples_mid; i++){
            Point v1 = get_vector(left_bound, top_bound);
            Point v2 = get_vector(left_bound, bottom_bound);
            Point v3 = get_vector(right_bound, top_bound);
            Point v4 = get_vector(right_bound, bottom_bound);
            Point v5 = get_vector(left_bound_min, top_bound_min);
            Point v6 = get_vector(left_bound_min, bottom_bound_min);
            Point v7 = get_vector(right_bound_min, top_bound_min);
            Point v8 = get_vector(right_bound_min, bottom_bound_min);

            Point new_sample1;
            new_sample1.x = left_bound.x + ((i + 1.0) /  (area_samples_mid + 1.0)) * v1.x;
            new_sample1.y = left_bound.y + ((i + 1.0) /  (area_samples_mid + 1.0)) * v1.y;
            new_sample1.z = left_bound.z + ((i + 1.0) /  (area_samples_mid + 1.0)) * v1.z;
            Point new_sample2;
            new_sample2.x = left_bound.x + ((i + 1.0) /  (area_samples_mid + 1.0)) * v2.x;
            new_sample2.y = left_bound.y + ((i + 1.0) /  (area_samples_mid + 1.0)) * v2.y;
            new_sample2.z = left_bound.z + ((i + 1.0) /  (area_samples_mid + 1.0)) * v2.z;
            Point new_sample3;
            new_sample3.x = right_bound.x + ((i + 1.0) /  (area_samples_mid + 1.0)) * v3.x;
            new_sample3.y = right_bound.y + ((i + 1.0) /  (area_samples_mid + 1.0)) * v3.y;
            new_sample3.z = right_bound.z + ((i + 1.0) /  (area_samples_mid + 1.0)) * v3.z;
            Point new_sample4;
            new_sample4.x = right_bound.x + ((i + 1.0) /  (area_samples_mid + 1.0)) * v4.x;
            new_sample4.y = right_bound.y + ((i + 1.0) /  (area_samples_mid + 1.0)) * v4.y;
            new_sample4.z = right_bound.z + ((i + 1.0) /  (area_samples_mid + 1.0)) * v4.z;

            Point new_sample5;
            new_sample5.x = left_bound_min.x + ((i + 1.0) /  (area_samples_mid + 1.0)) * v5.x;
            new_sample5.y = left_bound_min.y + ((i + 1.0) /  (area_samples_mid + 1.0)) * v5.y;
            new_sample5.z = left_bound_min.z + ((i + 1.0) /  (area_samples_mid + 1.0)) * v5.z;
            Point new_sample6;
            new_sample6.x = left_bound_min.x + ((i + 1.0) /  (area_samples_mid + 1.0)) * v6.x;
            new_sample6.y = left_bound_min.y + ((i + 1.0) /  (area_samples_mid + 1.0)) * v6.y;
            new_sample6.z = left_bound_min.z + ((i + 1.0) /  (area_samples_mid + 1.0)) * v6.z;
            Point new_sample7;
            new_sample7.x = right_bound_min.x + ((i + 1.0) /  (area_samples_mid + 1.0)) * v7.x;
            new_sample7.y = right_bound_min.y + ((i + 1.0) /  (area_samples_mid + 1.0)) * v7.y;
            new_sample7.z = right_bound_min.z + ((i + 1.0) /  (area_samples_mid + 1.0)) * v7.z;
            Point new_sample8;
            new_sample8.x = right_bound_min.x + ((i + 1.0) /  (area_samples_mid + 1.0)) * v8.x;
            new_sample8.y = right_bound_min.y + ((i + 1.0) /  (area_samples_mid + 1.0)) * v8.y;
            new_sample8.z = right_bound_min.z + ((i + 1.0) /  (area_samples_mid + 1.0)) * v8.z;

            bound_points_middle.push_back(new_sample1);
            bound_points_middle.push_back(new_sample2);
            bound_points_middle.push_back(new_sample3);
            bound_points_middle.push_back(new_sample4);
            bound_points_min.push_back(new_sample5);
            bound_points_min.push_back(new_sample6);
            bound_points_min.push_back(new_sample7);
            bound_points_min.push_back(new_sample8);
        }

        for(int i = 0; i < area_samples_first; i++){
            for(int j = 0; j < bound_points_min.size(); j++){
                Point v = get_vector(bound_points_min[j], bound_points_middle[j]);

                Point new_sample;
                new_sample.x = bound_points_min[j].x + ((i + 1.0) /  (area_samples_first + 1.0)) * v.x;
                new_sample.y = bound_points_min[j].y + ((i + 1.0) /  (area_samples_first + 1.0)) * v.y;
                new_sample.z = bound_points_min[j].z + ((i + 1.0) /  (area_samples_first + 1.0)) * v.z;

                new_sample.r = 200;
                new_sample.g = 100;
                new_sample.b = 255;
                new_sample.a = 255;

                bound_points.push_back(new_sample);
            }
        }

        for(int i = 0; i < area_samples_second; i++){
            for(int j = 0; j < bound_points_min.size(); j++){
                Point v = get_vector(bound_points_middle[j], origin_norm_bound);

                Point new_sample;
                new_sample.x = bound_points_middle[j].x + ((i + 1.0) /  (area_samples_second + 1.0)) * v.x;
                new_sample.y = bound_points_middle[j].y + ((i + 1.0) /  (area_samples_second + 1.0)) * v.y;
                new_sample.z = bound_points_middle[j].z + ((i + 1.0) /  (area_samples_second + 1.0)) * v.z;

                new_sample.r = 255;
                new_sample.g = 0;
                new_sample.b = 255;
                new_sample.a = 255;

                bound_points.push_back(new_sample);
            }
        }

        for(int i = 0; i < bound_points_middle.size(); i++){
            bound_points_middle[i].r = 255;
            bound_points_middle[i].g = 0;
            bound_points_middle[i].b = 0;
            bound_points_middle[i].a = 255;
            bound_points.push_back(bound_points_middle[i]);
        }

        for(int i = 0; i < bound_points_min.size(); i++){
            bound_points_min[i].r = 250;
            bound_points_min[i].g = 200;
            bound_points_min[i].b = 0;
            bound_points_min[i].a = 255;
            bound_points.push_back(bound_points_min[i]);
        }

        for(int i = 0; i < bound_points.size(); i++){
            // (*boundary_cloud).push_back(bound_points[i]);
            bound_points[i].r = 255 - (i * 50);
            bound_points[i].g = 255;
            bound_points[i].b = 255 - (i*100);
            (*areas_cloud).push_back(bound_points[i]);

            suitable_area_points.push_back(bound_points[i]);
            suitable_areas_points_indices.push_back(c);
        }

        // double origin_to_origin_dist = sqrt(uav_max_distance*uav_max_distance - dist_left_to_right*dist_left_to_right);
        // Point sample_area_origin;
        // sample_area_origin.x = boundary_samples_origins[c].x + origin_to_origin_dist*best_planes_normal[boundary_idx].normal_x;
        // sample_area_origin.y = boundary_samples_origins[c].y + origin_to_origin_dist*best_planes_normal[boundary_idx].normal_y;
        // sample_area_origin.z = boundary_samples_origins[c].z + origin_to_origin_dist*best_planes_normal[boundary_idx].normal_z;
        
        // visualization_msgs::Marker marker;
        // marker.header.frame_id = "base_frame";
        // marker.header.stamp = ros::Time::now();
        // marker.action = visualization_msgs::Marker::ADD;
        // marker.id = 505+ 10*c -1;
        // marker.type = visualization_msgs::Marker::ARROW;
        // marker.scale.x = 0.003;
        // marker.scale.y = 0.005;
        // marker.color.g = 1.0f;
        // marker.color.a = 1.0;
        // geometry_msgs::Point p0;
        // p0.x = boundary_samples_origins[c].x;
        // p0.y = boundary_samples_origins[c].y;
        // p0.z = boundary_samples_origins[c].z;
        // // p0.x = 0;
        // // p0.y = 0;
        // // p0.z = 0;
        // geometry_msgs::Point p1;
        // p1.x = sample_area_origin.x;
        // p1.y = sample_area_origin.y;
        // p1.z = sample_area_origin.z;
        // // p1.x = (best_planes_normal[c].normal_x);
        // // p1.y = (best_planes_normal[c].normal_y);
        // // p1.z = (best_planes_normal[c].normal_z);
        // marker.points.push_back(p0);
        // marker.points.push_back(p1);
        // markerArray.markers.push_back(marker);
        // marker_pub.publish(markerArray);


        // for(int j = 0; j < boundary_samples[c].size(); j++){
        //     Point curr_vec = get_vector(boundary_samples[c][j], sample_area_origin);
        //     for(int i = 1; i <= 3; i++){
        //         visualization_msgs::Marker marker2;
        //         marker2.header.frame_id = "base_frame";
        //         marker2.header.stamp = ros::Time::now();
        //         marker2.action = visualization_msgs::Marker::ADD;
        //         marker2.id = c+100 + 4*j + i;
        //         marker2.type = visualization_msgs::Marker::POINTS;
        //         marker2.scale.x = 0.005;
        //         marker2.scale.y = 0.005;
        //         marker2.scale.z = 0.005;
        //         marker2.color.r = 0.7f;
        //         marker2.color.b = 1.0f;
        //         marker2.color.a = 1.0;
        //         geometry_msgs::Point p2;
        //         p2.x = boundary_samples[c][j].x + (i / 4.0)*curr_vec.x;
        //         p2.y = boundary_samples[c][j].y + (i / 4.0)*curr_vec.y;
        //         p2.z = boundary_samples[c][j].z + (i / 4.0)*curr_vec.z;
        //         marker2.points.push_back(p2);
        //         markerArray2.markers.push_back(marker2);
        //     }
        // }
        // visualization_msgs::Marker marker2;
        // marker2.header.frame_id = "base_frame";
        // marker2.header.stamp = ros::Time::now();
        // marker2.action = visualization_msgs::Marker::ADD;
        // marker2.id = c+100 + 113;
        // marker2.type = visualization_msgs::Marker::POINTS;
        // marker2.scale.x = 0.005;
        // marker2.scale.y = 0.005;
        // marker2.scale.z = 0.005;
        // marker2.color.r = 0.7f;
        // marker2.color.b = 1.0f;
        // marker2.color.a = 1.0;
        // geometry_msgs::Point p2;
        // p2.x = sample_area_origin.x;
        // p2.y = sample_area_origin.y;
        // p2.z = sample_area_origin.z;
        // marker2.points.push_back(p2);
        // markerArray2.markers.push_back(marker2);

        
        for(int j = 0; j < 15000; j++){
            continue;
            // double min = -(sample_criterion / 2.0);
            // double max = sample_criterion / 2.0;
            double min = -1.0;
            double max = 1.0;
            // double x = min + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(max - min)));
            double x = min + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(max - min)));
            double y = min + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(max - min)));
            // double y = (min + 0.4) + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/((max - 0.4) - (min + 0.4))));
            double z = min + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(max - min)));
            Point p_rand;
            p_rand.x = boundary_samples_origins[c].x + x;
            p_rand.y = boundary_samples_origins[c].y + y;
            p_rand.z = boundary_samples_origins[c].z + z;

            // remove wrong side of tangent plane
            Point zero;
            zero.x = 0.0;
            zero.y = 0.0;
            zero.z = 0.0;
            Point normal;
            normal.x = best_planes_normal[boundary_idx].normal_x;
            normal.y = best_planes_normal[boundary_idx].normal_y;
            normal.z = best_planes_normal[boundary_idx].normal_z;
            pcl::Normal line_norm;
            line_norm.normal_x = rectangle_vectors[boundary_idx][0].x;
            line_norm.normal_y = rectangle_vectors[boundary_idx][0].y;
            line_norm.normal_z = rectangle_vectors[boundary_idx][0].z;
            Point p_proj = tangent_projection(line_norm, boundary_samples_origins[c], p_rand);

            // Point vec = get_vector(boundary_samples_origins[c], p_rand);
            Point vec = get_vector(boundary_samples_origins[c], p_proj);

            // double angle = acos( (vec.x*normal.x + vec.y*normal.y + vec.z*normal.z) / (vect_norm(normal, zero) * vect_norm(boundary_samples_origins[c], vec)) );
            double angle = acos( (vec.x*normal.x + vec.y*normal.y + vec.z*normal.z) / (vect_norm(normal, zero) * vect_norm(zero, vec)) );
            // std::cout << "angle: " << angle << " / " << (M_PI / 2) << " norm: " << vect_norm(normal, zero) << std::endl;
            // angle = acos( (vec.x*normal.x + vec.y*normal.y + vec.z*normal.z) / vect_norm(boundary_samples_origins[c], vec)) ;
            // std::cout << "angle: " << angle << " / " << (M_PI / 2) << std::endl;
            // std::cout << "----" << std::endl;
            if(angle > M_PI / 2){
            // if(fabs(angle) > M_PI / 2){//} || angle < - M_PI / 2){
                // std::cout << "break" << std::endl;
                j--;
                continue;
            }
            // else{

            //     visualization_msgs::Marker marker2;
            //     marker2.header.frame_id = "base_frame";
            //     marker2.header.stamp = ros::Time::now();
            //     marker2.action = visualization_msgs::Marker::ADD;
            //     marker2.id = c+100 + j;
            //     marker2.type = visualization_msgs::Marker::POINTS;
            //     marker2.scale.x = 0.005;
            //     marker2.scale.y = 0.005;
            //     marker2.scale.z = 0.005;
            //     marker2.color.r = 1.0f;
            //     marker2.color.b = 1.0f;
            //     marker2.color.a = 1.0;
            //     geometry_msgs::Point p2;
            //     p2.x = p_proj.x;
            //     p2.y = p_proj.y;
            //     p2.z = p_proj.z;
            //     marker2.points.push_back(p2);
            //     markerArray2.markers.push_back(marker2);

            //     visualization_msgs::Marker marker12;
            //     marker12.header.frame_id = "base_frame";
            //     marker12.header.stamp = ros::Time::now();
            //     marker12.action = visualization_msgs::Marker::ADD;
            //     marker12.id = 505 + 10*c + j;
            //     marker12.type = visualization_msgs::Marker::ARROW;
            //     marker12.scale.x = 0.003;
            //     marker12.scale.y = 0.005;
            //     marker12.color.r = 1.0f;
            //     marker12.color.g = 1.0f;
            //     marker12.color.a = 1.0;
            //     geometry_msgs::Point p0;
            //     p0.x = boundary_samples_origins[c].x;
            //     p0.y = boundary_samples_origins[c].y;
            //     p0.z = boundary_samples_origins[c].z;
            //     // p0.x = 0;
            //     // p0.y = 0;
            //     // p0.z = 0;
            //     geometry_msgs::Point p1;
            //     // p1.x = boundary_samples_origins[c].x + vec.x;
            //     // p1.y = boundary_samples_origins[c].y + vec.y;
            //     // p1.z = boundary_samples_origins[c].z + vec.z;
            //     p1.x = boundary_samples_origins[c].x + vec.x;
            //     p1.y = boundary_samples_origins[c].y + vec.y;
            //     p1.z = boundary_samples_origins[c].z + vec.z;
            //     // p1.x = (best_planes_normal[c].normal_x);
            //     // p1.y = (best_planes_normal[c].normal_y);
            //     // p1.z = (best_planes_normal[c].normal_z);
            //     marker12.points.push_back(p0);
            //     marker12.points.push_back(p1);
            //     markerArray.markers.push_back(marker12);
            // }


            // bool cont = false;
            // for(int idx = 0; idx < boundary_samples[c].size(); idx++){
            //     Point zero;
            //     zero.x = 0.0;
            //     zero.y = 0.0;
            //     zero.z = 0.0;
            //     Point normal;
            //     normal.x = best_planes_normal[boundary_idx].normal_x;
            //     normal.y = best_planes_normal[boundary_idx].normal_y;
            //     normal.z = best_planes_normal[boundary_idx].normal_z;
            //     pcl::Normal line_norm;
            //     line_norm.normal_x = rectangle_vectors[c][0].x;
            //     line_norm.normal_y = rectangle_vectors[c][0].y;
            //     line_norm.normal_z = rectangle_vectors[c][0].z;
            //     Point p_proj = tangent_projection(line_norm, boundary_samples_origins[c], p_rand);

            //     visualization_msgs::Marker marker2;
            //     marker2.header.frame_id = "base_frame";
            //     marker2.header.stamp = ros::Time::now();
            //     marker2.action = visualization_msgs::Marker::ADD;
            //     marker2.id = c+100 + j;
            //     marker2.type = visualization_msgs::Marker::POINTS;
            //     marker2.scale.x = 0.005;
            //     marker2.scale.y = 0.005;
            //     marker2.scale.z = 0.005;
            //     marker2.color.r = 1.0f;
            //     marker2.color.b = 1.0f;
            //     marker2.color.a = 1.0;
            //     geometry_msgs::Point p2;
            //     p2.x = p_proj.x;
            //     p2.y = p_proj.y;
            //     p2.z = p_proj.z;
            //     marker2.points.push_back(p2);
            //     markerArray2.markers.push_back(marker2);

            //     // Point vec = get_vector(boundary_samples_origins[c], p_rand);
            //     Point vec = get_vector(boundary_samples_origins[c], p_proj);

            //     // double angle = acos( (vec.x*normal.x + vec.y*normal.y + vec.z*normal.z) / (vect_norm(normal, zero) * vect_norm(boundary_samples_origins[c], vec)) );
            //     double angle = acos( (vec.x*normal.x + vec.y*normal.y + vec.z*normal.z) / vect_norm(boundary_samples_origins[c], vec)) ;
            //     // std::cout << "angle: " << angle << " / " << (M_PI / 2) << std::endl;
            //     if(fabs(angle) > M_PI / 2){//} || angle < - M_PI / 2){
            //         // std::cout << "break" << std::endl;
            //         cont = true;
            //         break;
            //     }
            //     else{
            //         visualization_msgs::Marker marker;
            //         marker.header.frame_id = "base_frame";
            //         marker.header.stamp = ros::Time::now();
            //         marker.action = visualization_msgs::Marker::ADD;
            //         marker.id = 505 + 10*c + j;
            //         marker.type = visualization_msgs::Marker::ARROW;
            //         marker.scale.x = 0.003;
            //         marker.scale.y = 0.005;
            //         marker.color.r = 1.0f;
            //         marker.color.g = 1.0f;
            //         marker.color.a = 1.0;
            //         geometry_msgs::Point p0;
            //         p0.x = boundary_samples_origins[c].x;
            //         p0.y = boundary_samples_origins[c].y;
            //         p0.z = boundary_samples_origins[c].z;
            //         // p0.x = 0;
            //         // p0.y = 0;
            //         // p0.z = 0;
            //         geometry_msgs::Point p1;
            //         // p1.x = boundary_samples_origins[c].x + vec.x;
            //         // p1.y = boundary_samples_origins[c].y + vec.y;
            //         // p1.z = boundary_samples_origins[c].z + vec.z;
            //         p1.x = boundary_samples_origins[c].x + vec.x;
            //         p1.y = boundary_samples_origins[c].y + vec.y;
            //         p1.z = boundary_samples_origins[c].z + vec.z;
            //         // p1.x = (best_planes_normal[c].normal_x);
            //         // p1.y = (best_planes_normal[c].normal_y);
            //         // p1.z = (best_planes_normal[c].normal_z);
            //         marker.points.push_back(p0);
            //         marker.points.push_back(p1);
            //         markerArray.markers.push_back(marker);
            //     }
            // }
            // if(cont){
            //     j--;
            //     continue;
            // }

            visualization_msgs::Marker marker3;
            marker3.header.frame_id = frame_id;
            marker3.header.stamp = ros::Time::now();
            marker3.action = visualization_msgs::Marker::ADD;
            marker3.id = c*10000+j;
            marker3.type = visualization_msgs::Marker::POINTS;
            marker3.scale.x = 0.005;
            marker3.scale.y = 0.005;
            marker3.scale.z = 0.005;
            marker3.color.r = 1.0f;
            marker3.color.g = 1.0f;
            marker3.color.a = 1.0;

            // distances
            bool ok_d = true;
            double min_dist_under_angle = uav_max_distance * sin(uav_min_angle);
            double uav_min_dist_to_hole = fmax(min_dist_under_angle, uav_min_distance);
            // bool ok_d = false;
            for(int idx = 0; idx < boundary_samples[c].size(); idx++){
                Point point = boundary_samples[c][idx];
                Point projection = tangent_projection(best_planes_normal[boundary_idx], boundary_samples_origins[c], p_rand);
                double max_dist = vect_norm(point, p_rand);
                double min_dist = vect_norm(projection, p_rand);

                // visualization_msgs::Marker marker5;
                // marker5.header.frame_id = "base_frame";
                // marker5.header.stamp = ros::Time::now();
                // marker5.action = visualization_msgs::Marker::ADD;
                // marker5.id = c + idx;
                // marker5.type = visualization_msgs::Marker::ARROW;
                // marker5.scale.x = 0.003;
                // marker5.scale.y = 0.005;
                // marker5.color.g = 1.0f;
                // marker5.color.a = 1.0;
                // geometry_msgs::Point p0;
                // p0.x = p_rand.x;
                // p0.y = p_rand.y;
                // p0.z = p_rand.z;
                // // p0.x = 0;
                // // p0.y = 0;
                // // p0.z = 0;
                // geometry_msgs::Point p1;
                // p1.x = point.x;
                // p1.y = point.y;
                // p1.z = point.z;
                // // p1.x = (best_planes_normal[c].normal_x);
                // // p1.y = (best_planes_normal[c].normal_y);
                // // p1.z = (best_planes_normal[c].normal_z);
                // marker5.points.push_back(p0);
                // marker5.points.push_back(p1);
                // markerArray.markers.push_back(marker5);

                // std::cout << "point: " << point.x << ", " << point.y << ", " << point.z << std::endl;

                if(min_dist > uav_min_distance && max_dist < uav_max_distance){// && max_dist < uav_max_distance){
                    // ok_d = false;
                    marker3.color.r = 1.0f;
                    marker3.color.g = 0.0f;
                    marker3.color.b = 0.0f;
                    // break;
                }
                if(min_dist > uav_min_dist_to_hole && max_dist < uav_max_distance){// && max_dist < uav_max_distance){
                    // ok_d = false;
                    marker3.color.r = 1.0f;
                    marker3.color.g = 0.0f;
                    marker3.color.b = 1.0f;
                    // break;
                }
                // if(max_dist < uav_max_distance){// && max_dist < uav_max_distance){
                //     // ok_d = false;
                //     marker3.color.r = 1.0f;
                //     marker3.color.g = 0.0f;
                //     marker3.color.b = 1.0f;
                //     // break;
                // }
                if(min_dist < uav_min_distance || max_dist > uav_max_distance){
                    ok_d = false;
                    // std::cout << "min: " << min_dist << ", max: " << max_dist << std::endl;
                    // j--;
                    break;
                }
            }
            if(ok_d){
                // marker3.color.r = 1.0f;
                // marker3.color.g = 0.0f;
                // marker3.color.b = 1.0f;
            }


            // angles
            bool ok_a = true;
            // bool ok_a = false;
            for(int idx = 0; idx < boundary_samples[c].size(); idx++){
                Point boundary_point = boundary_samples[c][idx];
                Point projection = tangent_projection(best_planes_normal[boundary_idx], boundary_samples_origins[c], p_rand);
                Point v_1 = get_vector(p_rand, projection);
                Point v_2 = get_vector(p_rand, boundary_point);
                double angle = (M_PI / 2) - acos( (v_1.x*v_2.x + v_1.y*v_2.y + v_1.z*v_2.z) / (vect_norm(boundary_point, p_rand) * vect_norm(projection, p_rand)) );
                if(angle < uav_min_angle){
                    // std::cout << "angle: " << angle << std::endl;
                    ok_a = false;
                    // j--;
                    break;
                }
            }
            if(ok_d && ok_a){
                // marker3.scale.x = 0.03;
                // marker3.scale.y = 0.03;
                // marker3.scale.z = 0.03;
                marker3.color.r = 1.0f;
                marker3.color.g = 1.0f;
                marker3.color.b = 1.0f;
                // marker3.color.r = (r / 255.0) * 1.0f;
                // marker3.color.g = (g / 255.0) * 1.0f;
                // marker3.color.b = (b / 255.0) * 1.0f;
                marker3.color.a = 1.0f;
                geometry_msgs::Point p3;
                p3.x = p_rand.x;
                p3.y = p_rand.y;
                p3.z = p_rand.z;
                // marker3.points.push_back(p3);
                // markerArray3.markers.push_back(marker3);

                p_rand.r = 255;
                p_rand.g = 255;
                p_rand.b = 255;
                p_rand.a = 255;

                // (*boundary_cloud).push_back(p_rand);

                // Point vec = get_vector(boundary_samples_origins[c], p_rand);

                // visualization_msgs::Marker marker;
                // marker.header.frame_id = "base_frame";
                // marker.header.stamp = ros::Time::now();
                // marker.action = visualization_msgs::Marker::ADD;
                // marker.id = 505 + 10*c + j;
                // marker.type = visualization_msgs::Marker::ARROW;
                // marker.scale.x = 0.003;
                // marker.scale.y = 0.005;
                // marker.color.g = 1.0f;
                // marker.color.a = 1.0;
                // geometry_msgs::Point p0;
                // p0.x = 0.0;
                // p0.y = 0.0;
                // p0.z = 0.0;
                // // p0.x = 0;
                // // p0.y = 0;
                // // p0.z = 0;
                // geometry_msgs::Point p1;
                // // p1.x = boundary_samples_origins[c].x + vec.x;
                // // p1.y = boundary_samples_origins[c].y + vec.y;
                // // p1.z = boundary_samples_origins[c].z + vec.z;
                // p1.x = vec.x;
                // p1.y = vec.y;
                // p1.z = vec.z;
                // // p1.x = (best_planes_normal[c].normal_x);
                // // p1.y = (best_planes_normal[c].normal_y);
                // // p1.z = (best_planes_normal[c].normal_z);
                // marker.points.push_back(p0);
                // marker.points.push_back(p1);
                // markerArray.markers.push_back(marker);

                // visualization_msgs::Marker marker2;
                // marker2.header.frame_id = "base_frame";
                // marker2.header.stamp = ros::Time::now();
                // marker2.action = visualization_msgs::Marker::ADD;
                // marker2.id = 1505 + 10*c + j;
                // marker2.type = visualization_msgs::Marker::ARROW;
                // marker2.scale.x = 0.003;
                // marker2.scale.y = 0.005;
                // marker2.color.r = 1.0f;
                // marker2.color.g = 1.0f;
                // marker2.color.a = 1.0;
                // geometry_msgs::Point p6;
                // p6.x = boundary_samples_origins[c].x;
                // p6.y = boundary_samples_origins[c].y;
                // p6.z = boundary_samples_origins[c].z;
                // // p0.x = 0;
                // // p0.y = 0;
                // // p0.z = 0;
                // geometry_msgs::Point p7;
                // p7.x = p_rand.x;
                // p7.y = p_rand.y;
                // p7.z = p_rand.z;
                // // p1.x = (best_planes_normal[c].normal_x);
                // // p1.y = (best_planes_normal[c].normal_y);
                // // p1.z = (best_planes_normal[c].normal_z);
                // marker2.points.push_back(p6);
                // marker2.points.push_back(p7);
                // markerArray.markers.push_back(marker2);

                suitable_area_points.push_back(p_rand);
                suitable_areas_points_indices.push_back(c);
            }
            else{
                j--;
                continue;
            }
            

            // geometry_msgs::Point p3;
            //     p3.x = p_rand.x;
            //     p3.y = p_rand.y;
            //     p3.z = p_rand.z;
            //     marker3.points.push_back(p3);
                // markerArray3.markers.push_back(marker3);

                // suitable_area_points.push_back(p_rand);
                // suitable_areas_points_indices.push_back(c);
            
        }
        // break;
        std::cout << "after sampling" << std::endl;
        
    }
    std::cout << "after all samplings" << std::endl;
    

    // Adding starting and ending point;
    Point drone_starting_position;
    drone_starting_position.x = uav_start_x;
    drone_starting_position.y = uav_start_y;
    drone_starting_position.z = uav_start_z;
    // Point drone_ending_position;
    // drone_ending_position.x = uav_end_x;
    // drone_ending_position.y = uav_end_y;
    // drone_ending_position.z = uav_end_z;
    suitable_area_points.insert(suitable_area_points.begin(), drone_starting_position);
    // suitable_area_points.push_back(drone_ending_position);
    std::cout << "GTSP matrix" << std::endl;
    // create GTSP matrix
    std::stringstream gtsp_matrix;
    for(int i = 0; i < suitable_area_points.size(); i++){
        // std::cout << "fst for" << std::endl;
        for(int j = 0; j < suitable_area_points.size(); j++){
            // std::cout << "snd for" << std::endl;
            if(i == j){
                gtsp_matrix << "1000000 ";
            }
            else{
                double dist = vect_norm(suitable_area_points[i], suitable_area_points[j]);
                double value = (std::round(dist * 1000.0) / 1000.0) * 1000;
                // gtsp_matrix << dist << "\t";
                gtsp_matrix << value << "\t";
            }
        }
        gtsp_matrix << "\n";
    }
    // std::cout << "2" << std::endl;
    std::vector<std::string> indices_in_clusters;
    // Adding starting position
    std::stringstream str_start;
    str_start << std::to_string(1) << " " << std::to_string(1);// << "-1\n";
    indices_in_clusters.push_back(str_start.str());
    // for(int i = 0; i < best_planes.size(); i++){
    // for(int i = 0; i < boundary_samples_origins.size(); i++){
    // std::cout << "boundary_samples_origins start" << std::endl;
    for(int i = 1; i < boundary_samples_origins.size() + 1; i++){
        // if(i == 0 || i == 1){
        //     std::stringstream str_str;
        //     int val = i + 1;
        //     str_str << std::to_string(val) << " " << std::to_string(100 + i) << " ";
        //     indices_in_clusters.push_back(str_str.str());
        //     std::cout << str_str.str() << std::endl;
        //     continue;
        // }
        std::stringstream str_str;
        int val = i + 1;
        // int val = i + 2;
        str_str << std::to_string(val) << " ";
        indices_in_clusters.push_back(str_str.str());
        // std::cout << str_str.str() << std::endl;
    }
    // std::cout << "3" << std::endl;
    // for(int i = 0; i < suitable_area_points.size(); i++){
        std::cout << "suitable_area_points start" << std::endl;
    for(int i = 1; i < suitable_area_points.size(); i++){   // -1 because of the ending point which is not in the indices vector
        // std::cout << "3.1" << std::endl;
        // int cluster_idx = suitable_areas_points_indices[i];
        int cluster_idx = suitable_areas_points_indices[i-1] + 1;
        // std::cout << "3.2" << std::endl;
        int val = i + 1;
        // std::cout << "3.3" << std::endl;
        // indices_in_clusters[cluster_idx] << std::to_string(val) << " ";
        std::string str_str = indices_in_clusters[cluster_idx] + std::to_string(val) + " ";
        // std::cout << "3.4" << std::endl;
        indices_in_clusters[cluster_idx] = str_str;
        // std::cout << "3.5" << std::endl;
        // indices_in_clusters[cluster_idx] =  indices_in_clusters[cluster_idx] + std::to_string(val) + " ";
        // std::cout << indices_in_clusters[cluster_idx] << std::endl;
    }
    std::cout << "4" << std::endl;
    // Adding end point
    // std::stringstream str_end;
    // str_end << std::to_string(boundary_samples_origins.size() + 2) << " " << std::to_string(suitable_area_points.size()) << " ";// << "-1\n";
    // indices_in_clusters.push_back(str_end.str());

    std::stringstream clusters;
    for(int i = 0; i < indices_in_clusters.size(); i++){
        // clusters << indices_in_clusters[i].str() << " -1\n";
        clusters << indices_in_clusters[i] << "-1\n";
        // std::cout << clusters.str() << std::endl;
    }

    std::cout << "5" << std::endl;
    std::string       problem_filename = "pcl_test";
    std::stringstream command, tour_path, problem_full_name;
    std::string       glkh_script_path      = "/home/honzuna/Stažené/GLKH-1.0";
    std::string       tsp_solution_filename = "pcl_test.tour";
    problem_full_name << glkh_script_path << "/" << problem_filename << ".gtsp";
    command << glkh_script_path << "/runGLKHrti " << glkh_script_path << "/" << problem_filename;
    tour_path << glkh_script_path << "/" << problem_filename << ".tour";
    std::stringstream instance_name;
    instance_name << std::to_string(boundary_samples.size()) << "pcl_test" << std::to_string(suitable_area_points.size()) << "\n";
    std::ofstream out(problem_full_name.str());
    if (out.is_open()) {
        out << "NAME: " << instance_name.str();//"NAME: tmp_instance\n";
        out << "TYPE: GTSP\n";
        out << "DIMENSION: " << std::to_string(suitable_area_points.size()) << "\n";
        // out << "GTSP_SETS: " << std::to_string(best_planes.size()) << "\n";
        out << "GTSP_SETS: " << std::to_string(boundary_samples.size()+1) << "\n";
        out << "EDGE_WEIGHT_TYPE: EXPLICIT\n";
        out << "EDGE_WEIGHT_FORMAT: FULL_MATRIX\n";
        out << "EDGE_WEIGHT_SECTION\n";
        // std::stringstream line;
        // for (size_t s = 0; s < path_lengths.size(); s++) {
        // line.str(std::string());
        // for (size_t g = 0; g < path_lengths.size(); g++) {
        //     line << std::to_string(static_cast<int>(1e2 * path_lengths[s][g])) << " ";
        // }
        out << gtsp_matrix.str();// << "\n";
        // }
        out << "GTSP_SET_SECTION:\n";
        // for (int k = 1; k < n_goals + 1; k++) {
        //     line.str(std::string());
        //     line << std::to_string(k) << " " << std::to_string(k);
        //     line << " -1\n";
        //     out << line.str();
        // }
        out << clusters.str();// << "\n";
        out << "EOF";
        out.close();
    } else {
        ROS_ERROR("[%s]: Unable to open file.", ros::this_node::getName().c_str());
    }
    std::cout << "command call" << std::endl;
    system(command.str().c_str());

    std::cout << "solution reading" << std::endl;
    std::vector<int> solution;
    std::ifstream         in(tour_path.str());
    if (in.is_open()) {
        std::string line;
        bool        tour_started = false;
        while (std::getline(in, line)) {
        if (line == "-1") {
            break;
        }
        if (tour_started) {
            solution.push_back(std::stoi(line) - 1);
        }
        if (line == "TOUR_SECTION") {
            tour_started = true;
        }
        }
        in.close();
    }
    std::cout << "solution reading ok" << std::endl;

    for(int i = 0; i < solution.size(); i++){
        int sol_idx = solution[i];
        std::cout << "sol: " << sol_idx << std::endl;
        Point p = suitable_area_points[sol_idx];
        // p.x = path_points[i].x;
        // p.y = path_points[i].y;
        // p.z = path_points[i].z;
        p.r = 255;
        p.g = 255;
        p.a = 255;
        (*path_cloud).push_back(p);
        // (*path_cloud).push_back((*areas_cloud)[sol_idx]);
        // markerArray3.markers[sol_idx].color.r = ((suitable_areas_points_indices[sol_idx] + 1.0) / solution.size() ) * 1.0f;
        // markerArray3.markers[sol_idx].color.g = 0.0f;
        // markerArray3.markers[sol_idx].color.b = ((suitable_areas_points_indices[sol_idx] + 1.0) / solution.size() ) * 1.0f;
    }
    
    
    
    // std::vector<Point> solution;
    // Point p1;
    // p1.x = 0.0;
    // p1.y = 3.0;
    // p1.z = 2.0;
    // Point p2;
    // p2.x = 3.0;
    // p2.y = 3.0;
    // p2.z = 2.0;
    // solution.push_back(p1);
    // solution.push_back(p2);
    
    std::cout << "Saving points" << std::endl;
    std::ofstream gtsp_solution("/home/honzuna/gtsp_solution");
    if (gtsp_solution.is_open()) {
        for(int i = 0; i < solution.size(); i++){
            // Point curr_point = (*boundary_cloud)[solution[i]];
            Point curr_point = suitable_area_points[solution[i]];
            std::cout << "position = (" << curr_point.x << ", " << curr_point.y << ", " << curr_point.z << "), heading = ??" << std::endl;
            gtsp_solution << curr_point.x << " " << curr_point.y << " " << curr_point.z << "\n";
        }
        // Adding ending point
        Point drone_ending_position;
        drone_ending_position.x = uav_end_x;
        drone_ending_position.y = uav_end_y;
        drone_ending_position.z = uav_end_z;
        gtsp_solution << drone_ending_position.x << " " << drone_ending_position.y << " " << drone_ending_position.z << "\n";
        gtsp_solution.close();
    } else {
        ROS_ERROR("[%s]: Unable to open file.", ros::this_node::getName().c_str());
    }
    std::cout << "Points saved" << std::endl;
    */
    
    std::cout << "Reading points" << std::endl;
    std::vector<Point> solution_points;
    std::ifstream gtsp_solution_in("/home/honzuna/gtsp_solution");
    if (gtsp_solution_in.is_open()) {
        std::string line;
        bool        tour_started = false;
        while (std::getline(gtsp_solution_in, line)) {
            if (line == "-1") {
                break;
            }
            std::cout << "line: " << line << std::endl;
            const char delim = ' '; 
            std::vector<std::string> out; 
            tokenize(line, delim, out);
            // std::stringstream   linestream(line);
            // std::string         data;
            // std::getline(linestream, data, ' ');
            // double x, y, z;
            // linestream >> x >> y >> z;
            Point p;
            p.x = std::stod(out[0]);
            p.y = std::stod(out[1]);
            p.z = std::stod(out[2]);
            solution_points.push_back(p);
        }
        gtsp_solution_in.close();
    }
    std::cout << "Points red" << std::endl;
    for(int i = 0; i < solution_points.size(); i++){
        Point curr_point = solution_points[i];
        std::cout << "position = (" << curr_point.x << ", " << curr_point.y << ", " << curr_point.z << "), heading = ??" << std::endl;
    }
    std::vector<Point> viewpoints = solution_points;
    /*
    std::cout << "Original points" << std::endl;
    // for(int i = 0; i < solution.size(); i++){
    //     Point curr_point = (*boundary_cloud)[solution[i]];
    //     std::cout << "position = (" << curr_point.x << ", " << curr_point.y << ", " << curr_point.z << "), heading = ??" << std::endl;
    // }

    ros::ServiceClient client = nh.serviceClient<mrs_msgs::GetPath>("/uav1/pathfinder/get_path");
    // mrs_msgs::GetPath srv;
    std::vector<Point> path_points;
    std::vector<double> path_headings;
    
    mrs_msgs::Reference ref_start;    
    mrs_msgs::Reference ref_end;
    ref_end.heading = uav_start_heading;
    // ref_end.position.x = uav_start_x;
    // ref_end.position.y = uav_start_y;
    // ref_end.position.z = uav_start_z;
    ref_end.position.x = solution_points[0].x;
    ref_end.position.y = solution_points[0].y;
    ref_end.position.z = solution_points[0].z;
    std::cout << "===== Calling planner ======" << std::endl;
    // for(int i = 0; i < solution.size(); i++){
    // for(int i = 0; i < solution_points.size(); i++){
    for(int i = 1; i < solution_points.size(); i++){
        mrs_msgs::GetPath srv;

        // Point curr_point = (*boundary_cloud)[solution[i]];
        // Point curr_point = suitable_area_points[solution[i]];
        // Point curr_point = solution[i];
        Point curr_point = solution_points[i];
        ref_start.heading = ref_end.heading;
        ref_start.position.x = ref_end.position.x;
        ref_start.position.y = ref_end.position.y;
        ref_start.position.z = ref_end.position.z;

        ref_end.heading = ref_end.heading; // find out how to deal with heading
        ref_end.position.x = curr_point.x;
        ref_end.position.y = curr_point.y;
        ref_end.position.z = curr_point.z;

        srv.request.start = ref_start;
        srv.request.goal = ref_end;
        client.waitForExistence();
        bool call = client.call(srv);

        for(int j = 0; j < srv.response.path.size(); j++){
            Point new_path_point;
            new_path_point.x = srv.response.path[j].position.x;
            new_path_point.y = srv.response.path[j].position.y;
            new_path_point.z = srv.response.path[j].position.z;
            double heading = srv.response.path[j].heading;
            path_points.push_back(new_path_point);
            path_headings.push_back(heading);
            std::cout << "position = (" << srv.response.path[j].position.x << ", " << srv.response.path[j].position.y << ", " << srv.response.path[j].position.z << "), heading = " << srv.response.path[j].heading << std::endl;
        } 
    }
    // mrs_msgs::GetPath srv;
    // ref_start.heading = ref_end.heading;
    // ref_start.position.x = ref_end.position.x;
    // ref_start.position.y = ref_end.position.y;
    // ref_start.position.z = ref_end.position.z;

    // ref_end.heading = uav_end_heading; // find out how to deal with heading
    // ref_end.position.x = uav_end_x;
    // ref_end.position.y = uav_end_y;
    // ref_end.position.z = uav_end_z;
    
    // srv.request.start = ref_start;
    // srv.request.goal = ref_end;
    // client.waitForExistence();
    // bool call = client.call(srv);

    // for(int j = 0; j < srv.response.path.size(); j++){
    //         Point new_path_point;
    //         new_path_point.x = srv.response.path[j].position.x;
    //         new_path_point.y = srv.response.path[j].position.y;
    //         new_path_point.z = srv.response.path[j].position.z;
    //         double heading = srv.response.path[j].heading;
    //         path_points.push_back(new_path_point);
    //         path_headings.push_back(heading);
    //         std::cout << "position = (" << srv.response.path[j].position.x << ", " << srv.response.path[j].position.y << ", " << srv.response.path[j].position.z << "), heading = " << srv.response.path[j].heading << std::endl;
    //     } 

    std::cout << "===== Final path =====" << std::endl;
    for(int i = 0; i < path_points.size(); i++){
        Point p;
        p.x = path_points[i].x;
        p.y = path_points[i].y;
        p.z = path_points[i].z;
        p.g = 255;
        p.a = 255;
        (*path_cloud).push_back(p);
        std::cout << "position = (" << path_points[i].x << ", " << path_points[i].y << ", " << path_points[i].z << "), heading = " << path_headings[i] << std::endl;
    }
    for(int i = 1; i < path_points.size(); i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.003;
        marker.scale.y = 0.005;
        marker.color.g = 1.0f;
        marker.color.a = 1.0;
        geometry_msgs::Point p0;
        p0.x = path_points[i-1].x;
        p0.y = path_points[i-1].y;
        p0.z = path_points[i-1].z;
        geometry_msgs::Point p1;
        p1.x = path_points[i].x;
        p1.y = path_points[i].y;
        p1.z = path_points[i].z;
        marker.points.push_back(p0);
        marker.points.push_back(p1);
        markerArrayPath.markers.push_back(marker);
        // marker_path.publish(markerArrayPath);
    }

    std::cout << "Saving path" << std::endl;
    std::ofstream path_founded("/home/honzuna/path_points");
    if (path_founded.is_open()) {
        for(int i = 0; i < path_points.size(); i++){
            // Point curr_point = (*boundary_cloud)[solution[i]];
            Point curr_point = path_points[i];
            std::cout << "position = (" << curr_point.x << ", " << curr_point.y << ", " << curr_point.z << "), heading = ??" << std::endl;
            path_founded << curr_point.x << " " << curr_point.y << " " << curr_point.z << "\n";
        }
        path_founded.close();
    } else {
        ROS_ERROR("[%s]: Unable to open file.", ros::this_node::getName().c_str());
    }
    std::cout << "Path saved" << std::endl;
    */

    // for(int i = 0; i < srv.response.path.size(); i++){
    //     std::cout << "position = (" << srv.response.path[i].position.x << ", " << srv.response.path[i].position.y << ", " << srv.response.path[i].position.z << "), heading = " << srv.response.path[i].heading << std::endl;
    // }

    // std::cout << "response name: " << client.getService() << std::endl;
    

    
    // std::vector<Point> path_points;
    // Point p1;
    // p1.x = 0.0;
    // p1.y = 0.0;
    // p1.z = 0.0;
    // Point p2;
    // p2.x = 10.0;
    // p2.y = 7.0;
    // p2.z = 3.0;
    // Point p3;
    // p2.x = 2.0;
    // p2.y = 4.0;
    // p2.z = 2.0;
    // path_points.push_back(p1);
    // path_points.push_back(p2);
    // path_points.push_back(p3);

    /*
    std::cout << "Reading path" << std::endl;
    std::vector<Point> path_solution;
    std::ifstream path_solution_in("/home/honzuna/gtsp_solution");
    if (path_solution_in.is_open()) {
        std::string line;
        bool        tour_started = false;
        while (std::getline(path_solution_in, line)) {
            if (line == "-1") {
                break;
            }
            std::cout << "line: " << line << std::endl;
            const char delim = ' '; 
            std::vector<std::string> out; 
            tokenize(line, delim, out);
            // std::stringstream   linestream(line);
            // std::string         data;
            // std::getline(linestream, data, ' ');
            // double x, y, z;
            // linestream >> x >> y >> z;
            Point p;
            p.x = std::stod(out[0]);
            p.y = std::stod(out[1]);
            p.z = std::stod(out[2]);
            path_solution.push_back(p);
        }
        path_solution_in.close();
    }
    std::cout << "Path red" << std::endl;

    
    ros::ServiceClient client_trajectory = nh.serviceClient<mrs_msgs::PathSrv>("/uav1/trajectory_generation/path");
    mrs_msgs::PathSrv srv_trajectory;
    mrs_msgs::Path path;
    std::vector<mrs_msgs::Reference> refs;
    for(int i = 0; i < path_solution.size(); i++){
        mrs_msgs::Reference ref;
        ref.position.x = path_solution[i].x;
        ref.position.y = path_solution[i].y;
        ref.position.z = path_solution[i].z;
        ref.heading = 0.0;
        refs.push_back(ref);
    }
    //std_msgs::Header header;
    //header.frame_id = "/uav1/gps_origin"; //"/uav1/trajectory_generation/path";
    //path.header = header;
    //path.input_id = 1;  // not compiled yet
    //path.use_heading = false;
    path.fly_now = true;
    // path.stop_at_waypoints = false;
    // path.loop = false;
    // path.points = refs;
    // path.override_constraints = true;
    // path.relax_heading = true;
    // path.override_max_velocity_horizontal = 10.0;
    // path.override_max_acceleration_horizontal = 10.0;
    // path.override_max_jerk_horizontal = 10.0;
    // path.override_max_velocity_vertical = 10.0;
    // path.override_max_acceleration_vertical = 10.0;
    // path.override_max_jerk_vertical = 10.0;
    path.points = refs;

    srv_trajectory.request.path = path;
    client_trajectory.waitForExistence();
    bool call_trajectory = client_trajectory.call(srv_trajectory);

    std::cout << "call: " << call_trajectory << std::endl;
    std::cout << "response success: " << srv_trajectory.response.success << std::endl;
    std::cout << "response message: " << srv_trajectory.response.message << std::endl;
    */

    
    // std::cout << "response frame_id: " << srv.response.frame_id << std::endl;
    // std::cout << "response path: " << srv.response.path.size() << std::endl;

    // for(int i = 0; i < srv.response.path.size(); i++){
    //     std::cout << "position = (" << srv.response.path[i].position.x << ", " << srv.response.path[i].position.y << ", " << srv.response.path[i].position.z << "), heading = " << srv.response.path[i].heading << std::endl;
    // }

    // for (auto& point: *final_cloud){        
    //     point.r = 0;
    //     point.g = 0;
    //     point.b = 255;
	// }

    // K = 11;
    // epsilon = 0.1;
    // pcl::PointCloud<pcl::Normal>::Ptr boundary_cloud_normals = get_normal_vectors(boundary_cloud, K, epsilon);
    // std::map<unsigned long int, std::vector<int>> boundary_cloud_neighbours = get_neighbours(boundary_cloud, K, epsilon);
    // std::map<unsigned long int, std::vector<std::pair<int, double>>> boundary_cloud_neighbours_distances = get_neighbours_distances(boundary_cloud, K, epsilon);
    // std::vector<double> boundary_cloud_average_distances = compute_average_distances(boundary_cloud_neighbours_distances);
    // std::vector<std::vector<int>> huh = get_boundary_points(boundary_cloud, K, epsilon, K/2.0, epsilon/2.0, k_angle, k_halfdisc, k_shape, trashold, weight_boundary);
    // std::vector<double> boundary_cloud_a = angle_criterion(boundary_cloud, K, epsilon, boundary_cloud_normals, boundary_cloud_neighbours);
    // std::vector<double> boundary_cloud_p = halfdisc_criterion(boundary_cloud, K, epsilon, boundary_cloud_normals, boundary_cloud_neighbours_distances, boundary_cloud_average_distances);
    // std::vector<double> boundary_cloud_s = shape_criterion(boundary_cloud, K, epsilon, boundary_cloud_normals, boundary_cloud_neighbours, boundary_cloud_neighbours_distances, boundary_cloud_average_distances);
   
    // std::cout << "coloration" << std::endl;
    // std::vector<std::vector<int>> huh = get_boundary_points(final_cloud, K, epsilon, K_2, epsilon_2, k_angle, k_halfdisc, k_shape, trashold, weight_boundary);
    // std::cout << "boundary size: " << huh.size() << std::endl;
    // for(auto vec : huh){
    //     std::cout << "vec size: " << vec.size() << std::endl;
    //     // std::cout << "vec size: " << vec.size() << " of " << boundary_p.size() << " and " << boundary_e.size() << ", " << boundary_points_start.size() << ", " << boundary_points_end.size() << std::endl;
    //     for(auto p : vec){
    //         // (*boundary_cloud)[p].r = 255;
    //         // (*boundary_cloud)[p].g = 255;
    //         // (*boundary_cloud)[p].b = 255;
    //         (*final_cloud)[p].r = 255;
    //         (*final_cloud)[p].g = 255;
    //         (*final_cloud)[p].b = 255;
    //     }
    // }

    // for(unsigned long int i = 0; i < (*boundary_cloud).size(); i++){
    //     double prob = k_halfdisc * boundary_cloud_p[i] + k_angle * boundary_cloud_a[i] + k_shape * boundary_cloud_s[i];
    //     // probabilities.push_back(prob);
    //     char boundary = prob > trashold ? 1 : 0;
    //     if(boundary == 1){
    //         (*boundary_cloud)[i].r = 255;
    //         (*boundary_cloud)[i].g = 255;
    //         (*boundary_cloud)[i].b = 255;
    //         // (*cloud)[i].r = 255;
    //         // (*cloud)[i].g = 255;
    //         // (*cloud)[i].b = 255;
    //         // (*cloud)[i].r = 0;
    //         // (*cloud)[i].g = 255;
    //         // (*cloud)[i].b = 0;
    //     }
    // }
    
    // for(auto best_plane : best_planes){
    //     Point pp1 = (*cloud)[best_plane[0]];
    //     pp1.r = 250;
    //     pp1.g = 180;
    //     pp1.b = 0;
    //     pp1.a = 255;
    //     Point pp2 = (*cloud)[best_plane[1]];
    //     pp1.r = 250;
    //     pp1.g = 180;
    //     pp1.b = 0;
    //     pp1.a = 255;
    //     Point pp3 = (*cloud)[best_plane[2]];
    //     pp1.r = 250;
    //     pp1.g = 180;
    //     pp1.b = 0;
    //     pp1.a = 255;
    //     (*boundary_cloud).push_back(pp1);
    //     (*boundary_cloud).push_back(pp2);
    //     (*boundary_cloud).push_back(pp3);
    // }
    
	
    // marker_pub.publish(markerArray);
    // marker_pub2.publish(markerArray2);
    // marker_pub3.publish(markerArray3);
    std::vector<bool> visited;
    for(int i = 0; i < viewpoints.size(); i++){
        visited.push_back(false);
    }
    ros::Subscriber drone_sub = nh.subscribe<nav_msgs::Odometry>("/uav1/odometry/odom_main", queue_length, boost::bind(callback, _1, pub_callback, uav_cloud));
    bool send = true;
    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::PointCloud2>("/uav1/os_cloud_nodelet/points", queue_length, boost::bind(callback_scan, _1, pub_callback_scan, uav_cloud, viewpoints, scan_cloud, visited));
    
    marker_path.publish(markerArrayPath);
    marker_pub.publish(markerArrayPlanesNormals);

    // for(auto &point : *scan_cloud){
    //     point.r = 255;
    //     point.g = 0;
    //     point.b = 0;
    // }
    
    std::cout << " <<< ===== DONE ===== >>>" << std::endl;
    ros::Rate loop_rate(4);
	// while (nh.ok())
    while (ros::ok())
	{
		// pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
		// pub.publish (*msg);
		// cloud->header.frame_id = "base_frame";
        // cloud_normals->header.frame_id = "base_frame";

		//colored_cloud->header.frame_id = "base_frame";
        
        // boundary_cloud->header.frame_id = "base_frame";
        
        boundary_cloud->header.frame_id = frame_id;
        areas_cloud->header.frame_id = frame_id;
        path_cloud->header.frame_id = frame_id;
        boundary_projections->header.frame_id = frame_id;

		pub.publish (*cloud);
        // pub.publish (*new_cloud);

        // pub.publish (*final_cloud);
        pub2.publish (*boundary_cloud);
        pub_areas.publish (*areas_cloud);
        pub_path.publish (*path_cloud);

        pub3.publish (*boundary_projections);
        pub_mis_data.publish(scan_to_send);

//         pc_pub.publish(*cloud_normals);
        
        // marker_pub.publish(markerArray);
        // marker_pub2.publish(markerArray2);
        // marker_pub3.publish(markerArray3);

		//pub.publish (*colored_cloud);
		//pub.publish (*cloud_blob);
		
        for(auto &point : *scan_cloud){
            point.r = 255;
            point.g = 0;
            point.b = 0;
            point.a = 255;
        }
        // pub_final_data.publish(*scan_cloud);
        // pub_callback_scan.publish(*scan_cloud);
        std::cout << scan_cloud->width << " x " << scan_cloud->height << std::endl;
        bool end = true;
        for(int i = 0; i < visited.size(); i++){
            std::cout << "not visited[i]: " << (!visited[i]) << std::endl;
            if(!visited[i]){
                end = false;
                break;
            }
        }
        if(end){
            break;
        }
        
        ros::spinOnce ();
		loop_rate.sleep ();
    }

    std::cout << "Filtering obtained data" << std::endl;
    pcl::PCLPointCloud2::Ptr cloud_filtered_2 (new pcl::PCLPointCloud2 ());
    // std::cout << "22 :)" << std::endl;
    pcl::PCLPointCloud2::Ptr pcl_pc2_2 (new pcl::PCLPointCloud2 ());
    // std::cout << "33 :)" << std::endl;
    pcl_pc2_2->width = scan_cloud->width;
    // std::cout << "44 :)" << std::endl;
    pcl_pc2_2->height = scan_cloud->height;
    // std::cout << "55 :)" << std::endl;
    pcl::toPCLPointCloud2((*scan_cloud), (*pcl_pc2_2));
    // std::cout << "22 :)" << std::endl;
    std::cerr << "PointCloud before filtering: " << scan_cloud->width * scan_cloud->height 
        << " data points (" << pcl::getFieldsList (*scan_cloud) << ")." << std::endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor_2;
    sor_2.setInputCloud (pcl_pc2_2);
    //   sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor_2.setLeafSize (0.1f, 0.1f, 0.1f);
    sor_2.filter (*cloud_filtered_2);

    std::cerr << "PointCloud after filtering: " << cloud_filtered_2->width * cloud_filtered_2->height 
        << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

    (*scan_cloud).clear();
    (*scan_cloud).width = pcl_pc2_2->width;
    (*scan_cloud).height = pcl_pc2_2->height;
    pcl::fromPCLPointCloud2((*cloud_filtered_2), (*scan_cloud));

    for(auto &point : *scan_cloud){
            point.r = 255;
            point.g = 0;
            point.b = 0;
            point.a = 255;
        }
    // pub_final_data.publish(*scan_cloud);
    pub_callback_scan.publish(*scan_cloud);
    pub2.publish (*scan_cloud);

    if(listen){
        pcl::io::savePCDFileASCII ("scan.pcd", *scan_cloud);
    }

    std::cout << "end" << std::endl;

	return 0;
}
