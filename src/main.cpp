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
    ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>("bound", queue_length);
    ros::Publisher pub3 = nh.advertise<sensor_msgs::PointCloud2>("proj", queue_length);
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

    double uav_min_distance;
    double uav_max_distance;
    double uav_min_angle;
    double uav_min_area_width;
    pl.loadParam(node_name + "/uav_min_distance", uav_min_distance);
    pl.loadParam(node_name + "/uav_max_distance", uav_max_distance);
    pl.loadParam(node_name + "/uav_min_angle", uav_min_angle);
    pl.loadParam(node_name + "/uav_min_area_width", uav_min_area_width);

    uav_min_angle *= (2 * M_PI) / 360.0 ;

    srand(time(NULL));
    
    std::cout << "uav_min_angle: " << uav_min_angle << std::endl;

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

    (*cloud).clear();
    (*cloud).width = pcl_pc2->width;
    (*cloud).height = pcl_pc2->height;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2((*cloud_filtered), (*cloud));






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

    for(auto current_boundary : boundaries){
        std::vector<int> best_plane;
        Point best_plane_origin;
        pcl::Normal best_plane_normal;
        // int error = 100;
        int maximum_points = 0;
        int minimum_distance = 1000;
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
            p.x /= 3;
            p.y /= 3;
            p.z /= 3;
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
            plane_normal.normal_x = 0;
            plane_normal.normal_y = 0;
            plane_normal.normal_z = 0;
            for(int j = 0; j < n_ransac; j++){
                plane_normal.normal_x += normals->points[random_indexes[j]].normal_x;
                plane_normal.normal_y += normals->points[random_indexes[j]].normal_y;
                plane_normal.normal_z += normals->points[random_indexes[j]].normal_z;
            }
            plane_normal.normal_x /= 3;
            plane_normal.normal_y /= 3;
            plane_normal.normal_z /= 3;

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
            // if(true){
            //     int curr_points = points_below_trashold.size();
            //     if(sum_of_distances < minimum_distance){
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

                    // minimum_distance = sum_of_distances;
                    maximum_points = curr_points;
                    best_plane = random_indexes;
                    best_plane_origin = p;
                    best_plane_normal = plane_normal;
                    // best_plane_origin = best_fit_origin;
                    // best_plane_normal = best_fit_normal;
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

    // std::vector<std::vector<int>> bounds;
    std::vector<std::vector<Point>> bounds;
    std::vector<std::vector<int>> bounds_indices;
    std::vector<std::vector<Point>> rectangle_vectors;

    PointCloud boundary_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloud boundary_projections (new pcl::PointCloud<pcl::PointXYZRGB>);
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
        for(auto idx : b){
            (*final_cloud)[idx].r = 0;
            (*final_cloud)[idx].g = 255;
            (*final_cloud)[idx].b = 0;
        }
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

    double sample_criterion = 2 * ( sqrt( uav_max_distance*uav_max_distance - uav_min_distance*uav_min_distance ) - (uav_min_area_width / 2) );
    std::cout << "sample criterion: " << sample_criterion << std::endl;
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

        visualization_msgs::Marker marker3;
        marker3.header.frame_id = "base_frame";
        marker3.header.stamp = ros::Time::now();
        marker3.action = visualization_msgs::Marker::ADD;
        marker3.id = c;
        marker3.type = visualization_msgs::Marker::POINTS;
        marker3.scale.x = 0.01;
        marker3.scale.y = 0.01;
        marker3.scale.z = 0.01;
        marker3.color.r = 0.0;
        marker3.color.g = 255.0;
        marker3.color.b = 255.0;
        marker3.color.a = 1.0;

        geometry_msgs::Point p2;
        p2.x = p_right.x;
        p2.y = p_right.y;
        p2.z = p_right.z;
        marker3.points.push_back(p2);
        markerArray3.markers.push_back(marker3);

        visualization_msgs::Marker marker4;
        marker4.header.frame_id = "base_frame";
        marker4.header.stamp = ros::Time::now();
        marker4.action = visualization_msgs::Marker::ADD;
        marker4.id = c+10;
        marker4.type = visualization_msgs::Marker::POINTS;
        marker4.scale.x = 0.01;
        marker4.scale.y = 0.01;
        marker4.scale.z = 0.01;
        marker3.color.r = 0.0;
        marker3.color.g = 255.0;
        marker3.color.b = 255.0;
        marker4.color.a = 1.0;

        geometry_msgs::Point p3;
        p3.x = p_top.x;
        p3.y = p_top.y;
        p3.z = p_top.z;
        marker4.points.push_back(p3);
        markerArray3.markers.push_back(marker4);

        // Point vector_left_to_right = get_vector(p_left, p_right);

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
        
        // Point rectangle_origin;
        // rectangle_origin.x = p_left.x + vector_left_to_right.x * (y / dist_left_to_right);
        // rectangle_origin.y = p_left.y + vector_left_to_right.y * (y / dist_left_to_right);
        // rectangle_origin.z = p_left.z + vector_left_to_right.z * (y / dist_left_to_right);
        // Point vec_bottom_origin = get_vector(p_bottom, rectangle_origin);

        Point rectangle_origin;
        rectangle_origin.x = (p_left.x + p_bottom.x + p_right.x + p_top.x) / 4.0;
        rectangle_origin.y = (p_left.y + p_bottom.y + p_right.y + p_top.y) / 4.0;
        rectangle_origin.z = (p_left.z + p_bottom.z + p_right.z + p_top.z) / 4.0;
        
        Point vec_left_origin = get_vector(p_left, rectangle_origin);
        Point vec_bottom_origin = get_vector(p_bottom, rectangle_origin);
        // Point new_top;
        // new_top.x = p_top.x + vector_left_to_right.x * (diff_y_x / dist_left_to_right);
        // new_top.y = p_top.y + vector_left_to_right.y * (diff_y_x / dist_left_to_right);
        // new_top.z = p_top.z + vector_left_to_right.z * (diff_y_x / dist_left_to_right);

        // Point vector_bottom_to_top = get_vector(p_bottom, new_top);

        for(double sample_height_start = 0.0; sample_height_start < dist_bottom_to_top; sample_height_start += sample_criterion){
            std::cout << "sample_height_start: " << sample_height_start << " / dist_bottom_to_top: " << dist_bottom_to_top << std::endl;
            for(double sample_width_start = 0.0; sample_width_start < dist_left_to_right; sample_width_start += sample_criterion){
                std::cout << "sample_width_start: " << sample_width_start << " / dist_left_to_right: " << dist_left_to_right << std::endl;

                int sample_max = 5;
                int r = 100 + rand() % 155;
                int g = 100 + rand() % 155;
                int b = 100 + rand() % 155;
                // int r = 255;
                // int g = 255;
                // int b = 0;

                Point h_bottom;
                h_bottom.x = p_bottom.x + vector_bottom_to_top.x * ( sample_height_start / dist_bottom_to_top);
                h_bottom.y = p_bottom.y + vector_bottom_to_top.y * ( sample_height_start / dist_bottom_to_top);
                h_bottom.z = p_bottom.z + vector_bottom_to_top.z * ( sample_height_start / dist_bottom_to_top);

                Point h_top;
                h_top.x = p_bottom.x + vector_bottom_to_top.x * ( ( sample_height_start + sample_criterion ) / dist_bottom_to_top);
                h_top.y = p_bottom.y + vector_bottom_to_top.y * ( ( sample_height_start + sample_criterion ) / dist_bottom_to_top);
                h_top.z = p_bottom.z + vector_bottom_to_top.z * ( ( sample_height_start + sample_criterion ) / dist_bottom_to_top);

                Point h = get_vector(h_bottom, h_top);
                // std::cout << "h = (" << h.x << ", " << h.y << ", " << h.z << ")" << std::endl;

                Point w_left;
                w_left.x = p_left.x + vector_left_to_right.x * ( sample_width_start / dist_left_to_right);
                w_left.y = p_left.y + vector_left_to_right.y * ( sample_width_start / dist_left_to_right);
                w_left.z = p_left.z + vector_left_to_right.z * ( sample_width_start / dist_left_to_right);

                Point w_right;
                w_right.x = p_left.x + vector_left_to_right.x * ( ( sample_width_start + sample_criterion ) / dist_left_to_right);
                w_right.y = p_left.y + vector_left_to_right.y * ( ( sample_width_start + sample_criterion ) / dist_left_to_right);
                w_right.z = p_left.z + vector_left_to_right.z * ( ( sample_width_start + sample_criterion ) / dist_left_to_right);

                Point w = get_vector(w_left, w_right);
                // std::cout << "w = (" << w.x << ", " << w.y << ", " << w.z << ")" << std::endl;

                std::vector<Point> samples;

                for(int i = 0; i < sample_max; i++){
                    double m = static_cast <double> (sample_max) ;
                    double coef =  i / m;

                    // Point curr_left;
                    // curr_left.x = p_left + 
                    
                    

                    Point left_sample;
                    // left_sample.x = p_left.x + ( vector_left_to_right.x * ( sample_height_start / dist_left_to_right ) );
                    left_sample.x = p_left.x + ( vector_left_to_right.x * ( sample_width_start / dist_left_to_right ) );
                    left_sample.y = p_left.y + ( vector_left_to_right.y * ( sample_width_start / dist_left_to_right ) );
                    left_sample.z = p_left.z + ( vector_left_to_right.z * ( sample_width_start / dist_left_to_right ) );

                    // left_sample.x = left_sample.x + ( vector_bottom_to_top.x * ( ( coef ) ) ) - vec_bottom_origin.x;
                    
                    left_sample.x = left_sample.x + ( ( sample_height_start / sample_criterion) * h.x + h.x * ( ( coef ) ) ) - vec_bottom_origin.x;
                    left_sample.y = left_sample.y + ( ( sample_height_start / sample_criterion) * h.y + h.y * ( ( coef ) ) ) - vec_bottom_origin.y;
                    left_sample.z = left_sample.z + ( ( sample_height_start / sample_criterion) * h.z + h.z * ( ( coef ) ) ) - vec_bottom_origin.z;

                    left_sample.r = r;
                    left_sample.g = g;
                    left_sample.b = b;
                    left_sample.a = 255;

                    // std::cout << "left_sample = (" << left_sample.x << ", " << left_sample.y << ", " << left_sample.z << ")" << std::endl;
                    (*boundary_cloud).push_back(left_sample);
                    
                    Point right_sample;
                    // right_sample.x = p_left.x + ( vector_left_to_right.x * ( ( sample_height_start + sample_criterion ) / dist_left_to_right ) );
                    right_sample.x = p_left.x + ( vector_left_to_right.x * ( ( sample_width_start + sample_criterion ) / dist_left_to_right ) );
                    right_sample.y = p_left.y + ( vector_left_to_right.y * ( ( sample_width_start + sample_criterion ) / dist_left_to_right ) );
                    right_sample.z = p_left.z + ( vector_left_to_right.z * ( ( sample_width_start + sample_criterion ) / dist_left_to_right ) );

                    right_sample.x = right_sample.x + ( ( sample_height_start / sample_criterion) * h.x + h.x * ( ( coef ) ) ) - vec_bottom_origin.x;
                    right_sample.y = right_sample.y + ( ( sample_height_start / sample_criterion) * h.y + h.y * ( ( coef ) ) ) - vec_bottom_origin.y;
                    right_sample.z = right_sample.z + ( ( sample_height_start / sample_criterion) * h.z + h.z * ( ( coef ) ) ) - vec_bottom_origin.z;

                    right_sample.r = r;
                    right_sample.g = g;
                    right_sample.b = b;
                    right_sample.a = 255;

                    (*boundary_cloud).push_back(right_sample);

                    Point bottom_sample;

                    bottom_sample.x = p_bottom.x + ( vector_bottom_to_top.x * ( sample_height_start / dist_bottom_to_top ) );
                    bottom_sample.y = p_bottom.y + ( vector_bottom_to_top.y * ( sample_height_start / dist_bottom_to_top ) );
                    bottom_sample.z = p_bottom.z + ( vector_bottom_to_top.z * ( sample_height_start / dist_bottom_to_top ) );

                    bottom_sample.x = bottom_sample.x + ( ( sample_width_start / sample_criterion) * w.x + w.x * ( ( coef ) ) ) - vec_left_origin.x;
                    bottom_sample.y = bottom_sample.y + ( ( sample_width_start / sample_criterion) * w.y + w.y * ( ( coef ) ) ) - vec_left_origin.y;
                    bottom_sample.z = bottom_sample.z + ( ( sample_width_start / sample_criterion) * w.z + w.z * ( ( coef ) ) ) - vec_left_origin.z;

                    bottom_sample.r = r;
                    bottom_sample.g = g;
                    bottom_sample.b = b;
                    bottom_sample.a = 255;

                    (*boundary_cloud).push_back(bottom_sample);

                    Point top_sample;

                    top_sample.x = p_bottom.x + ( vector_bottom_to_top.x * ( ( sample_height_start + sample_criterion ) / dist_bottom_to_top ) );
                    top_sample.y = p_bottom.y + ( vector_bottom_to_top.y * ( ( sample_height_start + sample_criterion ) / dist_bottom_to_top ) );
                    top_sample.z = p_bottom.z + ( vector_bottom_to_top.z * ( ( sample_height_start + sample_criterion ) / dist_bottom_to_top ) );

                    top_sample.x = top_sample.x + ( ( sample_width_start / sample_criterion) * w.x + w.x * ( ( coef ) ) ) - vec_left_origin.x;
                    top_sample.y = top_sample.y + ( ( sample_width_start / sample_criterion) * w.y + w.y * ( ( coef ) ) ) - vec_left_origin.y;
                    top_sample.z = top_sample.z + ( ( sample_width_start / sample_criterion) * w.z + w.z * ( ( coef ) ) ) - vec_left_origin.z;

                    top_sample.r = r;
                    top_sample.g = g;
                    top_sample.b = b;
                    top_sample.a = 255;

                    (*boundary_cloud).push_back(top_sample);

                    samples.push_back(left_sample);
                    samples.push_back(right_sample);
                    samples.push_back(bottom_sample);
                    samples.push_back(top_sample);

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
                Point origin;
                origin.x = 0;
                origin.y = 0;
                origin.z = 0;
                for(int i = 0; i < samples.size(); i++){
                    origin.x += samples[i].x;
                    origin.y += samples[i].y;
                    origin.z += samples[i].z;
                }
                origin.x /= samples.size();
                origin.y /= samples.size();
                origin.z /= samples.size();

                boundary_samples.push_back(samples);
                boundary_samples_origins.push_back(origin);
                boundary_samples_indices.push_back(c);
                // break;
            }
            // break;
        }

        std::cout << "after while" << std::endl;
    }

    std::cout << "sampling" << std::endl;
    for(int c = 0; c < boundary_samples.size(); c++){
        int boundary_idx = boundary_samples_indices[c];

        // visualization_msgs::Marker marker;
        // marker.header.frame_id = "base_frame";
        // marker.header.stamp = ros::Time::now();
        // marker.action = visualization_msgs::Marker::ADD;
        // marker.id = c;
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
        // marker2.color.b = 1.0f;
        // marker2.color.a = 1.0;
        // geometry_msgs::Point p2;
        // p2.x = boundary_samples_origins[c].x;
        // p2.y = boundary_samples_origins[c].y;
        // p2.z = boundary_samples_origins[c].z;
        // marker2.points.push_back(p2);
        // markerArray2.markers.push_back(marker2);

        // continue;

        for(int j = 0; j < 10; j++){
            double min = -(sample_criterion / 2.0);
            double max = sample_criterion / 2.0;
            // double x = min + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(max - min)));
            double x = min + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(max - min)));
            double y = min + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(max - min)));
            double z = min + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(max - min)));
            Point p_rand;
            p_rand.x = boundary_samples_origins[c].x + x;
            p_rand.y = boundary_samples_origins[c].y + y;
            p_rand.z = boundary_samples_origins[c].z + z;

            // remove wrong side of tangent plane
            bool cont = false;
            for(int idx = 0; idx < boundary_samples[c].size(); idx++){
                Point zero;
                zero.x = 0.0;
                zero.y = 0.0;
                zero.z = 0.0;
                Point normal;
                normal.x = best_planes_normal[boundary_idx].normal_x;
                normal.y = best_planes_normal[boundary_idx].normal_y;
                normal.z = best_planes_normal[boundary_idx].normal_z;
                Point vec = get_vector(boundary_samples_origins[c], p_rand);
                double angle = acos( (vec.x*normal.x + vec.y*normal.y + vec.z*normal.z) / (vect_norm(normal, zero) * vect_norm(boundary_samples_origins[c], vec)) );
                // std::cout << "angle: " << angle << std::endl;
                if(fabs(angle) > M_PI / 2){
                    cont = true;
                    break;
                }
            }
            if(cont){
                j--;
                continue;
            }

            visualization_msgs::Marker marker3;
            marker3.header.frame_id = "base_frame";
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

                if(min_dist < uav_min_distance || max_dist > uav_max_distance){
                    ok_d = false;
                    // std::cout << "min: " << min_dist << ", max: " << max_dist << std::endl;
                    // j--;
                    break;
                }
            }
            if(ok_d){
                marker3.color.r = 1.0f;
                marker3.color.g = 0.0f;
                marker3.color.b = 0.0f;
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
                marker3.scale.x = 0.03;
                marker3.scale.y = 0.03;
                marker3.scale.z = 0.03;
                marker3.color.r = 1.0f;
                marker3.color.g = 1.0f;
                marker3.color.b = 1.0f;
                geometry_msgs::Point p3;
                p3.x = p_rand.x;
                p3.y = p_rand.y;
                p3.z = p_rand.z;
                marker3.points.push_back(p3);
                markerArray3.markers.push_back(marker3);

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
            //     markerArray3.markers.push_back(marker3);

                // suitable_area_points.push_back(p_rand);
                // suitable_areas_points_indices.push_back(c);
        }
        // break;
        std::cout << "after all" << std::endl;
    }




    std::cout << "GTSP matrix" << std::endl;
    // create GTSP matrix
    std::stringstream gtsp_matrix;
    for(int i = 0; i < suitable_area_points.size(); i++){
        for(int j = 0; j < suitable_area_points.size(); j++){
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
    // for(int i = 0; i < best_planes.size(); i++){
    for(int i = 0; i < boundary_samples_origins.size(); i++){
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
        str_str << std::to_string(val) << " ";
        indices_in_clusters.push_back(str_str.str());
        // std::cout << str_str.str() << std::endl;
    }
    // std::cout << "3" << std::endl;
    for(int i = 0; i < suitable_area_points.size(); i++){
        // std::cout << "3.1" << std::endl;
        int cluster_idx = suitable_areas_points_indices[i];
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
    // std::cout << "4" << std::endl;
    std::stringstream clusters;
    for(int i = 0; i < indices_in_clusters.size(); i++){
        // clusters << indices_in_clusters[i].str() << " -1\n";
        clusters << indices_in_clusters[i] << "-1\n";
        // std::cout << clusters.str() << std::endl;
    }
    // std::cout << "5" << std::endl;
    std::string       problem_filename = "pcl_test";
    std::stringstream command, tour_path, problem_full_name;
    std::string       glkh_script_path      = "/home/honzuna/Stažené/GLKH-1.0";
    std::string       tsp_solution_filename = "pcl_test.tour";
    problem_full_name << glkh_script_path << "/" << problem_filename << ".gtsp";
    command << glkh_script_path << "/runGLKHrti " << glkh_script_path << "/" << problem_filename;
    tour_path << glkh_script_path << "/" << problem_filename << ".tour";
    std::stringstream instance_name;
    instance_name << std::to_string(best_planes.size()) << "pcl_test" << std::to_string(suitable_area_points.size()) << "\n";
    std::ofstream out(problem_full_name.str());
    if (out.is_open()) {
        out << "NAME: " << instance_name.str();//"NAME: tmp_instance\n";
        out << "TYPE: GTSP\n";
        out << "DIMENSION: " << std::to_string(suitable_area_points.size()) << "\n";
        // out << "GTSP_SETS: " << std::to_string(best_planes.size()) << "\n";
        out << "GTSP_SETS: " << std::to_string(boundary_samples.size()) << "\n";
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
    system(command.str().c_str());

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

    for(int i = 0; i < solution.size(); i++){
        int sol_idx = solution[i];
        std::cout << "sol: " << sol_idx << std::endl;
        markerArray3.markers[sol_idx].color.r = ((suitable_areas_points_indices[sol_idx] + 1.0) / solution.size() ) * 1.0f;
        markerArray3.markers[sol_idx].color.g = 0.0f;
        markerArray3.markers[sol_idx].color.b = ((suitable_areas_points_indices[sol_idx] + 1.0) / solution.size() ) * 1.0f;
    }





    // for (auto& point: *final_cloud){        
    //     point.r = 0;
    //     point.g = 0;
    //     point.b = 255;
	// }

    K = 11;
    epsilon = 0.1;
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
    

    
	
    marker_pub.publish(markerArray);
    marker_pub2.publish(markerArray2);
    marker_pub3.publish(markerArray3);

    std::cout << " <<< ===== DONE ===== >>>" << std::endl;
    ros::Rate loop_rate(4);
	while (nh.ok())
	{
		// pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
		// pub.publish (*msg);
		cloud->header.frame_id = "base_frame";
        // cloud_normals->header.frame_id = "base_frame";

		//colored_cloud->header.frame_id = "base_frame";
        boundary_cloud->header.frame_id = "base_frame";

        boundary_projections->header.frame_id = "base_frame";

		// pub.publish (*cloud);
        // pub.publish (*new_cloud);

        pub.publish (*final_cloud);

        pub2.publish (*boundary_cloud);

        pub3.publish (*boundary_projections);
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
