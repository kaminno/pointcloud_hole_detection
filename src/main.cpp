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

    double uav_min_distance;
    double uav_max_distance;
    double uav_min_angle;
    double uav_min_area_width;
    pl.loadParam(node_name + "/uav_min_distance", uav_min_distance);
    pl.loadParam(node_name + "/uav_max_distance", uav_max_distance);
    pl.loadParam(node_name + "/uav_min_angle", uav_min_angle);
    pl.loadParam(node_name + "/uav_min_area_width", uav_min_area_width);

    uav_min_angle *= (2 * M_PI) / 360.0 ;
    
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

    std::vector<std::vector<int>> bounds;
    PointCloud boundary_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // boundary_cloud.resize
    for(int i = 0; i < boundaries.size(); i++){
        for(int j = 0; j < boundaries[i].size(); j++){
            // (*final_cloud)[boundaries[i][j]].r = 0;
            // (*final_cloud)[boundaries[i][j]].g = 0;
            // (*final_cloud)[boundaries[i][j]].b = 255;
            boundary_cloud->push_back((*final_cloud)[boundaries[i][j]]);
        }
    }

    for(int i = 0; i < best_planes.size(); i++){
        std::vector<int> b;
        double min_x = 0;
        double min_y = 0;
        double max_x = 0;
        double max_y = 0;

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
        bounds.push_back(b);
    }

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
        double dist = vect_norm((*final_cloud)[bounds[c][0]], (*final_cloud)[bounds[c][2]]);
        int s_ = 0;
        // double s = 0;
        double s = sample_criterion;
        Point p_left = tangent_projection(best_planes_normal[c], best_planes_origin[c], (*final_cloud)[bounds[c][0]]);
        Point p_bottom = tangent_projection(best_planes_normal[c], best_planes_origin[c], (*final_cloud)[bounds[c][1]]);
        Point p_right = tangent_projection(best_planes_normal[c], best_planes_origin[c], (*final_cloud)[bounds[c][2]]);
        Point p_top = tangent_projection(best_planes_normal[c], best_planes_origin[c], (*final_cloud)[bounds[c][3]]);

        // Point p = get_vector((*final_cloud)[bounds[c][0]], (*final_cloud)[bounds[c][2]]);
        Point p = get_vector(p_left, p_right);

        if(vect_norm(p_left, p_right) < sample_criterion && vect_norm(p_bottom, p_top) < sample_criterion){
            // continue;
            s = dist;
        }

        // Point p_ = get_vector((*final_cloud)[bounds[c][1]], (*final_cloud)[bounds[c][3]]);
        double s_a = vect_norm(p_right, p_top);
        double s_b = vect_norm(p_left, p_top);
        double s_c = vect_norm(p_right, p_left);
        double v_c = sin( acos( ( s_c * s_c + s_a * s_a - s_b * s_b ) / ( 2 * s_a * s_c ) ) ) * s_a;
        double x = sqrt( s_b * s_b - v_c * v_c );
        s_a = vect_norm(p_right, p_bottom);
        s_b = vect_norm(p_left, p_bottom);
        v_c = sin( acos( ( s_c * s_c + s_a * s_a - s_b * s_b ) / ( 2 * s_a * s_c ) ) ) * s_a;
        double y = sqrt( s_b * s_b - v_c * v_c );
        double diff = y - x;
        
        Point square_origin;
        // square_origin.x = p_left.x + p.x * (diff / dist);
        // square_origin.y = p_left.y + p.y * (diff / dist);
        // square_origin.z = p_left.z + p.z * (diff / dist);
        square_origin.x = p_left.x + p.x * (y / dist);
        square_origin.y = p_left.y + p.y * (y / dist);
        square_origin.z = p_left.z + p.z * (y / dist);
        Point small_vec = get_vector(p_bottom, square_origin);
        Point new_top;
        new_top.x = p_top.x + p.x * (diff / dist);
        new_top.y = p_top.y + p.y * (diff / dist);
        new_top.z = p_top.z + p.z * (diff / dist);
        // Point p_ = get_vector(p_bottom, p_top);
        Point p_ = get_vector(p_bottom, new_top);
        std::cout << "s " << s << ", dist " << dist << std::endl;
        while(s < dist){
            // s = s_ + sample_criterion;
            // s_++;
            // s += sample_criterion;
            int sample_max = 50;
            
            for(int i = 0; i < sample_max; i++){
                double m = static_cast <double> (sample_max) ;
                double coef =  i / m;
                // add random translation in the other direction on the plane (now just one point on the line)
                // Point p_left = tangent_projection(best_planes_normal[c], best_planes_origin[c], (*final_cloud)[bounds[c][0]]);
                // Point p_bottom = tangent_projection(best_planes_normal[c], best_planes_origin[c], (*final_cloud)[bounds[c][1]]);
                // Point p_right = tangent_projection(best_planes_normal[c], best_planes_origin[c], (*final_cloud)[bounds[c][2]]);
                // Point p_top = tangent_projection(best_planes_normal[c], best_planes_origin[c], (*final_cloud)[bounds[c][3]]);

                // // Point p = get_vector((*final_cloud)[bounds[c][0]], (*final_cloud)[bounds[c][2]]);
                // Point p = get_vector(p_left, p_right);
                
                Point new_p;
                // new_p.x = (*final_cloud)[bounds[c][0]].x + ( p.x * ( sample_criterion / dist ) );
                // new_p.y = (*final_cloud)[bounds[c][0]].y + ( p.y * ( sample_criterion / dist ) );
                // new_p.z = (*final_cloud)[bounds[c][0]].z + ( p.z * ( sample_criterion / dist ) );
                // new_p.x = (*final_cloud)[bounds[c][0]].x + ( p.x * ( s / dist ) );
                // new_p.y = (*final_cloud)[bounds[c][0]].y + ( p.y * ( s / dist ) );
                // new_p.z = (*final_cloud)[bounds[c][0]].z + ( p.z * ( s / dist ) );
                new_p.x = p_left.x + ( p.x * ( s / dist ) );
                new_p.y = p_left.y + ( p.y * ( s / dist ) );
                new_p.z = p_left.z + ( p.z * ( s / dist ) );

                // // Point p_ = get_vector((*final_cloud)[bounds[c][1]], (*final_cloud)[bounds[c][3]]);
                // double s_a = vect_norm(p_right, p_top);
                // double s_b = vect_norm(p_left, p_top);
                // double s_c = vect_norm(p_right, p_left);
                // double v_c = sin( acos( ( s_c * s_c + s_a * s_a - s_b * s_b ) / ( 2 * s_a * s_c ) ) ) * s_a;
                // double x = sqrt( s_b * s_b - v_c * v_c );
                // s_a = vect_norm(p_right, p_bottom);
                // s_b = vect_norm(p_left, p_bottom);
                // v_c = sin( acos( ( s_c * s_c + s_a * s_a - s_b * s_b ) / ( 2 * s_a * s_c ) ) ) * s_a;
                // double y = sqrt( s_b * s_b - v_c * v_c );
                // double diff = y - x;
                
                // Point square_origin;
                // // square_origin.x = p_left.x + p.x * (diff / dist);
                // // square_origin.y = p_left.y + p.y * (diff / dist);
                // // square_origin.z = p_left.z + p.z * (diff / dist);
                // square_origin.x = p_left.x + p.x * (y / dist);
                // square_origin.y = p_left.y + p.y * (y / dist);
                // square_origin.z = p_left.z + p.z * (y / dist);
                // Point small_vec = get_vector(p_bottom, square_origin);
                // Point new_top;
                // new_top.x = p_top.x + p.x * (diff / dist);
                // new_top.y = p_top.y + p.y * (diff / dist);
                // new_top.z = p_top.z + p.z * (diff / dist);
                // // Point p_ = get_vector(p_bottom, p_top);
                // Point p_ = get_vector(p_bottom, new_top);
                // dist = vect_norm((*final_cloud)[bounds[c][1]], (*final_cloud)[bounds[c][3]]);
                // int random_num = rand() % 100;
                // new_p.x = new_p.x + ( p_.x * ( ( random_num / 100 ) * dist ) );
                // new_p.y = new_p.y + ( p_.y * ( ( random_num / 100 ) * dist ) );
                // new_p.z = new_p.z + ( p_.z * ( ( random_num / 100 ) * dist ) );
                // new_p.x = new_p.x + ( p_.x * ( ( random_num / 100.0 ) ) );
                // new_p.y = new_p.y + ( p_.y * ( ( random_num / 100.0 ) ) );
                // new_p.z = new_p.z + ( p_.z * ( ( random_num / 100.0 ) ) );
                // new_p.x = new_p.x + ( p_.x * ( ( coef ) ) );
                // new_p.y = new_p.y + ( p_.y * ( ( coef ) ) );
                // new_p.z = new_p.z + ( p_.z * ( ( coef ) ) );
                new_p.x = new_p.x + ( p_.x * ( ( coef ) ) ) - small_vec.x;
                new_p.y = new_p.y + ( p_.y * ( ( coef ) ) ) - small_vec.y;
                new_p.z = new_p.z + ( p_.z * ( ( coef ) ) ) - small_vec.z;

                visualization_msgs::Marker marker3;
                marker3.header.frame_id = "base_frame";
                marker3.header.stamp = ros::Time::now();
                marker3.action = visualization_msgs::Marker::ADD;
                marker3.id = c+1000 + 100*s + i;
                marker3.type = visualization_msgs::Marker::POINTS;
                marker3.scale.x = 0.005;
                marker3.scale.y = 0.005;
                marker3.scale.z = 0.005;
                marker3.color.r = 1.0f;
                marker3.color.g = 1.0f;
                marker3.color.a = 1.0;

                geometry_msgs::Point p2;
                // p2.x = new_p.x;
                // p2.y = new_p.y;
                // p2.z = new_p.z;
                p2.x = p_top.x;
                p2.y = p_top.y;
                p2.z = p_top.z;
                marker3.points.push_back(p2);
                // markerArray3.markers.push_back(marker3);
                new_p.r = 255;
                new_p.g = 255;
                new_p.a = 255;
                (*final_cloud).push_back(new_p);
                // (*boundary_cloud).push_back(new_p);
            }
            s += sample_criterion;
        }



        // ================================================

         dist = vect_norm((*final_cloud)[bounds[c][1]], new_top);
         s_ = 0;
        // double s = 0;
         s = sample_criterion;
         p_left = tangent_projection(best_planes_normal[c], best_planes_origin[c], new_top);
         p_bottom = tangent_projection(best_planes_normal[c], best_planes_origin[c], (*final_cloud)[bounds[c][0]]);
        //  p_right = tangent_projection(best_planes_normal[c], best_planes_origin[c], (*final_cloud)[bounds[c][3]]);
        p_right = tangent_projection(best_planes_normal[c], best_planes_origin[c], (*final_cloud)[bounds[c][1]]);
         p_top = tangent_projection(best_planes_normal[c], best_planes_origin[c], (*final_cloud)[bounds[c][2]]);

        // Point p = get_vector((*final_cloud)[bounds[c][0]], (*final_cloud)[bounds[c][2]]);
         p = get_vector(p_left, p_right);

        if(vect_norm(p_left, p_right) < sample_criterion && vect_norm(p_bottom, p_top) < sample_criterion){
            // continue;
            s = dist;
        }

        // Point p_ = get_vector((*final_cloud)[bounds[c][1]], (*final_cloud)[bounds[c][3]]);
         s_a = vect_norm(p_right, p_top);
         s_b = vect_norm(p_left, p_top);
         s_c = vect_norm(p_right, p_left);
         v_c = sin( acos( ( s_c * s_c + s_a * s_a - s_b * s_b ) / ( 2 * s_a * s_c ) ) ) * s_a;
         x = sqrt( s_b * s_b - v_c * v_c );
        s_a = vect_norm(p_right, p_bottom);
        s_b = vect_norm(p_left, p_bottom);
        v_c = sin( acos( ( s_c * s_c + s_a * s_a - s_b * s_b ) / ( 2 * s_a * s_c ) ) ) * s_a;
         y = sqrt( s_b * s_b - v_c * v_c );
         diff = y - x;
        
        //  square_origin;
        // square_origin.x = p_left.x + p.x * (diff / dist);
        // square_origin.y = p_left.y + p.y * (diff / dist);
        // square_origin.z = p_left.z + p.z * (diff / dist);
        // square_origin.x = p_left.x + p.x * (y / dist);
        // square_origin.y = p_left.y + p.y * (y / dist);
        // square_origin.z = p_left.z + p.z * (y / dist);
        new_top = p_top;
         small_vec = get_vector(p_bottom, square_origin);
        //  new_top;
        // new_top.x = p_top.x + p.x * (diff / dist);
        // new_top.y = p_top.y + p.y * (diff / dist);
        // new_top.z = p_top.z + p.z * (diff / dist);
        // Point p_ = get_vector(p_bottom, p_top);
         p_ = get_vector(p_bottom, new_top);
        std::cout << "s " << s << ", dist " << dist << std::endl;
        while(s < dist){
            int sample_max = 50;
            
            for(int i = 0; i < sample_max; i++){
                double m = static_cast <double> (sample_max) ;
                double coef =  i / m;
                Point new_p;
                new_p.x = p_left.x + ( p.x * ( s / dist ) );
                new_p.y = p_left.y + ( p.y * ( s / dist ) );
                new_p.z = p_left.z + ( p.z * ( s / dist ) );

                new_p.x = new_p.x + ( p_.x * ( ( coef ) ) ) - small_vec.x;
                new_p.y = new_p.y + ( p_.y * ( ( coef ) ) ) - small_vec.y;
                new_p.z = new_p.z + ( p_.z * ( ( coef ) ) ) - small_vec.z;

                visualization_msgs::Marker marker3;
                marker3.header.frame_id = "base_frame";
                marker3.header.stamp = ros::Time::now();
                marker3.action = visualization_msgs::Marker::ADD;
                marker3.id = c+1000 + 100*s + i;
                marker3.type = visualization_msgs::Marker::POINTS;
                marker3.scale.x = 0.005;
                marker3.scale.y = 0.005;
                marker3.scale.z = 0.005;
                marker3.color.r = 1.0f;
                marker3.color.g = 1.0f;
                marker3.color.a = 1.0;

                geometry_msgs::Point p2;
                // p2.x = new_p.x;
                // p2.y = new_p.y;
                // p2.z = new_p.z;
                p2.x = p_top.x;
                p2.y = p_top.y;
                p2.z = p_top.z;
                marker3.points.push_back(p2);
                // markerArray3.markers.push_back(marker3);
                new_p.r = 255;
                new_p.g = 255;
                new_p.a = 255;
                (*final_cloud).push_back(new_p);
                // (*boundary_cloud).push_back(new_p);
            }
            s += sample_criterion;
        }

        std::cout << "after while" << std::endl;

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
        // p0.x = 0;
        // p0.y = 0;
        // p0.z = 0;
        geometry_msgs::Point p1;
        p1.x = (best_planes_origin[c].x + best_planes_normal[c].normal_x);
        p1.y = (best_planes_origin[c].y + best_planes_normal[c].normal_y);
        p1.z = (best_planes_origin[c].z + best_planes_normal[c].normal_z);
        // p1.x = (best_planes_normal[c].normal_x);
        // p1.y = (best_planes_normal[c].normal_y);
        // p1.z = (best_planes_normal[c].normal_z);
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

        // continue;

        for(int j = 0; j < 3000; j++){
            // std::cout << "c: " << c << std::endl;
            // double x = (rand() % 100) / 1000.0;
            // double y = (rand() % 100) / 1000.0;
            // double z = (rand() % 100) / 1000.0;
            double min = -0.3;
            double max = 0.3;
            // double x = min + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(max - min)));
            double x = min + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(max - min)));
            double y = min + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(max - min)));
            double z = min + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(max - min)));
            Point p_rand;
            p_rand.x = best_planes_origin[c].x + x;
            p_rand.y = best_planes_origin[c].y + y;
            p_rand.z = best_planes_origin[c].z;// + z;

            // p_rand = tangent_projection(best_planes_normal[c], best_planes_origin[c], p_rand);

            // if(p_rand.x < (*final_cloud)[bounds[c][0]].x || p_rand.y < (*final_cloud)[bounds[c][1]].y ||
            //         p_rand.x > (*final_cloud)[bounds[c][2]].x || p_rand.y > (*final_cloud)[bounds[c][3]].y){
            //             j--;
            //             continue;
            // }


            // p_rand.z = 0;
            // Point p_proj = tangent_projection(best_planes_normal[c], best_planes_origin[c], p_rand);
            // double dist = vect_norm();
            // std::cout << "p ( " << p_rand.x << ", " << p_rand.y << ", " << p_rand.z << " )" << std::endl;

            // remove wrong side of tangent plane
            bool cont = false;
            for(int idx = 0; idx < boundaries[c].size(); idx++){
                Point zero;
                zero.x = 0.0;
                zero.y = 0.0;
                zero.z = 0.0;
                Point normal;
                normal.x = best_planes_normal[c].normal_x;
                normal.y = best_planes_normal[c].normal_y;
                normal.z = best_planes_normal[c].normal_z;
                Point vec = get_vector(best_planes_origin[c], p_rand);
                double angle = acos( (vec.x*normal.x + vec.y*normal.y + vec.z*normal.z) / (vect_norm(normal, zero) * vect_norm(best_planes_origin[c], vec)) );
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
            for(int idx = 0; idx < boundaries[c].size(); idx++){
                int point_idx = boundaries[c][idx];
                Point point = (*final_cloud)[point_idx];
                Point projection = tangent_projection(best_planes_normal[c], best_planes_origin[c], p_rand);
                double max_dist = vect_norm(point, p_rand);
                double min_dist = vect_norm(projection, p_rand);
                if(min_dist < uav_min_distance || max_dist > uav_max_distance){
                    ok_d = false;
                    // j--;
                    break;
                }
            }
            if(ok_d){
                marker3.color.r = 1.0f;
                marker3.color.g = 0.0f;
                marker3.color.b = 0.0f;
                // geometry_msgs::Point p3;
                // p3.x = p_rand.x;
                // p3.y = p_rand.y;
                // p3.z = p_rand.z;
                // marker3.points.push_back(p3);
                // markerArray3.markers.push_back(marker3);
            }

            // angles
            bool ok_a = true;
            // bool ok_a = false;
            for(int idx = 0; idx < boundaries[c].size(); idx++){
                int point_idx = boundaries[c][idx];
                Point boundary_point = (*final_cloud)[point_idx];
                Point projection = tangent_projection(best_planes_normal[c], best_planes_origin[c], p_rand);
                Point v_1 = get_vector(p_rand, projection);
                Point v_2 = get_vector(p_rand, boundary_point);
                double angle = (M_PI / 2) - acos( (v_1.x*v_2.x + v_1.y*v_2.y + v_1.z*v_2.z) / (vect_norm(boundary_point, p_rand) * vect_norm(projection, p_rand)) );
                if(angle < uav_min_angle){
                    ok_a = false;
                    // j--;
                    break;
                }
            }
            if(ok_d && ok_a){
                marker3.color.r = 1.0f;
                marker3.color.g = 1.0f;
                marker3.color.b = 1.0f;
                // geometry_msgs::Point p3;
                // p3.x = p_rand.x;
                // p3.y = p_rand.y;
                // p3.z = p_rand.z;
                // marker3.points.push_back(p3);
                // markerArray3.markers.push_back(marker3);
            }
            // else{
            //     j--;
            //     continue;
            // }
            
            geometry_msgs::Point p3;
            p3.x = p_rand.x;
            p3.y = p_rand.y;
            p3.z = p_rand.z;
            marker3.points.push_back(p3);
            markerArray3.markers.push_back(marker3);
        }
        std::cout << "after all" << std::endl;
    }


    for (auto& point: *final_cloud){        
        point.r = 0;
        point.g = 0;
        point.b = 255;
	}

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
    std::cout << "coloration" << std::endl;
    std::vector<std::vector<int>> huh = get_boundary_points(final_cloud, K, epsilon, K_2, epsilon_2, k_angle, k_halfdisc, k_shape, trashold, weight_boundary);
    std::cout << "boundary size: " << huh.size() << std::endl;
    for(auto vec : huh){
        std::cout << "vec size: " << vec.size() << std::endl;
        // std::cout << "vec size: " << vec.size() << " of " << boundary_p.size() << " and " << boundary_e.size() << ", " << boundary_points_start.size() << ", " << boundary_points_end.size() << std::endl;
        for(auto p : vec){
            // (*boundary_cloud)[p].r = 255;
            // (*boundary_cloud)[p].g = 255;
            // (*boundary_cloud)[p].b = 255;
            (*final_cloud)[p].r = 255;
            (*final_cloud)[p].g = 255;
            (*final_cloud)[p].b = 255;
        }
    }
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
        boundary_cloud->header.frame_id = "base_frame";

		// pub.publish (*cloud);
        // pub.publish (*new_cloud);
        pub.publish (*final_cloud);
        // pub.publish (*boundary_cloud);
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
