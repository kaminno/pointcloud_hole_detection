// standard libraries
#include <iostream>
#include <vector>
// #include <math.h>
#include <cmath>
#include <map>
#include <utility>

// ROS libraries
// #include <pcl_ros/point_cloud.h>

// pcl library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

#include "auxiliary_functions.h"

// typedef pcl::PointXYZRGB Point;
// typedef pcl::Normal Normal;
// typedef pcl::PointCloud<Point>::Ptr PointCloud;
// typedef pcl::PointCloud<Normal>::Ptr PointCloudNormal;

std::vector<Point> project_mis(std::vector<Point> mis, pcl::PointCloud<pcl::Normal>::Ptr normal_vectors, PointCloud plane_points){
    std::cout << "project_mis" << std::endl;
    std::vector<Point> projected_mis;
    
    auto start = std::chrono::high_resolution_clock::now();
    for(unsigned long int i = 0; i < (*plane_points).size(); i++){
        Point p = tangent_projection(normal_vectors->points[i], (*plane_points)[i], mis[i]);
        projected_mis.push_back(p);
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = end - start;
    std::cout << "slept for " << fp_ms.count() << " milliseconds." << std::endl;
    std::cout << "project_mis done" << std::endl;

    return projected_mis;
}

std::vector<double> get_probabilities(PointCloud cloud, std::vector<Point> mi_prime, std::vector<double> distances){
    std::cout << "get_probabilities" << std::endl;
    std::vector<double> probs;
    auto start = std::chrono::high_resolution_clock::now();
    for(unsigned long int i = 0; i < (*cloud).size(); i++){
        // double x_ = (*cloud)[i].x - mi_prime[i].x;
        // double y_ = (*cloud)[i].y - mi_prime[i].y;
        // double z_ = (*cloud)[i].z - mi_prime[i].z;
        // double norm = sqrt(x_*x_ + y_*y_ + z_*z_);
        double norm = vect_norm((*cloud)[i], mi_prime[i]);
        double not_prob = norm / ((4*distances[i]) / (3*M_PI));
        if(not_prob > 0.4){
            std::cout << not_prob << std::endl;
        }
        double prob = fmin(not_prob, 1);
        probs.push_back(prob);
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = end - start;
    std::cout << "slept for " << fp_ms.count() << " milliseconds." << std::endl;
    std::cout << "get_probabilities done" << std::endl;
    return probs;
}

// return vector of probabilities that point on idx i is a boundary point
std::vector<double> halfdisc_criterion(PointCloud cloud, unsigned int K, double epsilon){
    std::cout << "========== HALFDISC CRITERION ==========" << std::endl;
    // unsigned int K = 20;
    // double epsilon = 0.1;
    // compute normals
    pcl::PointCloud<pcl::Normal>::Ptr normals = get_normal_vectors(cloud, K, epsilon);
    // compute neighbours
    // std::map<unsigned long int, std::vector<int>> graph = get_neighbours(cloud, K, epsilon);
    // std::map<unsigned long int, std::vector<int>> graph = get_neighbours_distances(cloud, K, epsilon);
    std::map<unsigned long int, std::vector<std::pair<int, double>>> graph = get_neighbours_distances(cloud, K, epsilon);

    std::vector<double> average_distances = compute_average_distances(graph);
    // std::vector<double> angle_gaps = get_angle_gaps(cloud, normals, graph);
    // for(unsigned long int i = 0; i < (*cloud).size(); i++){
    //     double gap = get_angle_gaps();
    //     angle_gaps.push_back(gap);
    // }

    // std::vector<Point> mi_p = compute_mi(cloud, graph, average_distances);
    std::vector<Point> mi_p = compute_mi(cloud, graph, average_distances, normals);

    std::vector<Point> mi_p_prime = project_mis(mi_p, normals, cloud);

    //std::cout << "criterion angle " << angle_gaps[77] << std::endl;

    std::vector<double> probabilities = get_probabilities(cloud, mi_p_prime, average_distances);

    std::cout << "========== END ==========" << std::endl;

    return probabilities;
}