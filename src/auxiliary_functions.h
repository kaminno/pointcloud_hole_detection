#ifndef AUXILIARY_FUNCTIONS_H
#define AUXILIARY_FUNCTIONS_H

// standard libraries
#include <iostream>
#include <vector>
#include <math.h>
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

typedef pcl::PointXYZRGB Point;
// typedef pcl::Normal Normal;
typedef pcl::PointCloud<Point>::Ptr PointCloud;

std::map<unsigned long int, std::vector<int>> get_neighbours(PointCloud cloud, unsigned int K, double epsilon){
    std::cout << "get_neighbours" << std::endl;
    pcl::KdTreeFLANN<Point> kdtree;
    kdtree.setInputCloud (cloud);
    std::map<unsigned long int, std::vector<int>> graph;
    auto start = std::chrono::high_resolution_clock::now();
    unsigned long int num = 0;
    // std::cout << "cloud size " << cloud->width * cloud->height << std::endl;

    for (unsigned long int i = 0; i < (*cloud).size(); i++){
        std::vector< int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        kdtree.radiusSearch((*cloud)[i], epsilon, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        graph[i].insert(graph[i].end(), pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());

        std::vector<int> pointIdxKNNSearch(K);
        std::vector<float> pointKNNSquaredDistance(K);
        kdtree.nearestKSearch ((*cloud)[i], K, pointIdxKNNSearch, pointKNNSquaredDistance);
        graph[i].insert(graph[i].end(), pointIdxKNNSearch.begin(), pointIdxKNNSearch.end());

        int l1 = graph[i].size();
        graph[i].erase(std::remove(graph[i].begin(), graph[i].end(), i), graph[i].end());
        int l2 = graph[i].size();
        if(l1 == l2){
            std::cout << "same lengths :/" << std::endl;
        }
        
        for(int j = 0; j < pointIdxRadiusSearch.size(); j++){
            graph[pointIdxRadiusSearch[j]].push_back(i);
        }

        for(int j = 0; j < pointIdxKNNSearch.size(); j++){
            graph[pointIdxKNNSearch[j]].push_back(i);
        }
	}
	for (unsigned long int i = 0; i < (*cloud).size(); i++){
        std::sort(graph[i].begin(), graph[i].end());
        graph[i].erase(unique(graph[i].begin(), graph[i].end()), graph[i].end());
        num += graph[i].size();
    }
	auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = end - start;
    std::cout << "num " << num << std::endl;
    std::cout << "slept for " << fp_ms.count() << " milliseconds." << std::endl;
    std::cout << "get_neighbours done" << std::endl;
    // int q_i = 60;
    // for(int i = q_i; i < q_i + 30; i++){
    //     std::cout << "graph " << i << " size " << graph[i].size() << std::endl;
    // }
    return graph;
}

pcl::PointCloud<pcl::Normal>::Ptr get_normal_vectors(PointCloud cloud, unsigned int K, double epsilon){
    std::cout << "get_normal_vectors" << std::endl;
    pcl::NormalEstimation<Point, pcl::Normal> ne;
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    auto start = std::chrono::high_resolution_clock::now();

    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree);
    // ne.setKSearch(5*K);
    ne.setKSearch(20);
    // ne.setRadiusSearch(5*epsilon);
    ne.compute (*cloud_normals);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = end - start;
    std::cout << "slept for " << fp_ms.count() << " milliseconds." << std::endl;
    std::cout << "get_normal_vectors done" << std::endl;

    return cloud_normals;
}


Point tangent_projection(pcl::Normal normal_vector, Point plane_point, Point point){
    double numerator = (normal_vector.normal_x*plane_point.x - normal_vector.normal_x*point.x + normal_vector.normal_y*plane_point.y - normal_vector.normal_y*point.y + normal_vector.normal_z*plane_point.z - normal_vector.normal_z*point.z);
    double denominator = (normal_vector.normal_x*normal_vector.normal_x + normal_vector.normal_y*normal_vector.normal_y + normal_vector.normal_z*normal_vector.normal_z);
    double t = numerator / denominator;
    Point p;
    p.x = point.x + normal_vector.normal_x*t;
    p.y = point.y + normal_vector.normal_y*t;
    p.z = point.z + normal_vector.normal_z*t;

    return p;
}

std::map<unsigned long int, std::vector<std::pair<int, double>>> get_neighbours_distances(PointCloud cloud, unsigned int K, double epsilon){
    std::cout << "get_neighbours_distances" << std::endl;
    pcl::KdTreeFLANN<Point> kdtree;
    kdtree.setInputCloud (cloud);
    std::map<unsigned long int, std::vector<std::pair<int, double>>> graph;
    auto start = std::chrono::high_resolution_clock::now();

    // std::cout << "cloud size " << cloud->width * cloud->height << std::endl;

    for (unsigned long int i = 0; i < (*cloud).size(); i++){
        std::vector< int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        kdtree.radiusSearch((*cloud)[i], epsilon, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        if(pointIdxRadiusSearch.size() != pointRadiusSquaredDistance.size()){
            std::cout << "ERROR LENGTH RADIUS" << std::endl;
        }
        for(int j = 0; j < pointIdxRadiusSearch.size(); j++){
            // graph[i].push_back(std::pair<int, double>{pointIdxRadiusSearch[j], pointRadiusSquaredDistance[j]});
            graph[i].push_back(std::pair<int, double>{pointIdxRadiusSearch[j], sqrt(pointRadiusSquaredDistance[j])});
        }
        // (graph[i].begin()->first).insert(graph[i].begin()->first.end(), pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
        // (graph[i].begin()->second).insert(graph[i].begin()->second.end(), pointRadiusSquaredDistance.begin(), pointRadiusSquaredDistance.end());

        std::vector<int> pointIdxKNNSearch(K);
        std::vector<float> pointKNNSquaredDistance(K);
        kdtree.nearestKSearch ((*cloud)[i], K, pointIdxKNNSearch, pointKNNSquaredDistance);
        if(pointIdxKNNSearch.size() != pointKNNSquaredDistance.size()){
            std::cout << "ERROR LENGTH K" << std::endl;
        }
        for(int j = 0; j < pointIdxKNNSearch.size(); j++){
            // graph[i].push_back(std::pair<int, double>{pointIdxKNNSearch[j], pointKNNSquaredDistance[j]});
            graph[i].push_back(std::pair<int, double>{pointIdxKNNSearch[j], sqrt(pointKNNSquaredDistance[j])});
        }
        // graph[i].insert(graph[i].end(), pointIdxKNNSearch.begin(), pointIdxKNNSearch.end());
        // (graph[i].begin()->first).insert(graph[i].begin()->first.end(), pointIdxKNNSearch.begin(), pointIdxKNNSearch.end());
        // (graph[i].begin()->second).insert(graph[i].begin()->second.end(), pointKNNSquaredDistance.begin(), pointKNNSquaredDistance.end());

        int l1 = graph[i].size();
        std::vector<int> idxs;
        for(int j = 0; j < graph[i].size(); j++){
            if(graph[i][j].first == i){
                idxs.push_back(j);
            }
        }
        for(int j = 0; j < idxs.size(); j++){
            graph[i].erase(graph[i].begin() + idxs[j] - j);
        }
        int l2 = graph[i].size();
        if(l1 == l2){
            std::cout << "WRONG REMOVE OF POINT P" << std::endl;
        }

        for(int j = 0; j < pointIdxRadiusSearch.size(); j++){
            // graph[pointIdxRadiusSearch[j]].push_back(i);
            // graph[pointIdxRadiusSearch[j]].push_back(std::pair<int, double>{i, pointRadiusSquaredDistance[j]});
            graph[pointIdxRadiusSearch[j]].push_back(std::pair<int, double>{i, sqrt(pointRadiusSquaredDistance[j])});

        }

        for(int j = 0; j < pointIdxKNNSearch.size(); j++){
            // graph[pointIdxKNNSearch[j]].push_back(i);
            // graph[pointIdxKNNSearch[j]].push_back(std::pair<int, double>{i, pointKNNSquaredDistance[j]});
            graph[pointIdxKNNSearch[j]].push_back(std::pair<int, double>{i, sqrt(pointKNNSquaredDistance[j])});
        }
	}
    unsigned long int num = 0;
	for (unsigned long int i = 0; i < (*cloud).size(); i++){
        std::sort(graph[i].begin(), graph[i].end());
        graph[i].erase(unique(graph[i].begin(), graph[i].end()), graph[i].end());
        num += graph[i].size();
    }
    std::cout << "num: " << num << std::endl;
	auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = end - start;
    std::cout << "slept for " << fp_ms.count() << " milliseconds." << std::endl;
    std::cout << "get_neighbours_distances done" << std::endl;
    // int q_i = 60;
    // for(int i = q_i; i < q_i + 30; i++){
    //     std::cout << "graph " << i << " size " << graph[i].size() << std::endl;
    // }
    return graph;
}

double vect_norm(Point p1, Point p2){
    double x_ = p1.x - p2.x;
    double y_ = p1.y - p2.y;
    double z_ = p1.z - p2.z;
    double norm = sqrt(x_*x_ + y_*y_ + z_*z_);

    return norm;
}

std::vector<double> compute_average_distances(std::map<unsigned long int, std::vector<std::pair<int, double>>> neighbours_and_distances){
    std::cout << "compute_average_distances" << std::endl;
    std::vector<double> average_distances;

    auto start = std::chrono::high_resolution_clock::now();
    // std::cout << "map size: " << neighbours_and_distances.size() << std::endl;
    for(unsigned long int i = 0; i < neighbours_and_distances.size(); i++){
        // std::cout << "first cycle" << std::endl;
        double avg_dist = 0;
        // std::cout << "vector " << i <<" size: " << neighbours_and_distances[i].size() << std::endl;
        for(int j = 0; j < neighbours_and_distances[i].size(); j++){
            // std::cout << "inner cycle" << std::endl;
            avg_dist += neighbours_and_distances[i][j].second;
        }
        // std::cout << "division" << std::endl;
        // avg_dist /= (neighbours_and_distances[i].size() - 1);
        avg_dist /= neighbours_and_distances[i].size();
        // std::cout << "pushing" << std::endl;
        average_distances.push_back(avg_dist);
        // std::cout << "====================" << std::endl;
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = end - start;
    std::cout << "slept for " << fp_ms.count() << " milliseconds." << std::endl;
    std::cout << "compute_average_distances done" << std::endl;

    return average_distances;
}

std::vector<Point> compute_mi(PointCloud cloud, std::map<unsigned long int, std::vector<std::pair<int, double>>> neighbours, std::vector<double> average_distances, pcl::PointCloud<pcl::Normal>::Ptr normal_vectors){
    std::cout << "compute_mi" << std::endl;

    std::vector<Point> mis;

    auto start = std::chrono::high_resolution_clock::now();
    for(unsigned long int i = 0; i < (*cloud).size(); i++){
        Point mi_p;
        Point numerator;
        numerator.x = 0;
        numerator.y = 0;
        numerator.z = 0;
        double denominator = 0;
        Point p = (*cloud)[i];
        double sigma = average_distances[i] / 3;
        // std::cout << "sigma " << sigma << std::endl;
        for(int j = 0; j < neighbours[i].size(); j++){
            // std::cout << "first for " << j << ": " << neighbours[i][j].first << std::endl;
            Point q = (*cloud)[neighbours[i][j].first];
            double norm = vect_norm(q, p);
            // std::cout << "norm " << norm << std::endl;
            double g = exp(-(norm*norm)/(sigma*sigma));
            // double g = exp(-(norm*norm)/(sigma));
            // std::cout << "g " << g << std::endl;

            // numerator.x += (*cloud)[neighbours[i][j].first].x * g;
            // numerator.y += (*cloud)[neighbours[i][j].first].y * g;
            // numerator.z += (*cloud)[neighbours[i][j].first].z * g;
            numerator.x += (q.x * g);
            numerator.y += (q.y * g);
            numerator.z += (q.z * g);
            // numerator.x += q.x;
            // numerator.y += q.y;
            // numerator.z += q.z;
            // Point p = tangent_projection(normal_vectors->points[i], (*cloud)[i], (*cloud)[neighbours[i][j].first]);
            // double norm = vect_norm(p, (*cloud)[i]);
            // double sigma = average_distances[i] / 3;
            // double g = exp(-(norm*norm)/(sigma*sigma));
            // numerator.x += p.x * g;
            // numerator.y += p.y * g;
            // numerator.z += p.z * g;
            denominator += g;
            // denominator += 1;
        }
        mi_p.x = numerator.x / denominator;
        mi_p.y = numerator.y / denominator;
        mi_p.z = numerator.z / denominator;
        // std::cout << "mi " << i << " (" << mi_p.x << ", " << mi_p.y << ", " << mi_p.z << ")" << std::endl;
        // std::cout << "point " << i << " (" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
        // std::cout << "------------------------" << std::endl;
        mis.push_back(mi_p);
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = end - start;
    std::cout << "slept for " << fp_ms.count() << " milliseconds." << std::endl;
    std::cout << "compute_mi done" << std::endl;
    return mis;
}


#endif