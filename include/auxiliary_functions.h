#ifndef AUXILIARY_FUNCTIONS_H
#define AUXILIARY_FUNCTIONS_H

// standard libraries
#include <iostream>
#include <vector>
#include <math.h>
#include <map>
#include <utility>
#include <chrono>

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

class Edge{
public:
    int from = -2;
    int to = -2;
    double weight = 0.0;
    void print(){
        std::cout << "(" << from << ", " << to << ", " << weight << ")" << std::endl;
    }
};

int get_component_idx(std::vector<std::vector<int>> components_points, int point){
    // std::cout << "len: " << components_points.size() << std::endl;
	for(int i = 0; i < components_points.size(); i++){
            // std::cout << "\tin" << std::endl;
        	std::vector<int> vec = components_points[i];
            // std::cout << "\tvec length" << vec.size() << std::endl;
        	for(int j = 0; j < vec.size(); j++){
                // std::cout << vec[j] << " -> " << point << std::endl;
            	if(vec[j] == point){
                	return i;
                }
            }
        }
    return -1;
};

int get_cycle_length(std::vector<Edge> component_edges, int start, int end){
	std::vector<Edge> visited_edges;
    std::queue<int> q;
    q.push(start);
    while(!q.empty()){
    	int point = q.front();
        if(point == end){
        	break;
        }
        q.pop();
    	for(int i = 0; i < component_edges.size(); i++){
        	Edge edge = component_edges[i];
        	//if(std::find_if(component_edges.begin(), component_edges.end(), [](auto e){ return (e.from == point || e.to == point); }) != component_edges.end()){
            if(edge.from == point || edge.to == point){
//            	q.push(node);
                Edge e;
                e.from = point;
                e.to = edge.from == point ? edge.to : edge.from;
                q.push(e.to);
                visited_edges.push_back(e);
            }
        }
    }
    
    int length = 0;
    int i = 0;
    int s = end;
    bool f = true;
    while(f && i < 1000){
    	for(Edge e : visited_edges){
        	if(e.to == s){
            	length++;
                s = e.from;
                break;
            }
            if(s == start){
            	f = false;
                break;
            }
        }
        
        i++;
    }
    
	return length;
}

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
    ne.setKSearch(K);
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

Point get_vector(Point p1, Point p2){
    Point p;
    p.x = p2.x - p1.x;
    p.y = p2.y - p1.y;
    p.z = p2.z - p1.z;

    return p;
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
            // numerator.x += (q.x * g);
            // numerator.y += (q.y * g);
            // numerator.z += (q.z * g);
            numerator.x += q.x;
            numerator.y += q.y;
            numerator.z += q.z;
            // Point p = tangent_projection(normal_vectors->points[i], (*cloud)[i], (*cloud)[neighbours[i][j].first]);
            // double norm = vect_norm(p, (*cloud)[i]);
            // double sigma = average_distances[i] / 3;
            // double g = exp(-(norm*norm)/(sigma*sigma));
            // numerator.x += p.x * g;
            // numerator.y += p.y * g;
            // numerator.z += p.z * g;
            // denominator += g;
            denominator += 1;
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

Point rotate_point(double phi, Point point, pcl::Normal point_n, Point new_n){
    Point p;
    double cosphi = cos(M_PI - phi);
    double c1 = new_n.y*point_n.normal_z - new_n.z*point_n.normal_y;
    double c2 = new_n.z*point_n.normal_x - new_n.x*point_n.normal_z;
    double c3 = new_n.x*point_n.normal_y - new_n.y*point_n.normal_x;
    double norm_c = sqrt(c1*c1 + c2*c2 + c3*c3);
    double axx = c1/norm_c;
    double axy = c2/norm_c;
    double axz = c3/norm_c;
    double cs = cosphi;
    double s = sqrt(1-cs*cs);
    double C = 1-cs;

    // double p2x = (axx*axx*C+cs)*px + (axx*axy*C-axz*s)*py + (axx*axz*C+axy*s)*pz;
    // double p2y = (axy*axx*C+axz*s)*px + (axy*axy*C+cs)*py + (axy*axz*C-axx*s)*pz;
    // double p2z = (axz*axx*C-axy*s)*px + (axz*axy*C+axx*s)*py + (axz*axz*C+cs)*pz;
    p.x = (axx*axx*C+cs)*point.x + (axx*axy*C-axz*s)*point.y + (axx*axz*C+axy*s)*point.z;
    p.y = (axy*axx*C+axz*s)*point.x + (axy*axy*C+cs)*point.y + (axy*axz*C-axx*s)*point.z;
    p.z = (axz*axx*C-axy*s)*point.x + (axz*axy*C+axx*s)*point.y + (axz*axz*C+cs)*point.z;

    return p;
}

std::vector<double> get_angle_gaps(PointCloud cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, std::map<unsigned long int, std::vector<int>> neighbours){
    std::cout << "get_angle_gaps" << std::endl;
    // project neighbours to plane
    // rotate plane
    // compute angles
    // sort
    // compute differences
    // get biggest gap
    std::vector<double> gaps;
    // std::cout << "77 size " << neighbours[77].size() << std::endl;
    auto start = std::chrono::high_resolution_clock::now();

    int num1 = 0;
    int num2 = 0;
    int num3 = 0;
    int num4 = 0;
    for(unsigned long int i = 0; i < (*cloud).size(); i++){
        // point
        double d = (*cloud)[i].x;
        double e = (*cloud)[i].y;
        double f = (*cloud)[i].z;
        // normals
        double a = cloud_normals->points[i].normal_x;
        double b = cloud_normals->points[i].normal_y;
        double c = cloud_normals->points[i].normal_z;

        double translation_x = 0;
        double translation_y = 0;
        double translation_z = 0;

        std::vector<Point> transformed_points;
        for(unsigned long j = 0; j < neighbours[i].size(); j++){
            // project point to tangent plane
            // double x = (*cloud)[neighbours[i][j]].x;
            // double y = (*cloud)[neighbours[i][j]].y;
            // double z = (*cloud)[neighbours[i][j]].z;
            // double t = (a*d - a*x + b*e - b*y + c*f - c*z)/(a*a + b*b + c*c);
            // double px = x + a*t;
            // double py = y + b*t;
            // double pz = z + c*t;

            // Point p;
            // p.x = px;
            // p.y = py;
            // p.z = pz;

            // double p2x = px;
            // double p2y = py;
            // double p2z = pz;

            Point p = tangent_projection(cloud_normals->points[i], (*cloud)[i], (*cloud)[neighbours[i][j]]);
            // if(fabs(p_.x - p.x) < 0.00001 && fabs(p_.y - p.y) < 0.00001 && fabs(p_.z - p.z) < 0.00001){
            //     // std::cout << i << ": " << j << " - same points!" << std::endl;
            // }
            // else{
            //     std::cout << i << ": " << j << " - DIFF!" << std::endl;
            // }

            // rotate tangent plane
            double phi = acos(c/sqrt(a*a + b*b + c*c));
            // // if(fabs(phi) > 0.0){
            //     double cosphi = cos(M_PI - phi);
            //     double c1 = 0*c - 1*b;
            //     double c2 = 1*a - 0*c;
            //     double c3 = 0*b - 0*a;
            //     double norm_c = sqrt(c1*c1 + c2*c2 + c3*c3);
            //     double axx = c1/norm_c;
            //     double axy = c2/norm_c;
            //     double axz = c3/norm_c;
            //     double cs = cosphi;
            //     double s = sqrt(1-cs*cs);
            //     double C = 1-cs;

            //     // double p2x = (axx*axx*C+cs)*px + (axx*axy*C-axz*s)*py + (axx*axz*C+axy*s)*pz;
            //     // double p2y = (axy*axx*C+axz*s)*px + (axy*axy*C+cs)*py + (axy*axz*C-axx*s)*pz;
            //     // double p2z = (axz*axx*C-axy*s)*px + (axz*axy*C+axx*s)*py + (axz*axz*C+cs)*pz;
            //     p2x = (axx*axx*C+cs)*px + (axx*axy*C-axz*s)*py + (axx*axz*C+axy*s)*pz;
            //     p2y = (axy*axx*C+axz*s)*px + (axy*axy*C+cs)*py + (axy*axz*C-axx*s)*pz;
            //     p2z = (axz*axx*C-axy*s)*px + (axz*axy*C+axx*s)*py + (axz*axz*C+cs)*pz;

            //     // p.x = p2x;
            //     // p.y = p2y;
            //     // p.z = p2z;
            // // }

            Point nor;
            nor.x = 0;
            nor.y = 0;
            nor.z = 1;
            Point p_r = rotate_point(phi, p, cloud_normals->points[i], nor);
            // p.x = p2x;
            //     p.y = p2y;
            //     p.z = p2z;
            // if(fabs(p_.x - p.x) < 0.00001 && fabs(p_.y - p.y) < 0.00001 && fabs(p_.z - p.z) < 0.00001){
            //     // std::cout << i << ": " << j << " - same points!" << std::endl;
            // }
            // else{
            //     std::cout << i << ": " << j << " - DIFF!" << std::endl;
            // }
            // if(i == 77)
            // std::cout << "point " << p2x << ", " << p2y << ", " << p2z << std::endl;

            
            // transformed_points.push_back(p);
            transformed_points.push_back(p_r);

            if((*cloud)[neighbours[i][j]].x == (*cloud)[i].x && (*cloud)[neighbours[i][j]].y == (*cloud)[i].y && (*cloud)[neighbours[i][j]].z == (*cloud)[i].z){
                // translation_x = p2x;
                // translation_y = p2y;
                // translation_z = p2z;
                translation_x = p_r.x;
                translation_y = p_r.y;
                translation_z = p_r.z;
            }
        }
        // std::cout << "after inner cycle" << std::endl;
        // translate to origin
        for(Point& p : transformed_points){
            p.x -= translation_x;
            p.y -= translation_y;
            p.z -= translation_z;
            // if(i == 77)
            // std::cout << "point " << p.x << ", " << p.y << ", " << p.z << std::endl;
        }
        // std::cout << "after translation" << std::endl;
        // compute absolute angles
        // std::cout << "start" << std::endl;
        
        std::vector<double> absolute_angles;
        std::vector<double> absolute_angles_sorted;
        for(unsigned long j = 0; j < neighbours[i].size(); j++){
            // transformed_points[j].y *= 100;
            // transformed_points[j].x *= 100;
            double angle = atan2(transformed_points[j].y, transformed_points[j].x) + M_PI;
            if(angle > 2*M_PI){
                // std::cout << angle << std::endl;
                angle = 2*M_PI;
            }
            // if(fabs(angle) > M_PI){
            //     // std::cout << "angle: " << angle << std::endl;
            //     num1++;
            //     angle = 2*M_PI - fabs(angle);
            //     // std::cout << "angle: " << angle << std::endl;
            //     // std::cout << "--------------------" << std::endl;
            // }
            // absolute_angles.push_back(angle + M_PI);
            // absolute_angles_sorted.push_back(angle + M_PI);
            absolute_angles.push_back(angle);
            absolute_angles_sorted.push_back(angle);
            // if(i == 77)
            // std::cout << angle << ", ";
        }
        // std::cout << "after compute angles" << std::endl;
        // if(i == 77)
        // std::cout << std::endl;
        std::sort(absolute_angles_sorted.begin(), absolute_angles_sorted.end());
        // for(unsigned long j = 0; j < neighbours[i].size(); j++){
        //     if(i == 77)
        //     std::cout << absolute_angles_sorted[j] << ", ";
        // }
        // std::cout << "after sorting" << std::endl;
        // compute relative angles
        double max_angle = 0;
        double idx1 = -1;
        double idx2 = -1;
        for(unsigned int j = 0; j < absolute_angles_sorted.size(); j++){
            if(fabs(absolute_angles_sorted[j]) > 2*M_PI){
                // std::cout << "angle: " << max_angle << std::endl;
                num4++;
            }
            double a1 = fabs(2*M_PI - fabs(absolute_angles_sorted[j]) + fabs(absolute_angles_sorted[0]));
            a1 = a1 > M_PI ? 2*M_PI - a1 : a1;
            double a2 = fabs(absolute_angles_sorted[j+1] - absolute_angles_sorted[j]);
            a2 = a2 > M_PI ? 2*M_PI - a2 : a2;
            if(j == absolute_angles_sorted.size() - 1){
                // if(fabs(2*M_PI - fabs(absolute_angles_sorted[j]) - fabs(absolute_angles_sorted[0])) > max_angle){
                //     // max_angle = abs(absolute_angles_sorted[j] + M_PI - absolute_angles_sorted[0]);
                //     max_angle = fabs(2*M_PI - abs(absolute_angles_sorted[j]) - abs(absolute_angles_sorted[0]));
                if(a1 > max_angle){
                    // max_angle = abs(absolute_angles_sorted[j] + M_PI - absolute_angles_sorted[0]);
                    // max_angle = fabs(2*M_PI - fabs(absolute_angles_sorted[j]) + fabs(absolute_angles_sorted[0]));
                    max_angle = a1;
                    idx1 = j;
                    idx2 = 0;
                    if(fabs(max_angle) > 2*M_PI){
                        // std::cout << "angle: " << max_angle << std::endl;
                        num3++;
                    }
                }
            }
            else{
                // if(fabs(absolute_angles_sorted[j+1] - absolute_angles_sorted[j]) > max_angle){
                if(a2 > max_angle){
                    // max_angle = fabs(absolute_angles_sorted[j+1] - absolute_angles_sorted[j]);
                    max_angle = a2;
                    idx1 = j;
                    idx2 = j+1;
                    // std::cout << max_angle << std::endl;
                    // std::cout << fabs(max_angle) << std::endl;
                    // std::cout << absolute_angles_sorted[j+1] - absolute_angles_sorted[j] << std::endl;
                    // std::cout << absolute_angles_sorted[j+1] << std::endl;
                    // std::cout << absolute_angles_sorted[j] << std::endl;
                    if(fabs(max_angle) > 2*M_PI){
                        // std::cout << "angle: " << max_angle << std::endl;
                        num2++;
                        for(unsigned int j = 0; j < absolute_angles_sorted.size(); j++){
                            std::cout << "angle: " << absolute_angles_sorted[j] << std::endl;
                            if(j != absolute_angles_sorted.size()-1){
                                std::cout << absolute_angles_sorted[j+1] - absolute_angles_sorted[j] << std::endl;
                                std::cout << fabs(absolute_angles_sorted[j+1] - absolute_angles_sorted[j]) << std::endl;
                            }
                            std::cout << "----------" << std::endl;
                        }
                        exit(0);
                    }
                }
                
            }
        }
        // if(i == 77)
        // std::cout << "max angle: " << max_angle << std::endl;
        // if(max_angle > 4){
        //     std::cout << "max angle: " << max_angle << std::endl;
        // }
        gaps.push_back(max_angle);
    }

    std::cout << "num1: " << num1 << std::endl;
    std::cout << "num2: " << num2 << std::endl;
    std::cout << "num3: " << num3 << std::endl;
    std::cout << "num4: " << num3 << std::endl;
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = end - start;
    std::cout << "slept for " << fp_ms.count() << " milliseconds." << std::endl;
    std::cout << "get_angle_gaps done" << std::endl;
    return gaps;
}

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




#endif