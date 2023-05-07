#ifndef SHAPE_CRITERION_H
#define SHAPE_CRITERION_H

#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <chrono>

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/segmentation/region_growing_rgb.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/features/normal_3d.h>

#include <eigen3/Eigen/Eigenvalues>

#include "auxiliary_functions.h"

#include <mrs_lib/param_loader.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/node_handle.h>

double compute_prob(Point A_p){
	// ros::NodeHandle nh;
	// unsigned int queue_length = 1000;
    // ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", queue_length);
    // visualization_msgs::MarkerArray markerArray;

    Point A_boundary;
    Point A_interior;
    Point A_corner_noise;
    Point A_line;
    Point A_ridge;
    A_boundary.x = 2.0/3.0;
    A_boundary.y = 1.0/3.0;
    A_boundary.z = 0.0;
    A_interior.x = 1.0/2.0;
    A_interior.y = 1.0/2.0;
    A_interior.z = 0.0;
    A_corner_noise.x = 1.0/3.0;
    A_corner_noise.y = 1.0/3.0;
    A_corner_noise.z = 1.0/3.0;
    A_line.x = 1.0;
    A_line.y = 0.0;
    A_line.z = 0.0;
    A_ridge.x = 1.0/2.0;
    A_ridge.y = 1.0/4.0;
    A_ridge.z = 1.0/4.0;
    // A_boundary.x = 0.66;
    // A_boundary.y = 0.33;
    // A_boundary.z = 0;
    // A_interior.x = 0.5;
    // A_interior.y = 0.5;
    // A_interior.z = 0;
    // A_corner_noise.x = 0.33;
    // A_corner_noise.y = 0.33;
    // A_corner_noise.z = 0.33;
    // A_line.x = 1;
    // A_line.y = 0;
    // A_line.z = 0;
    // A_ridge.x = 0.5;
    // A_ridge.y = 0.25;
    // A_ridge.z = 0.25;

    // std::cout << A_boundary.x << std::endl;
    // std::cout << A_boundary.y << std::endl;
    // std::cout << A_boundary.z << std::endl;
    // std::cout << A_interior.x << std::endl;
    // std::cout << A_interior.y << std::endl;
    // std::cout << A_interior.z << std::endl;
    // std::cout << A_corner_noise.x << std::endl;
    // std::cout << A_corner_noise.y << std::endl;
    // std::cout << A_corner_noise.z << std::endl;
    // std::cout << A_line.x << std::endl;
    // std::cout << A_line.y << std::endl;
    // std::cout << A_line.z << std::endl;
    // std::cout << A_ridge.x << std::endl;
    // std::cout << A_ridge.y << std::endl;
    // std::cout << A_ridge.z << std::endl;
    // std::cout << "------------------------" << std::endl;
    

    Point c;
    // c.x = (4*A_p.x - A_boundary.x - A_interior.x - A_corner_noise.x - A_line.x) / 4;
    // c.y = (4*A_p.y - A_boundary.y - A_interior.y - A_corner_noise.y - A_line.y) / 4;
    // c.z = (4*A_p.z - A_boundary.z - A_interior.z - A_corner_noise.z - A_line.z) / 4;
    // c.x = (3*A_p.x - A_interior.x - A_corner_noise.x - A_line.x) / 3;
    // c.y = (3*A_p.y - A_interior.y - A_corner_noise.y - A_line.y) / 3;
    // c.z = (3*A_p.z - A_interior.z - A_corner_noise.z - A_line.z) / 3;
    c.x = (A_interior.x + A_corner_noise.x + A_line.x) / 3.0;
    c.y = (A_interior.y + A_corner_noise.y + A_line.y) / 3.0;
    c.z = (A_interior.z + A_corner_noise.z + A_line.z) / 3.0;
    // c.x = (A_boundary.x + A_interior.x + A_corner_noise.x + A_line.x) / 4;
    // c.y = (A_boundary.y + A_interior.y + A_corner_noise.y + A_line.y) / 4;
    // c.z = (A_boundary.z + A_interior.z + A_corner_noise.z + A_line.z) / 4;
    // c.x = (A_boundary.x + A_interior.x + A_corner_noise.x + A_line.x + A_p.x) / 5;
    // c.y = (A_boundary.y + A_interior.y + A_corner_noise.y + A_line.y + A_p.y) / 5;
    // c.z = (A_boundary.z + A_interior.z + A_corner_noise.z + A_line.z + A_p.z) / 5;

    // visualization_msgs::Marker marker;
    // marker.header.frame_id = "base_frame";
    // marker.header.stamp = ros::Time::now();
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.id = 1;
    // marker.type = visualization_msgs::Marker::POINTS;
    // marker.color.r = 1.0f;
    // marker.color.g = 1.0f;
    // marker.color.b = 0.0f;
    // marker.color.a = 1.0f;
    // marker.scale.x = 0.005;
    // marker.scale.y = 0.005;
    // marker.scale.z = 0.005;
    // geometry_msgs::Point p;
    // p.x = A_boundary.x;
    // p.y = A_boundary.y;
    // p.z = A_boundary.z;
    // marker.points.push_back(p);
    // markerArray.markers.push_back(marker);

    // visualization_msgs::Marker marker2;
    // marker2.header.frame_id = "base_frame";
    // marker2.header.stamp = ros::Time::now();
    // marker2.action = visualization_msgs::Marker::ADD;
    // marker2.id = 2;
    // marker2.type = visualization_msgs::Marker::POINTS;
    // marker2.color.r = 1.0f;
    // marker2.color.g = 1.0f;
    // marker2.color.b = 0.0f;
    // marker2.color.a = 1.0f;
    // marker2.scale.x = 0.005;
    // marker2.scale.y = 0.005;
    // marker2.scale.z = 0.005;
    // p.x = A_interior.x;
    // p.y = A_interior.y;
    // p.z = A_interior.z;
    // marker2.points.push_back(p);
    // markerArray.markers.push_back(marker2);

    // visualization_msgs::Marker marker3;
    // marker3.header.frame_id = "base_frame";
    // marker3.header.stamp = ros::Time::now();
    // marker3.action = visualization_msgs::Marker::ADD;
    // marker3.id = 3;
    // marker3.type = visualization_msgs::Marker::POINTS;
    // marker3.color.r = 1.0f;
    // marker3.color.g = 1.0f;
    // marker3.color.b = 0.0f;
    // marker3.color.a = 1.0f;
    // marker3.scale.x = 0.005;
    // marker3.scale.y = 0.005;
    // marker3.scale.z = 0.005;
    // p.x = A_corner_noise.x;
    // p.y = A_corner_noise.y;
    // p.z = A_corner_noise.z;
    // marker3.points.push_back(p);
    // markerArray.markers.push_back(marker3);

    // visualization_msgs::Marker marker4;
    // marker4.header.frame_id = "base_frame";
    // marker4.header.stamp = ros::Time::now();
    // marker4.action = visualization_msgs::Marker::ADD;
    // marker4.id = 4;
    // marker4.type = visualization_msgs::Marker::POINTS;
    // marker4.color.r = 1.0f;
    // marker4.color.g = 1.0f;
    // marker4.color.b = 0.0f;
    // marker4.color.a = 1.0f;
    // marker4.scale.x = 0.005;
    // marker4.scale.y = 0.005;
    // marker4.scale.z = 0.005;
    // p.x = A_line.x;
    // p.y = A_line.y;
    // p.z = A_line.z;
    // marker4.points.push_back(p);
    // markerArray.markers.push_back(marker4);

    // visualization_msgs::Marker marker5;
    // marker5.header.frame_id = "base_frame";
    // marker5.header.stamp = ros::Time::now();
    // marker5.action = visualization_msgs::Marker::ADD;
    // marker5.id = 5;
    // marker5.type = visualization_msgs::Marker::POINTS;
    // marker5.color.r = 1.0f;
    // marker5.color.g = 1.0f;
    // marker5.color.b = 0.0f;
    // marker5.color.a = 1.0f;
    // marker5.scale.x = 0.005;
    // marker5.scale.y = 0.005;
    // marker5.scale.z = 0.005;
    // p.x = A_ridge.x;
    // p.y = A_ridge.y;
    // p.z = A_ridge.z;
    // marker5.points.push_back(p);
    // markerArray.markers.push_back(marker5);

    // visualization_msgs::Marker marker6;
    // marker6.header.frame_id = "base_frame";
    // marker6.header.stamp = ros::Time::now();
    // marker6.action = visualization_msgs::Marker::ADD;
    // marker6.id = 6;
    // marker6.type = visualization_msgs::Marker::POINTS;
    // marker6.color.r = 0.0f;
    // marker6.color.g = 1.0f;
    // marker6.color.b = 0.0f;
    // marker6.color.a = 1.0f;
    // marker6.scale.x = 0.005;
    // marker6.scale.y = 0.005;
    // marker6.scale.z = 0.005;
    // p.x = c.x;
    // p.y = c.y;
    // p.z = c.z;
    // marker6.points.push_back(p);
    // markerArray.markers.push_back(marker6);

    // visualization_msgs::Marker marker7;
    // marker7.header.frame_id = "base_frame";
    // marker7.header.stamp = ros::Time::now();
    // marker7.action = visualization_msgs::Marker::ADD;
    // marker7.id = 7;
    // marker7.type = visualization_msgs::Marker::POINTS;
    // marker7.color.r = 1.0f;
    // marker7.color.g = 0.0f;
    // marker7.color.b = 0.0f;
    // marker7.color.a = 1.0f;
    // marker7.scale.x = 0.005;
    // marker7.scale.y = 0.005;
    // marker7.scale.z = 0.005;
    // p.x = A_p.x;
    // p.y = A_p.y;
    // p.z = A_p.z;
    // marker7.points.push_back(p);
    // markerArray.markers.push_back(marker7);



    double denominator = 0;
    
    double d_boundary = vect_norm(A_p, A_boundary);
    // std::cout << "d " << d_boundary << std::endl;
    // double sigma_boundary = 1 / (3*vect_norm(A_boundary, c)*vect_norm(A_boundary, c));
    // double sigma_boundary = (vect_norm(A_boundary, c)*vect_norm(A_boundary, c)) / 3.0;
    double sigma_boundary = (vect_norm(A_boundary, c)) / 3.0;
    // std::cout << "sigma " << sigma_boundary << std::endl;
    double prob_boundary = exp( -(d_boundary*d_boundary) / (sigma_boundary*sigma_boundary) );
    // std::cout << "prob " << prob_boundary << std::endl;
    denominator += prob_boundary;

    double d_interior = vect_norm(A_p, A_interior);
    // std::cout << "d " << d_interior << std::endl;
    // double sigma_interior = 1 / (3*vect_norm(A_interior, c)*vect_norm(A_interior, c));
    // double sigma_interior = (vect_norm(A_interior, c)*vect_norm(A_interior, c)) / 3.0;
    double sigma_interior = (vect_norm(A_interior, c)) / 3.0;
    // std::cout << "sigma " << sigma_interior << std::endl;
    double prob_interior = exp( (-d_interior*d_interior) / (sigma_interior*sigma_interior) );
    // std::cout << "prob " << prob_interior << std::endl;
    denominator += prob_interior;

    double d_corner_noise = vect_norm(A_p, A_corner_noise);
    // std::cout << "d " << d_corner_noise << std::endl;
    // double sigma_corner_noise = 1 / (3*vect_norm(A_corner_noise, c)*vect_norm(A_corner_noise, c));
    // double sigma_corner_noise = (vect_norm(A_corner_noise, c)*vect_norm(A_corner_noise, c)) / 3.0;
    double sigma_corner_noise = (vect_norm(A_corner_noise, c)) / 3.0;
    // std::cout << "sigma " << sigma_corner_noise << std::endl;
    double prob_corner_noise = exp( (-d_corner_noise*d_corner_noise) / (sigma_corner_noise*sigma_corner_noise) );
    // std::cout << "prob " << prob_corner_noise << std::endl;
    denominator += prob_corner_noise;

    double d_line = vect_norm(A_p, A_line);
    // std::cout << "d " << d_line << std::endl;
    // double sigma_line = 1 / (3*vect_norm(A_line, c)*vect_norm(A_line, c));
    // double sigma_line = (vect_norm(A_line, c)*vect_norm(A_line, c)) / 3.0;
    double sigma_line = (vect_norm(A_line, c)) / 3.0;
    // std::cout << "sigma " << sigma_line << std::endl;
    double prob_line = exp( (-d_line*d_line) / (sigma_line*sigma_line) );
    // std::cout << "prob " << prob_line << std::endl;
    // denominator += prob_line;

    double d_ridge = vect_norm(A_p, A_ridge);
    // std::cout << "d " << d_line << std::endl;
    // double sigma_ridge = 1 / (3*vect_norm(A_line, c)*vect_norm(A_line, c));
    // double sigma_ridge = (vect_norm(A_ridge, c)*vect_norm(A_ridge, c)) / 3.0;
    double sigma_ridge = (vect_norm(A_ridge, c)) / 3.0;
    // std::cout << "sigma " << sigma_ridge << std::endl;
    double prob_ridge = exp( (-d_ridge*d_ridge) / (sigma_ridge*sigma_ridge) );
    // std::cout << "prob " << prob_ridge << std::endl;
    denominator += prob_ridge;

    // std::cout << prob_boundary << std::endl;
    
    // std::cout << "final probs " << prob_boundary / denominator << std::endl;
    // std::cout << "final probs " << prob_interior / denominator << std::endl;
    // std::cout << "final probs " << prob_corner_noise / denominator << std::endl;
    // std::cout << "final probs " << prob_line / denominator << std::endl;
    // std::cout << "final probs " << prob_ridge / denominator << std::endl;
    // std::cout << "=======================" << std::endl;

    // marker_pub.publish(markerArray);
    return prob_boundary / denominator;
}

std::vector<double> get_probabilities(PointCloud cloud, std::map<unsigned long int, std::vector<int>> neighbours, std::vector<Point> mis){
    ros::NodeHandle nh;
	unsigned int queue_length = 1000;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", queue_length);
    visualization_msgs::MarkerArray markerArray;
    
    std::cout << "---" << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << "neighbours size " << neighbours.size() << std::endl;
    std::vector<double> probs;
    // Eigen::EigenSolver<Eigen::MatrixXd> es;
    unsigned long int n1 = 0;
    unsigned long int n2 = 0;
    // Eigen::MatrixXd C_p = compute_covariance_matrix(1, cloud, neighbours, mis[1]);

    for(unsigned long int i = 0; i < (*cloud).size(); i++){
        // std::vector<std::vector<double>> m = compute_covariance_matrix(i, cloud, neighbours, mis[i]);
        // Eigen::MatrixXd C_p = compute_covariance_matrix(i, cloud, neighbours, mis[i]);

        auto w = [] (Point p, Point q, double dist) -> double { 
                // if(p.x - q.x == 0 && p.y - q.y == 0 && p.z - q.z == 0){
                //     return 0;
                // }
                // std::cout << "returning " << 1 / vect_norm(p, q) << std::endl;
                // return 1;// / vect_norm(p, q); 
                // return vect_norm(p, q);
                // return 1.0 / vect_norm(p, q);
                double norm = vect_norm(p, q);
                // return exp(- ((9.0 * (norm * norm)) / (dist * dist)));
                return exp(- ( (norm * norm)) / (dist * dist));
                // return exp(- ((9.0 * (norm * norm)) / (dist * dist * dist * dist)));
            };
        // for(int i = 0; i < neighbours[point_idx].size(); i++){
        double c11 = 0;
        double c12 = 0;
        double c13 = 0;
        double c22 = 0;
        double c23 = 0;
        double c33 = 0;
        // Point centroid;
        // centroid.x = 0;
        // centroid.y = 0;
        // centroid.z = 0;
        // for(int j = 0; j < neighbours[i].size(); j++){
        //     centroid.x += (*cloud)[neighbours[i][j]].x;
        //     centroid.y += (*cloud)[neighbours[i][j]].y;
        //     centroid.z += (*cloud)[neighbours[i][j]].z;
        // }
        // centroid.x /= static_cast <double> (neighbours[i].size());
        // centroid.y /= static_cast <double> (neighbours[i].size());
        // centroid.z /= static_cast <double> (neighbours[i].size());

        // std::cout << "mis[i] " << mis[i].x << ", " << mis[i].y << ", " << mis[i].z << std::endl;
        // std::cout << "centroid " << centroid.x << ", " << centroid.y << ", " << centroid.z << std::endl;
        // std::cout << "----------" << std::endl;

        // visualization_msgs::Marker marker;
        // marker.header.frame_id = "base_frame";
        // marker.header.stamp = ros::Time::now();
        // marker.action = visualization_msgs::Marker::ADD;
        // marker.id = 10*i + 1;
        // marker.type = visualization_msgs::Marker::POINTS;
        // marker.color.r = 0.0f;
        // marker.color.g = 1.0f;
        // marker.color.b = 0.0f;
        // marker.color.a = 1.0f;
        // marker.scale.x = 0.005;
        // marker.scale.y = 0.005;
        // marker.scale.z = 0.005;
        // geometry_msgs::Point p;
        // p.x = (*cloud)[i].x;
        // p.y = (*cloud)[i].y;
        // p.z = (*cloud)[i].z;
        // marker.points.push_back(p);
        // markerArray.markers.push_back(marker);

        // visualization_msgs::Marker marker2;
        // marker2.header.frame_id = "base_frame";
        // marker2.header.stamp = ros::Time::now();
        // marker2.action = visualization_msgs::Marker::ADD;
        // marker2.id = 10*i + 2;
        // marker2.type = visualization_msgs::Marker::POINTS;
        // marker2.color.r = 0.0f;
        // marker2.color.g = 1.0f;
        // marker2.color.b = 1.0f;
        // marker2.color.a = 1.0f;
        // marker2.scale.x = 0.005;
        // marker2.scale.y = 0.005;
        // marker2.scale.z = 0.005;
        // geometry_msgs::Point p2;
        // p2.x = mis[i].x;
        // p2.y = mis[i].y;
        // p2.z = mis[i].z;
        // marker2.points.push_back(p2);
        // markerArray.markers.push_back(marker2);
        double avg_dist = 0.0;
        for(int j = 0; j < neighbours[i].size(); j++){
            avg_dist += vect_norm((*cloud)[i], (*cloud)[neighbours[i][j]]);
            // avg_dist += vect_norm(mis[i], (*cloud)[neighbours[i][j]]);
        }
        avg_dist /= neighbours[i].size();

        for(int j = 0; j < neighbours[i].size(); j++){
            Point q = (*cloud)[neighbours[i][j]];
            double d1 = mis[i].x - q.x;
            double d2 = mis[i].y - q.y;
            double d3 = mis[i].z - q.z;
            // double d1 = centroid.x - q.x;
            // double d2 = centroid.y - q.y;
            // double d3 = centroid.z - q.z;
            // double d1 = (*cloud)[i].x - q.x;
            // double d2 = (*cloud)[i].y - q.y;
            // double d3 = (*cloud)[i].z - q.z;
            // c11 += d1*d1;
            // c12 += d1*d2;
            // c13 += d1*d3;
            // c22 += d2*d2;
            // c23 += d2*d3;
            // c33 += d3*d3;
            double wq = w(mis[i], q, avg_dist);
            // double wq = w((*cloud)[i], q, avg_dist);
            c11 += wq*d1*d1;
            c12 += wq*d1*d2;
            c13 += wq*d1*d3;
            c22 += wq*d2*d2;
            c23 += wq*d2*d3;
            c33 += wq*d3*d3;
            // centroid.x += q.x;
            // centroid.y += q.y;
            // centroid.z += q.z;
        }
        // centroid.x /= neighbours[i].size();
        // centroid.y /= neighbours[i].size();
        // centroid.z /= neighbours[i].size();
        Eigen::MatrixXd C_p (3, 3);
        // c11 /= static_cast <double> (neighbours[i].size());
        // c12 /= static_cast <double> (neighbours[i].size());
        // c13 /= static_cast <double> (neighbours[i].size());
        // c22 /= static_cast <double> (neighbours[i].size());
        // c23 /= static_cast <double> (neighbours[i].size());
        // c33 /= static_cast <double> (neighbours[i].size());
        C_p << c11, c12, c13, c12, c22, c23, c13, c23, c33;
        // Eigen::MatrixXd C_p (3, 3);
        // // {{m[0][0], m[0][1], m[0][2]}, {m[1][0], m[1][1], m[1][2]}, {m[2][0], m[2][1], m[2][2]}};
        // C_p << m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2];
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es;
        // Eigen::EigenSolver<Eigen::MatrixXd> es;
        es.compute(C_p, /* computeEigenvectors = */ false);
        // SORT EIGENVALUES!!
        // std::cout << "The eigenvalues of C_p are: " << es.eigenvalues().transpose() << std::endl;
        // double l1 = es.eigenvalues()[2].real();
        // double l2 = es.eigenvalues()[1].real();
        // double l3 = es.eigenvalues()[0].real();
        double l1 = es.eigenvalues()[2];
        double l2 = es.eigenvalues()[1];
        double l3 = es.eigenvalues()[0];
        // double l1 = es.eigenvalues()[0];
        // double l2 = es.eigenvalues()[1];
        // double l3 = es.eigenvalues()[2];
        if(l1 < l2 || l1 < l3 || l2 < l3){
            std::cout << l1 << ", " << l2 << ", " << l3 << std::endl;
            std::cerr << "ERROR: eigenvalues are sort wrongly" << std::endl;
            exit(100);
        }
        // std::cout << l1 << ", " << l2 << ", " << l3 << std::endl;
        double alpha = l1 + l2 + l3;
        // std::cout << alpha << std::endl;
        Point A_p;
        A_p.x = l1 / alpha;
        A_p.y = l2 / alpha;
        A_p.z = l3 / alpha;
        // std::cout << "A_p " << A_p.x << ", " << A_p.y << ", " << A_p.z << std::endl;



        // c11 = 0.0;
        // c12 = 0.0;
        // c13 = 0.0;
        // c22 = 0.0;
        // c23 = 0.0;
        // c33 = 0.0;
        // for(int j = 0; j < neighbours[i].size(); j++){
        //     Point q = (*cloud)[neighbours[i][j]];
        //     double d1 = mis[i].x - q.x;
        //     double d2 = mis[i].y - q.y;
        //     double d3 = mis[i].z - q.z;
        //     double wq = w((*cloud)[i], q);
        //     c11 += wq*d1*d1;
        //     c12 += wq*d1*d2;
        //     c13 += wq*d1*d3;
        //     c22 += wq*d2*d2;
        //     c23 += wq*d2*d3;
        //     c33 += wq*d3*d3;
        // }
        // Eigen::MatrixXd C_ (3, 3);
        // C_ << c11, c12, c13, c12, c22, c23, c13, c23, c33;
        // es.compute(C_, /* computeEigenvectors = */ false);
        // l1 = es.eigenvalues()[2];
        // l2 = es.eigenvalues()[1];
        // l3 = es.eigenvalues()[0];
        // if(l1 < l2 || l1 < l3 || l2 < l3){
        //     std::cout << l1 << ", " << l2 << ", " << l3 << std::endl;
        //     std::cerr << "ERROR: eigenvalues are sort wrongly" << std::endl;
        //     exit(100);
        // }
        // alpha = l1 + l2 + l3;
        // A_p.x = l1 / alpha;
        // A_p.y = l2 / alpha;
        // A_p.z = l3 / alpha;
        // std::cout << "A_p " << A_p.x << ", " << A_p.y << ", " << A_p.z << std::endl;
        // std::cout << "------" << std::endl;



        // A_p.x *= 100;
        // A_p.y *= 100;
        // A_p.z *= 100;
        // Point centroid;
        // centroid.x = 0;
        // centroid.y = 0;
        // centroid.z = 0;
        // for(int j = 0; j < neighbours[i].size(); j++){
        //     centroid.x += (*cloud)[neighbours[i][j]].x;
        //     centroid.y += (*cloud)[neighbours[i][j]].y;
        //     centroid.z += (*cloud)[neighbours[i][j]].z;
        // }
        // centroid.x /= neighbours[i].size();
        // centroid.y /= neighbours[i].size();
        // centroid.z /= neighbours[i].size();

        // double prob = compute_prob(A_p, centroid);
        double prob = compute_prob(A_p);
        // double prob = compute_prob(A_p, A_p);
        // double prob = compute_prob(A_p, (*cloud)[i]);
        if(prob > 0.01){
            n1++;
        }
        else{
            n2++;
        }
        // double prob = 0;
        probs.push_back(prob);
        // es.compute(A + MatrixXf::Identity(4,4), false); // re-use es to compute eigenvalues of A+I
        // cout << "The eigenvalues of A+I are: " << es.eigenvalues().transpose() << endl;
        // if(i == 10){
        //     break;
        // }
        // if(i % 10 == 0){
        //     std::cout << "------ " << i << " ------" << std::endl;
        // }
        marker_pub.publish(markerArray);
        // exit(100);
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = end - start;
    std::cout << "slept for " << fp_ms.count() << " milliseconds." << std::endl;
    std::cout << "--- done" << std::endl;

    std::cout << "n1 " << n1 << "\nn2 " << n2 << std::endl;
    return probs;
}


// return vector of probabilities that point on idx i is a boundary point
std::vector<double> shape_criterion(PointCloud cloud, unsigned int K, double epsilon, pcl::PointCloud<pcl::Normal>::Ptr normals,
            std::map<unsigned long int, std::vector<int>> neighbours, std::map<unsigned long int, std::vector<std::pair<int, double>>> graph,
            std::vector<double> average_distances){
    std::cout << "========== SHAPE CRITERION ==========" << std::endl;

    // pcl::PointCloud<pcl::Normal>::Ptr normals = get_normal_vectors(cloud, K, epsilon);
    
    // std::map<unsigned long int, std::vector<std::pair<int, double>>> graph = get_neighbours_distances(cloud, K, epsilon);

    // std::vector<double> average_distances = compute_average_distances(graph);

    std::vector<Point> mi_p = compute_mi(cloud, graph, average_distances, normals);
    // std::vector<Point> mi_p_prime = project_mis(mi_p, normals, cloud);

    // std::map<unsigned long int, std::vector<int>> neighbours = get_neighbours(cloud, K, epsilon);

    // std::vector<double> probabilities = some_func(cloud, neighbours, mi_p);
    std::vector<double> probabilities = get_probabilities(cloud, neighbours, mi_p);
    // std::vector<double> probabilities = some_func(cloud, neighbours, mi_p_prime);
    // std::cout << "prob size " << probabilities.size() << std::endl;
    // std::vector<double> probabilities = get_probabilities();


    // unsigned long int non = 0;
    // unsigned long int yes = 0;
    // std::cout << "nd " << graph.size() << " and n " << neighbours.size() << std::endl;
    // for(unsigned long int i = 0; i < graph.size(); i++){
    //     if(graph[i].size() != neighbours[i].size()){
    //         std::cout << "nd " << graph[i].size() << " and n " << neighbours[i].size() << std::endl;
    //         non++;
    //     }
    //     else{
    //         yes++;
    //     }
    // }

    // std::cout << "yes " << yes << "\nnon " << non << std::endl;


    std::cout << "========== END ==========" << std::endl;

    return probabilities;
}

#endif