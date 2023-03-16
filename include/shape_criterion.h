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

double compute_prob(Point A_p){
    Point A_boundary;
    Point A_interior;
    Point A_corner_noise;
    Point A_line;
    Point A_ridge;
    A_boundary.x = 2.0/3.0;
    A_boundary.y = 1.0/3.0;
    A_boundary.z = 0;
    A_interior.x = 1.0/2.0;
    A_interior.y = 1.0/2.0;
    A_interior.z = 0;
    A_corner_noise.x = 1.0/3.0;
    A_corner_noise.y = 1.0/3.0;
    A_corner_noise.z = 1.0/3.0;
    A_line.x = 1.0;
    A_line.y = 0;
    A_line.z = 0;
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
    c.x = (A_interior.x + A_corner_noise.x + A_line.x) / 3;
    c.y = (A_interior.y + A_corner_noise.y + A_line.y) / 3;
    c.z = (A_interior.z + A_corner_noise.z + A_line.z) / 3;
    // c.x = (A_boundary.x + A_interior.x + A_corner_noise.x + A_line.x) / 4;
    // c.y = (A_boundary.y + A_interior.y + A_corner_noise.y + A_line.y) / 4;
    // c.z = (A_boundary.z + A_interior.z + A_corner_noise.z + A_line.z) / 4;
    // c.x = (A_boundary.x + A_interior.x + A_corner_noise.x + A_line.x + A_p.x) / 5;
    // c.y = (A_boundary.y + A_interior.y + A_corner_noise.y + A_line.y + A_p.y) / 5;
    // c.z = (A_boundary.z + A_interior.z + A_corner_noise.z + A_line.z + A_p.z) / 5;

    double denominator = 0;
    
    double d_boundary = vect_norm(A_p, A_boundary);
    // std::cout << "d " << d_boundary << std::endl;
    // double sigma_boundary = 1 / (3*vect_norm(A_boundary, c)*vect_norm(A_boundary, c));
    // double sigma_boundary = (vect_norm(A_boundary, c)*vect_norm(A_boundary, c)) / 3;
    double sigma_boundary = (vect_norm(A_boundary, c)) / 3;
    // std::cout << "sigma " << sigma_boundary << std::endl;
    double prob_boundary = exp( (-d_boundary*d_boundary) / (sigma_boundary*sigma_boundary) );
    // std::cout << "prob " << prob_boundary << std::endl;
    denominator += prob_boundary;

    double d_interior = vect_norm(A_p, A_interior);
    // std::cout << "d " << d_interior << std::endl;
    // double sigma_interior = 1 / (3*vect_norm(A_interior, c)*vect_norm(A_interior, c));
    // double sigma_interior = (vect_norm(A_interior, c)*vect_norm(A_interior, c)) / 3;
    double sigma_interior = (vect_norm(A_interior, c)) / 3;
    // std::cout << "sigma " << sigma_interior << std::endl;
    double prob_interior = exp( (-d_interior*d_interior) / (sigma_interior*sigma_interior) );
    // std::cout << "prob " << prob_interior << std::endl;
    denominator += prob_interior;

    double d_corner_noise = vect_norm(A_p, A_corner_noise);
    // std::cout << "d " << d_corner_noise << std::endl;
    // double sigma_corner_noise = 1 / (3*vect_norm(A_corner_noise, c)*vect_norm(A_corner_noise, c));
    // double sigma_corner_noise = (vect_norm(A_corner_noise, c)*vect_norm(A_corner_noise, c)) / 3;
    double sigma_corner_noise = (vect_norm(A_corner_noise, c)) / 3;
    // std::cout << "sigma " << sigma_corner_noise << std::endl;
    double prob_corner_noise = exp( (-d_corner_noise*d_corner_noise) / (sigma_corner_noise*sigma_corner_noise) );
    // std::cout << "prob " << prob_corner_noise << std::endl;
    denominator += prob_corner_noise;

    // double d_line = vect_norm(A_p, A_line);
    // // std::cout << "d " << d_line << std::endl;
    // // double sigma_line = 1 / (3*vect_norm(A_line, c)*vect_norm(A_line, c));
    // // double sigma_line = (vect_norm(A_line, c)*vect_norm(A_line, c)) / 3;
    // double sigma_line = (vect_norm(A_line, c)) / 3;
    // // std::cout << "sigma " << sigma_line << std::endl;
    // double prob_line = exp( (-d_line*d_line) / (sigma_line*sigma_line) );
    // // std::cout << "prob " << prob_line << std::endl;
    // denominator += prob_line;

    double d_ridge = vect_norm(A_p, A_ridge);
    // std::cout << "d " << d_line << std::endl;
    // double sigma_line = 1 / (3*vect_norm(A_line, c)*vect_norm(A_line, c));
    // double sigma_line = (vect_norm(A_line, c)*vect_norm(A_line, c)) / 3;
    double sigma_ridge = (vect_norm(A_ridge, c)) / 3;
    // std::cout << "sigma " << sigma_line << std::endl;
    double prob_ridge = exp( (-d_ridge*d_ridge) / (sigma_ridge*sigma_ridge) );
    // std::cout << "prob " << prob_line << std::endl;
    denominator += prob_ridge;

    // std::cout << prob_boundary << std::endl;
    
    // std::cout << "final probs " << prob_boundary / denominator << std::endl;
    // std::cout << "final probs " << prob_interior / denominator << std::endl;
    // std::cout << "final probs " << prob_corner_noise / denominator << std::endl;
    // std::cout << "final probs " << prob_line / denominator << std::endl;
    // std::cout << "final probs " << prob_ridge / denominator << std::endl;
    // std::cout << "=======================" << std::endl;
    return prob_boundary / denominator;
}

std::vector<double> get_probabilities(PointCloud cloud, std::map<unsigned long int, std::vector<int>> neighbours, std::vector<Point> mis){
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

        auto w = [] (Point p, Point q) -> double { 
                // if(p.x - q.x == 0 && p.y - q.y == 0 && p.z - q.z == 0){
                //     return 0;
                // }
                // std::cout << "returning " << 1 / vect_norm(p, q) << std::endl;
                return 1;// / vect_norm(p, q); 
            };
        // for(int i = 0; i < neighbours[point_idx].size(); i++){
        double c11 = 0;
        double c12 = 0;
        double c13 = 0;
        double c22 = 0;
        double c23 = 0;
        double c33 = 0;
        Point centroid;
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
            double wq = w((*cloud)[i], q);
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
        // c11 /= neighbours[i].size();
        // c12 /= neighbours[i].size();
        // c13 /= neighbours[i].size();
        // c22 /= neighbours[i].size();
        // c23 /= neighbours[i].size();
        // c33 /= neighbours[i].size();
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