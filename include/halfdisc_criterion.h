#ifndef HALFDISC_CRITERION_H
#define HALFDISC_CRITERION_H

#include <iostream>
#include <vector>
#include <cmath>
#include <map>
#include <chrono>

#include "auxiliary_functions.h"

std::vector<double> get_probabilities(PointCloud cloud, std::vector<Point> mi_prime, std::vector<double> distances){
    std::cout << "get_probabilities" << std::endl;
    std::vector<double> probs;
    auto start = std::chrono::high_resolution_clock::now();
    for(unsigned long int i = 0; i < (*cloud).size(); i++){
        double norm = vect_norm((*cloud)[i], mi_prime[i]);
        double not_prob = norm / ((4*distances[i]) / (3*M_PI));
        // if(not_prob > 0.4){
        //     std::cout << not_prob << std::endl;
        // }
        double prob = fmin(not_prob, 1);
        probs.push_back(prob);
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = end - start;
    std::cout << "slept for " << fp_ms.count() << " milliseconds." << std::endl;
    std::cout << "get_probabilities done" << std::endl;
    return probs;
}

std::vector<double> halfdisc_criterion(PointCloud cloud, unsigned int K, double epsilon, pcl::PointCloud<pcl::Normal>::Ptr normals,
            std::map<unsigned long int, std::vector<std::pair<int, double>>> graph, std::vector<double> average_distances){
    std::cout << "========== HALFDISC CRITERION ==========" << std::endl;

    // pcl::PointCloud<pcl::Normal>::Ptr normals = get_normal_vectors(cloud, K, epsilon);
    
    // std::map<unsigned long int, std::vector<std::pair<int, double>>> graph = get_neighbours_distances(cloud, K, epsilon);

    // std::vector<double> average_distances = compute_average_distances(graph);
    
    std::vector<Point> mi_p = compute_mi(cloud, graph, average_distances, normals);

    std::vector<Point> mi_p_prime = project_mis(mi_p, normals, cloud);

    std::vector<double> probabilities = get_probabilities(cloud, mi_p_prime, average_distances);

    std::cout << "========== END ==========" << std::endl;

    return probabilities;
}

#endif