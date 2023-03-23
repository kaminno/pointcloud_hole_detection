#ifndef ANGLE_CRITERION_H
#define ANGLE_CRITERION_H

#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <chrono>

#include "auxiliary_functions.h"

std::vector<double> get_probabilities(std::vector<double> gaps, std::map<unsigned long int, std::vector<int>> neighbours){
    std::cout << "get_probabilities" << std::endl;
    std::vector<double> probs;
    auto start = std::chrono::high_resolution_clock::now();
    for(unsigned long int i = 0; i < gaps.size(); i++){
        double not_prob = ( gaps[i] - (2 * M_PI / neighbours[i].size())) / (M_PI - (2 * M_PI / neighbours[i].size()));
        // double not_prob = ( gaps[i] - (2 * M_PI / neighbours[i].size())) / (2*M_PI - (2 * M_PI / neighbours[i].size()));
        double prob = fmin(not_prob, 1);
        probs.push_back(prob);
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = end - start;
    std::cout << "slept for " << fp_ms.count() << " milliseconds." << std::endl;
    std::cout << "get_probabilities done" << std::endl;
    return probs;
}

std::vector<double> angle_criterion(PointCloud cloud, unsigned int K, double epsilon, pcl::PointCloud<pcl::Normal>::Ptr normals,
            std::map<unsigned long int, std::vector<int>> graph){
    std::cout << "========== ANGLE CRITERION ==========" << std::endl;

    // pcl::PointCloud<pcl::Normal>::Ptr normals = get_normal_vectors(cloud, K, epsilon);

    // std::map<unsigned long int, std::vector<int>> graph = get_neighbours(cloud, K, epsilon);

    std::vector<double> angle_gaps = get_angle_gaps(cloud, normals, graph);

    std::vector<double> probabilities = get_probabilities(angle_gaps, graph);

    std::cout << "========== END ==========" << std::endl;

    return probabilities;
}

#endif