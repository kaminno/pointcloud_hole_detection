#ifndef BOUNDARY_COHERENCE_H
#define BOUNDARY_COHERENCE_H

#include <vector>

#include "auxiliary_functions.h"
#include "angle_criterion.h"
#include "halfdisc_criterion.h"
#include "shape_criterion.h"

std::vector<char> get_boundary_points(PointCloud cloud, int K, double epsilon, double k_angle,
                                        double k_halfdisc, double k_shape, double trashold){

    pcl::PointCloud<pcl::Normal>::Ptr normals = get_normal_vectors(cloud, K, epsilon);
    std::map<unsigned long int, std::vector<int>> neighbours = get_neighbours(cloud, K, epsilon);
    std::map<unsigned long int, std::vector<std::pair<int, double>>> neighbours_distances = get_neighbours_distances(cloud, K, epsilon);
    std::vector<double> average_distances = compute_average_distances(neighbours_distances);

    // std::vector<double> angle_probabilities = angle_criterion(cloud, K, epsilon);
    // std::vector<double> halfdisc_probabilities = halfdisc_criterion(cloud, K, epsilon);
    // std::vector<double> shape_probabilities = shape_criterion(cloud, K, epsilon);

    std::vector<double> angle_probabilities = angle_criterion(cloud, K, epsilon, normals, neighbours);
    std::vector<double> halfdisc_probabilities = halfdisc_criterion(cloud, K, epsilon, normals, neighbours_distances, average_distances);
    std::vector<double> shape_probabilities = shape_criterion(cloud, K, epsilon, normals, neighbours, neighbours_distances, average_distances);

    

    // std::vector<char> boundary_points;
    // for(unsigned long int i = 0; i < (*cloud).size(); i++){
    //     double prob = k_halfdisc * halfdisc_probabilities[i] + k_angle * angle_probabilities[i] + k_shape * shape_probabilities[i];
    //     char boundary = prob > trashold ? 1 : 0;
    //     boundary_points.push_back(boundary);
    // }
    // return boundary_points;

    std::vector<double> probabilities;
    std::vector<char> ret;
    for(unsigned long int i = 0; i < (*cloud).size(); i++){
        double prob = k_halfdisc * halfdisc_probabilities[i] + k_angle * angle_probabilities[i] + k_shape * shape_probabilities[i];
        probabilities.push_back(prob);
        char boundary = prob > trashold ? 1 : 0;
        if(boundary == 1){
            (*cloud)[i].r = 255;
            (*cloud)[i].g = 255;
            (*cloud)[i].b = 255;
            // (*cloud)[i].r = 0;
            // (*cloud)[i].g = 255;
            // (*cloud)[i].b = 0;
        }
        else{
            // (*cloud)[i].r = 0;
            // (*cloud)[i].g = 0;
            // (*cloud)[i].b = 255;
        }
        ret.push_back(boundary);
    }
    std::cout << "len: " << ret.size() << " num " << std::count(ret.begin(), ret.end(), 1) << std::endl;

    std::vector<unsigned long int> boundary_points;
    for(unsigned long int i = 0; i < (*cloud).size(); i++){
        double prob = k_halfdisc * halfdisc_probabilities[i] + k_angle * angle_probabilities[i] + k_shape * shape_probabilities[i];
        if(prob > trashold){
            boundary_points.push_back(i);
        }
    }
    bool change = true;
    std::vector<unsigned long int> points_to_remove;
    std::vector<unsigned long int> changed_points;
    

    std::map<unsigned long int, std::vector<int>> msg_neighbours = get_neighbours(cloud, K, epsilon);
    while(change){
        change = false;
        // double idx1 = -1;
        // double idx2 = -1;
        for(unsigned long int l = 0; l < boundary_points.size(); l++){
            // double idx1 = -1;
            // double idx2 = -1;
            // double angle1 = -1;
            // double angle2 = -1;
            unsigned long int i = boundary_points[l];
            bool ok = true;
            // for(unsigned long int j = 0; j < changed_points.size(); j++){
            //     if(std::find(neighbours[j].begin(), neighbours[j].end(), i) != neighbours[j].end()){
            //         std::cout << "point found" << std::endl;
            //         ok = true;
            //         break;
            //     }
            // }
            if(!ok){
                continue;
            }
            // point
            double d = (*cloud)[i].x;
            double e = (*cloud)[i].y;
            double f = (*cloud)[i].z;
            // normals
            double a = normals->points[i].normal_x;
            double b = normals->points[i].normal_y;
            double c = normals->points[i].normal_z;

            double translation_x = 0;
            double translation_y = 0;
            double translation_z = 0;

            std::vector<Point> transformed_points;
            for(unsigned long int j = 0; j < msg_neighbours[i].size(); j++){
                // if(std::find(boundary_points.begin(), boundary_points.end(), msg_neighbours[i][j]) == boundary_points.end()){
                //     continue;
                // }
                Point p = tangent_projection(normals->points[i], (*cloud)[i], (*cloud)[msg_neighbours[i][j]]);
                double phi = acos(c/sqrt(a*a + b*b + c*c));

                Point nor;
                nor.x = 0;
                nor.y = 0;
                nor.z = 1;
                Point p_r = rotate_point(phi, p, normals->points[i], nor);
                transformed_points.push_back(p_r);

                if((*cloud)[msg_neighbours[i][j]].x == (*cloud)[i].x && (*cloud)[msg_neighbours[i][j]].y == (*cloud)[i].y && (*cloud)[msg_neighbours[i][j]].z == (*cloud)[i].z){
                    translation_x = p_r.x;
                    translation_y = p_r.y;
                    translation_z = p_r.z;
                }
            }
                // translate to origin
            for(Point& p : transformed_points){
                p.x -= translation_x;
                p.y -= translation_y;
                p.z -= translation_z;
            }
            std::vector<double> absolute_angles;
            std::vector<double> absolute_angles_sorted;
            for(unsigned long j = 0; j < msg_neighbours[i].size(); j++){
                double angle = atan2(transformed_points[j].y, transformed_points[j].x) + M_PI;
                if(angle > 2*M_PI){
                    angle = 2*M_PI;
                }
                // if(fabs(angle) > M_PI){
                //     // std::cout << "angle: " << angle << std::endl;
                //     // num1++;
                //     angle = M_PI - (angle - M_PI);
                //     // std::cout << "angle: " << angle << std::endl;
                //     // std::cout << "--------------------" << std::endl;
                // }
                // absolute_angles.push_back(angle + M_PI);
                // absolute_angles_sorted.push_back(angle + M_PI);
                absolute_angles.push_back(angle);
                absolute_angles_sorted.push_back(angle);
            }
            std::sort(absolute_angles_sorted.begin(), absolute_angles_sorted.end());
            double max_angle = 0;
            // double idx1 = -1;
            // double idx2 = -1;
            double idx1 = -1;
            double idx2 = -1;
            double angle1 = -1;
            double angle2 = -1;
            for(unsigned int j = 0; j < absolute_angles_sorted.size(); j++){
                if(fabs(absolute_angles_sorted[j]) > 2*M_PI){
                    // std::cout << "angle: " << max_angle << std::endl;
                    // num4++;
                }
                double a1 = fabs(2*M_PI - fabs(absolute_angles_sorted[j]) + fabs(absolute_angles_sorted[0]));
                a1 = a1 > M_PI ? 2*M_PI - a1 : a1;
                double a2 = fabs(absolute_angles_sorted[j+1] - absolute_angles_sorted[j]);
                a2 = a2 > M_PI ? 2*M_PI - a2 : a2;
                if(j == absolute_angles_sorted.size() - 1){
                    // if(fabs(2*M_PI - fabs(absolute_angles_sorted[j]) - fabs(absolute_angles_sorted[0])) > max_angle){
                    //     // max_angle = abs(absolute_angles_sorted[j] + M_PI - absolute_angles_sorted[0]);
                    //     max_angle = fabs(2*M_PI - abs(absolute_angles_sorted[j]) - abs(absolute_angles_sorted[0]));
                    // if(fabs(2*M_PI - fabs(absolute_angles_sorted[j]) + fabs(absolute_angles_sorted[0])) > max_angle){
                    //     // max_angle = abs(absolute_angles_sorted[j] + M_PI - absolute_angles_sorted[0]);
                    //     max_angle = fabs(2*M_PI - abs(absolute_angles_sorted[j]) + abs(absolute_angles_sorted[0]));
                    if(a1 > max_angle){
                        max_angle = a1;
                        idx1 = j;
                        idx2 = 0;
                        angle1 = absolute_angles_sorted[j];
                        angle2 = absolute_angles_sorted[0];
                        if(fabs(max_angle) > 2*M_PI){
                            // std::cout << "angle: " << max_angle << std::endl;
                            // num3++;
                        }
                    }
                }
                else{
                    // if(fabs(absolute_angles_sorted[j+1] - absolute_angles_sorted[j]) > max_angle){
                    //     max_angle = fabs(absolute_angles_sorted[j+1] - absolute_angles_sorted[j]);
                    if(a2 > max_angle){
                        max_angle = a2;
                        idx1 = j;
                        idx2 = j+1;
                        angle1 = absolute_angles_sorted[j];
                        angle2 = absolute_angles_sorted[j+1];
                        // std::cout << max_angle << std::endl;
                        // std::cout << fabs(max_angle) << std::endl;
                        // std::cout << absolute_angles_sorted[j+1] - absolute_angles_sorted[j] << std::endl;
                        // std::cout << absolute_angles_sorted[j+1] << std::endl;
                        // std::cout << absolute_angles_sorted[j] << std::endl;
                        if(fabs(max_angle) > 2*M_PI){
                            // std::cout << "angle: " << max_angle << std::endl;
                            // num2++;
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
            // gaps.push_back(max_angle);

            auto it1 = find(absolute_angles.begin(), absolute_angles.end(), angle1);
            idx1 = it1 - absolute_angles.begin();
            auto it2 = find(absolute_angles.begin(), absolute_angles.end(), angle2);
            idx2 = it2 - absolute_angles.begin();
            
            // vzít uhly odpovidajici indexum, pro ne najit body, a ty pak dosazovat
            bool ok1 = false;
            bool ok2 = false;
            for(unsigned long int j = 0; j < boundary_points.size(); j++){
                // if(boundary_points[j] == neighbours[l][idx1]){
                if(boundary_points[j] == msg_neighbours[i][idx1]){
                    ok1 = true;
                }
                // if(boundary_points[j] == neighbours[l][idx2]){
                if(boundary_points[j] == msg_neighbours[i][idx2]){
                    ok2 = true;
                }
                if(ok1 && ok2){
                    // std::cout << "oks!" << std::endl;
                    break;
                }
            }
            // if(!(ok1 && ok2)){
            //     change = true;
            //     points_to_remove.push_back(i);
            //     ret[i] = 0;
            // }
            if((ok1 && ok2)){                
            }
            else{
                change = true;
                points_to_remove.push_back(i);
                changed_points.push_back(i);
                ret[i] = 0;
            }
        }

        for(unsigned int j = 0; j < points_to_remove.size(); j++){
            boundary_points.erase(std::remove(boundary_points.begin(), boundary_points.end(), points_to_remove[j]), boundary_points.end());
            // boundary_points.erase(boundary_points.begin() + )
        }
        points_to_remove.clear();
        std::cout << "len: " << ret.size() << " num " << std::count(ret.begin(), ret.end(), 1) << std::endl;
    }
    std::cout << boundary_points.size() << std::endl;

    // weigth creating; idx 0 corresponds to point at boundary_points idx 0
    // std::vector<std::map<int, double>> edge_weights;
    // for(unsigned long int l = 0; l < boundary_points.size(); l++){
    //     unsigned long int i = boundary_points[l];
    //     // (*cloud)[i].r = 255;
    //     // (*cloud)[i].g = 0;
    //     // (*cloud)[i].b = 0;
    //     // std::vector<double> weights;
    //     std::map<int, double> weights;
    //     for(int j = 0; j < neighbours[i].size(); j++){
    //         unsigned long int neighbour = neighbours[i][j];
    //         if(std::find(boundary_points.begin(), boundary_points.end(), neighbour) != boundary_points.end()){
    //             double w_prob = 2 - probabilities[i] - probabilities[neighbour];
    //             // std::cout << "w_prob: " << w_prob;
    //             double w_density = (2 * vect_norm((*cloud)[i], (*cloud)[neighbour])) / (average_distances[i] + average_distances[neighbour]);
    //             // std::cout << " w_density: " << w_density << std::endl;
    //             double weight = w_prob + w_density;
    //             // weights.push_back(weight);
    //             // weights[neighbour].insert(weight);
    //             weights.insert({neighbour, weight});
    //             // if(w_prob < 1.1 && w_density < 1){
    //             //     (*cloud)[i].r = 255;
    //             //     (*cloud)[i].g = 255;
    //             //     (*cloud)[i].b = 0;
    //             // }
    //         }
    //     }
    //     // std::cout << "=== total edges: " << weights.size() << std::endl;
    //     edge_weights.push_back(weights);
    // }

    // std::vector<std::vector<int>> component_forest;
    // std::vector<std::vector<std::pair<int, int>>> forest_edges;
    // for(int l = 0; l < boundary_points.size(); l++){
    //     int i = boundary_points[l];
    //     // find out if point is not in component_fofest
    //     std::vector<int> component;
    //     std::vector<std::pair<int, int>> edges;
    //     std::vector<std::pair<int, int>> edges_queue;
    //     for(auto &j : weights){
    //         edges_queue.push_back({j.first, j.second});
    //     }
    //     component.push_back(i);

    //     while(true){
    //         double min_edge_weight = std::min_element(edges_queue.begin(), edges_queue.end(), [](const auto& p, const auto& q) { return p.second < q.second; });
    //         int edge_idx;
    //         for (auto &j : edge_weights[l]) {
    //             if (j.second == min_edge_weight) {
    //                 edge_idx = j.first;
    //                 break; // to stop searching
    //             }
    //         }
    //         component.push_back(neighbours[i][edge_idx]);
    //         // indexof edge_idx



    //     }




    //     component_forest.push_back(component);
    //     break;
    // }

    // for(int i = 0; i < component_forest.size(); i++){
    //     for(int j = 0; j < component_forest[i].size(); j++){
    //         (*cloud)[i].r = 255;
    //         (*cloud)[i].g = 255;
    //         (*cloud)[i].b = 0;
    //     }
    // }
    
    // return boundary_points;
    // return probabilities;
    return ret;
}


#endif