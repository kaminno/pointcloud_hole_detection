#ifndef BOUNDARY_COHERENCE_H
#define BOUNDARY_COHERENCE_H

#include <vector>

#include "auxiliary_functions.h"
#include "angle_criterion.h"
#include "halfdisc_criterion.h"
#include "shape_criterion.h"

// std::vector<char> get_boundary_points(PointCloud cloud, int K, double epsilon, int K_2, double epsilon_2, double k_angle,
//                                         double k_halfdisc, double k_shape, double trashold){
std::vector<std::vector<int>> get_boundary_points(PointCloud cloud, int K, double epsilon, int K_2, double epsilon_2, double k_angle,
                                        double k_halfdisc, double k_shape, double trashold, double weight_boundary, int minimum_loop_length){

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
    std::vector<int> ret;
    for(unsigned long int i = 0; i < (*cloud).size(); i++){
        double prob = k_halfdisc * halfdisc_probabilities[i] + k_angle * angle_probabilities[i] + k_shape * shape_probabilities[i];
        probabilities.push_back(prob);
        char boundary = prob > trashold ? 1 : 0;
        if(boundary == 1){
            // (*cloud)[i].r = 255;
            // (*cloud)[i].g = 255;
            // (*cloud)[i].b = 255;
            
            // (*cloud)[i].r = 255;
            // (*cloud)[i].g = 0;
            // (*cloud)[i].b = 0;

            // (*cloud)[i].r = 255;
            // (*cloud)[i].g = 50;
            // (*cloud)[i].b = 50;
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
    
    std::map<unsigned long int, std::vector<int>> msg_neighbours = get_neighbours(cloud, K_2, epsilon_2);
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << "again angle criterion" << std::endl;

    // while(change){
    //     change = false;
    //     // double idx1 = -1;
    //     // double idx2 = -1;
    //     for(unsigned long int l = 0; l < boundary_points.size(); l++){
    //         // double idx1 = -1;
    //         // double idx2 = -1;
    //         // double angle1 = -1;
    //         // double angle2 = -1;
    //         unsigned long int i = boundary_points[l];
    //         // bool ok = false;
    //         // for(unsigned long int j = 0; j < changed_points.size(); j++){
    //         //     int m = changed_points[j];
    //         //     // if(std::find(neighbours[j].begin(), neighbours[j].end(), i) != neighbours[j].end()){
    //         //     if(std::find(neighbours[m].begin(), neighbours[m].end(), i) != neighbours[m].end()){
    //         //         std::cout << "point found" << std::endl;
    //         //         ok = true;
    //         //         changed_points.erase(changed_points.begin() + j);
    //         //         break;
    //         //     }
    //         // }
    //         // if(!ok && changed_points.size() > 0){
    //         //     continue;
    //         // }
            
    //         // point
    //         double d = (*cloud)[i].x;
    //         double e = (*cloud)[i].y;
    //         double f = (*cloud)[i].z;
    //         // normals
    //         double a = normals->points[i].normal_x;
    //         double b = normals->points[i].normal_y;
    //         double c = normals->points[i].normal_z;

    //         double translation_x = 0;
    //         double translation_y = 0;
    //         double translation_z = 0;

    //         std::vector<Point> transformed_points;
    //         for(unsigned long int j = 0; j < msg_neighbours[i].size(); j++){
    //             // if(std::find(boundary_points.begin(), boundary_points.end(), msg_neighbours[i][j]) == boundary_points.end()){
    //             //     continue;
    //             // }
    //             Point p = tangent_projection(normals->points[i], (*cloud)[i], (*cloud)[msg_neighbours[i][j]]);
    //             double phi = acos(c/sqrt(a*a + b*b + c*c));

    //             Point nor;
    //             nor.x = 0;
    //             nor.y = 0;
    //             nor.z = 1;
    //             Point p_r = rotate_point(phi, p, normals->points[i], nor);
    //             transformed_points.push_back(p_r);

    //             if((*cloud)[msg_neighbours[i][j]].x == (*cloud)[i].x && (*cloud)[msg_neighbours[i][j]].y == (*cloud)[i].y && (*cloud)[msg_neighbours[i][j]].z == (*cloud)[i].z){
    //                 translation_x = p_r.x;
    //                 translation_y = p_r.y;
    //                 translation_z = p_r.z;
    //             }
    //         }
    //             // translate to origin
    //         for(Point& p : transformed_points){
    //             p.x -= translation_x;
    //             p.y -= translation_y;
    //             p.z -= translation_z;
    //         }
    //         std::vector<double> absolute_angles;
    //         std::vector<double> absolute_angles_sorted;
    //         for(unsigned long j = 0; j < msg_neighbours[i].size(); j++){
    //             double angle = atan2(transformed_points[j].y, transformed_points[j].x) + M_PI;
    //             if(angle > 2*M_PI){
    //                 angle = 2*M_PI;
    //             }
    //             // if(fabs(angle) > M_PI){
    //             //     // std::cout << "angle: " << angle << std::endl;
    //             //     // num1++;
    //             //     angle = M_PI - (angle - M_PI);
    //             //     // std::cout << "angle: " << angle << std::endl;
    //             //     // std::cout << "--------------------" << std::endl;
    //             // }
    //             // absolute_angles.push_back(angle + M_PI);
    //             // absolute_angles_sorted.push_back(angle + M_PI);
    //             absolute_angles.push_back(angle);
    //             absolute_angles_sorted.push_back(angle);
    //         }
    //         std::sort(absolute_angles_sorted.begin(), absolute_angles_sorted.end());
    //         double max_angle = 0;
    //         // double idx1 = -1;
    //         // double idx2 = -1;
    //         double idx1 = -1;
    //         double idx2 = -1;
    //         double angle1 = -1;
    //         double angle2 = -1;
    //         for(unsigned int j = 0; j < absolute_angles_sorted.size(); j++){
    //             if(fabs(absolute_angles_sorted[j]) > 2*M_PI){
    //                 // std::cout << "angle: " << max_angle << std::endl;
    //                 // num4++;
    //             }
    //             double a1 = fabs(2*M_PI - fabs(absolute_angles_sorted[j]) + fabs(absolute_angles_sorted[0]));
    //             a1 = a1 > M_PI ? 2*M_PI - a1 : a1;
    //             double a2 = fabs(absolute_angles_sorted[j+1] - absolute_angles_sorted[j]);
    //             a2 = a2 > M_PI ? 2*M_PI - a2 : a2;
    //             if(j == absolute_angles_sorted.size() - 1){
    //                 // if(fabs(2*M_PI - fabs(absolute_angles_sorted[j]) - fabs(absolute_angles_sorted[0])) > max_angle){
    //                 //     // max_angle = abs(absolute_angles_sorted[j] + M_PI - absolute_angles_sorted[0]);
    //                 //     max_angle = fabs(2*M_PI - abs(absolute_angles_sorted[j]) - abs(absolute_angles_sorted[0]));
    //                 // if(fabs(2*M_PI - fabs(absolute_angles_sorted[j]) + fabs(absolute_angles_sorted[0])) > max_angle){
    //                 //     // max_angle = abs(absolute_angles_sorted[j] + M_PI - absolute_angles_sorted[0]);
    //                 //     max_angle = fabs(2*M_PI - abs(absolute_angles_sorted[j]) + abs(absolute_angles_sorted[0]));
    //                 if(a1 > max_angle){
    //                     max_angle = a1;
    //                     idx1 = j;
    //                     idx2 = 0;
    //                     angle1 = absolute_angles_sorted[j];
    //                     angle2 = absolute_angles_sorted[0];
    //                     if(fabs(max_angle) > 2*M_PI){
    //                         // std::cout << "angle: " << max_angle << std::endl;
    //                         // num3++;
    //                     }
    //                 }
    //             }
    //             else{
    //                 // if(fabs(absolute_angles_sorted[j+1] - absolute_angles_sorted[j]) > max_angle){
    //                 //     max_angle = fabs(absolute_angles_sorted[j+1] - absolute_angles_sorted[j]);
    //                 if(a2 > max_angle){
    //                     max_angle = a2;
    //                     idx1 = j;
    //                     idx2 = j+1;
    //                     angle1 = absolute_angles_sorted[j];
    //                     angle2 = absolute_angles_sorted[j+1];
    //                     // std::cout << max_angle << std::endl;
    //                     // std::cout << fabs(max_angle) << std::endl;
    //                     // std::cout << absolute_angles_sorted[j+1] - absolute_angles_sorted[j] << std::endl;
    //                     // std::cout << absolute_angles_sorted[j+1] << std::endl;
    //                     // std::cout << absolute_angles_sorted[j] << std::endl;
    //                     if(fabs(max_angle) > 2*M_PI){
    //                         // std::cout << "angle: " << max_angle << std::endl;
    //                         // num2++;
    //                         for(unsigned int j = 0; j < absolute_angles_sorted.size(); j++){
    //                             std::cout << "angle: " << absolute_angles_sorted[j] << std::endl;
    //                             if(j != absolute_angles_sorted.size()-1){
    //                                 std::cout << absolute_angles_sorted[j+1] - absolute_angles_sorted[j] << std::endl;
    //                                 std::cout << fabs(absolute_angles_sorted[j+1] - absolute_angles_sorted[j]) << std::endl;
    //                             }
    //                             std::cout << "----------" << std::endl;
    //                         }
    //                         exit(0);
    //                     }
    //                 }
                    
    //             }
    //         }
    //         // if(i == 77)
    //         // std::cout << "max angle: " << max_angle << std::endl;
    //         // if(max_angle > 4){
    //         //     std::cout << "max angle: " << max_angle << std::endl;
    //         // }
    //         // gaps.push_back(max_angle);

    //         auto it1 = find(absolute_angles.begin(), absolute_angles.end(), angle1);
    //         idx1 = it1 - absolute_angles.begin();
    //         auto it2 = find(absolute_angles.begin(), absolute_angles.end(), angle2);
    //         idx2 = it2 - absolute_angles.begin();
            
    //         // vzít uhly odpovidajici indexum, pro ne najit body, a ty pak dosazovat
    //         bool ok1 = false;
    //         bool ok2 = false;
    //         for(unsigned long int j = 0; j < boundary_points.size(); j++){
    //             // if(boundary_points[j] == neighbours[l][idx1]){
    //             if(boundary_points[j] == msg_neighbours[i][idx1]){
    //                 ok1 = true;
    //             }
    //             // if(boundary_points[j] == neighbours[l][idx2]){
    //             if(boundary_points[j] == msg_neighbours[i][idx2]){
    //                 ok2 = true;
    //             }
    //             if(ok1 && ok2){
    //                 // std::cout << "oks!" << std::endl;
    //                 break;
    //             }
    //         }
    //         // if(!(ok1 && ok2)){
    //         //     change = true;
    //         //     points_to_remove.push_back(i);
    //         //     ret[i] = 0;
    //         // }
    //         if((ok1 && ok2)){                
    //         }
    //         else{
    //             change = true;
    //             points_to_remove.push_back(i);
    //             changed_points.push_back(i);
    //             ret[i] = 0;
    //         }
    //     }

    //     for(unsigned int j = 0; j < points_to_remove.size(); j++){
    //         boundary_points.erase(std::remove(boundary_points.begin(), boundary_points.end(), points_to_remove[j]), boundary_points.end());
    //         // boundary_points.erase(boundary_points.begin() + )
    //     }
    //     points_to_remove.clear();
    //     std::cout << "len: " << ret.size() << " num " << std::count(ret.begin(), ret.end(), 1) << std::endl;
    // }

    changed_points = boundary_points;
    std::vector<unsigned long int> points_to_change;
    while(change){
        change = false;
        for(unsigned long int l = 0; l < changed_points.size(); l++){
            unsigned long int i = changed_points[l];
            
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
                // for(int i_ = 0; i_ < neighbours[i].size(); i_++){
                //     if(std::find(boundary_points.begin(), boundary_points.end(), neighbours[i][i_]) != boundary_points.end()){
                //         points_to_change.push_back(neighbours[i][i_]);
                //     }
                // }
                points_to_change.insert(points_to_change.end(), neighbours[i].begin(), neighbours[i].end());
                ret[i] = 0;
            }
        }

        for(unsigned int j = 0; j < points_to_remove.size(); j++){
            boundary_points.erase(std::remove(boundary_points.begin(), boundary_points.end(), points_to_remove[j]), boundary_points.end());
            // boundary_points.erase(boundary_points.begin() + )
        }
        changed_points.clear();
        // std::cout << "changed_points size " << changed_points.size() << std::endl;
        int j_count = 0;
        int if_count = 0;
        for(unsigned int j = 0; j < boundary_points.size(); j++){
            j_count++;
            auto it = std::find(points_to_change.begin(), points_to_change.end(), boundary_points[j]);// - points_to_change.begin();
            auto in = std::find(changed_points.begin(), changed_points.end(), boundary_points[j]);
            if(it != points_to_change.end() && in == changed_points.end()){
                changed_points.push_back(boundary_points[j]);
                if_count++;
            }
            // boundary_points.erase(std::remove(boundary_points.begin(), boundary_points.end(), points_to_remove[j]), boundary_points.end());
            // boundary_points.erase(boundary_points.begin() + )
        }
        std::cout << "j_count: " << j_count << " if_count " << if_count << std::endl;
        // changed_points = points_to_change;
        points_to_change.clear();

        points_to_remove.clear();
        std::cout << "len: " << ret.size() << " ret num " << std::count(ret.begin(), ret.end(), 1) <<
                    " bp " << boundary_points.size() << " next step points " << changed_points.size() << std::endl;
    }

    std::cout << "edges" << std::endl;
    std::map<unsigned long int, std::vector<int>> edges = get_neighbours(cloud, K, epsilon);    // neighbours
    std::vector<int> graph_points;  // boundary points
    for(unsigned long int i = 0; i < (*cloud).size(); i++){
        if(ret[i] == 1){
            graph_points.push_back(i);
        }
    }

    std::cout << "again angle criterion done" << std::endl;
    std::cout << boundary_points.size() << std::endl;
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = end - start;
    // std::cout << "num " << num << std::endl;
    std::cout << "slept for " << fp_ms.count() << " milliseconds." << std::endl;
    std::cout << "weights" << std::endl;
    std::map<int, std::vector<double>> weights;    // weights in the format of neighbours
    for(unsigned long int l = 0; l < graph_points.size(); l++){
        unsigned long int i = graph_points[l];
        for(int j = 0; j < edges[i].size(); j++){
            unsigned long int neighbour = edges[i][j];
            if(std::find(graph_points.begin(), graph_points.end(), neighbour) != graph_points.end()){
                double w_prob = 2 - probabilities[i] - probabilities[neighbour];
                // double w_prob = 0.0;
                // std::cout << "w_prob: " << w_prob;
                double w_density = (2.0 * vect_norm((*cloud)[i], (*cloud)[neighbour])) / (average_distances[i] + average_distances[neighbour]);
                // double w_density = 0.0;
                // std::cout << " w_density: " << w_density << std::endl;
                double weight = w_prob + w_density;
                weights[i].push_back(weight);
            }
        }
    }

    // std::vector<std::vector<int>> edges = get_neighbours(cloud, K, epsilon);    // neighbours
    
    // for(unsigned long int i = 0; i < (*cloud).size(); i++){
    //     // double prob = k_halfdisc * halfdisc_probabilities[i] + k_angle * angle_probabilities[i] + k_shape * shape_probabilities[i];
    //     // if(prob > trashold){
    //     //     graph_points.push_back(i);
    //     // }
    //     if(boundaries[i] == 1){
    //         graph_points.push_back(i);
    //     }
    // }

    // edges to whole graph, meaning for all points which has boundary = 1
    std::vector<Edge> graph;
    std::cout << "graph" << std::endl;
    for(int i = 0; i < graph_points.size(); i++){
        auto edge_from = graph_points[i];
        int edge_num = 0;
        for(int j = 0; j < edges[edge_from].size(); j++){
            auto edge_to = edges[edge_from][j];
            if(std::find(graph_points.begin(), graph_points.end(), edge_to) != graph_points.end()){
                // if(std::find_if(graph.begin(), graph.end(), [edge_from, edge_to](auto edge){ return ( (edge_from == edge.from && edge_to == edge.to) || (edge_from == edge.to && edge_to == edge.from) ); }) == graph.end()){
                    int l = edge_from; //graph_points[i];
                    Edge edge;
                    edge.from = l;
                    edge.to = edges[l][j];
                    if(edge.from == edge.to){
                        continue;
                    }
                    // edge.weight = weights[l][j];
                    edge.weight = weights[l][edge_num];
                    if(weights[l][edge_num] < weight_boundary){
                        graph.push_back(edge);
                    }
                    edge_num++;
                    // graph.push_back(edge);
                // }
            }
        }
    }

    for(int i = 0; i < graph_points.size(); i++){
        auto point = graph_points[i];
        if(std::find_if(graph.begin(), graph.end(), [point](auto edge){ return ( (point == edge.from || point == edge.to) ); }) == graph.end()){
            graph_points.erase(graph_points.begin() + i);
            auto it = std::find(graph_points.begin(), graph_points.end(), point) - graph_points.begin();
            std::cout << "i: " << i << ", it: " << it << std::endl;
            i--;
        }
    }
    std::cout << "graph points: " << graph_points.size() << std::endl;

    std::cout << "sortion" << std::endl;
    std::sort(graph.begin(), graph.end(), [](auto a, auto b){return a.weight < b.weight;});
    Edge min_edge;
    for(Edge edge : graph){
        if(edge.weight < min_edge.weight){
            std::cerr << "ERROR: edges are not sorted!" << std::endl;
        }
        min_edge = edge;
    }
    // list of vectors (for each edge in graph) / ints (for each point in graph)
    std::vector<std::vector<Edge>> components_edges;
    std::vector<std::vector<int>> components_points;
    std::vector<double> components_max_cycle_len;
    std::vector<Edge> components_max_cycle_edge;
    std::cout << "points components" << std::endl;
  	for(int i = 0; i < graph_points.size(); i++){
      	std::vector<int> vc;
        vc.push_back(graph_points[i]);
        components_points.push_back(vc);
	}
    std::cout << "edges components" << std::endl;
    for(int i = 0; i < graph.size(); i++){
        std::vector<Edge> vec;
        components_edges.push_back(vec);
    }
    for(int i = 0; i < graph_points.size(); i++){
        components_max_cycle_len.push_back(0.0);
	}
    for(int i = 0; i < graph_points.size(); i++){
      	Edge e;
        e.from = -1;
        // e.print();
        components_max_cycle_edge.push_back(e);
	}
    std::cout << "MSG" << std::endl;
    std::cout << "Total edges to process: " << components_edges.size() << std::endl;
    std::cout << "Total points to process: " << components_points.size() << std::endl;

    int i = 0;
    int cont = 0;
    int iff = 0;
    int elsee = 0;
    // MST finding
    for(Edge edge : graph){
        i++;
        if(i % 1000 == 0){
            std::cout << "i: " << i << " / " << components_edges.size() << " ? " << graph.size() << std::endl;
        }
        // std::cout << "in" << std::endl;
        int idx_from = get_component_idx(components_points, edge.from);
        // std::cout << "after idx from" << std::endl;
        int idx_to = get_component_idx(components_points, edge.to);
        if(idx_to == -1 || idx_from == -1){
            cont++;
            continue;
        }
        if(idx_from != idx_to){
        	components_points[idx_from].insert(components_points[idx_from].end(), components_points[idx_to].begin(), components_points[idx_to].end());
            // components_points[idx_from].insert(components_points[idx_from].end(), v.begin(), v.end());
            // std::cout << "before erase" << std::endl;
            // std::cout << "components_points.size() " << components_points.size() << std::endl;
            components_points.erase(components_points.begin() + idx_to);
            // std::cout << "components_points.size() " << components_points.size() << std::endl;
            // components_points[idx_to].clear();
            
            // std::cout << "between" << std::endl;
            // if(true){//!components_edges[idx_from].empty()){
            	components_edges[idx_from].push_back(edge);
            	components_edges[idx_from].insert(components_edges[idx_from].end(), components_edges[idx_to].begin(), components_edges[idx_to].end());
                // components_edges[idx_from].insert(components_edges[idx_from].end(), w.begin(), w.end());
                // std::cout << "components_edges.size() " << components_edges.size() << std::endl;
            	components_edges.erase(components_edges.begin() + idx_to);
                // std::cout << "components_edges.size() " << components_edges.size() << std::endl;

                // std::cout << "components_max_cycle_edge.size() " << components_max_cycle_edge.size() << std::endl;
                components_max_cycle_edge.erase(components_max_cycle_edge.begin() + idx_to);
                components_max_cycle_edge[idx_from].from = -1;
                // std::cout << "components_max_cycle_edge.size() " << components_max_cycle_edge.size() << std::endl;
                // std::cout << "components_max_cycle_len.size() " << components_max_cycle_len.size() << std::endl;
                components_max_cycle_len.erase(components_max_cycle_len.begin() + idx_to);
                components_max_cycle_len[idx_from] = 0;
        }
        // else{
        //     elsee++;
        //     // continue;
        //     // if(components_max_cycle_edge[idx_from].from != -1){
        //     //         Edge xtc = components_max_cycle_edge[idx_from];
        //     //         // std::cout << "2" << std::endl;
        //     //         // auto max_it = std::find_if(components_edges[idx_from].begin(), components_edges[idx_from].end(), [xtc](Edge ee){ return ((xtc.from == ee.from && xtc.to == ee.to) || (xtc.to == ee.from && xtc.from == ee.to)); }) - components_edges[idx_from].begin();
        //     //         auto max_it = std::find_if(components_edges[idx_from].begin(), components_edges[idx_from].end(), [xtc](Edge ee){ return (xtc.from == ee.from && xtc.to == ee.to); }) - components_edges[idx_from].begin();

        //     //         components_edges[idx_from].erase(components_edges[idx_from].begin() + max_it);
        //     //     }

        //     double d = 0.0;

        //     for(int i = 0; i < components_edges[idx_from].size(); i++){
        //         int from = components_edges[idx_from][i].from;
        //         int to = components_edges[idx_from][i].to;
        //         // components_edges[idx_from][i].print();
        //         int idx = std::find_if(neighbours_distances[from].begin(), neighbours_distances[from].end(), [to](std::pair<int, double> a) { return a.first == to;}) - neighbours_distances[from].begin();
        //         // std::cout << "idx " << idx << std::endl;
        //         d += neighbours_distances[from][idx].second;
        //         // std::cout << "d " << d << std::endl;
        //     }
        //     d /= components_edges[idx_from].size();
        //     double e = (2 * M_PI * epsilon) / d;
        //     int start = edge.from;
        //     int end = edge.to;
        //     // std::vector<Edge> component_edges = components_edges[idx_from];
        //     std::vector<Edge> visited_edges;
        //     std::queue<int> q;
        //     // std::cout << "--- before q push ---" << std::endl;
        //     q.push(start);
        //     int cou = 0;
        //     while(!q.empty() && cou < 10000){
        //         cou++;
        //         // std::cout << "--- in while ---" << std::endl;
        //         int point = q.front();
        //         // std::cout << "--- after front ---" << std::endl;
        //         if(point == end){
        //             // std::cout << "--- end ---" << std::endl;
        //             break;
        //         }
        //         q.pop();
        //         // std::cout << "--- after pop ---" << std::endl;
        //         for(int j = 0; j < components_edges[idx_from].size(); j++){
        //             // std::cout << "--- in for ---" << std::endl;
        //             Edge comp_edge = components_edges[idx_from][j];
        //             // std::cout << "--- edges[j] ---" << std::endl;
        //             // if(std::find_if(component_edges.begin(), component_edges.end(), [](auto e){ return (e.from == point || e.to == point); }) != component_edges.end()){
        //             if(std::find_if(visited_edges.begin(), visited_edges.end(), [comp_edge](auto e){ return ((comp_edge.from == e.from && comp_edge.to == e.to) || (comp_edge.from == e.to && comp_edge.to == e.from)); }) != visited_edges.end()){
        //                 continue;
        //             }
        //             if(comp_edge.from == point || comp_edge.to == point){
        //                 // std::cout << "--- if ---" << std::endl;
        // //            	q.push(node);
        //                 Edge e;
        //                 e.from = point;
        //                 e.to = comp_edge.from == point ? comp_edge.to : comp_edge.from;
        //                 q.push(e.to);
        //                 // std::cout << "--- after queue push ---" << std::endl;
        //                 visited_edges.push_back(e);
        //                 // std::cout << "--- after visited push ---" << std::endl;
        //             }
        //         }
        //     }
        //     // std::cout << "ok 3" << std::endl;
        //     int length = 0;
        //     int j = 0;
        //     int s = end;
        //     bool f = true;
        //     // std::cout << "--- before while ---" << std::endl;
        //     // std::cout << "visited edges len: " << visited_edges.size() << std::endl;
        //     while(f && j < 100000){
        //         // std::cout << "j: " << j << std::endl;
        //         for(Edge e : visited_edges){
        //             // std::cout << "--- in for ---" << std::endl;
        //             if(e.to == s){
        //                 // std::cout << "--- len++ and break ---" << std::endl;
        //                 length++;
        //                 s = e.from;
        //                 break;
        //             }
        //             if(s == start){
        //                 // std::cout << "--- end and break ---" << std::endl;
        //                 f = false;
        //                 break;
        //             }
        //         }
                
        //         j++;
        //     }

        //     // if(length > e){
        //     //     components_edges[idx_from].push_back(edge);
        //     // }
        //     // if(length > e && length > components_max_cycle_len[idx_from]){
        //     //     components_max_cycle_len[idx_from] = length;
        //     //     components_max_cycle_edge[idx_from].from = edge.from;
        //     //     components_max_cycle_edge[idx_from].to = edge.to;
        //     //     components_max_cycle_edge[idx_from].weight = edge.weight;
        //     // 	components_edges[idx_from].push_back(edge);
        //     // }
        //     // else if(components_max_cycle_edge[idx_from].from != -1){
        //     //     auto sdlfkj = components_max_cycle_edge[idx_from];
        //     //     components_edges[idx_from].push_back(sdlfkj);
        //     // }
        // }
        // std::cout << "----------" << std::endl;
    }

    std::cout << "components_max_cycle_edge.size() " << components_max_cycle_edge.size() << std::endl;
    std::cout << "components_max_cycle_length.size() " << components_max_cycle_len.size() << std::endl;
    std::cout << "components_edges.size() " << components_edges.size() << std::endl;
    std::cout << "finding cycle start" << std::endl;
    // MSG with the biggest cycle
    for(int i = 0; i < graph.size(); i++){
        // std::cout << " in outer " << std::endl;
        Edge edge = graph[i];
        int idx_from = get_component_idx(components_points, edge.from);
        // std::cout << "after idx from" << std::endl;
        int idx_to = get_component_idx(components_points, edge.to);
        if(idx_to == -1 || idx_from == -1){
            cont++;
            continue;
        }
            if(components_max_cycle_edge[idx_from].from != -1){
                    Edge xtc = components_max_cycle_edge[idx_from];
                    // std::cout << "del" << std::endl;
                    // std::cout << "2" << std::endl;
                    // auto max_it = std::find_if(components_edges[idx_from].begin(), components_edges[idx_from].end(), [xtc](Edge ee){ return ((xtc.from == ee.from && xtc.to == ee.to) || (xtc.to == ee.from && xtc.from == ee.to)); }) - components_edges[idx_from].begin();
                    auto max_it = std::find_if(components_edges[idx_from].begin(), components_edges[idx_from].end(), [xtc](Edge ee){ return (xtc.from == ee.from && xtc.to == ee.to); }) - components_edges[idx_from].begin();
                    components_edges[idx_from].erase(components_edges[idx_from].begin() + max_it);
                    // }
                    // std::cout << "4" << std::endl;
                }

            double d = 0.0;

            for(int k = 0; k < components_edges[idx_from].size(); k++){
                int from = components_edges[idx_from][k].from;
                int to = components_edges[idx_from][k].to;
                // components_edges[idx_from][i].print();
                int idx = std::find_if(neighbours_distances[from].begin(), neighbours_distances[from].end(), [to](std::pair<int, double> a) { return a.first == to;}) - neighbours_distances[from].begin();
                // std::cout << "idx " << idx << std::endl;
                // d += neighbours_distances[from][idx].second;
                d += components_edges[idx_from][k].weight;
                // std::cout << "d " << d << std::endl;
            }
            d /= components_edges[idx_from].size();
            double e = (2 * M_PI * epsilon) / d;

            // std::cout << "--- start ---" << std::endl;
            int start = edge.from;
            int end = edge.to;
            // std::vector<Edge> component_edges = components_edges[idx_from];
            std::vector<Edge> visited_edges;
            std::queue<int> q;
            // std::cout << "--- before q push ---" << std::endl;
            q.push(start);
            int cou = 0;
            while(!q.empty() && cou < 10000){
                cou++;
                // std::cout << "--- in while ---" << std::endl;
                int point = q.front();
                // std::cout << "--- after front ---" << std::endl;
                if(point == end){
                    // std::cout << "--- end ---" << std::endl;
                    break;
                }
                q.pop();
                // std::cout << "--- after pop ---" << std::endl;
                for(int k = 0; k < components_edges[idx_from].size(); k++){
                    // std::cout << "--- in for ---" << std::endl;
                    Edge comp_edge = components_edges[idx_from][k];
                    // std::cout << "--- edges[j] ---" << std::endl;
                    // if(std::find_if(component_edges.begin(), component_edges.end(), [](auto e){ return (e.from == point || e.to == point); }) != component_edges.end()){
                    if(std::find_if(visited_edges.begin(), visited_edges.end(), [comp_edge](auto e){ return ((comp_edge.from == e.from && comp_edge.to == e.to) || (comp_edge.from == e.to && comp_edge.to == e.from)); }) != visited_edges.end()){
                        continue;
                    }
                    if(comp_edge.from == point || comp_edge.to == point){
                        // std::cout << "--- if ---" << std::endl;
        //            	q.push(node);
                        Edge e;
                        e.from = point;
                        e.to = comp_edge.from == point ? comp_edge.to : comp_edge.from;
                        q.push(e.to);
                        // std::cout << "--- after queue push ---" << std::endl;
                        visited_edges.push_back(e);
                        // std::cout << "--- after visited push ---" << std::endl;
                    }
                }
            }
            int length = 0;
            int b = 0;
            int s = end;
            bool f = true;
            while(f && b < 100000){
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
                
                b++;
            }

            // if(length > e){
            //     components_edges[idx_from].push_back(edge);
            // }

            if(length > e && length > minimum_loop_length && length > components_max_cycle_len[idx_from]){

                components_max_cycle_len[idx_from] = length;
                components_max_cycle_edge[idx_from].from = edge.from;
                components_max_cycle_edge[idx_from].to = edge.to;
                components_max_cycle_edge[idx_from].weight = edge.weight;
            	components_edges[idx_from].push_back(edge);
            }
            else if(components_max_cycle_edge[idx_from].from != -1){

                auto sdlfkj = components_max_cycle_edge[idx_from];
                components_edges[idx_from].push_back(sdlfkj);
            }
    }
    std::cout << "finding cycle end" << std::endl;

    std::cout << "total points count " << components_points.size() << std::endl;
    std::cout << "edges count " << components_edges[0].size() << std::endl;
    std::cout << "total edges count " << components_edges.size() << std::endl;

    std::vector<int> visited_points;
    std::vector<std::vector<Edge>> boundary_edges;
    std::vector<int> boundary_points_start;
    std::vector<int> boundary_points_end;
    for(int i = 0; i < components_points.size(); i++){

        std::vector<Edge> final_edges;
        auto component_p = components_points[i];
        auto component_e = components_edges[i];
        std::vector<int> colors;
        for(int j = 0; j < component_p.size(); j++){
            colors.push_back(0);
        }

        std::queue<int> q;
        q.push(component_p[0]);
        colors[0] = 1;
        bool f = true;
        int n = 0;
        while(!q.empty() && f){
            auto new_q = q;
            int point = q.front();
            auto itt = std::find(component_p.begin(), component_p.end(), point) - component_p.begin();
            colors[itt] = 2;
            q.pop();
            for(int j = 0; j < component_p.size(); j++){
                auto neigh = component_p[j];
                if(neigh == point){
                    continue;
                }
                    if(std::find_if(component_e.begin(), component_e.end(), [point, neigh](auto e){ return (e.from == point && e.to == neigh) || (e.to == point && e.from == neigh) ; }) != component_e.end()){

                        auto it = j;
                        if(colors[it] == 0){
                            Edge e;
                            e.from = neigh;
                            e.to = point;
                            q.push(e.from);

                            colors[it] = 1;
                            final_edges.push_back(e);
                        }
                        else if(colors[it] == 1){
                            f = false;
                            colors[it] == 2;
                            Edge e;
                            e.from = neigh;
                            e.to = point;
                            final_edges.push_back(e);

                            boundary_points_start.push_back(point);
                            boundary_points_end.push_back(neigh);
                            break;
                        }
                    }
//                 }
            }
            n++;
            if(n == 100000){
                break;
            }
        }
        if(!f){
            boundary_edges.push_back(final_edges);
        }
        
    }

    std::cout << " before last run " << std::endl;
    
    std::cout << " boundary_points_start len " << boundary_points_start.size() << std::endl;
    std::cout << " boundary_points_end len " << boundary_points_end.size() << std::endl;
    std::cout << " boundary_edges len " << boundary_edges.size() << std::endl;
    std::vector<std::vector<Edge>> boundary_e;
    std::vector<std::vector<int>> boundary_p;
    for(int i = 0; i < boundary_points_start.size(); i++){
        auto start = boundary_points_start[i];
        auto end = boundary_points_end[i];
        std::vector<Edge> edgs;
        std::vector<int> pnts;
        int q = 0;
        Edge edg;
        edg.from = start;
        edg.to = end;
        edgs.push_back(edg);
        while(true){
            auto it_s = std::find_if(boundary_edges[i].begin(), boundary_edges[i].end(), [start](Edge c){return c.from == start || c.to == start;}) - boundary_edges[i].begin();
            auto it_e = std::find_if(boundary_edges[i].begin(), boundary_edges[i].end(), [end](Edge c){return c.from == end || c.to == end;}) - boundary_edges[i].begin();

            Edge e_s = boundary_edges[i][it_s];
            // e_s.print();
            pnts.push_back(start);
            edgs.push_back(e_s);
            start = e_s.to;
            
            Edge e_e = boundary_edges[i][it_e];
            // e_e.print();
            pnts.push_back(end);
            edgs.push_back(e_e);
            end = e_e.to;
            
            if(start == end){
                pnts.push_back(start);
                break;
            }
            q++;
            if(q > 10000){
                std::cout << "ERROR: q = 10 000" << std::endl;
                break;
            }
        }
        boundary_e.push_back(edgs);
        boundary_p.push_back(pnts);
    }







    // std::vector<Point> mi_p = compute_mi(cloud, neighbours_distances, average_distances, normals);
    // for(int kk = 0; kk < boundary_p.size(); kk++){
    // for(int i_ = 0; i_ < boundary_p[kk].size(); i_++){
    //     int i = boundary_p[kk][i_];
    //     auto w = [] (Point p, Point q) -> double { 
    //             // if(p.x - q.x == 0 && p.y - q.y == 0 && p.z - q.z == 0){
    //             //     return 0;
    //             // }
    //             // std::cout << "returning " << 1 / vect_norm(p, q) << std::endl;
    //             return 1;// / vect_norm(p, q); 
    //             // return 1.0 / vect_norm(p, q);
    //         };
    //     // for(int i = 0; i < neighbours[point_idx].size(); i++){
    //     double c11 = 0;
    //     double c12 = 0;
    //     double c13 = 0;
    //     double c22 = 0;
    //     double c23 = 0;
    //     double c33 = 0;
    //     Point centroid;
    //     for(int j = 0; j < neighbours[i].size(); j++){
    //         Point q = (*cloud)[neighbours[i][j]];
    //         double d1 = mi_p[i].x - q.x;
    //         double d2 = mi_p[i].y - q.y;
    //         double d3 = mi_p[i].z - q.z;
    //         // double d1 = (*cloud)[i].x - q.x;
    //         // double d2 = (*cloud)[i].y - q.y;
    //         // double d3 = (*cloud)[i].z - q.z;
    //         double wq = w((*cloud)[i], q);
    //         c11 += wq*d1*d1;
    //         c12 += wq*d1*d2;
    //         c13 += wq*d1*d3;
    //         c22 += wq*d2*d2;
    //         c23 += wq*d2*d3;
    //         c33 += wq*d3*d3;
    //     }
    //     c11 /= static_cast <double> (neighbours[i].size());
    //     c12 /= static_cast <double> (neighbours[i].size());
    //     c13 /= static_cast <double> (neighbours[i].size());
    //     c22 /= static_cast <double> (neighbours[i].size());
    //     c23 /= static_cast <double> (neighbours[i].size());
    //     c33 /= static_cast <double> (neighbours[i].size());
    //     Eigen::MatrixXd C_p (3, 3);
    //     C_p << c11, c12, c13, c12, c22, c23, c13, c23, c33;
    //     Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es;
    //     // Eigen::EigenSolver<Eigen::MatrixXd> es;
    //     es.compute(C_p, /* computeEigenvectors = */ false);
    //     double l1 = es.eigenvalues()[2];
    //     double l2 = es.eigenvalues()[1];
    //     double l3 = es.eigenvalues()[0];
    //     if(l1 < l2 || l1 < l3 || l2 < l3){
    //         std::cout << l1 << ", " << l2 << ", " << l3 << std::endl;
    //         std::cerr << "ERROR: eigenvalues are sort wrongly" << std::endl;
    //         exit(100);
    //     }
    //     // std::cout << l1 << ", " << l2 << ", " << l3 << std::endl;
    //     double alpha = l1 + l2 + l3;
    //     // std::cout << alpha << std::endl;
    //     Point A_p;
    //     A_p.x = l1 / alpha;
    //     A_p.y = l2 / alpha;
    //     A_p.z = l3 / alpha;
        
    //     ros::NodeHandle nh;
    // // mrs_lib::ParamLoader pl(nh, node_name);
	// unsigned int queue_length = 1000;
    // ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", queue_length);
    // visualization_msgs::MarkerArray markerArray;

    // Point A_boundary;
    // Point A_interior;
    // Point A_corner_noise;
    // Point A_line;
    // Point A_ridge;
    // A_boundary.x = 2.0/3.0;
    // A_boundary.y = 1.0/3.0;
    // A_boundary.z = 0.0;
    // A_interior.x = 1.0/2.0;
    // A_interior.y = 1.0/2.0;
    // A_interior.z = 0.0;
    // A_corner_noise.x = 1.0/3.0;
    // A_corner_noise.y = 1.0/3.0;
    // A_corner_noise.z = 1.0/3.0;
    // A_line.x = 1.0;
    // A_line.y = 0.0;
    // A_line.z = 0.0;
    // A_ridge.x = 1.0/2.0;
    // A_ridge.y = 1.0/4.0;
    // A_ridge.z = 1.0/4.0;
    // // A_boundary.x = 0.66;
    // // A_boundary.y = 0.33;
    // // A_boundary.z = 0;
    // // A_interior.x = 0.5;
    // // A_interior.y = 0.5;
    // // A_interior.z = 0;
    // // A_corner_noise.x = 0.33;
    // // A_corner_noise.y = 0.33;
    // // A_corner_noise.z = 0.33;
    // // A_line.x = 1;
    // // A_line.y = 0;
    // // A_line.z = 0;
    // // A_ridge.x = 0.5;
    // // A_ridge.y = 0.25;
    // // A_ridge.z = 0.25;

    // // std::cout << A_boundary.x << std::endl;
    // // std::cout << A_boundary.y << std::endl;
    // // std::cout << A_boundary.z << std::endl;
    // // std::cout << A_interior.x << std::endl;
    // // std::cout << A_interior.y << std::endl;
    // // std::cout << A_interior.z << std::endl;
    // // std::cout << A_corner_noise.x << std::endl;
    // // std::cout << A_corner_noise.y << std::endl;
    // // std::cout << A_corner_noise.z << std::endl;
    // // std::cout << A_line.x << std::endl;
    // // std::cout << A_line.y << std::endl;
    // // std::cout << A_line.z << std::endl;
    // // std::cout << A_ridge.x << std::endl;
    // // std::cout << A_ridge.y << std::endl;
    // // std::cout << A_ridge.z << std::endl;
    // // std::cout << "------------------------" << std::endl;
    

    // Point c;
    // // c.x = (4*A_p.x - A_boundary.x - A_interior.x - A_corner_noise.x - A_line.x) / 4;
    // // c.y = (4*A_p.y - A_boundary.y - A_interior.y - A_corner_noise.y - A_line.y) / 4;
    // // c.z = (4*A_p.z - A_boundary.z - A_interior.z - A_corner_noise.z - A_line.z) / 4;
    // // c.x = (3*A_p.x - A_interior.x - A_corner_noise.x - A_line.x) / 3;
    // // c.y = (3*A_p.y - A_interior.y - A_corner_noise.y - A_line.y) / 3;
    // // c.z = (3*A_p.z - A_interior.z - A_corner_noise.z - A_line.z) / 3;
    // c.x = (A_interior.x + A_corner_noise.x + A_line.x) / 3.0;
    // c.y = (A_interior.y + A_corner_noise.y + A_line.y) / 3.0;
    // c.z = (A_interior.z + A_corner_noise.z + A_line.z) / 3.0;
    // // c.x = (A_boundary.x + A_interior.x + A_corner_noise.x + A_line.x) / 4;
    // // c.y = (A_boundary.y + A_interior.y + A_corner_noise.y + A_line.y) / 4;
    // // c.z = (A_boundary.z + A_interior.z + A_corner_noise.z + A_line.z) / 4;
    // // c.x = (A_boundary.x + A_interior.x + A_corner_noise.x + A_line.x + A_p.x) / 5;
    // // c.y = (A_boundary.y + A_interior.y + A_corner_noise.y + A_line.y + A_p.y) / 5;
    // // c.z = (A_boundary.z + A_interior.z + A_corner_noise.z + A_line.z + A_p.z) / 5;

    // visualization_msgs::Marker marker;
    // marker.header.frame_id = "base_frame";
    // marker.header.stamp = ros::Time::now();
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.id = 10*i + 1;
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
    // marker2.id = 10*i + 2;
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
    // marker3.id = 10*i + 3;
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
    // marker4.id = 10*i + 4;
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
    // marker5.id = 10*i + 5;
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
    // marker6.id = 10*i + 6;
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
    // marker7.id = 10*i + 7;
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

    // marker_pub.publish(markerArray);
    // }}

    // return ret;
    return boundary_p;
}


#endif