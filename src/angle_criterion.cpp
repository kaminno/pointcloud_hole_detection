// standard libraries
#include <iostream>
#include <vector>
// #include <math.h>
#include <cmath>

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

// pcl::PointCloud<pcl::Normal>::Ptr get_normal_vectors(PointCloud cloud, unsigned int K, double epsilon){
//     std::cout << "get_normal_vectors" << std::endl;
//     pcl::NormalEstimation<Point, pcl::Normal> ne;
//     pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());
//     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//     auto start = std::chrono::high_resolution_clock::now();

//     ne.setInputCloud (cloud);
//     ne.setSearchMethod (tree);
//     // ne.setKSearch(5*K);
//     ne.setKSearch(10);
//     // ne.setRadiusSearch(5*epsilon);
//     ne.compute (*cloud_normals);

//     auto end = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double, std::milli> fp_ms = end - start;
//     std::cout << "slept for " << fp_ms.count() << " milliseconds." << std::endl;
//     std::cout << "get_normal_vectors done" << std::endl;

//     return cloud_normals;
// }

// Point tangent_projection(pcl::Normal normal_vector, Point plane_point, Point point){
//     double numerator = (normal_vector.normal_x*plane_point.x - normal_vector.normal_x*point.x + normal_vector.normal_y*plane_point.y - normal_vector.normal_y*point.y + normal_vector.normal_z*plane_point.z - normal_vector.normal_z*point.z);
//     double denominator = (normal_vector.normal_x*normal_vector.normal_x + normal_vector.normal_y*normal_vector.normal_y + normal_vector.normal_z*normal_vector.normal_z);
//     double t = numerator / denominator;
//     Point p;
//     p.x = point.x + normal_vector.normal_x*t;
//     p.y = point.y + normal_vector.normal_y*t;
//     p.z = point.z + normal_vector.normal_z*t;

//     return p;
// }

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
            double angle = atan2(transformed_points[j].y, transformed_points[j].x);
            if(fabs(angle) > M_PI){
                // std::cout << "angle: " << angle << std::endl;
                num1++;
                angle = M_PI - (angle - M_PI);
                // std::cout << "angle: " << angle << std::endl;
                // std::cout << "--------------------" << std::endl;
            }
            absolute_angles.push_back(angle + M_PI);
            absolute_angles_sorted.push_back(angle + M_PI);
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
            if(j == absolute_angles_sorted.size() - 1){
                if(fabs(2*M_PI - fabs(absolute_angles_sorted[j]) - fabs(absolute_angles_sorted[0])) > max_angle){
                    // max_angle = abs(absolute_angles_sorted[j] + M_PI - absolute_angles_sorted[0]);
                    max_angle = fabs(2*M_PI - abs(absolute_angles_sorted[j]) - abs(absolute_angles_sorted[0]));
                    idx1 = j;
                    idx2 = 0;
                    if(fabs(max_angle) > 2*M_PI){
                        // std::cout << "angle: " << max_angle << std::endl;
                        num3++;
                    }
                }
            }
            else{
                if(fabs(absolute_angles_sorted[j+1] - absolute_angles_sorted[j]) > max_angle){
                    max_angle = fabs(absolute_angles_sorted[j+1] - absolute_angles_sorted[j]);
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

std::vector<double> get_probabilities(std::vector<double> gaps, std::map<unsigned long int, std::vector<int>> neighbours){
    std::cout << "get_probabilities" << std::endl;
    std::vector<double> probs;
    auto start = std::chrono::high_resolution_clock::now();
    for(unsigned long int i = 0; i < gaps.size(); i++){
        double not_prob = ( gaps[i] - (2 * M_PI / neighbours[i].size())) / (M_PI - (2 * M_PI / neighbours[i].size()));
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
std::vector<double> angle_criterion(PointCloud cloud, unsigned int K, double epsilon){
    std::cout << "========== ANGLE CRITERION ==========" << std::endl;
    // unsigned int K = 20;
    // double epsilon = 0.1;
    // compute normals
    pcl::PointCloud<pcl::Normal>::Ptr normals = get_normal_vectors(cloud, K, epsilon);
    // compute neighbours
    std::map<unsigned long int, std::vector<int>> graph = get_neighbours(cloud, K, epsilon);

    std::vector<double> angle_gaps = get_angle_gaps(cloud, normals, graph);
    // for(unsigned long int i = 0; i < (*cloud).size(); i++){
    //     double gap = get_angle_gaps();
    //     angle_gaps.push_back(gap);
    // }

    //std::cout << "criterion angle " << angle_gaps[77] << std::endl;

    std::vector<double> probabilities = get_probabilities(angle_gaps, graph);

    std::cout << "========== END ==========" << std::endl;

    return probabilities;
}