/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * syntheticOptimizationEval.cpp
 *
 *  Created on: Feb 21, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <algorithm>

#include "Open3D/Open3D.h"

#include "mrob/create_points.hpp"
#include "mrob/plane_registration.hpp"


void print_statistics(std::vector<std::vector<double>> &res)
{
    uint_t N = res[0].size();
    for (uint_t i = 0; i < res.size() ; ++i)
    {
        std::sort( res[i].begin(), res[i].end() );
        uint_t medianIndex = N/2, loIndex = N/4, hiIndex = N-N/4;
        std::cout  << ", " << res[i][medianIndex] << ", " << res[i][loIndex] << ", " << res[i][hiIndex];
    }
}


void fill_point_cloud(mrob::CreatePoints &scene, std::shared_ptr<open3d::geometry::PointCloud> pointcloud, uint_t t, Mat31 color)
{
    pointcloud->Clear();
    auto points = scene.get_point_cloud(t);
    // fill open3d message
    for (auto p : points)
    {
        pointcloud->points_.push_back(p);
        pointcloud->colors_.push_back(color);
    }
}


int main(int argc, char *argv[]) {
    
// configure parameters
typedef std::chrono::microseconds Ttim;


    // 1) generate points
    uint_t numPlanes = 3, numPoses = 3, numPoints = 400, N = 20;
    double pointNoiseStd = 0.01, noiseBias = 0.2, alpha=0.2, beta = 0.7;



std::cout << "# Per experiment: Initial_error[0], error_alg1[1],error_alg2[2],error_alg3[3]";

for (numPoses = 2; numPoses < 41; numPoses +=2 )
{
  
  for (numPoints = 400; numPoints < 8000; numPoints*=2 )
  {
  for (numPlanes = 3; numPlanes < 6; numPlanes++ )
  {
    for (uint_t i = 0; i < N; ++i)
    {
    
        auto t1 = std::chrono::steady_clock::now();
        mrob::CreatePoints scene(numPoints,numPlanes,numPoses,pointNoiseStd,noiseBias);
        auto t2 = std::chrono::steady_clock::now();
        auto dif = std::chrono::duration_cast<Ttim>(t2 - t1);
        
        mrob::SE3 T_ini;
        
        // Method 1: Naive Gradient descent with fixed stepsize, for 2,3,4 poses doe not work, fix it!
        {
            mrob::PlaneRegistration planeRegistration;
            scene.create_plane_registration(planeRegistration);
            //planeRegistration.set_solving_method(mrob::PlaneRegistration::SolveMode::GRADIENT_DESCENT_NAIVE);
            planeRegistration.set_solving_method(mrob::PlaneRegistration::SolveMode::BENGIOS_NAG);
            planeRegistration.set_alpha_parameter(alpha);
            planeRegistration.set_beta_parameter(beta);
            
            // for 2 poses regular params dont work well- > fix it
            if (numPoses == 2)
            {
                planeRegistration.set_alpha_parameter(0.1);
                planeRegistration.set_beta_parameter(0.0);
            }
            
            t1 = std::chrono::steady_clock::now();
            planeRegistration.solve_initialize();
            // for later ICP initialization
            T_ini = planeRegistration.get_transformations()->back();
            std::cout << "\n" << scene.get_ground_truth_trajectory().back().distance(planeRegistration.get_transformations()->back());
            
            uint_t numberIterations = planeRegistration.solve_interpolate();
            t2 = std::chrono::steady_clock::now();
            dif = std::chrono::duration_cast<Ttim>(t2 - t1);
            
            std::cout << ", " << scene.get_ground_truth_trajectory().back().distance(planeRegistration.get_transformations()->back());
        }
        
        // Method 2: ICP with some variants
        {
            // create point clouds            
            auto sourcePc = std::make_shared<open3d::geometry::PointCloud>();
            auto targetPc = std::make_shared<open3d::geometry::PointCloud>();
            fill_point_cloud(scene, targetPc, 0, Mat31(0,0,1.0));
            fill_point_cloud(scene, sourcePc, numPoses-1, Mat31(0.95,0,0));
            
            t1 = std::chrono::steady_clock::now();
            
              
            sourcePc->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
            targetPc->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
            auto result = open3d::registration::RegistrationICP(*sourcePc , *targetPc, 0.25, T_ini.T(), open3d::registration::TransformationEstimationPointToPlane());
            t2 = std::chrono::steady_clock::now();
            dif = std::chrono::duration_cast<Ttim>(t2 - t1);
            
            mrob::SE3 T_res(Mat4(result.transformation_));
            // save statistics
            std::cout << ", " << scene.get_ground_truth_trajectory().back().distance(T_res);
        
        
        
        // Method 3: ICP planes
        
            t1 = std::chrono::steady_clock::now();
            auto result2 = open3d::registration::RegistrationICP(*sourcePc , *targetPc, 0.05, T_ini.T(), open3d::registration::TransformationEstimationPointToPoint(true));
            auto result2 = open3d::RegistrationICP(*sourcePc , *targetPc, 0.05, T_ini.T(), open3d::TransformationEstimationPointToPoint(true));
            t2 = std::chrono::steady_clock::now();
            dif = std::chrono::duration_cast<Ttim>(t2 - t1);
            
            mrob::SE3 T_res2(Mat4(result2.transformation_));
            // save statistics
            std::cout << ", " << scene.get_ground_truth_trajectory().back().distance(T_res2);
        }
       
    }//end for
  }
  }
}
    return 0;
}
