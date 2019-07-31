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


int main(int argc, char *argv[]) {
    
// configure parameters
typedef std::chrono::microseconds Ttim;


    // 1) generate points
    uint_t numPlanes = 4, numPoses = 3, numPoints = 400, N = 20;
    double pointNoiseStd = 0.01, noiseBias = 0.2, alpha=0.2, beta = 0.7;


    // results stored on vectors for extraction of statistics later
    std::vector<std::vector<double>> res_alg1, res_alg2;
    
    

std::cout << "#Results ofganized as vectors as, for each we provide median, 0.25 percentile and 0.75:\n#[ALgId[0], Nplanes[1], Nposes[2], N[3], iters[4],  time[7],  IniError[10] ,   error[13], trajErrorIni[16], trajError[19]]";

for (numPoses = 2; numPoses < 41; numPoses +=2 )
{
  
  res_alg1.resize(6, std::vector<double>());
  res_alg2.resize(6, std::vector<double>());
  //for (noiseBias = 0.0; noiseBias < 0.5 ; noiseBias+=0.01 )    
  //for (numPoints = 400; numPoints < 8000; numPoints*=2 )
  {
  for (numPlanes = 3; numPlanes < 6; numPlanes++ )
  {
    for (uint_t i = 0; i < N; ++i)
    {
    
        auto t1 = std::chrono::steady_clock::now();
        mrob::CreatePoints scene(numPoints,numPlanes,numPoses,pointNoiseStd,noiseBias);
        auto t2 = std::chrono::steady_clock::now();
        auto dif = std::chrono::duration_cast<Ttim>(t2 - t1);
        
        
        // Method 1: Naive Gradient descent with fixed stepsize, for 2,3,4 poses doe not work, fix it!
        {
            mrob::PlaneRegistration planeRegistration;
            scene.create_plane_registration(planeRegistration);
            planeRegistration.set_solving_method(mrob::PlaneRegistration::SolveMode::BENGIOS_NAG);
            planeRegistration.set_alpha_parameter(alpha);
            planeRegistration.set_beta_parameter(beta);

             // for 2 poses regular params dont work well- > fix it
            if (numPoses == 2)
            {
                planeRegistration.set_beta_parameter(0.0);
            }
            
            t1 = std::chrono::steady_clock::now();
            planeRegistration.solve_initialize();
            t2 = std::chrono::steady_clock::now();
            dif = std::chrono::duration_cast<Ttim>(t2 - t1);
            res_alg1[2].push_back( planeRegistration.get_current_error()/numPoints/numPoses);
            res_alg1[4].push_back( planeRegistration.calculate_poses_rmse(scene.get_ground_truth_trajectory()) );
            
            t1 = std::chrono::steady_clock::now();
            uint_t numberIterations = planeRegistration.solve_interpolate();
            t2 = std::chrono::steady_clock::now();
            dif = std::chrono::duration_cast<Ttim>(t2 - t1);
            
            // save statistics
            res_alg1[0].push_back( (double)numberIterations);
            res_alg1[1].push_back( dif.count());
            res_alg1[3].push_back( planeRegistration.get_current_error()/numPoints/numPoses);
            res_alg1[5].push_back( planeRegistration.calculate_poses_rmse(scene.get_ground_truth_trajectory()) );
        }
        
        // Method 2: Naive Gradient descent with fixed stepsize
        {
            mrob::PlaneRegistration planeRegistration;
            scene.create_plane_registration(planeRegistration);
            planeRegistration.set_solving_method(mrob::PlaneRegistration::SolveMode::GRADIENT_DESCENT_NAIVE);
            planeRegistration.set_alpha_parameter(alpha);

            
            t1 = std::chrono::steady_clock::now();
            planeRegistration.solve_initialize();
            t2 = std::chrono::steady_clock::now();
            dif = std::chrono::duration_cast<Ttim>(t2 - t1);
            res_alg2[2].push_back( planeRegistration.get_current_error()/numPoints/numPoses);
            res_alg2[4].push_back( planeRegistration.calculate_poses_rmse(scene.get_ground_truth_trajectory()) );
            
            t1 = std::chrono::steady_clock::now();
            uint_t numberIterations = planeRegistration.solve_interpolate();
            t2 = std::chrono::steady_clock::now();
            dif = std::chrono::duration_cast<Ttim>(t2 - t1);
            
            // save statistics
            res_alg2[0].push_back( (double)numberIterations);
            res_alg2[1].push_back( dif.count());
            res_alg2[3].push_back( planeRegistration.get_current_error()/numPoints/numPoses);
            res_alg2[5].push_back( planeRegistration.calculate_poses_rmse(scene.get_ground_truth_trajectory()) );
        }
        
        
       
    }//end for
  }
  }
  // process results,
  std::cout  << "\n1, " << numPlanes << ", " << numPoses << ", " << numPoints;
  print_statistics(res_alg1);
  std::cout  << "\n2, " << numPlanes << ", " << numPoses << ", " << numPoints;
  print_statistics(res_alg2);
}
    return 0;
}
