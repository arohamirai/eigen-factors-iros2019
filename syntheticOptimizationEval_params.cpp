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

#include "mrob/create_points.hpp"
#include "mrob/plane_registration.hpp"

/**
 * This program is a testbest to play arouind with parameters and select "best" alpha and beta.
 * some lines have to be uncommented, for parameter sweeping.
 */ 

int main(int argc, char *argv[]) {
    
// configure parameters
typedef std::chrono::microseconds Ttim;


    // 1) generate points
    uint_t numPlanes = 3, numPoses = 4, numPoints = 400, N = 20, numAlg = 20;
    double pointNoiseStd = 0.01, noiseBias = 0.01, alpha=0.2, beta = 0.0;

    //matrix for storing results, row #experiment, colum test (i.e.:
    //             0,    1,        2,     3,    4
    //         iters,error,trajError, fails, time
    MatX results = MatX::Zero(numAlg,9);

//for (noiseBias = 0.0; noiseBias < 0.5 ; noiseBias+=0.05 )    
//for (numPoints = 50; numPoints < 8000; numPoints*=2 )
//for (numPoses = 17; numPoses < 20; numPoses+=2 )
//for (numPlanes = 3; numPlanes < 6; numPlanes++ )
{
    for (uint_t i = 0; i < N; ++i)
    {
    
        auto t1 = std::chrono::steady_clock::now();
        mrob::CreatePoints scene(numPoints,numPlanes,numPoses,pointNoiseStd,noiseBias);
        auto t2 = std::chrono::steady_clock::now();
        auto dif = std::chrono::duration_cast<Ttim>(t2 - t1);
        
        // results for Grad /N, but it follows an exact 0.2/t shape, except for 2-3 poses
        // poses(alpha)  2(0.04) 4(0.05)  6(0.03) 8(0.03)  12(0.02) 16(0.012)
        //    20(0.016) 24(0.01) 28(0.012) 32(0.01) 36(0.005) 40(.01)
        // Method 0: Naive Gradient descent with fixed stepsize 0.6
        uint_t algId = 0;
        //for (alpha = 0.01; alpha < 3.5 && algId < numAlg; alpha +=0.02)
        for (beta = 0.00; beta < 1.0 && algId < numAlg; beta +=0.1)
        {
            mrob::PlaneRegistration planeRegistration;
            scene.create_plane_registration(planeRegistration);
            //planeRegistration.set_solving_method(mrob::PlaneRegistration::SolveMode::GRADIENT_DESCENT_NAIVE);
            planeRegistration.set_solving_method(mrob::PlaneRegistration::SolveMode::BENGIOS_NAG);
            planeRegistration.set_alpha_parameter(alpha);
            planeRegistration.set_beta_parameter(beta);

            
            t1 = std::chrono::steady_clock::now();
            planeRegistration.solve_initialize();
            t2 = std::chrono::steady_clock::now();
            dif = std::chrono::duration_cast<Ttim>(t2 - t1);
            
            results(algId,1) += dif.count();
            results(algId,3) += planeRegistration.get_current_error();
            t1 = std::chrono::steady_clock::now();
            uint_t numberIterations = planeRegistration.solve_interpolate();
            t2 = std::chrono::steady_clock::now();
            dif = std::chrono::duration_cast<Ttim>(t2 - t1);
            
            // save statistics
            results(algId,0) += numberIterations;
            results(algId,2) += dif.count();
            results(algId,4) += planeRegistration.get_current_error();
            results(algId,5) += planeRegistration.calculate_poses_rmse(scene.get_ground_truth_trajectory());
            results(algId,6) += numberIterations==1e3 ? 1.0 : 0.0;
            results(algId,7) += alpha;
            results(algId,8) += beta;
            algId++;
        }
       
    }//end for
    // process results
    results /= N;
    std::cout << "Average results for planes= " << numPlanes << ", poses= " << numPoses << ", number of points = " 
              << numPoints << ", std = " << pointNoiseStd << ", bias = " << noiseBias
              << " \n  [  iters,  timeIni,   time,  IniError ,   error,  trajError,   fails, alpha,  beta]\n" 
              << results << std::endl;
    results.setZero();// for next batch of evaluations
}
    return 0;
}
