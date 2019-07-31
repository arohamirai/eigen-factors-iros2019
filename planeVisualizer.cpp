/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * planeVisualizer.cpp
 *
 *  Created on: Feb 16, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

#include <Core/Core.h>
#include <IO/IO.h>
#include <Visualization/Visualization.h>

#include "mrob/create_points.hpp"
#include "mrob/plane_registration.hpp"
#include "mrob/SE3.hpp"


void fill_point_cloud(mrob::CreatePoints &scene, std::shared_ptr<open3d::PointCloud> pointcloud, uint_t t, Mat31 color)
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

void fill_point_cloud_pose(mrob::CreatePoints &scene, mrob::PlaneRegistration &planeReg, std::shared_ptr<open3d::PointCloud> pointcloud)
{
    Mat31 colorIni(1.0,0.0,0.0), colorEnd(0.0,0.0,0.8);
    double cont_p = 0.0;
    pointcloud->Clear();
    
    // fill open3d message
    for (uint_t t = 0;  t < scene.get_number_poses() ; ++t)
    {
        // get point cloud at time t
        auto points = scene.get_point_cloud(t);
        Mat31 color = (1.0 - cont_p) * colorIni + cont_p * colorEnd;
        // get transformation
        mrob::SE3 T = planeReg.get_transformations()->at(t);
        for (auto rawPoint : points)
        {
            Mat31 p = T.transform(rawPoint);
            pointcloud->points_.push_back(p);
            pointcloud->colors_.push_back(color);
        }
        cont_p += 1.0/((double)scene.get_number_poses()-1);
    }
}


void create_trajectory(std::vector<mrob::SE3> &traj, std::shared_ptr<open3d::TriangleMesh> &markers, double size)
{
    markers->Clear();
    for ( mrob::SE3& pose: traj)
    {
        auto mesh = open3d::CreateMeshCoordinateFrame(size);
        mesh->Transform(pose.T());
        *markers += *mesh;
    }
}

int main(int argc, char *argv[]) {
    
    typedef std::chrono::microseconds Ttim;

    open3d::SetVerbosityLevel(open3d::VerbosityLevel::VerboseAlways);

    // generate points from mrob generate_planes
    uint_t numPlanes = 3, numPoses = 10, numPoints = 400;
    double pointNoiseStd = 0.01, noiseBias = 0.1;
    mrob::CreatePoints scene(numPoints,numPlanes,numPoses,pointNoiseStd,noiseBias);
    //scene.print();
    
    double alpha = 0.2;
    // generate continious plane regsitration
    mrob::PlaneRegistration planeRegistration;
    scene.create_plane_registration(planeRegistration);
    planeRegistration.set_solving_method(mrob::PlaneRegistration::SolveMode::BENGIOS_NAG);
    planeRegistration.set_alpha_parameter(alpha);
    planeRegistration.set_beta_parameter(0.7);
    // NAG does not work so well for the case of 2 poses. TODO fix it
    if (numPoses == 2)
        planeRegistration.set_beta_parameter(0.0);
    

    // fill pointcloud with current palnes data
    auto pointcloud = std::make_shared<open3d::PointCloud>();
    auto t1 = std::chrono::steady_clock::now();
    fill_point_cloud_pose(scene, planeRegistration, pointcloud);
    auto t2 = std::chrono::steady_clock::now();
    auto dif = std::chrono::duration_cast<Ttim>(t2 - t1);
    std::cout << "Time spent on building PC " << dif.count() << std::endl;
    
    // calcualte a solution and plot it
    auto traj = std::make_shared<open3d::TriangleMesh>();
    auto trajGT = std::make_shared<open3d::TriangleMesh>();
    create_trajectory(scene.get_ground_truth_trajectory(), trajGT,0.1);
    create_trajectory(*planeRegistration.get_transformations(), traj,0.5);
    open3d::DrawGeometries({pointcloud,traj,trajGT}, "PointCloud", 1600, 900);
    
    // initialize solution
    planeRegistration.solve_initialize();
    mrob::SE3 T_ini = planeRegistration.get_transformations()->back();
    fill_point_cloud_pose(scene, planeRegistration, pointcloud);
    create_trajectory(*planeRegistration.get_transformations(), traj,0.5);
    open3d::DrawGeometries({pointcloud,traj,trajGT}, "PointCloud", 1600, 900);
    for ( uint_t i = 0; i < 100; ++i)
    {
        t1 = std::chrono::steady_clock::now();
        uint_t numberIterations = planeRegistration.solve_interpolate(true);
        t2 = std::chrono::steady_clock::now();
        dif = std::chrono::duration_cast<Ttim>(t2 - t1);
        std::cout << "Time spent on calculating a solution " << dif.count() 
                  << ", number of iterations: " << numberIterations 
                  << ", estimated error = " << planeRegistration.get_current_error() 
                  << ", trajectory error = "<< planeRegistration.calculate_poses_rmse(scene.get_ground_truth_trajectory()) << std::endl;
        if ((i+1)%100 == 0)
        {
            fill_point_cloud_pose(scene, planeRegistration, pointcloud);
            create_trajectory(*planeRegistration.get_transformations(), traj,0.5);
            open3d::DrawGeometries({pointcloud,traj,trajGT}, "PointCloud", 1600, 900);
        }
    }

// a second method to compare against
if (0)
{
    open3d::PrintInfo("New optimization method.\n");
    mrob::PlaneRegistration planeRegistrationM2;
    scene.create_plane_registration(planeRegistrationM2);
    planeRegistrationM2.set_solving_method(mrob::PlaneRegistration::SolveMode::GRADIENT_DESCENT_NAIVE);
    planeRegistrationM2.set_alpha_parameter(alpha);
    planeRegistrationM2.set_beta_parameter(0.7);
    
    
    fill_point_cloud_pose(scene, planeRegistrationM2, pointcloud);
    create_trajectory(*planeRegistrationM2.get_transformations(), traj,0.5);
    open3d::DrawGeometries({pointcloud,traj,trajGT}, "PointCloud", 1600, 900);
    planeRegistrationM2.solve_initialize();
    for ( uint_t i = 0; i < 100; ++i)
    {
        t1 = std::chrono::steady_clock::now();
        uint_t numberIterations = planeRegistrationM2.solve_interpolate(true);
        t2 = std::chrono::steady_clock::now();
        dif = std::chrono::duration_cast<Ttim>(t2 - t1);
        std::cout << "Time spent on calculating a solution " << dif.count() 
          << ", number of iterations: " << numberIterations 
          << ", estimated error = " << planeRegistrationM2.get_current_error() 
          << ", trajectory error = "<< planeRegistrationM2.calculate_poses_rmse(scene.get_ground_truth_trajectory()) << std::endl;
        if ((i+1)%100 == 0)
        {
            //planeRegistrationM2.set_alpha_parameter(alpha);
            create_trajectory(*planeRegistrationM2.get_transformations(), traj,0.5);
            fill_point_cloud_pose(scene, planeRegistrationM2, pointcloud);
            open3d::DrawGeometries({pointcloud,traj,trajGT}, "PointCloud", 1600, 900);
        }
    }
}
if(1)
{
    // ICP solution:
    // 1) create source point cloud and target point cloud
    open3d::PrintInfo("ICP method.\n");
    auto sourcePc = std::make_shared<open3d::PointCloud>();
    auto targetPc = std::make_shared<open3d::PointCloud>();
    fill_point_cloud(scene, targetPc, 0, Mat31(0,0,1.0));
    fill_point_cloud(scene, sourcePc, numPoses-1, Mat31(0.95,0,0));
    // how to plot the trajectory?
    open3d::DrawGeometries({sourcePc,targetPc,trajGT}, "PointCloud", 1600, 900);
    
    t1 = std::chrono::steady_clock::now();
    T_ini.print();
    //auto result = open3d::RegistrationICP(*sourcePc , *targetPc, 0.05, T_ini.T(), open3d::TransformationEstimationPointToPoint(true));
    open3d::EstimateNormals(*sourcePc, open3d::KDTreeSearchParamHybrid(0.1, 30));
    open3d::EstimateNormals(*targetPc, open3d::KDTreeSearchParamHybrid(0.1, 30));
    auto result = open3d::RegistrationICP(*sourcePc , *targetPc, 0.5, T_ini.T(), open3d::TransformationEstimationPointToPlane());
    t2 = std::chrono::steady_clock::now();
    dif = std::chrono::duration_cast<Ttim>(t2 - t1);
    std::cout << "Time spent on calculating a solution " << dif.count() << std::endl;
    std::cout << "Transformation found " << result.transformation_;
    
    sourcePc->Transform(result.transformation_);
    mrob::SE3 T_res(Mat4(result.transformation_));
    // error?
    std::cout << "\nTime spent on calculating a solution " << dif.count() 
          << ", trajectory error ICP = "<< scene.get_ground_truth_trajectory().back().distance(T_res)
          << ", error from method NAG = " << scene.get_ground_truth_trajectory().back().distance(planeRegistration.get_transformations()->back()) << std::endl;
    
    open3d::DrawGeometries({sourcePc,targetPc,trajGT}, "PointCloud", 1600, 900);
}    
    

    return 0;
}
