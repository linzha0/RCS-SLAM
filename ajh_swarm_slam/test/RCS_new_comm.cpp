#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>

#include <gtsam_unstable/slam/DirectCommFactor.h>

int main(int argc, char** argv) {

    // Some setting
    int maxIterations = 100;
    double commdist = 1.0;

    // Some gtsam param
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    auto priorNoise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(0.01, 0.01, 0.005));

    auto odomNoise1 = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(0.01, 0.01, 0.005));

    auto odomNoise2 = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(0.01, 0.3, 0.005));

    auto rangeNoise = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);

    auto commNoise = gtsam::noiseModel::Isotropic::Sigma(1, 0.3);

    // ================================================================================ //
    // Robot 1
    // ================================================================================ //

    {
        // Add pose 0
        gtsam::Pose2 robo1_pose0(0.0, 0.0, 0.0);    
        initial.insert(
            1000, robo1_pose0);    
        graph.addPrior(
            1000, robo1_pose0, priorNoise);    

        // Add pose 1
        gtsam::Pose2 robo1_pose1(0.5, 0.0, 0.0);    
        initial.insert(
            1001, robo1_pose1);    
        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
            1000, 1001, robo1_pose0.between(robo1_pose1), odomNoise1));

        // Add pose 2
        gtsam::Pose2 robo1_pose2(1.0, 0.0, 0.0);    
        initial.insert(
            1002, robo1_pose2);    
        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
            1001, 1002, robo1_pose1.between(robo1_pose2), odomNoise1));

        // Add pose 3
        gtsam::Pose2 robo1_pose3(1.5, 0.0, 0.0);    
        initial.insert(
            1003, robo1_pose3);    
        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
            1002, 1003, robo1_pose2.between(robo1_pose3), odomNoise1));

        // Add pose 4
        gtsam::Pose2 robo1_pose4(2.0, 0.0, 0.0);    
        initial.insert(
            1004, robo1_pose4);    
        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
            1003, 1004, robo1_pose3.between(robo1_pose4), odomNoise1));

        // Add pose 5
        gtsam::Pose2 robo1_pose5(2.5, 0.0, 0.0);    
        initial.insert(
            1005, robo1_pose5);    
        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
            1004, 1005, robo1_pose4.between(robo1_pose5), odomNoise1));
    }
    

    // ================================================================================ //
    // Robot 2
    // ================================================================================ //

    {
        // Add pose 0
        gtsam::Pose2 robo2_pose0(0.0, 0.8, 0.0);    
        initial.insert(
            2000, robo2_pose0);    
        graph.addPrior(
            2000, robo2_pose0, priorNoise);    

        // Add pose 1
        gtsam::Pose2 robo2_pose1(0.5, 1.2, 0.0);    
        initial.insert(
            2001, robo2_pose1);    
        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
            2000, 2001, robo2_pose0.between(robo2_pose1), odomNoise2));

        // Add pose 2
        gtsam::Pose2 robo2_pose2(1.0, 1.1, 0.0);    
        initial.insert(
            2002, robo2_pose2);    
        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
            2001, 2002, robo2_pose1.between(robo2_pose2), odomNoise2));

        // Add pose 3
        gtsam::Pose2 robo2_pose3(1.5, 0.9, 0.0);    
        initial.insert(
            2003, robo2_pose3);    
        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
            2002, 2003, robo2_pose2.between(robo2_pose3), odomNoise2));

        // Add pose 4
        gtsam::Pose2 robo2_pose4(2.0, 1.1, 0.0);    
        initial.insert(
            2004, robo2_pose4);    
        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
            2003, 2004, robo2_pose3.between(robo2_pose4), odomNoise2));

        // Add pose 5
        gtsam::Pose2 robo2_pose5(2.5, 1.2, 0.0);    
        initial.insert(
            2005, robo2_pose5);    
        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
            2004, 2005, robo2_pose4.between(robo2_pose5), odomNoise2));
    }
    

    // ================================================================================ //
    // Direct comm constraint: if noise odom distance larger than 1.0m, add 1m range constraint
    // ================================================================================ //

    {
        gtsam::Pose2 p1, p2;

        // Pair 0: no constraint added
        p1 = initial.at<gtsam::Pose2>(1000);
        p2 = initial.at<gtsam::Pose2>(2000);
        graph.add(gtsam::DirectCommFactor(
            1000, 2000, commdist, commNoise));

        // Pair 1
        p1 = initial.at<gtsam::Pose2>(1001);
        p2 = initial.at<gtsam::Pose2>(2001);
        graph.add(gtsam::DirectCommFactor(
            1001, 2001, commdist, commNoise));

        // Pair 2
        p1 = initial.at<gtsam::Pose2>(1002);
        p2 = initial.at<gtsam::Pose2>(2002);
        graph.add(gtsam::DirectCommFactor(
            1002, 2002, commdist, commNoise));

        // Pair 3: no constraint added
        p1 = initial.at<gtsam::Pose2>(1003);
        p2 = initial.at<gtsam::Pose2>(2003);
        graph.add(gtsam::DirectCommFactor(
            1003, 2003, commdist, commNoise));

        // Pair 4
        p1 = initial.at<gtsam::Pose2>(1004);
        p2 = initial.at<gtsam::Pose2>(2004);
        graph.add(gtsam::DirectCommFactor(
            1004, 2004, commdist, commNoise));  

        // Pair 4
        p1 = initial.at<gtsam::Pose2>(1005);
        p2 = initial.at<gtsam::Pose2>(2005);
        graph.add(gtsam::DirectCommFactor(
            1005, 2005, commdist, commNoise));    
    }
    

    // ================================================================================ //
    // Optimization
    // ================================================================================ //    
    gtsam::GaussNewtonParams params_gn;
    params_gn.setVerbosity("TERMINATION");
    // params_gn.maxIterations = maxIterations;
    
    graph.print("\nFactor Graph:\n");  // print
    initial.print("\nInitial Estimate:\n");  // print
    gtsam::Values result = gtsam::GaussNewtonOptimizer(graph, initial, params_gn).optimize();
    result.print("Final Result:\n");    


    return 0;
}