#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>


int main(int argc, char** argv) {
    gtsam::NonlinearFactorGraph graph;

    // ======================================== //
    // Robot 1
    // ======================================== //

    // Add prior
    gtsam::Pose2 prior1(0.0, 0.0, 0.0);  // prior at origin
    auto priorNoise1 = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(0.3, 0.3, 0.1));
    graph.addPrior(1, prior1, priorNoise1);    

    // Add odometry factors between consecutive poses
    gtsam::Pose2 odometry1(2.0, 0.0, 0.0);
    auto odometryNoise1 = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(0.2, 0.2, 0.1));
    // Create odometry (Between) factors between consecutive poses
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
        1, 2, odometry1, odometryNoise1));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
        2, 3, odometry1, odometryNoise1));
    graph.print("\nFactor Graph:\n");  // print

    // Add initial poses for 
    gtsam::Values initial;
    initial.insert(1, gtsam::Pose2(0.5, 0.0, 0.2));
    initial.insert(2, gtsam::Pose2(2.3, 0.1, -0.2));
    initial.insert(3, gtsam::Pose2(4.1, 0.1, 0.1));
    initial.print("\nInitial Estimate:\n");  // print

    // Optimize using Levenberg-Marquardt optimization
    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print("Final Result:\n");    

    // Calculate and print marginal covariances for all variables
    std::cout.precision(2);
    gtsam::Marginals marginals(graph, result);
    std::cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << std::endl;
    std::cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << std::endl;
    std::cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << std::endl;

    return 0;    
}