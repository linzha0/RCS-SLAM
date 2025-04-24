#include <iostream>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <iomanip>

#include <Eigen/Dense>

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
#include <gtsam_unstable/slam/OneHopFactor.h>
#include <gtsam/slam/dataset.h>

// ===================================================================== //
// Data structures
// ===================================================================== //
struct CommData {
    int id1;
    int id2;
    double range;
};

struct OnehopData {
    int id1;
    int id2;
};

// ===================================================================== //
// Global parameters
// ===================================================================== //
std::vector<std::string> odom_csv_;
std::vector<std::vector<std::pair<double,gtsam::Pose2>>> robot_odoms_;
std::string comm_csv_;
std::string graph_csv_;
std::string onehop_csv_;
std::string est_new_path_;
std::string est_old_path_;
std::string est_range_path_;
std::vector<double> odom_noise_;
std::vector<double> prior_noise_;
double range_noise_, new_comm_noise_, old_comm_noise_, new_onehop_noise_, old_onehop_noise_;
bool use_graph_, use_onehop_;
std::ofstream file_;

// ===================================================================== //
// Functions
// ===================================================================== //

void load_config(std::string path) 
{
    YAML::Node config = YAML::LoadFile(path);

    // ===================================================================== //
    // Parse graph related
    // ===================================================================== //
    //! Parse the odom csv
    if (config["odom"]) {
        for (const auto& path_node : config["odom"]) {
            odom_csv_.push_back(path_node.as<std::string>());
        }
    } 
 
    //! Parse the graph csv
    if(config["graph"]) {
        graph_csv_ = config["graph"].as<std::string>();
    }

    //! Parse use_graph
    use_graph_ = config["use_graph"].as<bool>();
    if(use_graph_) {
        printf("use_graph:%s\n", use_graph_? "Yes":"No");
        printf(" graph_csv: %s\n", graph_csv_.c_str());
    }
    else {
        printf("use_graph:%s\n", use_graph_? "Yes":"No");
        if(odom_csv_.size()>0) {
            for (size_t i = 0; i < odom_csv_.size(); ++i) {
                std::cout << " oodm file " << i+1 << ": " << odom_csv_[i] << std::endl;
            }   
        }
        else {
            std::cerr << " Error: no 'odom' given" << std::endl;
            exit(0);
        }        
    }

    // ===================================================================== //
    // Parse comm factor related
    // ===================================================================== //

    //! Parse the comm csv
    comm_csv_ = config["comm"].as<std::string>();

    //! Parse the oneop csv
    onehop_csv_ = config["onehop"].as<std::string>();

    // ===================================================================== //
    // Parse noise related
    // ===================================================================== //

    //! Parse the noise
    if (config["odom_noise"] && config["odom_noise"].IsSequence()) {
        odom_noise_ = config["odom_noise"].as<std::vector<double>>();

        //! Test:
        std::cout << "Odom noise: ";
        for (double val : odom_noise_) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    } else {
        std::cerr << "odom_noise not found or not a sequence!" << std::endl;
    }    

    //! Parse prior noise
    if (config["prior_noise"] && config["prior_noise"].IsSequence()) {
        prior_noise_ = config["prior_noise"].as<std::vector<double>>();

        //! Test:
        std::cout << "prior noise: ";
        for (double val : prior_noise_) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    } else {
        std::cerr << "prior_noise not found or not a sequence!" << std::endl;
    } 

    //! Parse range noise
    range_noise_ = config["range_noise"].as<double>();
    printf("range_noise: %f\n", range_noise_);

    //! Parse comm noise
    new_comm_noise_ = config["new_comm_noise"].as<double>();
    old_comm_noise_ = config["old_comm_noise"].as<double>();
    printf("new_comm_noise: %f, old_comm_noise_: %f\n", 
        new_comm_noise_, old_comm_noise_);

    //! Parse one-hop noise
    new_onehop_noise_ = config["new_onehop_noise"].as<double>();
    old_onehop_noise_ = config["old_onehop_noise"].as<double>();
    use_onehop_ = config["use_onehop"].as<bool>();
    printf("new_onehop_noise: %f, old_onehop_noise: %f, use_onehop:%s\n", 
        new_onehop_noise_, old_onehop_noise_, use_onehop_? "Yes":"No");

    // ===================================================================== //
    // Parse saved files
    // ===================================================================== //

    //! Parse file path
    est_new_path_ = config["new_est"].as<std::string>();
    est_old_path_ = config["old_est"].as<std::string>();
    est_range_path_ = config["range_est"].as<std::string>();
}

void parse_odom(std::string& csv, std::vector<std::pair<double,gtsam::Pose2>>& pose_data)
{
    // ===================================================================== //
    // Check if csv files are vaild
    // ===================================================================== //

    std::ifstream file(csv);
    if (!file.is_open()) {
        std::cerr << "Failed to open CSV file: " << csv << std::endl;
        return;
    }

    // ===================================================================== //
    // Parse the header
    // ===================================================================== //

    //! Read header
    std::string line;
    std::getline(file, line);
    
    //! Map column name to its index
    std::unordered_map<std::string, size_t> column_map;
    std::stringstream header_stream(line);
    std::string col;
    size_t index = 0;
    //! Separate the string by ','
    while (std::getline(header_stream, col, ',')) {
        column_map[col] = index++;
    }

    //! Check required columns exist
    std::vector<std::string> required = {
        "Time", "pose.x", "pose.y",
        "orientation.x", "orientation.y", 
        "orientation.z", "orientation.w"
    };

    for (const auto& key : required) {
        if (column_map.find(key) == column_map.end()) {
            std::cerr << "Missing column: " << key << std::endl;
            return ;
        }
    }

    // ===================================================================== //
    // Parse the required data
    // ===================================================================== //
    while (std::getline(file, line)) {
        std::stringstream line_stream(line);
        std::string cell;
        std::vector<std::string> row;
        while (std::getline(line_stream, cell, ',')) {
            row.push_back(cell);
        }

        //! Get orientation
        double x = std::stod(row[column_map["orientation.x"]]);
        double y = std::stod(row[column_map["orientation.y"]]);
        double z = std::stod(row[column_map["orientation.z"]]);
        double w = std::stod(row[column_map["orientation.w"]]);
        Eigen::Quaterniond q(w,x,y,z);
        q.normalize();
        Eigen::Matrix3d rot = q.toRotationMatrix();            

        // --------------------------------------------------------------- //
        // Use Roll-Pitch-Yaw order
        // --------------------------------------------------------------- //
        // Eigen::Vector3d euler = rot.eulerAngles(0, 1, 2);
        // double roll = euler[0] ;
        // double pitch = euler[1];
        // double yaw = euler[2];
        // printf("r:%f, p: %f, y:%f\n", roll* 180.0 / M_PI, pitch* 180.0 / M_PI, yaw* 180.0 / M_PI);

        // --------------------------------------------------------------- //
        // Use Yaw-Pitch-Roll order
        // --------------------------------------------------------------- //
        // Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0);
        // double yaw = euler[0] ;
        // double pitch = euler[1];
        // double roll = euler[2];
        // // printf("r:%f, p: %f, y:%f\n", roll* 180.0 / M_PI, pitch* 180.0 / M_PI, yaw* 180.0 / M_PI);

        // --------------------------------------------------------------- //
        // Use Adam order
        // --------------------------------------------------------------- //
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        double yaw = atan2(siny_cosp, cosy_cosp);
        // printf("org yaw: %f\n", yaw* 180.0 / M_PI);
        // if (yaw < 0) {
        //     yaw = 2 * M_PI + yaw;
        //     printf("new yaw: %f\n", yaw* 180.0 / M_PI);
        // }

        //! Save to data
        std::pair<double, gtsam::Pose2> pose_pair;
        gtsam::Pose2 pose(
            std::stod(row[column_map["pose.x"]]), 
            std::stod(row[column_map["pose.y"]]), 
            yaw); 
        pose_pair.first = std::stod(row[column_map["Time"]]);
        pose_pair.second = pose;
        pose_data.push_back(pose_pair);
    }

    file.close();

    // ===================================================================== //
    // Test data
    // ===================================================================== //

    //! Print first few entries
    // for (size_t i = 0; i < std::min(pose_data.size(), size_t(5)); ++i) {
    //     const auto& d = pose_data[i];
    //     std::cout << "Time: " << std::fixed << std::setprecision(9) << d.first
    //               << ", position[m]: (" << d.second.x() << ", " << d.second.y() << ")"
    //               << ", yaw[deg]: (" << d.second.theta() * 180.0/ M_PI << ")\n";
    // }
}

void parse_comm(std::string& csv, std::vector<CommData>& comm_data)
{
    // ===================================================================== //
    // Check if csv files are vaild
    // ===================================================================== //

    std::ifstream file(csv);
    if (!file.is_open()) {
        std::cerr << "Failed to open CSV file: " << csv << std::endl;
        return;
    }

    // ===================================================================== //
    // Parse the data
    // ===================================================================== //
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        CommData d;
        if (!(iss >> d.id1 >> d.id2 >> d.range)) {
            std::cerr << "Invalid line format: " << line << std::endl;
            continue;
        }
        comm_data.push_back(d);
    }

    file.close();

    // ===================================================================== //
    // Test
    // ===================================================================== //
    printf("\nParse_comm test:\n");
    for(size_t i =0 ; i < comm_data.size(); i++) {
        if(i>4 && i < comm_data.size()-5) {
            continue;
        }
        std::cout << i+1 << ": id1= " << comm_data[i].id1 << ", id2= " << comm_data[i].id2 <<  ", range= " << comm_data[i].range << std::endl;
    }        
}

void parse_onehop(std::string& csv, std::vector<OnehopData>& onehop_data)
{
    // ===================================================================== //
    // Check if csv files are vaild
    // ===================================================================== //

    std::ifstream file(csv);
    if (!file.is_open()) {
        std::cerr << "Failed to open CSV file: " << csv << std::endl;
        return;
    }    

    // ===================================================================== //
    // Parse the data
    // ===================================================================== //
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        OnehopData d;
        if (!(iss >> d.id1 >> d.id2)) {
            std::cerr << "Invalid line format: " << line << std::endl;
            continue;
        }
        onehop_data.push_back(d);
    }

    file.close(); 

    // ===================================================================== //
    // Test
    // ===================================================================== //
    printf("\nParse_onehop test:\n");
    for(size_t i =0 ; i < onehop_data.size(); i++) {
        if(i>4 && i < onehop_data.size()-5) {
            continue;
        }        
        std::cout << i+1 << ": id1= " << onehop_data[i].id1 << ", id2= " << onehop_data[i].id2 << std::endl;
    }          
}

void graph_from_odom(
    gtsam::NonlinearFactorGraph::shared_ptr& graph, 
    gtsam::Values::shared_ptr& initial,
    const gtsam::SharedNoiseModel& prior_noise, 
    const gtsam::SharedNoiseModel& odom_noise)
{
    // ----------------------------------------------------------------- //
    // Add odom factors for each robots
    // ----------------------------------------------------------------- //
    //! Loop each robot
    for(size_t robo_num=0; robo_num<robot_odoms_.size(); robo_num++) {
        //! Add prior factor
        auto prior_id = (robo_num+1) * 1000 + 0;
        initial->insert(prior_id, robot_odoms_.at(robo_num).at(0).second);
        graph->addPrior(prior_id, robot_odoms_.at(robo_num).at(0).second, prior_noise);  
        // printf("prior_id: %ld\n", prior_id);

        //! Loop each pose from second one
        for(size_t pos_num=1; pos_num<robot_odoms_.at(robo_num).size(); pos_num++) {
            //! Add odom factor
            auto id = (robo_num+1) * 1000 + pos_num;
            auto odom = robot_odoms_.at(robo_num).at(pos_num-1).second.between(robot_odoms_.at(robo_num).at(pos_num).second);
            initial->insert(id, robot_odoms_.at(robo_num).at(pos_num).second);
            graph->add(
                gtsam::BetweenFactor<gtsam::Pose2>(id-1, id, odom, odom_noise));
            // printf("id: %ld\n", id);
        }
    }
}

void graph_from_g2o(
    gtsam::NonlinearFactorGraph::shared_ptr& graph, 
    gtsam::Values::shared_ptr& initial,
    const gtsam::SharedNoiseModel& prior_noise)
{
    //! Load graph from g2o file
    bool is3D = false;
    std::tie(graph, initial) = gtsam::readG2o(graph_csv_, is3D);

    //! Add prior for all the robots
    //! TODO: the graph already has the node from odom, why still add it as prior ?
    // for(size_t robo_num=0; robo_num<robot_odoms_.size(); robo_num++) {

    //     //! Add prior factor based on graph
    //     auto prior_id = (robo_num+1) * 1000 + 0;
    //     if(initial->exists(prior_id)) {
    //         auto pose = initial->at<gtsam::Pose2>(prior_id);   
    //         graph->addPrior(prior_id, pose, prior_noise);     
    //         printf("prior_id: %ld, x:%f, y:%f, theta:%f\n", 
    //             prior_id, pose.x(), pose.y(), pose.theta());                                  
    //     }
        
    //     //! Add prior factor based on odom 
    //     // auto prior_id = (robo_num+1) * 1000 + 0;
    //     // if(initial->exists(prior_id)) {
    //     //     auto pose = robot_odoms_.at(robo_num).at(0).second;
    //     //     graph->addPrior(prior_id, pose, prior_noise);  
    //     //     printf("prior_id: %ld, x:%f, y:%f, theta:%f\n", 
    //     //         prior_id, pose.x(), pose.y(), pose.theta());
    //     // }
    // }

    graph->addPrior(1000,  gtsam::Pose2(-1.0,-1.0,0.0),  prior_noise);
    graph->addPrior(2000,  gtsam::Pose2(-1.0, -0.5,0.0), prior_noise);
    graph->addPrior(3000,  gtsam::Pose2(-1.0,0.0,0.0),   prior_noise);
    graph->addPrior(4000,  gtsam::Pose2(-1.0,0.5,0.0),   prior_noise);
    graph->addPrior(5000,  gtsam::Pose2(-1.0,1.0,0.0),   prior_noise);
    graph->addPrior(6000,  gtsam::Pose2(-0.5,-1.0,0.0),  prior_noise);
    graph->addPrior(7000,  gtsam::Pose2(-0.5,-0.5,0.0),  prior_noise);
    graph->addPrior(8000,  gtsam::Pose2(-0.5,0.0,0.0),   prior_noise);
    graph->addPrior(9000,  gtsam::Pose2(-0.5,0.5,0.0),   prior_noise);
    graph->addPrior(10000, gtsam::Pose2(-0.5,1.0,0.0),   prior_noise);
    graph->addPrior(11000, gtsam::Pose2(0.0,-1.0,0.0),   prior_noise);
    graph->addPrior(12000, gtsam::Pose2(0.0,-0.5,0.0),   prior_noise);
    graph->addPrior(13000, gtsam::Pose2(0.0,0.0,0.0),    prior_noise);
    graph->addPrior(14000, gtsam::Pose2(0.0,0.5,0.0),    prior_noise);
    graph->addPrior(15000, gtsam::Pose2(0.0,1.0,0.0),    prior_noise);
    graph->addPrior(16000, gtsam::Pose2(0.5,-1.0,0.0),   prior_noise);
    graph->addPrior(17000, gtsam::Pose2(0.5,-0.5,0.0),   prior_noise);
    graph->addPrior(18000, gtsam::Pose2(0.5,0.0,0.0),    prior_noise);
    graph->addPrior(19000, gtsam::Pose2(0.5,0.5,0.0),    prior_noise);
    graph->addPrior(20000, gtsam::Pose2(0.5,1.0,0.0),    prior_noise);
    graph->addPrior(21000, gtsam::Pose2(0.25,0.25,0.0),  prior_noise);    
}

void save_odom(const gtsam::Values& result, std::string& path) {

    std::string robot_id = "0";
    std::string file_name = "/tmp/rcs_file.csv";
    int count = 0;
    
    for (const auto& key_value : result) {
        gtsam::Key key = key_value.key;
        if (result.exists<gtsam::Pose2>(key)) {
            // ============================================================= //
            // Extract the robot ID
            // ============================================================= //
            //! Get the key for each robots
            std::string str_key = std::to_string(key);
            //! Make sure it more than 3 digits
            if (str_key.length() <= 3)
                continue;
            //! Remove the key and left the robot ID
            auto curr_robot_id = str_key.substr(0, str_key.length() - 3);

            // ============================================================= //
            // Save pose to current file
            // ============================================================= //

            //! Check if to start a new csv file
            if (curr_robot_id != robot_id) {
                //! Close the old file if opened
                if(file_.is_open()) {
                    file_.close();    
                    if(count != 0) {
                        std::cout<<"Robot"+robot_id<<" saved: "<< count <<std::endl;
                    }                     
                }

                //! Reset

                robot_id = curr_robot_id;
                count = 0;

                //! Create a new file name
                file_name = path + "Robot" + robot_id + ".csv";

                //! Write the header
                file_.open(file_name);
                file_ << "x,y,theta\n";     
            }

            //! Save the data
            gtsam::Pose2 pose = result.at<gtsam::Pose2>(key);
            file_ << pose.x() << "," << pose.y() << "," << pose.theta() << "\n";
            count++;
        }
    }

    //! Close the old file if opened
    if(file_.is_open()) {
        file_.close();    
        if(count != 0) {
            std::cout<<"Robot"+robot_id<<" saved: "<< count <<std::endl;
        }                     
    } 

    //! TEST: print all the values
    // for (const auto& key_value : result) {
    //     gtsam::Key key = key_value.key;
    //     if (result.exists<gtsam::Pose2>(key)) {
    //         gtsam::Pose2 pose = result.at<gtsam::Pose2>(key);
    //         std::cout << "Key " << key << " (Symbol: " << gtsam::DefaultKeyFormatter(key) << "): "
    //                   << "(" << pose.x() << ", " << pose.y() << ", " << pose.theta() << ")" << std::endl;
    //     } else {
    //         std::cout << "Key " << key << " is not a Pose2." << std::endl;
    //     }
    // }
}

int main(const int argc, const char *argv[])
{
    printf("\n=================== Load config ==================\n");

    // ===================================================================== //
    // Load config
    // ===================================================================== //

    // config yaml path
    std::string config_path;
    if (argc > 1) {
        config_path = argv[1];
        printf("read file: %s\n", config_path.c_str());
    } else {
        printf("not file given: exit now\n");
        exit(0);
    }

    //! Load config
    load_config(config_path);

    // ===================================================================== //
    // Parse odom data
    // ===================================================================== //
    printf("\n=================== Parse Odom ==================\n");

    for (auto& csv : odom_csv_) {
        std::vector<std::pair<double,gtsam::Pose2>> poses;
        parse_odom(csv, poses);
        robot_odoms_.emplace_back(poses);
    }
    //! TEST:
    for(size_t i = 0; i < robot_odoms_.size(); i++) {
        printf("robot_%ld: pose size:%ld\n", i+1, robot_odoms_.at(i).size());
    }

    // ===================================================================== //
    // Parse comm data
    // ===================================================================== //
    printf("\n================== Parse comm ===================\n");

    std::vector<CommData> comm_data;
    parse_comm(comm_csv_, comm_data);

    // ===================================================================== //
    // Parse onehop data
    // ===================================================================== //
    printf("\n================== Parse onehop ===================\n");

    std::vector<OnehopData> onehop_data;
    parse_onehop(onehop_csv_, onehop_data);

    // ===================================================================== //
    // GTSAM preparation
    // ===================================================================== //

    auto priorNoise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(prior_noise_.at(0), prior_noise_.at(1), prior_noise_.at(2)));

    auto odomNoise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(odom_noise_.at(0), odom_noise_.at(1), odom_noise_.at(2)));        

    auto rangeNoise = gtsam::noiseModel::Isotropic::Sigma(1, range_noise_);

    auto newCommNoise = gtsam::noiseModel::Isotropic::Sigma(1, new_comm_noise_);
    auto oldCommNoise = gtsam::noiseModel::Isotropic::Sigma(1, old_comm_noise_);

    auto newOnehopNoise = gtsam::noiseModel::Isotropic::Sigma(1, new_onehop_noise_);
    auto oldOnehopNoise = gtsam::noiseModel::Isotropic::Sigma(1, old_onehop_noise_);

    double commdist = 1.0;

    gtsam::NonlinearFactorGraph::shared_ptr graph_range = 
        boost::make_shared<gtsam::NonlinearFactorGraph>();
    gtsam::Values::shared_ptr initial_range =  
        boost::make_shared<gtsam::Values>();

    gtsam::NonlinearFactorGraph::shared_ptr graph_new_est = 
        boost::make_shared<gtsam::NonlinearFactorGraph>();
    gtsam::Values::shared_ptr initial_new_est = 
        boost::make_shared<gtsam::Values>();

    gtsam::NonlinearFactorGraph::shared_ptr graph_old_est = 
        boost::make_shared<gtsam::NonlinearFactorGraph>();
    gtsam::Values::shared_ptr initial_old_est = 
        boost::make_shared<gtsam::Values>();

    // ===================================================================== //
    // Range constraint
    // ===================================================================== //
    {
        // ----------------------------------------------------------------- //
        // Add odom factors for each robots
        // ----------------------------------------------------------------- //
        if (use_graph_) {
            graph_from_g2o(graph_range, initial_range, priorNoise);
        }
        else {
            graph_from_odom(graph_range, initial_range, priorNoise, odomNoise);
        }

        // ----------------------------------------------------------------- //
        // Add range factors for each robots
        // ----------------------------------------------------------------- //
        printf("\n=================== Add constraint-Range ==================\n");
        for(const auto& comm : comm_data) {
            //! Check if this comm exist
            if(!initial_range->exists(comm.id1) || !initial_range->exists(comm.id2)) {
                printf("not exit: %d %d\n", comm.id1, comm.id2);
                continue;
            }
            //! Add range factor
            graph_range->add(gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2>(
                comm.id1, comm.id2, comm.range, rangeNoise));
            // printf("range: ids exist and adding factor now\n");
        }

        // ----------------------------------------------------------------- //
        // Optimization
        // ----------------------------------------------------------------- //
        //! NOTE: if you have good noise model, GN should work, otherwise use LM
        printf("\n=================== Optimization-Range ==================\n");

        // //! Use Gauss Newton
        // gtsam::GaussNewtonParams params_gn;
        // params_gn.setVerbosity("TERMINATION");
        // gtsam::Values result = gtsam::GaussNewtonOptimizer(*graph_range, *initial_range, params_gn).optimize();
        // // graph_range->print("\nFactor Graph:\n");
        // // initial_range.print("\nInitial Estimate:\n");    
        // // result.print("Final Result:\n");   

        //! Use Levenberg Marquard
        gtsam::LevenbergMarquardtParams params_lm;
        // params_lm.verbosity = gtsam::NonlinearOptimizerParams::Verbosity::ERROR;
        params_lm.setVerbosity("TERMINATION");
        gtsam::LevenbergMarquardtOptimizer optimizer(*graph_range, *initial_range, params_lm);
        gtsam::Values result = optimizer.optimize();
        // graph_range->print("\nFactor Graph:\n");
        // initial_range.print("\nInitial Estimate:\n");    
        // result.print("Final Result:\n");           

        // ----------------------------------------------------------------- //
        // Save the data
        // ----------------------------------------------------------------- //
        printf("\n=================== Save data-Range ==================\n");
        save_odom(result, est_range_path_);
    }

    // ===================================================================== //
    // New method for estimation
    // ===================================================================== //
    {
        // ----------------------------------------------------------------- //
        // Add odom factors for each robots
        // ----------------------------------------------------------------- //
        if (use_graph_) {
            graph_from_g2o(graph_new_est, initial_new_est, priorNoise);
        }
        else {
            graph_from_odom(graph_new_est, initial_new_est, priorNoise, odomNoise);
        }

        // ----------------------------------------------------------------- //
        // Add new direct comm factors for each robots
        // ----------------------------------------------------------------- //
        printf("\n============== Add constraint-new direct comm ===========\n");

        for(const auto& comm : comm_data) {
            //! Check if this comm exist
            if(!initial_new_est->exists(comm.id1) || !initial_new_est->exists(comm.id2)) {
                continue;
            }
            //! Add direct comm factor
            graph_new_est->add(gtsam::DirectCommFactor(
                comm.id1, comm.id2, commdist, newCommNoise));
            // printf("new-est: ids exist and adding factor now\n");
        }

        // ----------------------------------------------------------------- //
        // Add new onehop comm factors for each robots
        // ----------------------------------------------------------------- //
        if(use_onehop_) {
            printf("\n============== Add constraint-new onehop comm ===========\n");

            for(const auto& onehop : onehop_data) {
                //! Check if this comm exist
                if(!initial_new_est->exists(onehop.id1) || 
                   !initial_new_est->exists(onehop.id2)) {
                    continue;
                }
                //! Add onehop factor
                graph_new_est->add(gtsam::OneHopFactor(
                    onehop.id1, onehop.id2, commdist, newOnehopNoise));
            }
        }

        // ----------------------------------------------------------------- //
        // Optimization
        // ----------------------------------------------------------------- //
        //! NOTE: if you have good noise model, GN should work, otherwise use LM
        printf("\n=================== Optimization-New Est ==================\n");

        // //! Use Gauss Newton
        // gtsam::GaussNewtonParams params_gn;
        // params_gn.setVerbosity("TERMINATION");
        // gtsam::Values result = gtsam::GaussNewtonOptimizer(*graph_new_est, *initial_new_est, params_gn).optimize();
        // // graph_new_est->print("\nFactor Graph:\n");
        // // initial_new_est->print("\nInitial Estimate:\n");    
        // // result.print("Final Result:\n");   

        // ! Use Levenberg Marquard
        gtsam::LevenbergMarquardtParams params_lm;
        // params_lm.verbosity = gtsam::NonlinearOptimizerParams::Verbosity::ERROR;
        params_lm.setVerbosity("TERMINATION");
        gtsam::LevenbergMarquardtOptimizer optimizer(*graph_new_est, *initial_new_est, params_lm);
        gtsam::Values result = optimizer.optimize();
        // graph_new_est->print("\nFactor Graph:\n");
        // initial_new_est->print("\nInitial Estimate:\n");    
        // result.print("Final Result:\n");   

        // ----------------------------------------------------------------- //
        // Save the data
        // ----------------------------------------------------------------- //
        printf("\n=================== Save data-New Est ==================\n");
        save_odom(result, est_new_path_);            
    }

    // ===================================================================== //
    // Old method for estimation
    // ===================================================================== //

    {

        // ----------------------------------------------------------------- //
        // Add odom factors for each robots
        // ----------------------------------------------------------------- //
        if (use_graph_) {
            graph_from_g2o(graph_old_est, initial_old_est, priorNoise);
        }
        else {
            graph_from_odom(graph_old_est, initial_old_est, priorNoise, odomNoise);
        }        

        // ----------------------------------------------------------------- //
        // Add old direct comm factors for each robots
        // ----------------------------------------------------------------- //
        printf("\n=============== Add constraint-old direct comm ==============\n");

        for(const auto& comm : comm_data) {
            //! Check if this comm exist
            if(!initial_old_est->exists(comm.id1) || 
               !initial_old_est->exists(comm.id2)) {
                continue;
            }
            // printf("old-est: ids exist and adding factor now\n");

            //! Add direct comm constraint consided as range factor
            gtsam::Pose2 p1 = initial_old_est->at<gtsam::Pose2>(comm.id1);
            gtsam::Pose2 p2 = initial_old_est->at<gtsam::Pose2>(comm.id2);
            double range = (sqrt(pow(p1.x() - p2.x(),2) + pow(p1.y() - p2.y(),2)));
            if(range > commdist) {
                graph_old_est->add(gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2>(
                    comm.id1, comm.id2, commdist, oldCommNoise));
            }
        }        

        // ----------------------------------------------------------------- //
        // Add new onehop comm factors for each robots
        // ----------------------------------------------------------------- //
        if(use_onehop_) {
            printf("\n============ Add constraint-old onehop comm ===========\n");

            for(const auto& onehop : onehop_data) {
                //! Check if this comm exist
                if(!initial_old_est->exists(onehop.id1) || 
                   !initial_old_est->exists(onehop.id2)) {
                    continue;
                }
                //! Add onehop factor
                gtsam::Pose2 p1 = initial_old_est->at<gtsam::Pose2>(onehop.id1);
                gtsam::Pose2 p2 = initial_old_est->at<gtsam::Pose2>(onehop.id2);
                double range = (sqrt(pow(p1.x() - p2.x(),2) + pow(p1.y() - p2.y(),2)));

                if(range > 2*commdist) {
                    graph_old_est->add(gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2>(
                        onehop.id1, onehop.id2, 2*commdist, oldOnehopNoise));
                }
                else if (range < commdist) {
                    graph_old_est->add(gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2>(
                        onehop.id1, onehop.id2, commdist, oldOnehopNoise));                    
                }
            }
        }

        // ----------------------------------------------------------------- //
        // Optimization
        // ----------------------------------------------------------------- //
        //! NOTE: if you have good noise model, GN should work, otherwise use LM
        printf("\n=================== Optimization-Old Est ==================\n");

        // //! Use Gauss Newton
        // gtsam::GaussNewtonParams params_gn;
        // params_gn.setVerbosity("TERMINATION");
        // gtsam::Values result = gtsam::GaussNewtonOptimizer(*graph_old_est, *initial_old_est, params_gn).optimize();
        // // graph_new_est->print("\nFactor Graph:\n");
        // // initial_new_est->print("\nInitial Estimate:\n");    
        // // result.print("Final Result:\n");   

        //! Use Levenberg Marquard
        gtsam::LevenbergMarquardtParams params_lm;
        // params_lm.verbosity = gtsam::NonlinearOptimizerParams::Verbosity::ERROR;
        params_lm.setVerbosity("TERMINATION");
        gtsam::LevenbergMarquardtOptimizer optimizer(*graph_old_est, *initial_old_est, params_lm);
        gtsam::Values result = optimizer.optimize();
        // graph_old_est->print("\nFactor Graph:\n");
        // initial_old_est->print("\nInitial Estimate:\n");    
        // result.print("Final Result:\n");   

        // ----------------------------------------------------------------- //
        // Save the data
        // ----------------------------------------------------------------- //
        printf("\n=================== Save data-Old Est ==================\n");
        save_odom(result, est_old_path_);        
    }

    return 0;
}