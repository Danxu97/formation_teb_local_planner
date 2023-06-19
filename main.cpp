#include <iostream>
#include "inc/teb_config.h"
#include "inc/pose_se2.h"
#include "inc/robot_footprint_model.h"
#include "inc/obstacles.h"
#include "inc/optimal_planner.h"
#include <boost/smart_ptr.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace teb_local_planner;
using namespace std;



ViaPointContainer readNumericFile(const std::string& filename) {
    ViaPointContainer data;
    std::ifstream file(filename);
    float reso = 0.05;

    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            double column1, column2;
            if (iss >> column1 >> column2) {
                data.emplace_back(column1*reso,column2*reso);
            } else {
                std::cerr << "Error parsing line: " << line << std::endl;
            }
        }
        file.close();
    } else {
        std::cerr << "Failed to open file: " << filename << std::endl;
    }

    return data;
}

void writeVector3fToFile(const std::string& filename, const std::vector<Eigen::Vector3f>& data) {
    std::ofstream file(filename);

    if (file.is_open()) {
        for (const Eigen::Vector3f& vector : data) {
            file << vector.x() << " " << vector.y() << " " << vector.z() << std::endl;
        }
        file.close();
        std::cout << "Data written to file: " << filename << std::endl;
    } else {
        std::cerr << "Failed to open file: " << filename << std::endl;
    }
}

void writeVectorXfToFile(const std::string& filename, const std::vector<vector<float>>& data) {
    std::ofstream file(filename);

    if (file.is_open()) {
        for (int i=0;i<data.size();i++) {
            for(int j=0;j<data[0].size();j++){
                file<<data[i][j]<<" ";
            }
            file << std::endl;
        }
        file.close();
        std::cout << "Data written to file: " << filename << std::endl;
    } else {
        std::cerr << "Failed to open file: " << filename << std::endl;
    }
}

int main()
{   
    string fpath_path = "/home/ldx/workspace/formation_teb/src/data/fpath.txt";
    string teb1_path = "/home/ldx/workspace/formation_teb/src/data/traj1.txt";
    string teb2_path = "/home/ldx/workspace/formation_teb/src/data/traj2.txt";
    string teb3_path = "/home/ldx/workspace/formation_teb/src/data/traj3.txt";
    string teb4_path = "/home/ldx/workspace/formation_teb/src/data/traj4.txt";

    std::ofstream file("/home/ldx/workspace/formation_teb/src/data/dubug_data.txt", std::ios::trunc);
    file.close();

    ViaPointContainer via_points1,via_points2,via_points3,via_points4,data;
    FormationTrajsContainer Trajs(4); 
    std::vector<vector<float>> traj1,traj2,traj3,traj4;
    data= readNumericFile(fpath_path);
    for(int i=0; i<data.size();i+=4){
        via_points1.emplace_back(data[i][0],data[i][1]);
        via_points2.emplace_back(data[i+1][0],data[i+1][1]);
        via_points3.emplace_back(data[i+2][0],data[i+2][1]);
        via_points4.emplace_back(data[i+3][0],data[i+3][1]);
    }
    // 参数配置
    int len = via_points1.size();
    std::cout<<"path size:"<<len<<std::endl;
    TebConfig config;
    
    
    std::vector<ObstaclePtr> obst_vector;
    //obst_vector.emplace_back(boost::make_shared<PointObstacle>(0,0));//flat radius = 0.5m

    // Setup robot shape model
    RobotFootprintModelPtr robot_model = boost::make_shared<CircularRobotFootprint>(0.4);
    auto visual = TebVisualizationPtr(new TebVisualization(config));
    auto planner1 = new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points1, &Trajs, 1);
    auto planner2 = new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points2, &Trajs, 2);
    auto planner3 = new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points3, &Trajs, 3);
    auto planner4 = new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points4, &Trajs, 4);
    cv::Mat show_map = cv::Mat::zeros(cv::Size(500,500),CV_8UC3);

    // param
    int offset = 0;
    int iter = 0;
    while (iter<10)
    {   
        iter++;
        try
        {
            {//PATH1
                PoseSE2 start1(via_points1[0][0],via_points1[0][1],0);
                PoseSE2 end1(via_points1[len-1][0],via_points1[len-1][1],0);

                planner1->plan(start1,end1);
                std::vector<Eigen::Vector3f> path;
                planner1->getFullTrajectory(path);
                //writeVector3fToFile("data/teb1.txt",path);
                planner1->getFullTrajectoryWithVW(traj1);
                Trajs[0] = traj1;
            }
            {//PATH2
                PoseSE2 start2(via_points2[0][0],via_points2[0][1],0);
                PoseSE2 end2(via_points2[len-1][0],via_points2[len-1][1],0);

                planner2->plan(start2,end2);
                std::vector<Eigen::Vector3f> path;
                planner2->getFullTrajectory(path);
                //writeVector3fToFile("data/teb2.txt",path);
                planner2->getFullTrajectoryWithVW(traj2);
                Trajs[1] = traj2;
            }

            {//PATH3
                PoseSE2 start3(via_points3[0][0],via_points3[0][1],0);
                PoseSE2 end3(via_points3[len-1][0],via_points3[len-1][1],0);

                planner3->plan(start3,end3);
                std::vector<Eigen::Vector3f> path;
                planner3->getFullTrajectory(path);
                //writeVector3fToFile("data/teb3.txt",path);
                planner3->getFullTrajectoryWithVW(traj3);
                Trajs[2] = traj3;
            }
            {//PATH4
                PoseSE2 start4(via_points4[0][0],via_points4[0][1],0);
                PoseSE2 end4(via_points4[len-1][0],via_points4[len-1][1],0);

                planner4->plan(start4,end4);
                std::vector<Eigen::Vector3f> path;
                planner4->getFullTrajectory(path);
                //writeVector3fToFile("data/teb4.txt",path);
                planner4->getFullTrajectoryWithVW(traj4);
                Trajs[3] = traj4;
            }
        }
        catch (...)
        {
            break;
        }
    }
    writeVectorXfToFile(teb1_path,traj1);
    writeVectorXfToFile(teb2_path,traj2);
    writeVectorXfToFile(teb3_path,traj3);
    writeVectorXfToFile(teb4_path,traj4);
    return 0;
}
