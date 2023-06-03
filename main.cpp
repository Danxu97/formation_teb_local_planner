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
    string fpath_path = "/home/ldx/Downloads/TEBwithoutROS/data/fpath.txt";
    

    ViaPointContainer via_points1,via_points2,via_points3,via_points4,data;
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
    auto planner1 = new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points1);
    auto planner2 = new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points2);
    auto planner3 = new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points3);
    auto planner4 = new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points4);
    cv::Mat show_map = cv::Mat::zeros(cv::Size(500,500),CV_8UC3);

    // param
    int offset = 0;
    int iter = 0;
    while (iter<1)
    {   
        iter++;
        memset(show_map.data,255,500*500*3);
        //PATH1
        for(int i=0;i<via_points1.size()-1;i++){
                int x = (int)(via_points1[i][0] * 100.f + offset);
                int y = (int)(via_points1[i][1] * 100.f + offset);
                int next_x = (int)(via_points1[i+1][0] * 100.f + offset);
                int next_y = (int)(via_points1[i+1][1] * 100.f + offset);
                cv::line(show_map,cv::Point(x,y),cv::Point(next_x,next_y),cv::Scalar(255,127,128));
        }
        //PATH2
        for(int i=0;i<via_points2.size()-1;i++){
                int x = (int)(via_points2[i][0] * 100.f + offset);
                int y = (int)(via_points2[i][1] * 100.f + offset);
                int next_x = (int)(via_points2[i+1][0] * 100.f + offset);
                int next_y = (int)(via_points2[i+1][1] * 100.f + offset);
                cv::line(show_map,cv::Point(x,y),cv::Point(next_x,next_y),cv::Scalar(255,127,128));
        }
        //PATH3
        for(int i=0;i<via_points3.size()-1;i++){
                int x = (int)(via_points3[i][0] * 100.f + offset);
                int y = (int)(via_points3[i][1] * 100.f + offset);
                int next_x = (int)(via_points3[i+1][0] * 100.f + offset);
                int next_y = (int)(via_points3[i+1][1] * 100.f + offset);
                cv::line(show_map,cv::Point(x,y),cv::Point(next_x,next_y),cv::Scalar(255,127,128));
        }
        //PATH3
        for(int i=0;i<via_points4.size()-1;i++){
                int x = (int)(via_points4[i][0] * 100.f + offset);
                int y = (int)(via_points4[i][1] * 100.f + offset);
                int next_x = (int)(via_points4[i+1][0] * 100.f + offset);
                int next_y = (int)(via_points4[i+1][1] * 100.f + offset);
                cv::line(show_map,cv::Point(x,y),cv::Point(next_x,next_y),cv::Scalar(255,127,128));
        }
        try
        {
            {//PATH1
                PoseSE2 start1(via_points1[0][0],via_points1[0][1],0);
                PoseSE2 end1(via_points1[len-1][0],via_points1[len-1][1],0);

                planner1->plan(start1,end1);
                // vi
                std::vector<Eigen::Vector3f> path;
                planner1->getFullTrajectory(path);
                //writeVector3fToFile("data/teb1.txt",path);
                
                std::vector<vector<float>> traj;
                planner1->getFullTrajectoryWithVW(traj);
                writeVectorXfToFile("data/traj1.txt",traj);
                for(int i = 0;i < path.size() - 1;i ++)
                {
                    int x = (int)(path.at(i)[0] * 100.f + offset);
                    int y = (int)(path.at(i)[1] * 100.f + offset);
                    int next_x = (int)(path.at(i+1)[0] * 100.f + offset);
                    int next_y = (int)(path.at(i+1)[1] * 100.f + offset);
                    cv::line(show_map,cv::Point(x,y),cv::Point(next_x,next_y),cv::Scalar(0,255*i/path.size(),255-255*i/path.size()));
                }
            }
            {//PATH2
                PoseSE2 start2(via_points2[0][0],via_points2[0][1],0);
                PoseSE2 end2(via_points2[len-1][0],via_points2[len-1][1],0);

                planner2->plan(start2,end2);
                // vi
                std::vector<Eigen::Vector3f> path;
                planner2->getFullTrajectory(path);
                //writeVector3fToFile("data/teb2.txt",path);

                std::vector<vector<float>> traj;
                planner2->getFullTrajectoryWithVW(traj);
                writeVectorXfToFile("data/traj2.txt",traj);
                for(int i = 0;i < path.size() - 1;i ++)
                {
                    int x = (int)(path.at(i)[0] * 100.f + offset);
                    int y = (int)(path.at(i)[1] * 100.f + offset);
                    int next_x = (int)(path.at(i+1)[0] * 100.f + offset);
                    int next_y = (int)(path.at(i+1)[1] * 100.f + offset);
                    cv::line(show_map,cv::Point(x,y),cv::Point(next_x,next_y),cv::Scalar(0,255*i/path.size(),255-255*i/path.size()));
                }
            }

            {//PATH3
                PoseSE2 start3(via_points3[0][0],via_points3[0][1],0);
                PoseSE2 end3(via_points3[len-1][0],via_points3[len-1][1],0);

                planner3->plan(start3,end3);
                // vi
                std::vector<Eigen::Vector3f> path;
                planner3->getFullTrajectory(path);
                //writeVector3fToFile("data/teb3.txt",path);

                std::vector<vector<float>> traj;
                planner3->getFullTrajectoryWithVW(traj);
                writeVectorXfToFile("data/traj3.txt",traj);
                for(int i = 0;i < path.size() - 1;i ++)
                {
                    int x = (int)(path.at(i)[0] * 100.f + offset);
                    int y = (int)(path.at(i)[1] * 100.f + offset);
                    int next_x = (int)(path.at(i+1)[0] * 100.f + offset);
                    int next_y = (int)(path.at(i+1)[1] * 100.f + offset);
                    cv::line(show_map,cv::Point(x,y),cv::Point(next_x,next_y),cv::Scalar(0,255*i/path.size(),255-255*i/path.size()));
                }
            }
            {//PATH4
                PoseSE2 start4(via_points4[0][0],via_points4[0][1],0);
                PoseSE2 end4(via_points4[len-1][0],via_points4[len-1][1],0);

                planner4->plan(start4,end4);
                // vi
                std::vector<Eigen::Vector3f> path;
                planner4->getFullTrajectory(path);
                //writeVector3fToFile("data/teb4.txt",path);

                std::vector<vector<float>> traj;
                planner4->getFullTrajectoryWithVW(traj);
                writeVectorXfToFile("data/traj4.txt",traj);
                for(int i = 0;i < path.size() - 1;i ++)
                {
                    int x = (int)(path.at(i)[0] * 100.f + offset);
                    int y = (int)(path.at(i)[1] * 100.f + offset);
                    int next_x = (int)(path.at(i+1)[0] * 100.f + offset);
                    int next_y = (int)(path.at(i+1)[1] * 100.f + offset);
                    cv::line(show_map,cv::Point(x,y),cv::Point(next_x,next_y),cv::Scalar(0,255*i/path.size(),255-255*i/path.size()));
                }
            }



            
            //cv::createTrackbar("start theta","path",&start_theta,100);
            //cv::createTrackbar("end theta","path",&end_theta,100);
            cv::imshow("path",show_map);
        }
        catch (...)
        {
            break;
        }
        cv::waitKey(0);
    }
    return 0;
}
