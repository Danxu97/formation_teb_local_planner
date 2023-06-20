/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 * 
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann
 *********************************************************************/
#ifndef EDGE_KEEP_FORMATION_H_
#define EDGE_KEEP_FORMATION_H_

#include "../obstacles.h"
#include "../robot_footprint_model.h"
#include "vertex_pose.h"
#include "base_teb_edges.h"
#include "penalties.h"
#include "../teb_config.h"

namespace teb_local_planner
{

/**
 * @class EdgeKeepFormation
 * @brief Edge defining the cost function for keeping a formation.
 * @see TebOptimalPlanner::AddEdgesFormation
 * @remarks Do not forget to call setTebConfig(), setVertexIdx() and 
 * @warning Experimental
 */   
class EdgeKeepFormation : public BaseTebUnaryEdge<1, const std::vector<std::vector<std::vector<float>>>*,VertexTimeDiff>  //1指定error的个数，会结合information这个权重矩阵进行总error计算
{
public:
    
  /**
   * @brief Construct edge and specify the index of robot
   */    
  EdgeKeepFormation() : index_(3)
  {
    //this->resize(2);
    _measurement = NULL;
  }

  /**
   * @brief Construct edge and specify the index of robot
   * @param index_ index of robot(0, 1, 2, 3)
   */    
  EdgeKeepFormation(int index) : index_(index)
  {
    //this->resize(3);  //3表示参与优化的有三个量
    _measurement = NULL;
  }
 
   //step 2: m:[x1,y1,'''x4,y4]
  /**
   * @brief Actual cost function
   */   
  void computeError()
  {
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[0]);
    Eigen::Matrix4d SNL0,SNL;

    //rect
    SNL0 <<  1.0000,   -0.2929,   -0.4142,   -0.2929,
            -0.2929,    1.0000,   -0.2929,   -0.4142,
            -0.4142,   -0.2929,    1.0000,   -0.2929,
            -0.2929,   -0.4142,   -0.2929,    1.0000;

    Eigen::Matrix<double, 4, 2> m;
    int piece = 10;//分成p份
    for(int i=0;i<piece;i++){ 
      double time_now = time_i_ + deltaT->estimate()*i/(piece-1);
      getStFromTraj(time_now,&m);
      SNL = calculateSNL(m,true);
      _error[0] += 1*(SNL0 - SNL).norm();
    }


    
    //----------dubug log---------------//
    DataEntry data;
    data.id = index_;
    data.time_i = time_i_;
    data.dt = deltaT->estimate();
    data.pose = m.row((index_)%4);
    data.measurement = m;
    data.ff = _error[0];
    // 将数据写入文件
    writeDataToFile(data, "/home/ldx/workspace/formation_teb/src/data/dubug_data.txt");
  }
  

#ifdef USE_ANALYTIC_JACOBI
#if 0

  /**
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgePointObstacle()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    
    Eigen::Vector2d deltaS = *_measurement - bandpt->position(); 
    double angdiff = atan2(deltaS[1],deltaS[0])-bandpt->theta();
    
    double dist_squared = deltaS.squaredNorm();
    double dist = sqrt(dist_squared);
    
    double aux0 = sin(angdiff);
    double dev_left_border = penaltyBoundFromBelowDerivative(dist*fabs(aux0),cfg_->obstacles.min_obstacle_dist,cfg_->optim.penalty_epsilon);

    if (dev_left_border==0)
    {
      _jacobianOplusXi( 0 , 0 ) = 0;
      _jacobianOplusXi( 0 , 1 ) = 0;
      _jacobianOplusXi( 0 , 2 ) = 0;
      return;
    }
    
    double aux1 = -fabs(aux0) / dist;
    double dev_norm_x = deltaS[0]*aux1;
    double dev_norm_y = deltaS[1]*aux1;
    
    double aux2 = cos(angdiff) * g2o::sign(aux0);
    double aux3 = aux2 / dist_squared;
    double dev_proj_x = aux3 * deltaS[1] * dist;
    double dev_proj_y = -aux3 * deltaS[0] * dist;
    double dev_proj_angle = -aux2;
    
    _jacobianOplusXi( 0 , 0 ) = dev_left_border * ( dev_norm_x + dev_proj_x );
    _jacobianOplusXi( 0 , 1 ) = dev_left_border * ( dev_norm_y + dev_proj_y );
    _jacobianOplusXi( 0 , 2 ) = dev_left_border * dev_proj_angle;
  }
#endif
#endif
  
  /**
   * @brief Set Formation for the underlying cost function
   * @param obstacles Const pointer to an ObstContainer
   */     
  void setFormation(const std::vector<std::vector<std::vector<float>>>* formation_trajs_ptr)
  {
    _measurement = formation_trajs_ptr;
  }
    
  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacles 2D position vector containing the position of the obstacles
   */
  void setParameters(const TebConfig& cfg,const std::vector<std::vector<std::vector<float>>>* formation_trajs_ptr,double time_i)
  {
    cfg_ = &cfg;
    _measurement = formation_trajs_ptr;
    time_i_ = time_i;
    // std::cout<<"* _measurement     1:"<<std::endl <<* _measurement<<std::endl;
  }
  
protected:

  int index_; //index of robot(0, 1, 2, 3)

private:
  double time_i_;
  struct DataEntry {
    int id;
    double time_i;
    double dt;
    Eigen::Vector2d pose;
    Eigen::Matrix<double, 4, 2> measurement;
    double ff;
  };
  void writeDataToFile(const DataEntry& data, const std::string& filename) {
    // 打开文件并追加数据
    std::ofstream file(filename, std::ios::app);
    if (!file.is_open()) {
        std::cerr << "无法打开文件" << std::endl;
        return;
    }

    // 将数据按行写入文件
    file << data.id << " "
        << data.time_i << " "
        << data.dt << " "
        << data.pose[0] << " " << data.pose[1] << " "
        << data.measurement.row(0)[0] << " " << data.measurement.row(0)[1] << " "
        << data.measurement.row(1)[0] << " " << data.measurement.row(1)[1] << " "
        << data.measurement.row(2)[0] << " " << data.measurement.row(2)[1] << " "
        << data.measurement.row(3)[0] << " " << data.measurement.row(3)[1] << " "
        << data.ff << std::endl;

    // 关闭文件
    file.close();

    //std::cout << "数据已追加到文件" << std::endl;
  }

  void calculateCircleIntersection(Eigen::Matrix<double,3,2>& matrix, double r1, double r2) {
    Eigen::Vector2d point1 = matrix.row(0);//第1行
    Eigen::Vector2d point2 = matrix.row(1); 

    // 计算两个圆心之间的距离
    double distance = (point2 - point1).norm();

    // 如果两个圆心距离大于两个圆的半径之和，说明两个圆不相交，返回
    if (distance > r1 + r2) {
        return;
    }

    // 计算圆心连线与水平线之间的夹角
    double theta = std::atan2(point2.y() - point1.y(), point2.x() - point1.x());

    // 计算交点的坐标
    double intersection_x1 = point1.x() + r1 * std::cos(theta);
    double intersection_y1 = point1.y() + r1 * std::sin(theta);
    double intersection_x2 = point2.x() - r2 * std::cos(theta);
    double intersection_y2 = point2.y() - r2 * std::sin(theta);

    // 将交点坐标放入矩阵的第三行
    double d1 = abs(intersection_x1-matrix(2,0)) + abs(intersection_y1-matrix(2,1));
    double d2 = abs(intersection_x2-matrix(2,0)) + abs(intersection_y2-matrix(2,1));
    if(d1 < d2) matrix.row(2) << intersection_x1, intersection_y1;
    else matrix.row(2) << intersection_x2, intersection_y2;
    //std::cout<<"matrix row(2):"<<matrix.row(2)<<std::endl;
  }

  void getStFromTraj(double time,Eigen::Matrix<double, 4, 2>* St){
    const std::vector<std::vector<std::vector<float>>> trajs = *_measurement;
    for(int i=0;i<trajs.size();i++){
        double yaw=0,dx=0,dy=0,w=0,dt;
        //init position
        (*St)(i,0) = trajs[i][0][0];
        (*St)(i,1) = trajs[i][0][1];
        yaw = trajs[i][0][2];
        for(int j=0;j<trajs[i].size()-1;j++){
            //先找time所在索引段
            if(time>=trajs[i][j][6] && time<=trajs[i][j+1][6]){
                                    dt = time - trajs[i][j][6];
                dx = (trajs[i][j+1][0]-trajs[i][j][0])*dt/(trajs[i][j+1][6] - trajs[i][j][6]);
                dy = (trajs[i][j+1][1]-trajs[i][j][1])*dt/(trajs[i][j+1][6] - trajs[i][j][6]);
                yaw =(trajs[i][j+1][2]-trajs[i][j][2])*dt/(trajs[i][j+1][6] - trajs[i][j][6]);

                (*St)(i,0)  += dx;
                (*St)(i,1)  += dy;
                break;
            }else{
                dt = trajs[i][j+1][6] - trajs[i][j][6];
                dx = (trajs[i][j+1][0]-trajs[i][j][0])*dt/(trajs[i][j+1][6] - trajs[i][j][6]);
                dy = (trajs[i][j+1][1]-trajs[i][j][1])*dt/(trajs[i][j+1][6] - trajs[i][j][6]);
                yaw =(trajs[i][j+1][2]-trajs[i][j][2])*dt/(trajs[i][j+1][6] - trajs[i][j][6]);

                (*St)(i,0)  += dx;
                (*St)(i,1)  += dy;
            }
        }
    }
  }

  Eigen::MatrixXd calculateSNL(const Eigen::MatrixXd& matrix,bool sn=true) {
    int numPoints = matrix.rows();  // 矩阵的行数

    // 创建邻接矩阵和度矩阵
    Eigen::MatrixXd adjacencyMatrix = Eigen::MatrixXd::Zero(numPoints, numPoints);
    Eigen::MatrixXd degreeMatrix = Eigen::MatrixXd::Zero(numPoints,numPoints);
    for (int i = 0; i < numPoints; ++i) {
        for (int j = 0; j < numPoints; ++j) {
          double distance = (matrix.row(i) - matrix.row(j)).norm();  // 计算两行之间的距离
          //double distance = (matrix.row(i) - matrix.row(j)).cwiseAbs2().sum();
          // std::cout<<"row(i):"<<matrix.row(i)<<"row(j):"<<matrix.row(j)<<"distance:"<<distance<<std::endl;
          adjacencyMatrix(i, j) = distance;
          degreeMatrix(i,i) += adjacencyMatrix(i,j);
        }
    }

    // 计算拉普拉斯矩阵
    Eigen::MatrixXd laplacianMatrix = Eigen::MatrixXd::Zero(numPoints, numPoints);
    laplacianMatrix = degreeMatrix-adjacencyMatrix;
    if(!sn) return laplacianMatrix;

    // 对称归一化处理
    Eigen::MatrixXd SNL = Eigen::MatrixXd::Zero(numPoints, numPoints);
    for (int i = 0; i < numPoints; ++i) {
      for (int j = 0; j < numPoints; ++j) {
        if(i==j)
          SNL(i,j)=1;
        else
          SNL(i,j)=-adjacencyMatrix(i, j) * std::pow(degreeMatrix(i,i),-0.5) * std::pow(degreeMatrix(j,j),-0.5);
      }
    }

    return SNL;
  }
  
public: 	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
    

} // end namespace

#endif
