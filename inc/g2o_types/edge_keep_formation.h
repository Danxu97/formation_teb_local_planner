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
class EdgeKeepFormation : public BaseTebUnaryEdge<1, const Eigen::Matrix<double, 4, 2>*, VertexPose>  //1指定error的个数，会结合information这个权重矩阵进行总error计算
{
public:
    
  /**
   * @brief Construct edge and specify the index of robot
   */    
  EdgeKeepFormation() : index_(3)
  {
    _measurement = NULL;
  }

  /**
   * @brief Construct edge and specify the index of robot
   * @param index_ index of robot(0, 1, 2, 3)
   */    
  EdgeKeepFormation(int index) : index_(index)
  {
    _measurement = NULL;
  }
 
   //step 2: m:[x1,y1,'''x4,y4]
  /**
   * @brief Actual cost function
   */   
  void computeError()
  {
    //std::cout<<"* _measurement"<<std::endl <<* _measurement<<std::endl;

    //ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig(), setFormation() and on EdgeFormation()");    

    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    Eigen::Matrix4d SNL0;

    //rect
    SNL0 <<  1.0000,   -0.2929,   -0.4142,   -0.2929,
            -0.2929,    1.0000,   -0.2929,   -0.4142,
            -0.4142,   -0.2929,    1.0000,   -0.2929,
            -0.2929,   -0.4142,   -0.2929,    1.0000;

    Eigen::Matrix4d SNL = Eigen::Matrix4d::Zero();

    Eigen::Matrix4d Adj = Eigen::Matrix4d::Zero();

    Eigen::Vector4d Deg = Eigen::Vector4d::Zero();

    Eigen::Vector2d pose0, pose1;


    for (int i = 0; i < 4; ++i)
    {
      if (i == index_)
        pose0 = bandpt->position();
      else
        pose0 = _measurement->row(i);

      for (int j = 0; j < 4; ++j) 
      {
        if (j == index_)
          pose1 = bandpt->position();
        else
          pose1 = _measurement->row(j);
        
        Adj(i, j) = (pose0 - pose1).norm();

        Deg(i) = Deg(i) + Adj(i, j);
      }
    }

    for (int i = 0; i < 4; ++i) 
    {
      for (int j = 0; j < 4; ++j) 
      {
        if (i == j)
          SNL(i, j) = 1;
        else
          SNL(i, j) = -Adj(i, j) * pow(Deg(i), -0.5) * pow(Deg(j), -0.5);
      }
    }

    _error[0] = (SNL0 - SNL).norm();

    DataEntry data;
    data.id = index_;
    data.pose0 = bandpt->position();
    data.measurement = *_measurement;
    data.ff = _error[0];
    // 将数据写入文件
    writeDataToFile(data, "/home/ldx/workspace/formation_teb/src/data/dubug_data.txt");
    //std::cout<<"_measurement    2:"<<std::endl <<* _measurement<<std::endl;
    //ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeFormation::computeError() _error[0]=%f\n",_error[0]);
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
  void setFormation(const Eigen::Matrix<double, 4, 2>* formation_position_ptr)
  {
    _measurement = formation_position_ptr;
  }
    
  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacles 2D position vector containing the position of the obstacles
   */
  void setParameters(const TebConfig& cfg, const Eigen::Matrix<double, 4, 2>* formation_position_ptr)
  {
    cfg_ = &cfg;
    _measurement = formation_position_ptr;
    // std::cout<<"* _measurement     1:"<<std::endl <<* _measurement<<std::endl;
  }
  
protected:

  int index_; //index of robot(0, 1, 2, 3)

private:
  struct DataEntry {
    int id;
    Eigen::Vector2d pose0;
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
        << data.pose0[0] << " " << data.pose0[1] << " "
        << data.measurement.row(0)[0] << " " << data.measurement.row(0)[1] << " "
        << data.measurement.row(1)[0] << " " << data.measurement.row(1)[1] << " "
        << data.measurement.row(2)[0] << " " << data.measurement.row(2)[1] << " "
        << data.measurement.row(3)[0] << " " << data.measurement.row(3)[1] << " "
        << data.ff << std::endl;

    // 关闭文件
    file.close();

    //std::cout << "数据已追加到文件" << std::endl;
  }
  
public: 	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
    

} // end namespace

#endif
