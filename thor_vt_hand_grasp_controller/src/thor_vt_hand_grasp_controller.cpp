    //=================================================================================================
// Copyright (c) 2013, David Conner, TORC Robotics
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the names of TU Darmstadt, Virginia Tech, Oregon State, nor TORC Robotics,
//       nor the names of its contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <thor_vt_hand_grasp_controller.h>

namespace thor_vt_hand_grasp_controller{


    THOR_VT_Hand_GraspController::THOR_VT_Hand_GraspController()
      : VigirManipulationController() // explicitly initialize the base class
    {
    }

    THOR_VT_Hand_GraspController::~THOR_VT_Hand_GraspController()
    {
        std::cout << "Shutting down the VT Hand grasping controller ..." << std::endl;
    }


    //////////////////////////////////////////////////////////////////////////
    // VT Hand class functions
    //////////////////////////////////////////////////////////////////////////

    void THOR_VT_Hand_GraspController::initializeVT_Hand_GraspController(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    {
      // Initialize the generic manipulation controller components
      initializeManipulationController(nh,nhp);

      ROS_INFO("Initializing VT Hand Grasp controller");

      //Initializing Trajectory action for fingers
      this->trajectory_action_.trajectory.joint_names.resize(hand_joint_names_.size());
      this->trajectory_action_.trajectory.points.resize(1);
      this->trajectory_action_.trajectory.points[0].positions.resize(hand_joint_names_.size());
      this->trajectory_action_.trajectory.points[0].time_from_start = ros::Duration(0.5);
      this->trajectory_action_.goal_time_tolerance                  = ros::Duration(5.0);

      ROS_INFO("Trajectory action initialized");

      for(int i = 0; i < hand_joint_names_.size(); i++){
          ROS_INFO("Joint %d: %s",i,hand_joint_names_[i].c_str());
          this->trajectory_action_.trajectory.joint_names[i]         = hand_joint_names_[i];

          //THIS ARE SPECIFIC FROM VT HAND
          this->trajectory_action_.trajectory.points[0].positions[0]  = 2.685;		//f0_j0
          this->trajectory_action_.trajectory.points[0].positions[1]  = 2.685;		//f1_j0
          //this->trajectory_action_.trajectory.points[0].positions[2]  = 0.8;		//f0_j1
          //this->trajectory_action_.trajectory.points[0].positions[3]  = 0.8;		//f1_j1
      }

      ROS_INFO("Close joint positions initialized");

      this->trajectory_client_ = new  thor_vt_hand_grasp_controller::TrajectoryActionClient("/thor_mang/"+this->hand_side_+"_hand_traj_controller/follow_joint_trajectory", true);
      while(!this->trajectory_client_->waitForServer(ros::Duration(5.0)))
         ROS_INFO("Waititing for %s TrajectoryActionServer", this->hand_side_.c_str());

      //Sending Initial finger postions, MAX joint limit (CLOSE) from URDF
      if(this->trajectory_client_->isServerConnected())
      {
          ROS_INFO("Sending trajectory action");
          this->trajectory_action_.trajectory.header.stamp = ros::Time::now();
          this->trajectory_client_->sendGoal(trajectory_action_,
                                       boost::bind(&THOR_VT_Hand_GraspController::trajectoryDoneCb, this, _1, _2),
                                       boost::bind(&THOR_VT_Hand_GraspController::trajectoryActiveCB, this),
                                       boost::bind(&THOR_VT_Hand_GraspController::trajectoryFeedbackCB, this, _1));
      }
      else
      {
          ROS_ERROR("TrajectoryActionClient: Server not yet connected!");
      }

    }
    void THOR_VT_Hand_GraspController::graspCommandCallback(const vigir_grasp_msgs::GraspState &grasp)
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);

        //THIS IS VT HAND SPECIFIC

        switch(grasp.grasp_state.data){
        case vigir_grasp_msgs::GraspState::GRASP_ID:
            this->trajectory_action_.trajectory = grasp.grasp.grasp_posture;
        break;
        case vigir_grasp_msgs::GraspState::OPEN:
            this->trajectory_action_.trajectory.points[0].positions[0]  = 0.0;
            this->trajectory_action_.trajectory.points[0].positions[1]  = 0.0;
        break;
        case vigir_grasp_msgs::GraspState::CLOSE:
            this->trajectory_action_.trajectory.points[0].positions[0]  = 2.685;
            this->trajectory_action_.trajectory.points[0].positions[1]  = 2.685;
        break;
        case vigir_grasp_msgs::GraspState::PERCENTAGE:
            this->trajectory_action_.trajectory.points[0].positions[0]  = float(grasp.grip.data > 100 ? 100 : grasp.grip.data)*0.02685;
            this->trajectory_action_.trajectory.points[0].positions[1]  = float(grasp.grip.data > 100 ? 100 : grasp.grip.data)*0.02685;
        break;
        default:return;
        }

        //Create ROS trajectory and publish
        if(this->trajectory_client_->isServerConnected())
        {
            this->trajectory_action_.trajectory.header.stamp = ros::Time::now();
            this->trajectory_client_->sendGoal(trajectory_action_,
                                         boost::bind(&THOR_VT_Hand_GraspController::trajectoryDoneCb, this, _1, _2),
                                         boost::bind(&THOR_VT_Hand_GraspController::trajectoryActiveCB, this),
                                         boost::bind(&THOR_VT_Hand_GraspController::trajectoryFeedbackCB, this, _1));
        }
        else
        {
            ROS_ERROR("TrajectoryActionClient: Server not connected!");
        }

        return;
    }

    void THOR_VT_Hand_GraspController::trajectoryActiveCB()
    {
        //ROS_INFO("TrajectoryActionClient: Status changed to active.");
    }

    void THOR_VT_Hand_GraspController::trajectoryFeedbackCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
    {
        ROS_INFO("TrajectoryActionClient: Feedback received.");// pos[0]= %f", feedback->actual.positions[0]);
    }

    void THOR_VT_Hand_GraspController::trajectoryDoneCb(const actionlib::SimpleClientGoalState& state,
                                                      const control_msgs::FollowJointTrajectoryResultConstPtr& result)
    {
        ROS_INFO("Fingers Trajectory finished in state [%s]", state.toString().c_str());
    }


    vigir_manipulation_controller::GraspQuality THOR_VT_Hand_GraspController::processHandTactileData()
    {
        return vigir_manipulation_controller::NO_GRASP_QUALITY;
    }

} /// end namespace thor_vt_hand_grasp_controller

int main(int argc, char** argv)
{
  ros::init(argc, argv, "thor_vt_hand_controller");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  thor_vt_hand_grasp_controller::THOR_VT_Hand_GraspController vt_hand_controller;

  ROS_WARN(" Initialize the VT hand grasp controller ...");
  vt_hand_controller.initializeVT_Hand_GraspController(nh, nhp);

  ROS_WARN(" Start the ros spinner ...");
  ros::spin();
  return 0;
}
