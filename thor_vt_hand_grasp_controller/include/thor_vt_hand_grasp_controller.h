//=================================================================================================
// Copyright (c) 2015, Alberto Romay, TU Darmstadt
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

#ifndef THOR_VT_HAND_GRASP_CONTROLLER_H__
#define THOR_VT_HAND_GRASP_CONTROLLER_H__

#include <vigir_manipulation_controller/vigir_manipulation_controller.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

namespace thor_vt_hand_grasp_controller{

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryActionClient;

  //////////////////////////////////////////////////////////////////////////
  // Defines the VT_Hand class as a derived class of VigirManipulationController plus
  // extra VT_Hand specific info
  //////////////////////////////////////////////////////////////////////////

  class THOR_VT_Hand_GraspController: virtual public vigir_manipulation_controller::VigirManipulationController
  {
    public:

        THOR_VT_Hand_GraspController();
        ~THOR_VT_Hand_GraspController();
        void initializeVT_Hand_GraspController(ros::NodeHandle& nh, ros::NodeHandle& nhp);

    protected:

        void graspCommandCallback(const vigir_grasp_msgs::GraspState &grasp)  ;
        vigir_manipulation_controller::GraspQuality processHandTactileData() ;


    private:

        //Trajectory Action
        control_msgs::FollowJointTrajectoryGoal    trajectory_action_;
        TrajectoryActionClient*                    trajectory_client_;

        void trajectoryActiveCB();
        void trajectoryFeedbackCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);
        void trajectoryDoneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr &result);


  };

}
#endif

