/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld + Hamburg University
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
 *   * Neither the name of Bielefeld University nor the names of its
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
 *********************************************************************/

/* Authors: Robert Haschke, Michael Goerner */

#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>
#include <moveit/planning_scene/planning_scene.h>
#include <rviz_marker_tools/marker_creation.h>

#include <eigen_conversions/eigen_msg.h>

namespace moveit {
namespace task_constructor {
namespace stages {

GeneratePose::GeneratePose(const std::string& name) : MonitoringGenerator(name) {
	auto& p = properties();
  p.declare<double>("angle_delta", 0.0, "angular steps (rad)");
	p.declare<geometry_msgs::PoseStamped>("pose", "target pose to pass on in spawned states");
}

void GeneratePose::reset() {
	upstream_solutions_.clear();
	MonitoringGenerator::reset();
}

void GeneratePose::onNewSolution(const SolutionBase& s) {
	// It's safe to store a pointer to this solution, as the generating stage stores it
	upstream_solutions_.push(&s);
}

bool GeneratePose::canCompute() const {
	return upstream_solutions_.size() > 0;
}

void GeneratePose::compute() {
	if (upstream_solutions_.empty())
		return;

	planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();

  const auto& props = properties();
  geometry_msgs::PoseStamped target_pose_msg;
	geometry_msgs::PoseStamped target_pose = properties().get<geometry_msgs::PoseStamped>("pose");
	if (target_pose.header.frame_id.empty())
		target_pose.header.frame_id = scene->getPlanningFrame();
	else if (!scene->knowsFrameTransform(target_pose.header.frame_id)) {
		ROS_WARN_NAMED("GeneratePose", "Unknown frame: '%s'", target_pose.header.frame_id.c_str());
		return;
  }

  target_pose_msg.header.frame_id = target_pose.header.frame_id;
  Eigen::Isometry3d seed_target_pose;
  tf::poseMsgToEigen(target_pose.pose, seed_target_pose);

  double angle_delta = props.get<double>("angle_delta");
  if(angle_delta != 0.) {
    //if the user has set angle_delta sample the target pose about the z-axis
    double current_angle_ = 0.0;
    while (current_angle_ < 2. * M_PI && current_angle_ > -2. * M_PI) {
      // rotate object pose about z-axis
      Eigen::Isometry3d target_pose(seed_target_pose * Eigen::AngleAxisd(current_angle_, Eigen::Vector3d::UnitZ()));
      current_angle_ += angle_delta;

      InterfaceState state(scene);
      tf::poseEigenToMsg(target_pose, target_pose_msg.pose);
      state.properties().set("target_pose", target_pose_msg);

      SubTrajectory trajectory;
      trajectory.setCost(0.0);
      trajectory.setComment(std::to_string(current_angle_));

      // add frame at target pose
      rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_msg, 0.1, "target frame");

      spawn(std::move(state), std::move(trajectory));
    }
  }
  else {
    //else the user has only set the target pose
    InterfaceState state(scene);
    state.properties().set("target_pose", target_pose);

    SubTrajectory trajectory;
    trajectory.setCost(0.0);

    rviz_marker_tools::appendFrame(trajectory.markers(), target_pose, 0.1, "pose frame");

    spawn(std::move(state), std::move(trajectory));
  }
}
}
}
}
