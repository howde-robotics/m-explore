/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
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
 *   * Neither the name of the Jiri Horner nor the names of its
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
 *********************************************************************/

#include <explore/explore.h>

#include <thread>

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
  : private_nh_("~")
  , tf_listener_(ros::Duration(10.0))
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , move_base_client_("move_base")
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  progress_timeout_ = ros::Duration(timeout);
  private_nh_.param("visualize", visualize_, false);
  private_nh_.param("potential_scale", potential_scale_, 1e-3);
  private_nh_.param("orientation_scale", orientation_scale_, 0.0);
  private_nh_.param("gain_scale", gain_scale_, 1.0);
  private_nh_.param("min_frontier_size", min_frontier_size, 0.5);
  private_nh_.param("sweep_dist_threshold", sweep_dist_threshold_, 1.0);
  private_nh_.param("last_goal_time_limit", lastGoalTimeLimit_, 10.0);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size);

  if (visualize_) {
    marker_array_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  // dragoon stuff
  statePublisher_ =
      private_nh_.advertise<dragoon_messages::stateCmd>("/commands", 1);
  sweepDistPublisher_ =
      private_nh_.advertise<std_msgs::Float32>("/sweep_dist_travelled", 1);
  stateSubscriber_ = private_nh_.subscribe("/behavior_state", 1,
                                           &Explore::stateCallback, this);
  odomSubscriber_ =
      private_nh_.subscribe("/odom", 1, &Explore::odomCallback, this);
  lastOdomTime_ = ros::Time::now();

  ROS_INFO("Waiting to connect to move_base server");
  // move_base_client_.waitForServer();
  ROS_INFO("Connected to move_base server");

  exploring_timer_ =
      relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                               [this](const ros::TimerEvent&) { makePlan(); });
}

Explore::~Explore()
{
  stop();
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;
  std_msgs::ColorRGBA cyan;
  cyan.r = 0;
  cyan.g = 1.0;
  cyan.b = 1.0;
  cyan.a = 1.0;

  ROS_DEBUG("visualising %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(0);
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::Marker::POINTS;
    m.id = int(id);
    m.pose.position = {};
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    /* Skip publishing of circular markers on frontier to avoid confusion */
    // m.type = visualization_msgs::Marker::SPHERE;
    // m.id = int(id);
    // m.pose.position = frontier.initial;
    // // scale frontier according to its cost (costier frontiers will be smaller)
    // double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.2);
    // m.scale.x = scale;
    // m.scale.y = scale;
    // m.scale.z = scale;
    // m.points = {};
    // m.color = cyan;
    // markers.push_back(m);
    // ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
}

void Explore::makePlan()
{
  if (finishedMission) {
    ROS_ERROR("**************** MISSION IS DONE, PLS STOP *****************");
    //return;
  }

  if (currentDragoonState != EXPLORE_STATE) {
    last_progress_ = ros::Time::now();
    return;
  }

  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  ROS_DEBUG("found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    processEndOfExplore();
    ROS_ERROR("**************** frontier is empty **********************");
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });
  if (frontier == frontiers.end()) {
    ROS_ERROR("******************** frontier at end *********************");
    processEndOfExplore();
    return;
  }
  geometry_msgs::Point target_position = frontier->centroid;

  // time out if we are not making any progress
  bool same_goal = prev_goal_ == target_position;
  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = ros::Time::now();
    prev_distance_ = frontier->min_distance;
  }
  // black list if we've made no progress for a long time
  if (ros::Time::now() - last_progress_ > progress_timeout_) {
    frontier_blacklist_.push_back(target_position);
    ROS_DEBUG("Adding current goal to black list");
    makePlan();
    return;
  }

  // we don't need to do anything if we still pursuing the same goal
  // and planner has been running
  if (same_goal && !justStartedExplore_) {
    return;
  }

  // We go to sweep when we changed goal; it's not the first goal in this
  // explore session; and we have travelled a certain distance this explore
  // session
  if (sweep_dist_travelled_ > sweep_dist_threshold_) {
    // ROS_WARN("CHANGEEEEE TOOOOO SWEEEEEEEEEEEP");
    // Only reset this distance counter when it meets the threshold.
    // Hence, when we come back to explore from approach, this is not reset.
    sweep_dist_travelled_ = 0.0;
    dragoon_messages::stateCmd stateMsg;
    stateMsg.event = "GOAL REACHED";
    stateMsg.value = true;
    statePublisher_.publish(stateMsg);
    return;
  }

  /* If we are exploring, send a goal to Move base */
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.position = target_position;
  goal.target_pose.pose.orientation.w = 1.;
  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();

  /* Only send the goal if we are in the explore state */
  // send goal to move_base if we have something new to pursue
  move_base_client_.sendGoal(
      goal, [this, target_position](
                const actionlib::SimpleClientGoalState& status,
                const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status, result, target_position);
      });

  justStartedExplore_ = false;
}

bool Explore::goalOnBlacklist(const geometry_msgs::Point& goal)
{
  constexpr static size_t tolerace = 5;
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
                          const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::Point& frontier_goal)
{
  ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
  if (status == actionlib::SimpleClientGoalState::ABORTED) {
    frontier_blacklist_.push_back(frontier_goal);
    ROS_DEBUG("Adding current goal to black list");
  }

  // find new goal immediatelly regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  ROS_ERROR("*********************************************** IN ORIGINAL REEACHED GOAL ***********************************");
  oneshot_ = relative_nh_.createTimer(
      ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
      true);

  // change the state by publishing the goal reached event
  if (currentDragoonState == EXPLORE_STATE) {
    dragoon_messages::stateCmd stateMsg;
    stateMsg.event = "GOAL REACHED";
    stateMsg.value = true;
    statePublisher_.publish(stateMsg);
  }
}

void Explore::reachedLastGoal(
    const actionlib::SimpleClientGoalState& status,
    const move_base_msgs::MoveBaseResultConstPtr& result,
    const geometry_msgs::Point& frontier_goal)
{
  ROS_ERROR("*********************************************** IN NEW REEACHED GOAL ***********************************");
  sendLastSweepAndStop();
}

void Explore::start()
{
  exploring_timer_.start();
}

void Explore::processEndOfExplore()
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.position = prev_goal_;
  goal.target_pose.pose.orientation.w = 1.;
  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();

  // send goal to move_base the last goal
  move_base_client_.sendGoal(
      goal, [this](const actionlib::SimpleClientGoalState& status,
                   const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedLastGoal(status, result, prev_goal_);
      });

  // Add time limit on Dragoon's attempt to go to the last goal
  // Because the last goal might not be reachable
  ros::Duration(lastGoalTimeLimit_).sleep();

  ROS_ERROR("*********************************************** AFTER SLEEP ***********************************");
  sendLastSweepAndStop();
}

void Explore::sendLastSweepAndStop()
{
  ROS_ERROR("**************************** sendLastSweepAndStop ************************************");
  move_base_client_.cancelAllGoals();
  exploring_timer_.stop();
  if (!finishedMission) {
  // Send goal reached to go to sweep
  dragoon_messages::stateCmd stateMsg;
  stateMsg.event = "CONCLUDE SWEEP";
  stateMsg.value = true;
  statePublisher_.publish(stateMsg);
  // send conclude sweep to go to idle at after sweep
  stateMsg.event = "GOAL REACHED";
  statePublisher_.publish(stateMsg);
  }
  finishedMission = true;
}

void Explore::stop()
{
  move_base_client_.cancelAllGoals();
  exploring_timer_.stop();
  ROS_INFO("Exploration stopped.");
  dragoon_messages::stateCmd stateMsg;
  stateMsg.event = "STOP";
  stateMsg.value = true;
  statePublisher_.publish(stateMsg);
}

void Explore::stateCallback(const std_msgs::Int32ConstPtr msg)
{
  /* Set the current dragoon state when the state is updated */
  ROS_DEBUG_STREAM("[EXPLORE] Updating Dragoon state from "
                   << currentDragoonState << " to " << msg->data);
  currentDragoonState = (State)msg->data;
  if (msg->data == EXPLORE_STATE) {
    // reset timer used for blacklist
    last_progress_ = ros::Time::now();

    justStartedExplore_ = true;

    // force to make a plan
    oneshot_ = relative_nh_.createTimer(
        ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
        true);
  }
}

void Explore::odomCallback(const nav_msgs::Odometry::ConstPtr msg)
{
  ros::Duration dt = ros::Time::now() - lastOdomTime_;
  if (dt.toSec() > 0.2) {
    ROS_WARN("Odom in explore is too old");
  } else {
    sweep_dist_travelled_ +=
        std::max(msg->twist.twist.linear.x, 0.0) * dt.toSec();
    std_msgs::Float32 msg;
    msg.data = sweep_dist_travelled_;
    sweepDistPublisher_.publish(msg);
  }
  lastOdomTime_ = ros::Time::now();
}

}  // namespace explore

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  explore::Explore explore;
  ros::spin();

  return 0;
}
