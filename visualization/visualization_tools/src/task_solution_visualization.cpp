/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Dave Coleman, Robert Haschke */

#include <moveit_task_constructor/visualization_tools/display_solution.h>
#include <moveit_task_constructor/visualization_tools/task_solution_visualization.h>
#include <moveit_task_constructor/visualization_tools/task_solution_panel.h>

#include <moveit/rviz_plugin_render_tools/planning_scene_render.h>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
#include <moveit/rviz_plugin_render_tools/planning_link_updater.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/trajectory_tools.h>

#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>
#include <rviz/properties/property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/window_manager_interface.h>
#include <rviz/panel_dock_widget.h>

namespace moveit_rviz_plugin
{
TaskSolutionVisualization::TaskSolutionVisualization(rviz::Property* parent, rviz::Display* display)
  : display_(display)
{
  // trajectory properties
  interrupt_display_property_ = new rviz::BoolProperty("Interrupt Display", false,
                                                       "Immediately show newly planned trajectory, "
                                                       "interrupting the currently displayed one.",
                                                       parent);

  loop_display_property_ = new rviz::BoolProperty("Loop Animation", false,
                                                  "Indicates whether the last received path is to be animated in a loop",
                                                  parent, SLOT(changedLoopDisplay()), this);

  trail_display_property_ = new rviz::BoolProperty("Show Trail", false, "Show a path trail",
                                                   parent, SLOT(changedShowTrail()), this);

  state_display_time_property_ = new rviz::EditableEnumProperty("State Display Time", "0.05 s",
                                                                "The amount of wall-time to wait in between displaying "
                                                                "states along a received trajectory path",
                                                                parent, SLOT(changedStateDisplayTime()), this);
  state_display_time_property_->addOptionStd("REALTIME");
  state_display_time_property_->addOptionStd("0.05 s");
  state_display_time_property_->addOptionStd("0.1 s");
  state_display_time_property_->addOptionStd("0.5 s");

  trail_step_size_property_ = new rviz::IntProperty("Trail Step Size", 1,
                                                    "Specifies the step size of the samples shown in the trajectory trail.",
                                                    parent, SLOT(changedTrailStepSize()), this);
  trail_step_size_property_->setMin(1);


  // robot properties
  robot_property_ = new rviz::Property("Robot", QString(), QString(), parent);
  robot_visual_enabled_property_ = new rviz::BoolProperty("Show Robot Visual", true,
                                                          "Indicates whether the geometry of the robot as defined for "
                                                          "visualisation purposes should be displayed",
                                                          robot_property_, SLOT(changedRobotVisualEnabled()), this);

  robot_collision_enabled_property_ = new rviz::BoolProperty("Show Robot Collision", false,
                                                             "Indicates whether the geometry of the robot as defined "
                                                             "for collision detection purposes should be displayed",
                                                             robot_property_, SLOT(changedRobotCollisionEnabled()), this);

  robot_alpha_property_ = new rviz::FloatProperty("Robot Alpha", 0.5f, "Specifies the alpha for the robot links",
                                                  robot_property_, SLOT(changedRobotAlpha()), this);
  robot_alpha_property_->setMin(0.0);
  robot_alpha_property_->setMax(1.0);

  robot_color_property_ = new rviz::ColorProperty("Fixed Robot Color", QColor(150, 50, 150), "The color of the animated robot",
                                                  robot_property_, SLOT(changedRobotColor()), this);

  enable_robot_color_property_ = new rviz::BoolProperty("Use Fixed Robot Color", false,
                                                        "Specifies whether the fixed robot color should be used."
                                                        " If not, the original color is used.",
                                                        robot_property_, SLOT(enabledRobotColor()), this);


  // planning scene properties
  scene_enabled_property_ = new rviz::BoolProperty("Scene", true, "Show Planning Scene", parent,
                                                   SLOT(changedSceneEnabled()), this);

  scene_alpha_property_ = new rviz::FloatProperty("Scene Alpha", 0.9f, "Specifies the alpha for the scene geometry",
                                                  scene_enabled_property_, SLOT(renderCurrentScene()), this);
  scene_alpha_property_->setMin(0.0);
  scene_alpha_property_->setMax(1.0);

  scene_color_property_ = new rviz::ColorProperty("Scene Color", QColor(50, 230, 50),
                                                  "The color for the planning scene obstacles (if a color is not defined)",
                                                  scene_enabled_property_, SLOT(renderCurrentScene()), this);

  attached_body_color_property_ = new rviz::ColorProperty("Attached Body Color", QColor(150, 50, 150), "The color for the attached bodies",
                                                          scene_enabled_property_, SLOT(changedAttachedBodyColor()), this);

  octree_render_property_ = new rviz::EnumProperty("Voxel Rendering", "Occupied Voxels", "Select voxel type.",
                                                   scene_enabled_property_, SLOT(renderCurrentScene()), this);

  octree_render_property_->addOption("Occupied Voxels", OCTOMAP_OCCUPIED_VOXELS);
  octree_render_property_->addOption("Free Voxels", OCTOMAP_FREE_VOXELS);
  octree_render_property_->addOption("All Voxels", OCTOMAP_FREE_VOXELS | OCTOMAP_OCCUPIED_VOXELS);

  octree_coloring_property_ = new rviz::EnumProperty("Voxel Coloring", "Z-Axis", "Select voxel coloring mode",
                                                     scene_enabled_property_, SLOT(renderCurrentScene()), this);

  octree_coloring_property_->addOption("Z-Axis", OCTOMAP_Z_AXIS_COLOR);
  octree_coloring_property_->addOption("Cell Probability", OCTOMAP_PROBABLILTY_COLOR);
}

TaskSolutionVisualization::~TaskSolutionVisualization()
{
  clearTrail();
  solution_to_display_.reset();
  displaying_solution_.reset();

  scene_render_.reset();
  robot_render_.reset();
  if (slider_dock_panel_)
    delete slider_dock_panel_;
}

void TaskSolutionVisualization::onInitialize(Ogre::SceneNode* scene_node, rviz::DisplayContext* context)
{
  // Save pointers for later use
  scene_node_ = scene_node;
  context_ = context;

  // Load trajectory robot
  robot_render_.reset(new RobotStateVisualization(scene_node_, context_, "Solution Trajectory", robot_property_));
  robot_render_->setVisualVisible(robot_visual_enabled_property_->getBool());
  robot_render_->setCollisionVisible(robot_collision_enabled_property_->getBool());
  robot_render_->setVisible(false);

  scene_render_.reset(new PlanningSceneRender(scene_node_, context_, robot_render_));
  scene_render_->getGeometryNode()->setVisible(false);

  rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  if (window_context)
  {
    slider_panel_ = new TaskSolutionPanel(window_context->getParentWindow());
    slider_dock_panel_ =
        window_context->addPane(display_->getName() + " - Slider", slider_panel_);
    slider_dock_panel_->setIcon(display_->getIcon());
    connect(slider_dock_panel_, SIGNAL(visibilityChanged(bool)), this,
            SLOT(sliderPanelVisibilityChange(bool)));
    slider_panel_->onInitialize();
  }
}

void TaskSolutionVisualization::setName(const QString& name)
{
  if (slider_dock_panel_)
    slider_dock_panel_->setWindowTitle(name + " - Slider");
}

void TaskSolutionVisualization::onRobotModelLoaded(robot_model::RobotModelConstPtr robot_model)
{
  // Error check
  if (!robot_model)
  {
    ROS_ERROR_STREAM_NAMED("task_solution_visualization", "No robot model found");
    return;
  }

  scene_.reset(new planning_scene::PlanningScene(robot_model));

  robot_render_->load(*robot_model->getURDF());  // load rviz robot
  enabledRobotColor();  // force-refresh to account for saved display configuration
}

void TaskSolutionVisualization::reset()
{
  clearTrail();
  solution_to_display_.reset();
  displaying_solution_.reset();
  animating_ = false;

  robot_render_->setVisualVisible(robot_visual_enabled_property_->getBool());
  robot_render_->setCollisionVisible(robot_collision_enabled_property_->getBool());
  robot_render_->setVisible(false);
}

void TaskSolutionVisualization::clearTrail()
{
  for (std::size_t i = 0; i < trail_.size(); ++i)
    delete trail_[i];
  trail_.clear();
}

void TaskSolutionVisualization::changedLoopDisplay()
{
  robot_render_->setVisible(display_->isEnabled() && displaying_solution_ && animating_);
  if (loop_display_property_->getBool() && slider_panel_)
    slider_panel_->pauseButton(false);
}

void TaskSolutionVisualization::changedShowTrail()
{
  clearTrail();

  if (!trail_display_property_->getBool())
    return;
  robot_trajectory::RobotTrajectoryPtr t = solution_to_display_;
  if (!t)
    t = displaying_solution_;
  if (!t)
    return;

  int stepsize = trail_step_size_property_->getInt();
  // always include last trajectory point
  trail_.resize((int)std::ceil((t->getWayPointCount() + stepsize - 1) / (float)stepsize));
  for (std::size_t i = 0; i < trail_.size(); i++)
  {
    int waypoint_i = std::min(i * stepsize, t->getWayPointCount() - 1);  // limit to last trajectory point
    rviz::Robot* r = new rviz::Robot(scene_node_, context_, "Trail Robot " + boost::lexical_cast<std::string>(i), NULL);
    r->load(*scene_->getRobotModel()->getURDF());
    r->setVisualVisible(robot_visual_enabled_property_->getBool());
    r->setCollisionVisible(robot_collision_enabled_property_->getBool());
    r->setAlpha(robot_alpha_property_->getFloat());
    r->update(PlanningLinkUpdater(t->getWayPointPtr(waypoint_i)));
    if (enable_robot_color_property_->getBool())
      setRobotColor(r, robot_color_property_->getColor());
    r->setVisible(display_->isEnabled() && (!animating_ || waypoint_i <= current_state_));
    trail_[i] = r;
  }
}

void TaskSolutionVisualization::changedTrailStepSize()
{
  if (trail_display_property_->getBool())
    changedShowTrail();
}

void TaskSolutionVisualization::changedRobotAlpha()
{
  robot_render_->setAlpha(robot_alpha_property_->getFloat());
  for (std::size_t i = 0; i < trail_.size(); ++i)
    trail_[i]->setAlpha(robot_alpha_property_->getFloat());
}

void TaskSolutionVisualization::changedRobotVisualEnabled()
{
  if (display_->isEnabled())
  {
    robot_render_->setVisualVisible(robot_visual_enabled_property_->getBool());
    robot_render_->setVisible(display_->isEnabled() && displaying_solution_ && animating_);
    for (std::size_t i = 0; i < trail_.size(); ++i)
      trail_[i]->setVisualVisible(robot_visual_enabled_property_->getBool());
  }
}

void TaskSolutionVisualization::changedRobotCollisionEnabled()
{
  if (display_->isEnabled())
  {
    robot_render_->setCollisionVisible(robot_collision_enabled_property_->getBool());
    robot_render_->setVisible(display_->isEnabled() && displaying_solution_ && animating_);
    for (std::size_t i = 0; i < trail_.size(); ++i)
      trail_[i]->setCollisionVisible(robot_collision_enabled_property_->getBool());
  }
}

void TaskSolutionVisualization::changedStateDisplayTime()
{
}

void TaskSolutionVisualization::onEnable()
{
  changedRobotAlpha();  // set alpha property
  changedSceneEnabled();  // show/hide planning scene

  robot_render_->setVisualVisible(robot_visual_enabled_property_->getBool());
  robot_render_->setCollisionVisible(robot_collision_enabled_property_->getBool());
  robot_render_->setVisible(displaying_solution_ && animating_);
  for (std::size_t i = 0; i < trail_.size(); ++i)
  {
    trail_[i]->setVisualVisible(robot_visual_enabled_property_->getBool());
    trail_[i]->setCollisionVisible(robot_collision_enabled_property_->getBool());
    trail_[i]->setVisible(true);
  }
}

void TaskSolutionVisualization::onDisable()
{
  if (scene_render_)
    scene_render_->getGeometryNode()->setVisible(false);

  robot_render_->setVisible(false);
  for (std::size_t i = 0; i < trail_.size(); ++i)
    trail_[i]->setVisible(false);

  displaying_solution_.reset();
  animating_ = false;
  if (slider_panel_)
    slider_panel_->onDisable();
}

void TaskSolutionVisualization::interruptCurrentDisplay()
{
  // update() starts a new trajectory as soon as it is available
  // interrupting may cause the newly received trajectory to interrupt
  // hence, only interrupt when current_state_ already advanced past first
  if (current_state_ > 0)
    animating_ = false;
}

float TaskSolutionVisualization::getStateDisplayTime()
{
  std::string tm = state_display_time_property_->getStdString();
  if (tm == "REALTIME")
    return -1.0;
  else
  {
    boost::replace_all(tm, "s", "");
    boost::trim(tm);
    float t = 0.05f;
    try
    {
      t = boost::lexical_cast<float>(tm);
    }
    catch (const boost::bad_lexical_cast& ex)
    {
      state_display_time_property_->setStdString("0.05 s");
    }
    return t;
  }
}

void TaskSolutionVisualization::dropTrajectory()
{
  drop_displaying_solution_ = true;
}

void TaskSolutionVisualization::update(float wall_dt, float ros_dt)
{
  if (drop_displaying_solution_)
  {
    animating_ = false;
    displaying_solution_.reset();
    robot_render_->setVisible(false);
    slider_panel_->update(0);
    drop_displaying_solution_ = false;
  }
  if (!animating_)
  {  // finished last animation?
    boost::mutex::scoped_lock lock(display_solution_mutex_);

    // new trajectory available to display?
    if (solution_to_display_ && !solution_to_display_->empty())
    {
      animating_ = true;
      displaying_solution_ = solution_to_display_;
      changedShowTrail();
      if (slider_panel_)
        slider_panel_->update(solution_to_display_->getWayPointCount());
    }
    else if (displaying_solution_)
    {
      if (loop_display_property_->getBool())
      {  // do loop? -> start over too
        animating_ = true;
      }
      else if (slider_panel_ && slider_panel_->isVisible())
      {
        if (slider_panel_->getSliderPosition() == (int)displaying_solution_->getWayPointCount() - 1)
        {  // show the last waypoint if the slider is enabled
          robot_render_->update(
                displaying_solution_->getWayPointPtr(displaying_solution_->getWayPointCount() - 1));
        }
        else
          animating_ = true;
      }
    }
    solution_to_display_.reset();

    if (animating_)
    {
      current_state_ = -1;
      current_state_time_ = std::numeric_limits<float>::infinity();
      robot_render_->update(displaying_solution_->getFirstWayPointPtr());
      robot_render_->setVisible(display_->isEnabled());
      if (slider_panel_)
        slider_panel_->setSliderPosition(0);
    }
  }

  if (animating_)
  {
    float tm = getStateDisplayTime();
    if (tm < 0.0)  // if we should use realtime
      tm = displaying_solution_->getWayPointDurationFromPrevious(current_state_ + 1);
    if (current_state_time_ > tm)
    {
      if (slider_panel_ && slider_panel_->isVisible() && slider_panel_->isPaused())
        current_state_ = slider_panel_->getSliderPosition();
      else
        ++current_state_;
      int waypoint_count = displaying_solution_->getWayPointCount();
      if (current_state_ < waypoint_count)
      {
        if (slider_panel_)
          slider_panel_->setSliderPosition(current_state_);
        robot_render_->update(displaying_solution_->getWayPointPtr(current_state_));
        for (std::size_t i = 0; i < trail_.size(); ++i)
          trail_[i]->setVisible(
                std::min(waypoint_count - 1, static_cast<int>(i) * trail_step_size_property_->getInt()) <=
                current_state_);
      }
      else
      {
        animating_ = false;  // animation finished
        robot_render_->setVisible(loop_display_property_->getBool());
        if (!loop_display_property_->getBool() && slider_panel_)
          slider_panel_->pauseButton(true);
      }
      current_state_time_ = 0.0f;
    }
    current_state_time_ += wall_dt;
  }
}

void TaskSolutionVisualization::renderPlanningScene(const planning_scene::PlanningSceneConstPtr &scene)
{
  if (!scene_render_ || !scene_enabled_property_->getBool())
    return;

  QColor color = scene_color_property_->getColor();
  rviz::Color env_color(color.redF(), color.greenF(), color.blueF());
  color = attached_body_color_property_->getColor();
  rviz::Color attached_color(color.redF(), color.greenF(), color.blueF());

  scene_render_->renderPlanningScene(scene, env_color, attached_color,
                                     static_cast<OctreeVoxelRenderMode>(octree_render_property_->getOptionInt()),
                                     static_cast<OctreeVoxelColorMode>(octree_coloring_property_->getOptionInt()),
                                     scene_alpha_property_->getFloat());
}

void TaskSolutionVisualization::showTrajectory(const moveit_task_constructor::Solution& msg)
{
  // Error check
  if (!scene_)
    return;

  if (msg.start_scene.robot_model_name != scene_->getRobotModel()->getName())
    ROS_WARN("Received a trajectory to display for model '%s' but model '%s' was expected",
             msg.start_scene.robot_model_name .c_str(),
             scene_->getRobotModel()->getName().c_str());

  scene_->setPlanningSceneMsg(msg.start_scene);

  robot_trajectory::RobotTrajectoryPtr t(new robot_trajectory::RobotTrajectory(scene_->getRobotModel(), ""));
  for (std::size_t i = 0; i < msg.sub_trajectory.size(); ++i)
  {
    if (t->empty())
    {
      t->setRobotTrajectoryMsg(scene_->getCurrentState(), msg.sub_trajectory[i].trajectory);
    }
    else
    {
      robot_trajectory::RobotTrajectory tmp(scene_->getRobotModel(), "");
      tmp.setRobotTrajectoryMsg(t->getLastWayPoint(), msg.sub_trajectory[i].trajectory);
      t->append(tmp, 0.0);
    }
  }

  if (!t->empty())
  {
    boost::mutex::scoped_lock lock(display_solution_mutex_);
    solution_to_display_.swap(t);
    if (interrupt_display_property_->getBool())
      interruptCurrentDisplay();
  }
}

void TaskSolutionVisualization::changedRobotColor()
{
  if (enable_robot_color_property_->getBool())
    setRobotColor(&(robot_render_->getRobot()), robot_color_property_->getColor());
}

void TaskSolutionVisualization::enabledRobotColor()
{
  if (enable_robot_color_property_->getBool())
    setRobotColor(&(robot_render_->getRobot()), robot_color_property_->getColor());
  else
    unsetRobotColor(&(robot_render_->getRobot()));
}

void TaskSolutionVisualization::changedAttachedBodyColor()
{

}

void TaskSolutionVisualization::unsetRobotColor(rviz::Robot* robot)
{
  for (auto& link : robot->getLinks())
    link.second->unsetColor();
}

void TaskSolutionVisualization::setRobotColor(rviz::Robot* robot, const QColor& color)
{
  for (auto& link : robot->getLinks())
    link.second->setColor(color.redF(), color.greenF(), color.blueF());
}

void TaskSolutionVisualization::sliderPanelVisibilityChange(bool enable)
{
  if (!slider_panel_)
    return;

  if (enable)
    slider_panel_->onEnable();
  else
    slider_panel_->onDisable();
}


void TaskSolutionVisualization::changedSceneEnabled()
{
  if (scene_render_)
    scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool());
}

void TaskSolutionVisualization::renderCurrentScene()
{
}

}  // namespace moveit_rviz_plugin