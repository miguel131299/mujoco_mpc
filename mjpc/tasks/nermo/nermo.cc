#include "mjpc/tasks/nermo/nermo.h"

#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"

namespace mjpc
{

std::string Nermo::XmlPath() const
{
  return GetModelPath("nermo/assets/task_nermo.xml");
}

std::string Nermo::Name() const
{
  return "Nermo";
}

void Nermo::ResidualFn::Residual(const mjModel* model, const mjData* data, double* residual) const {

}

void Nermo::TransitionLocked(mjModel* model, mjData* data) {}

void Nermo::ResetLocked(const mjModel* model) {}

// colors of visualisation elements drawn in ModifyScene()
// constexpr float kStepRgba[4] = {0.6, 0.8, 0.2, 1};  // step-height cylinders
// constexpr float kHullRgba[4] = {0.4, 0.2, 0.8, 1};  // convex hull
// constexpr float kAvgRgba[4] = {0.4, 0.2, 0.8, 1};   // average foot position
// constexpr float kCapRgba[4] = {0.3, 0.3, 0.8, 1};   // capture point
// constexpr float kPcpRgba[4] = {0.5, 0.5, 0.2, 1};   // projected capture point

// draw task-related geometry in the scene
void Nermo::ModifyScene(const mjModel* model, const mjData* data, mjvScene* scene) const {}

void Nermo::ResidualFn::AverageFootPos(double avg_foot_pos[3], double* foot_pos[kNumFoot]) const {

}

// return phase as a function of time
double Nermo::ResidualFn::GetPhase(double time) const {
  return 0;
}

// horizontal Walk trajectory
void Nermo::ResidualFn::Walk(double pos[2], double time) const {
}

// get gait
Nermo::ResidualFn::NermoGait Nermo::ResidualFn::GetGait() const {
  return ResidualFn::kGaitStand;
}

// return normalized target step height
double Nermo::ResidualFn::StepHeight(double time, double footphase,
                                 double duty_ratio) const {
  return 0;
}

void Nermo::ResidualFn::FootStep(double target_step_height[kNumFoot], double time,
                             NermoGait gait) const {

}



} // namespace mjpc
