#ifndef MJPC_TASKS_QUADRUPED_RFMPC_SPINE_H
#define MJPC_TASKS_QUADRUPED_RFMPC_SPINE_H

#include <string>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include <mutex> // Include the mutex header
#include <thread>
#include <string_view>

namespace mjpc {

class RFMPC_SPINE : public ThreadSafeTask {
 public:
  std::string Name() const override;
  std::string XmlPath() const override;

  class ResidualFn : public mjpc::BaseResidualFn {
    public:
      explicit ResidualFn(const RFMPC_SPINE* task)
        : mjpc::BaseResidualFn(task) {}
      ResidualFn(const ResidualFn&) = default;
      void Residual(const mjModel* model, const mjData* data,
                    double* residual) const override;

    private:
      friend class RFMPC_SPINE;

      //  ============  enums  ============
      // modes
      enum RFMPC_SPINEMode {
        kModeQuadruped = 0,
        kModeWalk,
        kNumMode
      };

      // feet
      enum RFMPC_SPINEFoot {
        kFootFL  = 0,
        kFootHL,
        kFootFR,
        kFootHR,
        kNumFoot
      };

      // gaits
      enum RFMPC_SPINEGait {
        kGaitStand = 0,
        kGaitWalk,
        kGaitTrot,
        kGaitCanter,
        kGaitGallop,
        kNumGait
      };

            // posture names
      enum RFMPC_POSTURE {
        kPostureNormal = 0,
        kPostureLow,
        kPostureHigh,
        kNumPosture
      };


      //  ============  constants  ============
      constexpr static RFMPC_SPINEFoot kFootAll[kNumFoot] = {kFootFL, kFootHL,
                                                    kFootFR, kFootHR};
      constexpr static RFMPC_SPINEFoot kFootHind[2] = {kFootHL, kFootHR};
      constexpr static RFMPC_SPINEGait kGaitAll[kNumGait] = { kGaitStand, kGaitWalk, kGaitTrot};

      // gait phase signature (normalized)
      constexpr static double kGaitPhase[kNumGait][kNumFoot] =
      {
      // FL     HL     FR     HR
        {0,     0,     0,     0   },   // stand
        {0,     0.75,  0.5,   0.25},   // walk
        {0,     0.5,   0.5,   0   },   // trot
        {0,     0.33,  0.33,  0.66},   // canter
        {0,     0.4,   0.05,  0.35}    // gallop
      };

      // gait parameters, set when switching into gait
      constexpr static double kGaitParam[kNumGait][7] =
      {
      // duty ratio  cadence  amplitude  balance   upright   height   posture
      // unitless    Hz       meter      unitless  unitless  unitless unitless
        {1,          1,       0,         0.3,       1,        1},      // stand
        {0.75,       1,       0.015,      0.15,     1,        1},      // walk
        {0.45,       2,       0.015,      0.2,      1,        1},      // trot
        {0.4,        4,       0.025,      0.03,     0.5,      0.2},      // canter
        {0.3,        3.5,     0.05,      0.03,      0.2,      0.1}       // gallop
      };

      // velocity ranges for automatic gait switching, meter/second
      constexpr static double kGaitAuto[kNumGait] =
      {
        0,     // stand
        0.01,  // walk
        0.01,  // trot
        0.3,   // canter
        1,     // gallop
      };


      // notes:
      // - walk is never triggered by auto-gait
      // - canter actually has a wider range than gallop

      // automatic gait switching: time constant for com speed filter
      constexpr static double kAutoGaitFilter = 0.1;    // second

      // automatic gait switching: minimum time between switches
      constexpr static double kAutoGaitMinTime = 1;     // second

      // target torso height over feet when quadrupedal
      constexpr static double kHeightQuadruped[kNumPosture] = 
      {
        0.148,   // low
        0.20,   // normal
        0.254    // high
      };  // meter
      // constexpr static double kHeightQuadruped = 0.20;  // meter

      // radius of foot geoms
      constexpr static double kFootRadius = 0.02;       // meter

      // below this target yaw velocity, walk straight
      constexpr static double kMinAngvel = 0.01;        // radian/second

      // posture gain factors for abduction, hip, knee
      constexpr static double kJointPostureGain[3] = {1, 1, 1};  // unitless

      constexpr static const char* kPostureNames[kNumPosture] = {"low", "home", "high"};

      constexpr static double kMinHeight = 0.15;
      constexpr static double kMediumHeight = 0.2;
      constexpr static double kMaxHeight = 0.25;

      //  ============  methods  ============
      // return internal phase clock
      double GetPhase(double time) const;

      // return current gait
      RFMPC_SPINEGait GetGait() const;

      // return current posture
      const char* GetPosture() const;

      // compute average foot position, depending on mode
      void AverageFootPos(double avg_foot_pos[3],
                          double* foot_pos[kNumFoot]) const;

      // return normalized target step height
      double StepHeight(double time, double footphase, double duty_ratio) const;

      // compute target step height for all feet
      void FootStep(double step[kNumFoot], double time, RFMPC_SPINEGait gait) const;

      // walk horizontal position given time
      void Walk(double pos[2], double time) const;

      // print residual
      void PrintResidual(double* residual, int size) const;

      double map(double value, double inMin, double inMax, double outMin, double outMax) const;

      //  ============  task state variables, managed by Transition  ============
      RFMPC_SPINEMode current_mode_ = kModeQuadruped;
      double last_transition_time_ = -1;

      // common mode states
      double mode_start_time_  = 0;
      double position_[3]       = {0};

      // walk states
      double heading_[2]        = {0};
      double speed_             = 0;
      double angvel_            = 0;

      // // backflip states
      // double ground_            = 0;
      // double orientation_[4]    = {0};
      // double save_gait_switch_  = 0;
      // std::vector<double> save_weight_;

      // gait-related states
      double current_gait_      = -1;
      double phase_start_       = 0;
      double phase_start_time_  = 0;
      double phase_velocity_    = 0;
      double com_vel_[2]        = {0};
      double gait_switch_time_  = 0;

      // posture
      double current_posture_ = 0;


      // spinal angle
      double spinal_angle_ = 0;

      enum RFMPC_SPINEJointIDs {
        kFrontLimbZHinge = 7,
        kFrontLimbYHinge,
        kAbductionFL,
        kHipFL,
        kKneeFL,
        kAbductionFR,
        kHipFR,
        kKneeFR,
        kHindLimbYHinge,
        kAbductionRL,
        kHipRL,
        kKneeRL,
        kAbductionRR,
        kHipRR,
        kKneeRR,
      };

      struct RFMPC_SPINEJointValues {
        double min_value;
        double max_value;
      };

      RFMPC_SPINEJointValues getJointValues(enum RFMPC_SPINEJointIDs joint_id) const 
      {
        switch (joint_id)
        {
          case kFrontLimbYHinge:
            return {-0.42, 0.42};
          case kHindLimbYHinge:
            return {0.42, -0.42};
          // case kHipFL:
            return {0.45, 0};
          case kHipFR:
            return {0.45, 0};
          case kHipRL:
            return {-0.45, 0.7};
          case kHipRR:
            return {-0.45, 0.7};
          default:
            printf("ERROR: joint id shouldnt get interpolated\n");
            return {-1, 1};
        }
      }

      //  ============  constants, computed in Reset()  ============
      int torso_body_id_        = -1;
      int head_site_id_         = -1;
      int goal_mocap_id_        = -1;
      int gait_param_id_        = -1;
      int gait_switch_param_id_ = -1;
      int cadence_param_id_     = -1;
      int amplitude_param_id_   = -1;
      int duty_param_id_        = -1;
      int posture_param_id_     = -1;
      int spinal_angle_param_id_ = -1;
      int chest_height_param_id_ = -1;
      int upright_cost_id_      = -1;
      int balance_cost_id_      = -1;
      int height_cost_id_       = -1;
      int foot_geom_id_[kNumFoot];
      int shoulder_body_id_[kNumFoot];

      // // derived kinematic quantities describing flip trajectory
      double gravity_           = 9.81;
      // double jump_vel_          = 0;
      // double flight_time_       = 0;
      // double jump_acc_          = 0;
      // double crouch_time_       = 0;
      // double leap_time_         = 0;
      // double jump_time_         = 0;
      // double crouch_vel_        = 0;
      // double land_time_         = 0;
      // double land_acc_          = 0;
      // double flight_rot_vel_    = 0;
      // double jump_rot_vel_      = 0;
      // double jump_rot_acc_      = 0;
      // double land_rot_acc_      = 0;

  }; // class ResidualFn

  RFMPC_SPINE() : residual_(this) {}

  void TransitionLocked(mjModel* model, mjData* data) override;

  // call base-class Reset, save task-related ids
  void ResetLocked(const mjModel* model) override;

  // draw task-related geometry in the scene
  void ModifyScene(const mjModel* model, const mjData* data,
                   mjvScene* scene) const override;

protected:
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn>(residual_);
  }

  ResidualFn* InternalResidual() override { return &residual_; }

private:
  friend class ResidualFn;
  ResidualFn residual_;

};

}  // namespace mjpc

#endif  // MJPC_TASKS_QUADRUPED_RFMPC_SPINE_H_
