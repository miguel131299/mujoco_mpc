#ifndef MJPC_TASKS_NERMO_H_
#define MJPC_TASKS_NERMO_H_

#include <string>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"

namespace mjpc {

class Nermo : public ThreadSafeTask {
  public:
    std::string Name() const override;
    std::string XmlPath() const override;

    class ResidualFn : public mjpc::BaseResidualFn {
      public:
        explicit ResidualFn(const Nermo* task)
          : mjpc::BaseResidualFn(task) {}
        
        ResidualFn(const ResidualFn&) = default;
        void Residual(const mjModel* model, const mjData* data,
                      double* residual) const override;

      private:
        friend class Nermo;

      //  ============  enums  ============
      // modes
      enum NermoMode {
        kModeQuadruped = 0,
        kModeWalk,
        kNumMode
      };

      // feet
      enum NermoFoot {
        kFootFL  = 0,
        kFootHL,
        kFootFR,
        kFootHR,
        kNumFoot
      };

      // gaits
      enum NermoGait {
        kGaitStand = 0,
        kGaitWalk,
        kGaitTrot,
        kNumGait
      };

      //  ============  constants  ============
      constexpr static NermoFoot kFootAll[kNumFoot] = {kFootFL, kFootHL,
                                                    kFootFR, kFootHR};
      constexpr static NermoFoot kFootHind[2] = {kFootHL, kFootHR};
      constexpr static NermoGait kGaitAll[kNumGait] = { kGaitStand, kGaitWalk, kGaitTrot};

      // gait phase signature (normalized)
      constexpr static double kGaitPhase[kNumGait][kNumFoot] =
      {
      // FL,  HL,  FR,  HR
        {0.0, 0.0, 0.0, 0.0},  // stand
        {0.0, 0.5, 0.0, 0.5},  // walk
        {0.0, 0.5, 0.5, 0.0}   // trot
      };

    // gait parameters, set when switching into gait
    constexpr static double kGaitParam[kNumGait][6] =
    {
    // duty ratio  cadence  amplitude  balance   upright   height
    // unitless    Hz       meter      unitless  unitless  unitless
      {1,          1,       0,         0,        1,        1},      // stand
      {0.75,       1,       0.03,      0,        1,        1},      // walk
      {0.45,       2,       0.03,      0.2,      1,        1},      // trot
    };

    // velocity ranges for automatic gait switching, meter/second
    constexpr static double kGaitAuto[kNumGait] =
    {
      0,     // stand
      0.02,  // walk
      0.02,  // trot
    };
    // notes:
    // - walk is never triggered by auto-gait
    // - canter actually has a wider range than gallop

    // automatic gait switching: time constant for com speed filter
    constexpr static double kAutoGaitFilter = 0.2;    // second

    // automatic gait switching: minimum time between switches
    constexpr static double kAutoGaitMinTime = 1;     // second

    // TODO: Get height for Nermo
    // target torso height over feet when quadrupedal
    constexpr static double kHeightQuadruped = 0.22;  // meter

    // radius of foot geoms (name="foot_placement") in Model
    constexpr static double kFootRadius = 0.002;       // meter

    // below this target yaw velocity, walk straight
    constexpr static double kMinAngvel = 0.01;        // radian/second

    // TODO: Research Joint Posture Gain does this work?
    // posture gain factors for (abduction -> not here), hip, knee
    constexpr static double kJointPostureGain[2] = {1, 1};  // unitless

    //  ============  methods  ============
    // return internal phase clock
    double GetPhase(double time) const;

    // return current gait
    NermoGait GetGait() const;

    // compute average foot position, depending on mode
    void AverageFootPos(double avg_foot_pos[3],
                        double* foot_pos[kNumFoot]) const;

    // return normalized target step height
    double StepHeight(double time, double footphase, double duty_ratio) const;

    // compute target step height for all feet
    void FootStep(double step[kNumFoot], double time, NermoGait gait) const;

    // walk horizontal position given time
    void Walk(double pos[2], double time) const;

    // print residual
    void PrintResidual(double* residual, int size) const;

    //  ============  task state variables, managed by Transition  ============
    NermoMode current_mode_ = kModeQuadruped;
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
    double current_gait_      = kGaitStand;
    double phase_start_       = 0;
    double phase_start_time_  = 0;
    double phase_velocity_    = 0;
    double com_vel_[2]        = {0};
    double gait_switch_time_  = 0;

    //  ============  constants, computed in Reset()  ============
    int torso_body_id_        = -1;
    int head_site_id_         = -1;
    int goal_mocap_id_        = -1;
    int gait_param_id_        = -1;
    int gait_switch_param_id_ = -1;
    int cadence_param_id_     = -1;
    int amplitude_param_id_   = -1;
    int duty_param_id_        = -1;
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


    };  // class ResidualFn
   
    Nermo() : residual_(this) {}

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
} // namespace mjpc

#endif  // MJPC_TASKS_NERMO_H_