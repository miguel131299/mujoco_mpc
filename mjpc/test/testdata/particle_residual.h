// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MJPC_TEST_TESTDATA_PARTICLE_RESIDUAL_H_
#define MJPC_TEST_TESTDATA_PARTICLE_RESIDUAL_H_

#include <string>
#include "mjpc/task.h"
#include <mujoco/mujoco.h>

class ParticleTestTask : public mjpc::ThreadSafeTask {
 public:
  ParticleTestTask() : residual_(this) {}

  std::string Name() const override {return ""; }
  std::string XmlPath() const override { return ""; }

 private:
  class ResidualFn : public mjpc::BaseResidualFn {
   public:
    explicit ResidualFn(const ParticleTestTask* task)
        : mjpc::BaseResidualFn(task) {}
    void Residual(const mjModel* model, const mjData* data,
                  double* residual) const override {
      // goal position
      mju_copy(residual, data->qpos, model->nq);
      residual[0] -= data->mocap_pos[0];
      residual[1] -= data->mocap_pos[1];

      // goal velocity error
      mju_copy(residual + 2, data->qvel, model->nv);
    }
  };

  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn>(residual_);
  }
  ResidualFn* InternalResidual() override { return &residual_; }

  ResidualFn residual_;
};

#endif  // MJPC_TEST_TESTDATA_PARTICLE_RESIDUAL_H_
