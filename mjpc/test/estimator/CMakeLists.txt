# Copyright 2023 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

test(estimator_force_test)
target_link_libraries(estimator_force_test load gmock)

test(estimator_optimize_test)
target_link_libraries(estimator_optimize_test load gmock)

test(estimator_prior_test)
target_link_libraries(estimator_prior_test load gmock)

test(estimator_sensor_test)
target_link_libraries(estimator_sensor_test load gmock)

test(estimator_shift_test)
target_link_libraries(estimator_shift_test load gmock)

test(estimator_trajectory_test)
target_link_libraries(estimator_trajectory_test gmock)

test(estimator_utilities_test)
target_link_libraries(estimator_utilities_test load gmock)
