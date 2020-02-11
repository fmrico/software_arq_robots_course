// Copyright 2019 Intelligent Robotics Lab
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

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"

#include "geometry_tf/transforms.h"

namespace geometry_tf
{

tf2::Vector3 transform_point(const tf2::Vector3 & input_point, const tf2::Transform & transform)
{
  return  transform * input_point;
}

}  // namespace geometry_tf
