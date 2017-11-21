/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <collision_benchmark/Shape.hh>

using collision_benchmark::Shape;

sdf::ElementPtr Shape::GetPoseSDF() const
{
  sdf::ElementPtr root(new sdf::Element());
  root->SetName("pose");
  std::stringstream vals;
  vals << pose.Pos().X() << " "
      <<pose.Pos().Y() << " "
      <<pose.Pos().Z() << " "
      <<pose.Rot().Euler().X() << " "
      <<pose.Rot().Euler().Y() << " "
      <<pose.Rot().Euler().Z();
  root->AddValue("pose", vals.str(), "0", "description");
  return root;
}

