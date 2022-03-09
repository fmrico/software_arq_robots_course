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

#include <time.h>
#include <math.h>

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "ros/ros.h"

#define OBJ_PUB_RATE  10.0

geometry_msgs::TransformStamped generate_object()
{
  float x, y, z;
  float ax, ay, az;

  unsigned int seed = time(NULL);

  // generate a position in [10, -10] in base_footprint
  x = 20.0 * (rand_r(&seed)/static_cast<float>(RAND_MAX)) - 10.0;
  y = 20.0 * (rand_r(&seed)/static_cast<float>(RAND_MAX)) - 10.0;
  z = 0.0;
  ax = 0.0;
  ay = 0.0;
  az = 2.0*M_PI * (rand_r(&seed)/static_cast<float>(RAND_MAX)) - M_PI;

  //tipo de datos de tf2 con padre basefootprint
  tf2::Stamped<tf2::Transform> object;
  object.frame_id_ = "base_footprint";
  object.stamp_ = ros::Time::now();

  object.setOrigin(tf2::Vector3(x, y, z));//decidimos la traslacion respecto a base_footprint en este caso

  tf2::Quaternion q;//creo un quaternion
  q.setRPY(ax, ay, az);//indico su posicion
  object.setRotation(q);//indico que la rotacion de este frame esta indicado en el cuaternion.
  
  //pasamos de tipo de dato de tf2 a mensaje de ross   
  geometry_msgs::TransformStamped object_msg = tf2::toMsg(object);
  object_msg.child_frame_id = "object";//indicamos que la infomacion de los mensajes del hijo estan en object

  return object_msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_pub");
  ros::NodeHandle n;

  //tambien se puede crear un tf2_ros::StaticTransformBroadcaster br para que se publice en tf2 static. 
  //en tf2, los frames publicados a los 10 segundos por convenio se suelen olvidar, en statictf2, no
  tf2_ros::TransformBroadcaster br;

  ros::Time obj_ts = ros::Time::now();

  ros::Rate loop_rate(20);

  geometry_msgs::TransformStamped object_msg = generate_object();

  while (ros::ok())
  {
    if ((ros::Time::now() - obj_ts).toSec() > OBJ_PUB_RATE)
    {
      obj_ts = ros::Time::now();

      object_msg = generate_object();

      //si pones el try-catch  dentro del if, se publicara la transformada cada 10 segundos y no con cada iteracion del bucle
      try
      {
       object_msg.header.stamp = ros::Time::now();
       br.sendTransform(object_msg);
      }
      catch(tf2::TransformException &exception)
      {
        ROS_ERROR("%s", exception.what());
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
