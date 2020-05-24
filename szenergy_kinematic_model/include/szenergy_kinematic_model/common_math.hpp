/*
 * common_math.hpp
 *
 *  Created on: May 24, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_SZENERGY_KINEMATIC_MODEL_COMMON_MATH_HPP_
#define INCLUDE_SZENERGY_KINEMATIC_MODEL_COMMON_MATH_HPP_

#include <geometry_msgs/Quaternion.h>

namespace jkk
{

static double getYawFromQuaternion(const geometry_msgs::Quaternion& q)
{
	return atan2(
		2*(q.w*q.z + q.x*q.y),
		1-2*(q.y*q.y + q.z*q.z)
	);
}

}



#endif /* INCLUDE_SZENERGY_KINEMATIC_MODEL_COMMON_MATH_HPP_ */
