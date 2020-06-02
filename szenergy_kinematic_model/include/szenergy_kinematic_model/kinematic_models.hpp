/*
 * KinematicModel.hpp
 *
 *  Created on: May 24, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_SZENERGY_KINEMATIC_MODEL_KINEMATICMODEL_HPP_
#define INCLUDE_SZENERGY_KINEMATIC_MODEL_KINEMATICMODEL_HPP_

#include <Eigen/Dense>

#include <memory>

namespace jkk
{

struct KinematicState
{
	// TODO: vector-based description would be more preferable
	Eigen::Isometry3d zero_state;
	Eigen::Vector3d linear_velocity;
	Eigen::Vector3d angular_velocity;

	double yaw;
	double x;
	double y;
	double dx;
	double dy;
	double dyaw;
};

class AbstractVehicleKinematicModel
{
protected:
	const double wheelbase;
	const double cog_ratio;
	// Kinematic state
	std::shared_ptr<KinematicState> kinematic_state;
public:
	AbstractVehicleKinematicModel(const double wheelbase,
			const double cog_ratio=0.5): wheelbase(wheelbase), cog_ratio(cog_ratio),
			kinematic_state(new KinematicState())
	{

	}
	virtual ~AbstractVehicleKinematicModel() {};

	void resetPose(const double x, const double y, const double yaw)
	{
		kinematic_state->x = x;
		kinematic_state->y = y;
		kinematic_state->yaw = yaw;
	}

	virtual void updateCommand(const double linearvelocity, const double wheelangle, const double dt) = 0;

	const std::shared_ptr<KinematicState> getState()
	{
		return kinematic_state;
	}
};

/**
 * Reference: Kinematic and Dynamic Vehicle Models for Autonomous Driving Control Design
 * Authors: J. Kong, M Pferiffer, G. Schildbach, F. Borelli
 */
class KinematicBicycleKinematicModel: public AbstractVehicleKinematicModel
{
protected:
	const double tau = 1.2;
public:
	KinematicBicycleKinematicModel(const double wheelbase,
		const double cog_ratio=0.5):
			AbstractVehicleKinematicModel(wheelbase, cog_ratio)
	{

	}



	virtual void updateCommand(const double linearvelocity, const double wheelangle, const double dt)
	{
		double beta = atan(tan(wheelangle)*cog_ratio);
		// Update first order state
		// TODO: why not integrate?
		// https://core.ac.uk/download/pdf/31082555.pdf
		kinematic_state->dyaw = (linearvelocity/wheelbase)*sin(wheelangle);
		double yaw_tp1 = kinematic_state->yaw + kinematic_state->dyaw * dt;

		// ORIGINALLY: kinematic_state->dyaw = (linearvelocity/wheelbase)*tan(wheelangle);
		kinematic_state->dx = (linearvelocity/(kinematic_state->dyaw*dt))*
				(sin(yaw_tp1) - sin(kinematic_state->yaw));
		kinematic_state->dy -= (linearvelocity/(kinematic_state->dyaw*dt))*
				(cos(yaw_tp1) - cos(kinematic_state->yaw));
		//std::cout << dt << '\t'  << kinematic_state->dx  <<'\n';
		kinematic_state->x += kinematic_state->dx;
		kinematic_state->y -= kinematic_state->dy;
		kinematic_state->yaw = yaw_tp1;
	}
};

}

#endif /* INCLUDE_SZENERGY_KINEMATIC_MODEL_KINEMATICMODEL_HPP_ */
