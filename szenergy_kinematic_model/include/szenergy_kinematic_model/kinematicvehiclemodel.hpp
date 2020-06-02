/*
 * kinematicvehiclemode.hpp
 *
 *  Created on: May 24, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_SZENERGY_KINEMATIC_MODEL_KINEMATICVEHICLEMODEL_HPP_
#define INCLUDE_SZENERGY_KINEMATIC_MODEL_KINEMATICVEHICLEMODEL_HPP_

#include "gen_kinematicvehiclemodel.hpp"

#include <szenergy_kinematic_model/KinematicVehicleModelConfig.h>
#include <szenergy_kinematic_model/kinematicparameters_struct.hpp>

#include <dynamic_reconfigure/server.h>

#include "kinematic_models.hpp"
#include "common_math.hpp"

#include <szenergy_kinematic_model/KinematicVehicleModelConfig.h>

#include <memory>

namespace jkk
{

class KinematicVehicleModel: public InterfaceRos_KinematicVehicleModel
{
private:
	ros::Time t_prev_update;
	std::shared_ptr< dynamic_reconfigure::Server<szenergy_kinematic_model::KinematicVehicleModelConfig> > dynamic_recfg;
protected:
	ConfigStructKinematicparameters kinematic_parameters;

	std::unique_ptr<AbstractVehicleKinematicModel> kinematic_model;

	void startAfterSynchronization()
	{
		pubsubstate->msg_port_update_odom.pose.pose = pubsubstate->msg_port_gnss_pose.pose;
		kinematic_model->resetPose(
			pubsubstate->msg_port_gnss_pose.pose.position.x,
			pubsubstate->msg_port_gnss_pose.pose.position.y,
			getYawFromQuaternion(pubsubstate->msg_port_gnss_pose.pose.orientation)
		);
		t_prev_update = ros::Time::now();
	}

	void timeoutAfterSync()
	{

	}

	void updateReconfig()
	{
		kinematic_model->updateParameters(kinematic_parameters);
	}



	void setSyncStateMachineCallbacks()
	{
		//
		pubsubstate->msg_port_update_odom.pose.pose = pubsubstate->msg_port_gnss_pose.pose;
		//
		sync_state_machine->setCbAllStateMessageReceived(std::bind(&KinematicVehicleModel::startAfterSynchronization, this));
		sync_state_machine->setCbTimeOut(std::bind(&KinematicVehicleModel::timeoutAfterSync, this));
	}
public:
	KinematicVehicleModel(std::shared_ptr<ros::NodeHandle> private_nh,
			std::shared_ptr<ros::NodeHandle> nh,
			std::unique_ptr<AbstractVehicleKinematicModel> vehiclemodel):
				InterfaceRos_KinematicVehicleModel(private_nh, nh),
				kinematic_model(std::move(vehiclemodel))
	{

	}



	virtual bool initPre()
	{
		return true;
	}

	bool startcondition()
	{
		return true;
	}

	virtual bool assignSyncGuards() override
	{
		sync_guard->setCrispGuardState_Start(std::bind(&KinematicVehicleModel::startcondition, this));
		return true;
	}

	virtual bool initPost() override
	{
		setSyncStateMachineCallbacks();
		pubsubstate->msg_port_current_velocity.header.frame_id = "base_link";
		pubsubstate->msg_port_update_odom.header.frame_id = "map";
		dynamic_recfg = std::make_shared<dynamic_reconfigure::Server<szenergy_kinematic_model::KinematicVehicleModelConfig>>();
		auto f = std::bind(&KinematicVehicleModel::callbackReconfigure, this, std::placeholders::_1, std::placeholders::_2);
		dynamic_recfg->setCallback(f);
		updateReconfig();
		return true;
	}



	void updateOdometry()
	{
		auto state = kinematic_model->getState();
		// This is the current velocity in the frame of the
		pubsubstate->msg_port_current_velocity.twist.linear.x = state->dx;
		pubsubstate->msg_port_current_velocity.twist.linear.y = state->dy;
		pubsubstate->msg_port_current_velocity.twist.angular.z = state->dyaw;


		pubsubstate->msg_port_update_odom.pose.pose.position.x = state->x;
		pubsubstate->msg_port_update_odom.pose.pose.position.y = state->y;
		getQuaternionFromYaw(state->yaw, pubsubstate->msg_port_update_odom.pose.pose.orientation);
		pubsubstate->msg_port_update_odom.twist.twist.linear.x = state->dx;
		pubsubstate->msg_port_update_odom.twist.twist.linear.y = state->dy;
		pubsubstate->msg_port_update_odom.twist.twist.angular.z = state->dyaw;
	}

	virtual void executeUpdatevehiclestatus(const autoware_msgs::VehicleStatus::ConstPtr& msg)
	{
		if (msg->header.stamp.toSec() < t_prev_update.toSec())
		{
			ROS_WARN("Jump backwards in time, resetting pose according to GNSS");
			kinematic_model->resetPose(
				pubsubstate->msg_port_gnss_pose.pose.position.x,
				pubsubstate->msg_port_gnss_pose.pose.position.y,
				getYawFromQuaternion(pubsubstate->msg_port_gnss_pose.pose.orientation)
			);
		}
		kinematic_model->updateCommand(msg->speed, msg->angle, msg->header.stamp.toSec() - t_prev_update.toSec());
		updateOdometry();
		pubsubstate->msg_port_update_odom.header.stamp = msg->header.stamp;
		publishVehicle_current_velocity();
		publishVehicle_odom();
		t_prev_update = msg->header.stamp;
	}

	void genParamConfig();

	void callbackReconfigure(szenergy_kinematic_model::KinematicVehicleModelConfig & config,uint32_t level);

};

}


#endif /* INCLUDE_SZENERGY_KINEMATIC_MODEL_KINEMATICVEHICLEMODEL_HPP_ */
