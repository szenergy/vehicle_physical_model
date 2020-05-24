#ifndef KINEMATICVEHICLEMODEL_HEADER_HPP
#define KINEMATICVEHICLEMODEL_HEADER_HPP

#include <ros/ros.h>
/// ROS msgs
#include <nav_msgs/Odometry.h>
#include <autoware_msgs/VehicleStatus.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <rei_statemachine_library/ros/ros_sync_state_machine.hpp>
// State-machine node element
#include <rei_common/gen_node_elements/interface_simple_ros_node.hpp>

#include <memory>

namespace jkk {

struct StateKinematicVehicleModel
{
	// Flags
	const bool debug;				///< Publish debug parameters
	const bool bypass_behavior;     ///< Bypass behavioral state machines
	// ROS input messages
	autoware_msgs::VehicleStatus msg_port_vehicle_status; ///< port_vehicle_status store to autoware_msgs::VehicleStatus
	geometry_msgs::PoseStamped msg_port_gnss_pose; ///< port_gnss_pose store to geometry_msgs::PoseStamped
	/// ROS output messages
	geometry_msgs::TwistStamped msg_port_current_velocity; ///< port_current_velocity store to geometry_msgs::TwistStamped
	nav_msgs::Odometry msg_port_update_odom; ///< port_update_odom store to nav_msgs::Odometry
	
	StateKinematicVehicleModel(const bool debug, const bool bypass_behavior): debug(debug), bypass_behavior(bypass_behavior) {}
};

/**
 *
 * @attribute port_vehicle_status: subscribes to topic vehicle_status 
 * @attribute port_gnss_pose: subscribes to topic gnss_pose 
 * @attribute port_current_velocity: publishes to topic vehicle/current_velocity
 * @attribute port_update_odom: publishes to topic vehicle/odom
 */
class InterfaceRos_KinematicVehicleModel: public rei::Interface_SimpleRosNode
{
private:
protected:
	/// ROS utils
	std::shared_ptr<ros::NodeHandle> private_nh;
	std::shared_ptr<ros::NodeHandle> nh;
	/// ROS Subscribers
	ros::Subscriber port_vehicle_status; ///< port_vehicle_status subscriber to autoware_msgs::VehicleStatus
	ros::Subscriber port_gnss_pose; ///< port_gnss_pose subscriber to geometry_msgs::PoseStamped
	/// ROS Publishers
	ros::Publisher port_current_velocity; ///< port_current_velocity publisher to geometry_msgs::TwistStamped
	ros::Publisher port_update_odom; ///< port_update_odom publisher to nav_msgs::Odometry
	std::unique_ptr<StateKinematicVehicleModel> pubsubstate;
	// State machines
	std::shared_ptr<rei::RosSyncStateMachine> sync_sm_kinematicstate;
	std::shared_ptr<rei::SyncStateMachine> sync_state_machine;
	std::shared_ptr<rei::PortStateMonitorRos> port_state_monitor;
	std::shared_ptr<rei::RosCommunicationGraphNotifier> notifier;
	std::shared_ptr<rei::RosSyncStateGuard> sync_guard;
	std::mutex sm_mutex;
	// Set ALL STATES CB
	virtual void setSyncStateMachineCallbacks() = 0;
public:
	InterfaceRos_KinematicVehicleModel(std::shared_ptr<ros::NodeHandle> private_nh, std::shared_ptr<ros::NodeHandle> nh): private_nh(private_nh), nh(nh) {}
	
	virtual ~InterfaceRos_KinematicVehicleModel() = 0;
	
	/*
	 * Override steps to initialize
	 *     STEPS:
	 *           1. Initialize descendant specific functionalities before middleware functionalities (initPre())
	 *           2. Initialize timeout state machine (initTimeoutStateMachine())
	 *           3. Assign guard related to timeout functions (assigSyncGuards())
	 *           4. Initialize middleware functionalities
	 *           5. Initialize descendant node-specific functionalities
	 */
	/*
	 * @brief: Initialize node pre
	 * @returns: Initialization successful
	 */
	virtual bool initPre() = 0;
	/*
	 * @brief: Initialize timeout statemachine
	 */
	virtual bool initTimeoutStateMachine() override;
	/*
	 * @brief: Assign sync guards
	 */
	virtual bool assignSyncGuards() = 0;
	
	/*
	 * @brief: initialize middleware
	 * @param debug: defines whether the debug information should be provided or not.
	 */
	virtual bool initMiddleware(const bool debug, const bool bypass_behavior) override;
	
	/*
	 * @brief: post initialize
	 */
	virtual bool initPost() = 0;
	
	/**
	 * Callback method for vehicle_status
	 */
	void cbPort_vehicle_status(const autoware_msgs::VehicleStatus::ConstPtr& msg); ///< port_vehicle_status subscriber to autoware_msgs::VehicleStatus
	virtual void executeUpdatevehiclestatus(const autoware_msgs::VehicleStatus::ConstPtr& msg) = 0;
	/**
	 * Callback method for gnss_pose
	 */
	void cbPort_gnss_pose(const geometry_msgs::PoseStamped::ConstPtr& msg); ///< port_gnss_pose subscriber to geometry_msgs::PoseStamped
	
	/**
	 * Publish method to publish message to vehicle/current_velocity
	 */
	void publishVehicle_current_velocity();
	/**
	 * Publish method to publish message to vehicle/odom
	 */
	void publishVehicle_odom();
};

}

#endif
