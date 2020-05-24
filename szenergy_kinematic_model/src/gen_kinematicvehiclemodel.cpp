#include <szenergy_kinematic_model/gen_kinematicvehiclemodel.hpp>

namespace jkk {

InterfaceRos_KinematicVehicleModel::~InterfaceRos_KinematicVehicleModel() {}

bool InterfaceRos_KinematicVehicleModel::initTimeoutStateMachine()
{
	// Initialize state machines 
	notifier = std::make_shared<rei::RosCommunicationGraphNotifier>("kinematicvehiclemodel/sync_state/", nh);
	notifier->initialize();
	port_state_monitor= std::make_shared<rei::PortStateMonitorRos>();
	sync_guard = std::make_shared<rei::RosSyncStateGuard>();
	sync_guard->setMonitor(port_state_monitor);
	sync_state_machine = std::make_shared<rei::SyncStateMachine>(notifier, sync_guard);
	// Sync state machine initialization
	sync_sm_kinematicstate = std::make_shared<rei::RosSyncStateMachine>(nh,
		sync_state_machine, port_state_monitor, notifier, 
		"KinematicVehicleModel/kinematicstate");
	if (sync_sm_kinematicstate!=nullptr)
	{
		if (!sync_sm_kinematicstate->initialize())
		{
			return false;
		}
		sync_sm_kinematicstate->addTopicGuard("/vehicle_status", 0.03333333333333333+0.1);
		ROS_INFO("Synchronize with topic: vehicle_status, with estimated freq 30 Hz");
		sync_sm_kinematicstate->addTopicGuard("/gnss_pose", 0.1+0.1);
		ROS_INFO("Synchronize with topic: gnss_pose, with estimated freq 10 Hz");
	}
	else
	{
		return false;
	}
	return true;
}

bool InterfaceRos_KinematicVehicleModel::initMiddleware(const bool debug, const bool bypass_behavior)
{
	/// Initialize internal pubsub state
	pubsubstate = std::unique_ptr<StateKinematicVehicleModel>(new StateKinematicVehicleModel(debug, bypass_behavior));
	if (pubsubstate==nullptr)
	{
		return false;
	}
	/// Initialize ROS publishers
	port_current_velocity = nh->advertise<geometry_msgs::TwistStamped>("vehicle/current_velocity", 10);
	port_update_odom = nh->advertise<nav_msgs::Odometry>("vehicle/odom", 10);
	/// Initialize ROS subscribers
	port_vehicle_status = nh->subscribe("vehicle_status", 10, &InterfaceRos_KinematicVehicleModel::cbPort_vehicle_status, this);
	port_gnss_pose = nh->subscribe("gnss_pose", 10, &InterfaceRos_KinematicVehicleModel::cbPort_gnss_pose, this);
	return true;
}



void InterfaceRos_KinematicVehicleModel::cbPort_vehicle_status(const autoware_msgs::VehicleStatus::ConstPtr& msg)
{
	pubsubstate->msg_port_vehicle_status = *msg;
	// Synchronize with state machine: sync_sm_kinematicstate
	sm_mutex.lock();
	sync_sm_kinematicstate->stepMessageTopic("/vehicle_status", msg->header);
	sm_mutex.unlock();
	if (sync_sm_kinematicstate->isReady()){
		executeUpdatevehiclestatus(msg);
	}
	
}
void InterfaceRos_KinematicVehicleModel::cbPort_gnss_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	pubsubstate->msg_port_gnss_pose = *msg;
	// Synchronize with state machine: sync_sm_kinematicstate
	sm_mutex.lock();
	sync_sm_kinematicstate->stepMessageTopic("/gnss_pose", msg->header);
	sm_mutex.unlock();
	
}

void InterfaceRos_KinematicVehicleModel::publishVehicle_current_velocity()
{
	port_current_velocity.publish(pubsubstate->msg_port_current_velocity);
}
void InterfaceRos_KinematicVehicleModel::publishVehicle_odom()
{
	port_update_odom.publish(pubsubstate->msg_port_update_odom);
}

}
