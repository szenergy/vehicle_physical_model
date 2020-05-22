#include <szenergy_kinematic_model/gen_interfaceros_kinematicvehiclemodel.h>

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
	return true;
}



void InterfaceRos_KinematicVehicleModel::cbPort_vehicle_status(const autoware_msgs::VehicleStatus::ConstPtr& msg)
{
	pubsubstate->msg_port_vehicle_status = *msg;
	executeUpdatevehiclestatus(msg);
	
}

void InterfaceRos_KinematicVehicleModel::publishVehicle/current_velocity()
{
	port_current_velocity.publish(pubsubstate->msg_port_current_velocity);
}
void InterfaceRos_KinematicVehicleModel::publishVehicle/odom()
{
	port_update_odom.publish(pubsubstate->msg_port_update_odom);
}

}
