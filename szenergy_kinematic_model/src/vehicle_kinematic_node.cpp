#include <ros/ros.h>


#include <szenergy_kinematic_model/kinematicvehiclemodel.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "szenergy_kinematic_bicycle");
	std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
		std::shared_ptr<ros::NodeHandle> private_nh = std::make_shared<ros::NodeHandle>("~");
	std::unique_ptr<jkk::AbstractVehicleKinematicModel> bicycle(new jkk::KinematicBicycleKinematicModel(2.7));
	jkk::KinematicVehicleModel vehiclemodel(private_nh, nh, std::move(bicycle));
	try
	{
		vehiclemodel.initialize(false, false);
		ros::spin();
		return 0;
	}
	catch(rei::ExceptionNodePreInitialization &e)
	{
		ROS_FATAL_STREAM("Unable to initialize HOTARU TEB LOCAL PLANNER" << e.what());
	}
	catch(rei::ExceptionNodeAssignSyncGuards &e)
	{
		ROS_FATAL_STREAM("Unable to initialize HOTARU TEB LOCAL PLANNER" << e.what());
	}
	catch(rei::ExceptionNodeMiddleWare &e)
	{
		ROS_FATAL_STREAM("Unable to initialize HOTARU TEB LOCAL PLANNER" << e.what());
	}
	return -1;
}
