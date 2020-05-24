#include "szenergy_kinematic_model/kinematicvehiclemodel.hpp"
#include "szenergy_kinematic_model/default_config_kinematicvehiclemodel.hpp"
#include <szenergy_kinematic_model/kinematicparameters_struct.hpp>

namespace jkk
{

void KinematicVehicleModel::genParamConfig()
{
	//
	// GROUP kinematicparameters
	// Load param wheelbase
	if (!private_nh->getParam("kinematicparameters/wheelbase", kinematic_parameters.wheelbase))
	{
		kinematic_parameters.wheelbase = kinematicparameters::DEFAULT_WHEELBASE;
	}
	ROS_INFO_STREAM("Using kinematicparameters/wheelbase:=" << kinematic_parameters.wheelbase);
	// Load param track_width
	if (!private_nh->getParam("kinematicparameters/track_width", kinematic_parameters.track_width))
	{
		kinematic_parameters.track_width = kinematicparameters::DEFAULT_TRACK_WIDTH;
	}
	ROS_INFO_STREAM("Using kinematicparameters/track_width:=" << kinematic_parameters.track_width);
}

}
