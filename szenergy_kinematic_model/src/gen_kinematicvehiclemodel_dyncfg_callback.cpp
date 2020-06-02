#include <szenergy_kinematic_model/kinematicvehiclemodel.hpp>

namespace jkk
{

void KinematicVehicleModel::callbackReconfigure(szenergy_kinematic_model::KinematicVehicleModelConfig & config,uint32_t level)
{
	if (kinematic_parameters.wheelbase != config.wheelbase)
	{
		ROS_INFO_STREAM("Updated message [kinematic_parameters.wheelbase]: wheelbase" << config.wheelbase);
	}
	kinematic_parameters.wheelbase = config.wheelbase;
	if (kinematic_parameters.track_width != config.track_width)
	{
		ROS_INFO_STREAM("Updated message [kinematic_parameters.track_width]: track_width" << config.track_width);
	}
	kinematic_parameters.track_width = config.track_width;
	if (kinematic_parameters.cog_ratio != config.cog_ratio)
	{
		ROS_INFO_STREAM("Updated message [kinematic_parameters.cog_ratio]: cog_ratio" << config.cog_ratio);
	}
	kinematic_parameters.cog_ratio = config.cog_ratio;
	updateReconfig();
}

}
