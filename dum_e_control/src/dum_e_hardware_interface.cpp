#include <dum_e_control/dum_e_hardware_interface.h>
#include <limits>

#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace dum_e
{
	GenericHWInterface::GenericHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
		: name_("dum_e_hardware_interface")
		, nh_(nh)
		, use_rosparam_joint_limits_(false)
		, use_soft_limits_if_available_(false)
	{
		if (urdf_model == NULL)
			loadURDF(nh,"robot_description");
		else
			urdf_model_ = urdf_model;

		ros::NodeHandle rpnh(nh_,"hardware_interface");
		std::size_t error = 0;
		error += !rosparam_shortcuts::get(name_,rpnh,"joints",joint_names_);
		rosparam_shortcuts::shutdownIfError(name_,error);
	}

	void GenericHWInterface::init()
	{
		num_joints_ = joint_names_.size();

		joint_position_prev_.resize(num_joints_,0.0);

		joint_positions_.resize(num_joints_,0.0);
		joint_velocity_.resize(num_joints_,0.0);
		joint_effort_.resize(num_joints_,0.0);

		joint_positions_command_.resize(num_joints_,0.0);
		joint_velocity_command_.resize(num_joints_,0.0);
		joint_effort_command_.resize(num_joints_,0.0);
	
		joint_positions_lower_limits_.resize(num_joints_,0.0);
		joint_positions_upper_limits_.resize(num_joints_,0.0);
		joint_velocity_limits_.resize(num_joints_,0.0);
		joint_effort_limits_.resize(num_joints_,0.0);

		for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
		{
			ROS_DEBUG_STREAM_NAMED(name_,"Loading joint name: " << joint_names_[joint_id]);

			joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
				joint_names_[joint_id], &joint_positions_[joint_id], &joint_velocity_[joint_id], &joint_effort_[joint_id]));
		

			hardware_interface::JointHandle joint_handle_position = hardware_interface::JointHandle(
				joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_positions_command_[joint_id]);
			position_joint_interface_.registerHandle(joint_handle_position);
			
			hardware_interface::JointHandle joint_handle_velocity = hardware_interface::JointHandle(
				joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_velocity_command_[joint_id]);
			velocity_joint_interface_.registerHandle(joint_handle_velocity);

			hardware_interface::JointHandle joint_handle_effort = hardware_interface::JointHandle(
				joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_effort_command_[joint_id]);
			effort_joint_interface_.registerHandle(joint_handle_effort);

			registerJointLimits(joint_handle_position, joint_handle_velocity, joint_handle_effort, joint_id);
		}

		registerInterface(&joint_state_interface_);
		registerInterface(&position_joint_interface_);
		registerInterface(&velocity_joint_interface_);
		registerInterface(&effort_joint_interface_);

		ROS_INFO_STREAM_NAMED(name_,"GenericHWInterface Ready.");
	}

	void GenericHWInterface::registerJointLimits(const hardware_interface::JointHandle &joint_handle_position,
												 const hardware_interface::JointHandle &joint_handle_velocity,
												 const hardware_interface::JointHandle &joint_handle_effort,
												 std::size_t joint_id)
	{
		joint_positions_lower_limits_[joint_id] = -std::numeric_limits<double>::max();
		joint_positions_upper_limits_[joint_id] = std::numeric_limits<double>::max();
		joint_velocity_limits_[joint_id] = std::numeric_limits<double>::max();
		joint_effort_limits_[joint_id] = std::numeric_limits<double>::max();

		joint_limits_interface::JointLimits joint_limits;
		joint_limits_interface::SoftJointLimits soft_limits;

		bool has_joint_limits = false;
		bool has_soft_limits = false;

		if (urdf_model_ == NULL)
		{
			ROS_WARN_STREAM_NAMED(name_, "No URDF model Loaded, unable to get joint limits");
			return;
		}

		urdf::JointConstSharedPtr urdf_joint = urdf_model_->getJoint(joint_names_[joint_id]);

		if (urdf_joint == NULL)
		{
			ROS_ERROR_STREAM_NAMED(name_,"URDF joint not found " << joint_names_[joint_id]);
			return;
		}

		if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits))
		{
			has_joint_limits = true;
			ROS_DEBUG_STREAM_NAMED(name_,"Joint " << joint_names_[joint_id] << " has URDF position limits ["
																	<< joint_limits.min_position << ", "
																	<< joint_limits.max_position << "]");
		
			if (joint_limits.has_velocity_limits)
				ROS_DEBUG_STREAM_NAMED(name_,"Joint " << joint_names_[joint_id] << " has URDF velocity limit ["
																		<< joint_limits.max_velocity << "]");
		}
		else
		{
			if (urdf_joint-> type != urdf::Joint::CONTINUOUS)
				ROS_WARN_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " does not have a URDF position limit ");
		}

		if (use_rosparam_joint_limits_)
		{
			if (joint_limits_interface::getJointLimits(joint_names_[joint_id], nh_, joint_limits))
			{
				has_joint_limits = true;
				ROS_DEBUG_STREAM_NAMED(name_, "Joint "
										<< joint_names_[joint_id] << " has rosparam position limits ["
										<< joint_limits.min_position << ", " << joint_limits.max_position << "]");
			
				if (joint_limits.has_velocity_limits)
					ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id]
															<< " has rosparam velocity limits ["
															<< joint_limits.max_velocity << "]");
			}
		}

		if (use_soft_limits_if_available_)
		{
			if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
			{
				has_soft_limits = true;
				ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has soft joint limits.");
			}
			else
			{
				ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " does not have soft joint limits");
			}
		}

		if (!has_joint_limits)
		{
			return;
		}

		if (joint_limits.has_velocity_limits)
		{
			joint_limits.min_position +=  std::numeric_limits<double>::epsilon();
			joint_limits.max_position -=  std::numeric_limits<double>::epsilon();

			joint_positions_lower_limits_[joint_id] = joint_limits.min_position;
			joint_positions_upper_limits_[joint_id] = joint_limits.max_position;
		}


		if (joint_limits.has_velocity_limits)
		{
			joint_velocity_limits_[joint_id] = joint_limits.max_velocity;
		}

		if (joint_limits.has_effort_limits)
		{
			joint_effort_limits_[joint_id] = joint_limits.max_effort;
		}

		if (has_soft_limits)
		{
			ROS_DEBUG_STREAM_NAMED(name_, "Using soft saturation limits");
			const joint_limits_interface::PositionJointSoftLimitsHandle soft_handle_position(joint_handle_position,
																							joint_limits, soft_limits);
			pos_jnt_soft_limits_.registerHandle(soft_handle_position);

			const joint_limits_interface::VelocityJointSoftLimitsHandle soft_handle_velocity(joint_handle_velocity,
																							joint_limits, soft_limits);
			vel_jnt_soft_limits_.registerHandle(soft_handle_velocity);

			const joint_limits_interface::EffortJointSoftLimitsHandle soft_handle_effort(joint_handle_effort, joint_limits,
                                                                                   soft_limits);
    		eff_jnt_soft_limits_.registerHandle(soft_handle_effort);
		}
		else
		{
			ROS_DEBUG_STREAM_NAMED(name_, "Using saturation limits (not soft limits)");
			const joint_limits_interface::PositionJointSaturationHandle sat_handle_position(joint_handle_position, joint_limits);
		
			pos_jnt_sat_interface_.registerHandle(sat_handle_position);

    		const joint_limits_interface::VelocityJointSaturationHandle sat_handle_velocity(joint_handle_velocity, joint_limits);
    		vel_jnt_sat_interface_.registerHandle(sat_handle_velocity);

    		const joint_limits_interface::EffortJointSaturationHandle sat_handle_effort(joint_handle_effort, joint_limits);
    		eff_jnt_sat_interface_.registerHandle(sat_handle_effort);	
		}

	}

	void GenericHWInterface::read(ros::Duration &elapsed_time)
	{

	}

	void GenericHWInterface::write(ros::Duration &elapsed_time)
	{
		enforceLimits(elapsed_time);

		for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
		{
			switch (sim_control_mode_)
			{
				case 0:
					positionControlSimulation (elapsed_time, joint_id);
					break;
				case 1:
					break;
				case 2:
					break;
			}
		}

	}

	void GenericHWInterface::enforceLimits(ros::Duration &period)
	{
		pos_jnt_sat_interface_.enforceLimits(period);
	}

	void GenericHWInterface::positionControlSimulation(ros::Duration &elapsed_time, const std::size_t joint_id)
	{
		const double max_delta_pos = joint_velocity_limits_[joint_id] * elapsed_time.toSec();

		p_error_ = joint_positions_command_[joint_id] - joint_positions_[joint_id];	
	
		const double delta_pos = std::max(std::min(p_error_, max_delta_pos), -max_delta_pos);
		joint_positions_[joint_id] += delta_pos;

		if (elapsed_time.toSec() > 0)
		{
			joint_velocity_[joint_id] = (joint_positions_[joint_id] - joint_position_prev_[joint_id]) / elapsed_time.toSec();
		}
		else
			joint_velocity_[joint_id] = 0;

		joint_position_prev_[joint_id] = joint_positions_[joint_id];
	}

	void GenericHWInterface::reset()
	{
		pos_jnt_sat_interface_.reset();
		pos_jnt_soft_limits_.reset();
	}

	void GenericHWInterface::printState()
	{
		ROS_INFO_STREAM_THROTTLE(1, std::endl << printStateHelper());
	}

	std::string GenericHWInterface::printStateHelper()
	{
		std::stringstream ss;
		std::cout.precision(15);

		for (std::size_t i = 0; i < num_joints_; ++i)
		{
			ss << "j" << i << ": " << std::fixed << joint_positions_[i] << "\t";
			ss << std::fixed << joint_velocity_[i] << "\t";
			ss << std::fixed << joint_effort_[i] << std::endl;
		}

		return ss.str();
	}

	std::string GenericHWInterface::printCommandHelper()
	{
		std::stringstream ss;
		std::cout.precision(15);
		ss << "	position 	velocity 	effort \n";

		for (std::size_t i = 0; i < num_joints_; ++i)
		{
			ss << "j" << i << ": " << std::fixed << joint_positions_command_[i] << "\t";
			ss << std::fixed << joint_velocity_command_[i] << "\t";
			ss << std::fixed << joint_effort_command_[i] << std::endl;
		}

		return ss.str();
	}

	void GenericHWInterface::loadURDF(ros::NodeHandle &nh, std::string param_name)
	{
		std::string urdf_string;
		urdf_model_ = new urdf::Model();

		while(urdf_string.empty() && ros::ok())
		{
			std::string search_param_name;

			if (nh.searchParam(param_name, search_param_name))
			{
				ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                            nh.getNamespace() << search_param_name);
      			nh.getParam(search_param_name, urdf_string);
			}
			else
			{
				ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                            nh.getNamespace() << param_name);
      			nh.getParam(param_name, urdf_string);
			}

			usleep(100000);
		}

		if (!urdf_model_->initString(urdf_string))
    		ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
  		else
    		ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
	}	
}