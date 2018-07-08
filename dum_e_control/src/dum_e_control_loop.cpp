#include <dum_e_control/dum_e_control_loop.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace dum_e
{
	GenericHWControlLoop::GenericHWControlLoop(ros::NodeHandle& nh, boost::shared_ptr<dum_e::GenericHWInterface> hardware_interface)
	: nh_(nh)
	, hardware_interface_(hardware_interface)
	{
		controller_manager_.reset(new controller_manager::ControllerManager(hardware_interface_.get(),nh_));

		ros::NodeHandle rpsnh(nh,name_);
		std::size_t error = 0;

		error += !rosparam_shortcuts::get(name_, rpsnh, "loop_hz", loop_hz_);
		error += !rosparam_shortcuts::get(name_, rpsnh, "cycle_time_error_threshold",cycle_time_error_threshold_);

		rosparam_shortcuts::shutdownIfError(name_, error);

		clock_gettime(CLOCK_MONOTONIC, &last_time_);

		desired_update_period_ = ros::Duration(1/loop_hz_);
	}

	void GenericHWControlLoop::run()
	{
		ros::Rate rate(loop_hz_);
		while(ros::ok())
		{
			update();
			rate.sleep();
		}
	}

	void GenericHWControlLoop::update()
	{
		clock_gettime(CLOCK_MONOTONIC, &current_time_);
		elapsed_time_ = ros::Duration(current_time_.tv_sec - last_time_.tv_sec + (current_time_.tv_nsec - last_time_.tv_nsec)/ BILLION);
		last_time_ = current_time_;

		const double cycle_time_error = (elapsed_time_ - desired_update_period_).toSec();
		
		if (cycle_time_error > cycle_time_error_threshold_)
		{
			ROS_WARN_STREAM_NAMED(name_, "Cycle time exceeded error threshold by: "
											<< cycle_time_error << ", cycle time: " << elapsed_time_
											<< ", threshold:" << cycle_time_error_threshold_);
		}

		hardware_interface_->read(elapsed_time_);

		controller_manager_->update(ros::Time::now(), elapsed_time_);

		hardware_interface_->write(elapsed_time_);
	}
}