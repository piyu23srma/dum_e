#include <time.h>
#include <dum_e_control/dum_e_hardware_interface.h>

namespace dum_e
{
	static const double BILLION = 1000000000.0;

	class GenericHWControlLoop
	{
	public:

		GenericHWControlLoop(ros::NodeHandle& nh, boost::shared_ptr<dum_e::GenericHWInterface> hardware_interface);
	
		void run();

	protected:
		void update();

		ros::NodeHandle nh_;

		std::string name_ = "dum_e_control_loop";

		ros::Duration desired_update_period_;
		double cycle_time_error_threshold_;

		ros::Duration elapsed_time_;

		double loop_hz_;
		struct timespec last_time_;
		struct timespec current_time_;

		boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
		
		boost::shared_ptr<dum_e::GenericHWInterface> hardware_interface_;
	};
}