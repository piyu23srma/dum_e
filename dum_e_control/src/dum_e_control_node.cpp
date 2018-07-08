#include <dum_e_control/dum_e_control_loop.h>
#include <dum_e_control/dum_e_hardware_interface.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dum_e_hw");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(2);
	spinner.start();

	boost::shared_ptr<dum_e::GenericHWInterface> gen_hw_interface(new dum_e::GenericHWInterface(nh));
	gen_hw_interface->init();

	dum_e::GenericHWControlLoop control_loop(nh, gen_hw_interface);
	control_loop.run(); 	

	return 0;
}