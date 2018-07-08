#include <dum_e/dum_e_hardware_interface.cpp>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dum_e_hw");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(2);
	spinner.start();

	boost::shared_ptr<dum_e::GenericHWInterface> gen_hw_interface(new dum_e::GenericHWInterface(nh));
	gen_hw_interface->init();

	return 0;
}