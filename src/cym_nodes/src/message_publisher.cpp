#include <ros/ros.h>
#include <math.h>
#include <cym_nodes/ExampleMessage.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "message_publisher");
	ros::NodeHandle n;
	ros::Publisher publisher_object = n.advertise<cym_nodes::ExampleMessage>("pub_message", 1);
	cym_nodes::ExampleMessage my_message;
	ros::Rate naptime(1.0);
	double counter = 0;
	my_message.header.stamp = ros::Time::now();
	my_message.header.seq = 0;
	my_message.header.frame_id = "first_frame";
	my_message.demo_int = 1;
	my_message.demo_double = 100.0;

	my_message.dbl_vec.resize(3);
	my_message.dbl_vec[0] = 1.414;
        my_message.dbl_vec[1] = 2.7;
        my_message.dbl_vec[2] = 3.14;
        my_message.dbl_vec.push_back(counter);
	double sqrt_arg;
	while(ros::ok())
	{
	counter += 1;
	my_message.header.stamp = ros::Time::now();
        my_message.header.seq++;
        my_message.demo_int = my_message.demo_int * 2;
        sqrt_arg = my_message.demo_double;
	my_message.demo_double = sqrt(sqrt_arg);
	my_message.dbl_vec.push_back(counter);
	publisher_object.publish(my_message);
	naptime.sleep();
	}
}
