// include the ROS2 client library and other required messages and libraries

#include<iostream>
#include<rclcpp/rclcpp.hpp>
#include<turtlesim/msg/pose.hpp>

// inherit a cpp Node class
class PosSubscriber : public rclcpp::Node{

	public:
		//Constructor (for setting up the node: defining publishers, subscribers, etc.)
		PosSubscriber():Node("pos_subscriber"){
			//define the subscriber
			pos_sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10, std::bind(&PosSubscriber::pose_callback,this, std::placeholders::_1));
			
		}
	
	private:
		// do this every time the messages received from the topic create an interrupt
		void pose_callback(const turtlesim::msg::Pose msg){
	  		RCLCPP_INFO(this->get_logger(), "x_t: %f, y_t: %f, theta_t %f", msg.x*100, msg.y*100, msg.theta*180/3.1415);
		}
	
		//define the subscriber object
		rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pos_sub_;

};

// main logic of the program
int main(int argc, char ** argv){
	std::cout<<"main function\n";
	//initialize the node;
	rclcpp::init(argc, argv);
	std::cout<<"Node initialized\n";
	// create an object for this node based on the definitions at the beginning
	auto node = std::make_shared<PosSubscriber>();
	rclcpp::spin(node); // make sure the node is chcked regularly
	rclcpp::shutdown(); // clean up
	

 	return 0;

}
