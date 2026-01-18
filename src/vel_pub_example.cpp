// include the ROS2 client library and other required messages and libraries

#include<iostream>
#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/twist.hpp>

// inherit a cpp Node class
class VelPublisher : public rclcpp::Node{

	public:
		//Constructor (for setting up the node: defining publishers, subscribers, etc.)
		VelPublisher():Node("vel_publisher"){
			//the timer for regular control loop (runs at 10 Hz here)
			timer_ = this->create_wall_timer(
			std::chrono::milliseconds(static_cast<int>(100)),
			std::bind(&VelPublisher::timer_callback,this));	
			
			//create a publisher to send commands to the turtlebot
			publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);
		}
	
	private:
		// do this every time the chrono timer creates and interrupt
		void timer_callback(){
			RCLCPP_INFO(this->get_logger(), "call_back %d", this->counter);
			// update the control commands (fixed values in this example)
			t_cmd.linear.x = 0.5;
			t_cmd.angular.z = 0.5;
			
			publisher_->publish(t_cmd);
			
			this->counter++;
		}
	
		//define the timer object
		rclcpp::TimerBase::SharedPtr timer_;
		//define the publisher object
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
		//variables for tracking the loop and sending commands
		geometry_msgs::msg::Twist t_cmd;
		int counter = 1;


};

// main logic of the program
int main(int argc, char ** argv){
	std::cout<<"main function\n";
	//initialize the node;
	rclcpp::init(argc, argv);
	std::cout<<"Node initialized\n";
	// create an object for this node based on the definitions at the beginning
	auto node = std::make_shared<VelPublisher>();
	rclcpp::spin(node); // make sure the node is chcked regularly
	rclcpp::shutdown(); // clean up
	

 	return 0;

}
