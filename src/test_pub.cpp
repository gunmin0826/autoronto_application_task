/*
	This program generates inputs to the "solver" via two topics "input" and 'target"
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <vector>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/multi_array_dimension.h"

using namespace std::chrono_literals;
using int8multi = std_msgs::msg::Int8MultiArray;
using mad = std_msgs::msg::MultiArrayDimension();
using int8 = std_msgs::msg::Int8;

const uint8_t LENGTH = 9;
class InputPublisher : public rclcpp::Node
{
public:
  InputPublisher()
  : Node("Input_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<int8multi>("input", 10);
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&InputPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {	
		size_t numSub = this->count_subscribers("input");
		if (numSub > 0) { // send message when there is a subscriber
			int8multi msg;
			
			// prepare message
			msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension()); // because we know the topic /input sends array (1-dim), we only push once
			msg.layout.dim[0].label = "length of 1-dim integer array";
			msg.layout.dim[0].size = LENGTH;
			msg.layout.dim[0].stride = 0;
			msg.layout.data_offset = 0; 
			std::vector<int8_t> vec;

			int8_t payload[LENGTH] = {1,2,3,5,8,13,21,34,55}; // these are possible elements for sum
			unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
			std::shuffle (std::begin(payload), std::end(payload), std::default_random_engine(seed)); // shuffle the order of payload
				
			// print the shuffled payload and assign into data vector
			std::string log_this = "";
			for(int i=0; i<LENGTH; i++){
					int8_t tmp = payload[i];
					vec.push_back(tmp);
					if (i==0){ log_this += "input: [" + std::to_string(tmp) + ", "; }
					else if (i == LENGTH-1){ log_this += std::to_string(tmp) + "]"; }
					else { log_this += std::to_string(tmp) + ", "; }	 
			}
			RCLCPP_INFO(this->get_logger(), "%s", log_this.c_str());

			msg.data = vec;
			publisher_ -> publish(msg);
		}
		else {RCLCPP_INFO(this->get_logger(), "no subscriber to 'input'");}
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<int8multi>::SharedPtr publisher_;
  size_t count_;
};

class TargetPublisher : public rclcpp::Node
{
public:
  TargetPublisher()
  : Node("Target_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<int8>("target", 10);
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&TargetPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
  	size_t numSub =  this->count_subscribers("target");
  	if (numSub > 0) {
			int8_t LUT[8] ={3,5,8,13,21,34,55,89}; // for testing purpose create array of fibonacci numbers
		  int8 message;
		  int index = rand() % 8; // the index in range [0,7]
			message.data = LUT[index];
		  RCLCPP_INFO(this->get_logger(), "target=%d", message.data);
		  publisher_->publish(message);
		}
		else {RCLCPP_INFO(this->get_logger(), "no subscriber to 'target'");}
	}
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<int8>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor; // for multithreading
  auto input_pub = std::make_shared<InputPublisher>();
  auto target_pub = std::make_shared<TargetPublisher>();
  
  executor.add_node(input_pub);
  executor.add_node(target_pub);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}





