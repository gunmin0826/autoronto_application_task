// this file contains solution for the task assigned to apply to 3DOD team of aUToronto
// this node subscribes to two topics "input" and "target" and outputs the indexes of input array
// which when the corresponding elements are added it is equal to the target
// output will be published to "solution" topic.
  
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"


using namespace message_filters;
using std::placeholders::_1;
using std::placeholders::_2;
using int8multi = std_msgs::msg::Int8MultiArray;
using int8 = std_msgs::msg::Int8;
using my_sync_policy = message_filters::sync_policies::ApproximateTime<int8multi, int8>;

class time_sync : public rclcpp::Node 
{
	public:
	time_sync() : Node("sync_node") // class constructor
	{
		rclcpp::QoS qos = rclcpp::QoS(10);
		
		// initilize publisher
		solution_pub = this->create_publisher<int8multi>("solution", qos);
		
		// initialize subscribers
		input_sub.subscribe(this, "input", qos.get_rmw_qos_profile());
		target_sub.subscribe(this, "target", qos.get_rmw_qos_profile());
		
		// initialize sync
		uint32_t queue_size = 10;
		sync = std::make_shared<message_filters::Synchronizer<my_sync_policy>>(my_sync_policy(queue_size), input_sub, target_sub);
    sync->setAgePenalty(0.50); // set max timestamp diff to be 50ms
  	sync->registerCallback(std::bind(&time_sync::SyncCallback, this, _1, _2));
	}
	
	private:
	rclcpp::Publisher<int8multi>::SharedPtr solution_pub; // publisher for "solution" topic
	message_filters::Subscriber<int8multi> input_sub; // subscriber for "input" topic
	message_filters::Subscriber<int8> target_sub; // subscriber for "target" topic
	

	// synchronizer for two incoming topics	
	std::shared_ptr<message_filters::Synchronizer<my_sync_policy>> sync;
      
  void SyncCallback(
  	const int8multi::ConstSharedPtr& input,
    const int8::ConstSharedPtr& target
    )
  {
  	
  	// print input.data
  	const int input_size = input->layout.dim[0].size;
		const int input_stride = input->layout.dim[0].stride; // because input should be 1-dim data, stride should be 0
		const int input_offset = input->layout.data_offset;
		std::vector<int8_t> input_vec;
		std::string log_this = "";
		for(int i=input_offset; i<input_size+input_offset; i++){ // read all elements of input->data, store values and log the values 
				int8_t tmp = input->data[i];
				input_vec.push_back(tmp);
				if (i==0){ log_this += "input: [" + std::to_string(tmp) + ", "; }
				else if (i == input_size-1){ log_this += std::to_string(tmp) + "]"; }
				else { log_this += std::to_string(tmp) + ", "; }	 
		}
		RCLCPP_INFO(this->get_logger(), "%s", log_this.c_str());
		
  	// get target.data
		const int8_t tar = target->data;
		
		// prepare other variables
		bool sol_found = false;
		
		// prepare message for /solution topic
		const int LENGTH = 2;
		const int STRIDE = 0;
		int8multi msg; // message for /solution topic
		msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
		msg.layout.dim[0].size = LENGTH;
		msg.layout.dim[0].stride = STRIDE;
		msg.layout.dim[0].label = "length";
		msg.layout.data_offset = 0; 
		std::vector<int8_t> data;

    // find solution
		for (int i=input_offset; i<input_size-1+input_offset; i++){ // nested-loop for finding solution 
				for (int8_t j=i+1; j<input_size+input_offset; j++){
						if ((input->data[i] + input->data[j]) == tar){ // solution found
								msg.data.push_back(i);
								msg.data.push_back(j);
								sol_found = true;
								break;
						}
				}
				if (sol_found){ break; }
		}	
		
		RCLCPP_INFO(this->get_logger(), "Target: %d  ||  Answer should be (%d,%d)\n", tar, msg.data[0], msg.data[1]);
		solution_pub->publish(msg);
  
  } // end of SyncCallback 
  
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<time_sync>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	
	return 0;
	

}
