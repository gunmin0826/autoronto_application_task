/*
		this code subscribes to topic "solution" and output message into log
*/

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
using std::placeholders::_1;
using int8multi = std_msgs::msg::Int8MultiArray;

class SolutionSubscriber : public rclcpp::Node
{
public:
  SolutionSubscriber()
  : Node("solution_subscriber")
  {
    subscription_ = this->create_subscription<int8multi>( 
      "solution", 10, std::bind(&SolutionSubscriber::topic_callback, this, _1));
  }
  

private:
  void topic_callback(const int8multi::ConstSharedPtr& solution)// const
  {
  	const int stride = solution->layout.dim[0].stride; // because it is 1 dimensional, stride should be zero
  	const int offset = solution->layout.data_offset;
    const int length = solution->layout.dim[0].size;
     
    // print element of solution->data vector to log
    // int8_t sol[length];
    std::string log_this = "";
    for (int i=offset; i<offset+length; i++){
    		int8_t tmp = solution->data[i];
    		if (i==0){ log_this += "solution: [" + std::to_string(tmp) + ", ";}
    		else if (i==offset+length-1){ log_this += std::to_string(tmp) + "]";}
    		else { log_this += std::to_string(tmp) + ", ";}
    		// sol[i-offset] = tmp;
    		
    }
    
    RCLCPP_INFO(this->get_logger(), "%s", log_this.c_str());
  }
  rclcpp::Subscription<int8multi>::SharedPtr subscription_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SolutionSubscriber>());
  rclcpp::shutdown();
  return 0;
}
