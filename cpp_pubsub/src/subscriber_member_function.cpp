#include "rclcpp/rclcpp.hpp"
#include "cpp_custinterface/srv/modstr.hpp"

using namespace std::chrono_literals;
using namespace std;
using namespace cpp_custinterface::srv;
using namespace rclcpp;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  if (argc != 2) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: modify_string X");
      return 1;
  }
  // Similar to the service node, the following lines of code create the node 
  // and then create the client for that node:
  shared_ptr<Node> node = Node::make_shared("mod_str_client");
  Client<Modstr>::SharedPtr client = node->create_client<Modstr>("mod_str");

  // Next, the request is created. Its structure is defined by the .srv file mentioned earlier.
  auto request = make_shared<Modstr::Request>();
  request->input_str = argv[1];
      
  // The while loop gives the client 1 second to search for service nodes in the network. 
  //If it can’t find any, it will continue waiting.
  //If the client is canceled (e.g. by you entering Ctrl+C into the terminal), 
  // it will return an error log message stating it was interrupted.
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    return 0;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // If the client is canceled (e.g. by you entering Ctrl+C into the terminal), 
  // it will return an error log message stating it was interrupted.
  auto result = client->async_send_request(request);
  // Wait for the result
  if (rclcpp::spin_until_future_complete(node, result) ==
  rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Modified string: %s", result.get()->output_str.c_str());
  } else {
  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}  
