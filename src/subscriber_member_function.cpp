/*  MIT License

    Copyright (c) 2022 Arshad S.

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/

/**
 * @copyright (c) 2022 Arshad S.
 * @file node_subs.cpp
 * @author Arshad S(arshad22@umd.edu)
 * @brief This is a ROS client node which uses a custom service to modify a
 * string
 * @version 1.0
 * @date 2022-12-01
 *
 */
#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/modify_string.hpp"

// Using Namespace to improve code readability
using namespace std;
using namespace rclcpp;
using namespace tutorial_interfaces::srv;

/**
 * @brief This class receives command line parameter and modifies the string
 *
 */
class MinimalParam : public Node {
 public:
  MinimalParam()  ///< Constructor for default init of tasks
      : Node("minimal_param_node") {
    this->declare_parameter(
        "my_parameter",
        "world");  

    string my_param =
        this->get_parameter("my_parameter")
            .get_parameter_value()
            .get<string>();  

    RCLCPP_WARN(this->get_logger(), "Going to Modify String from Command Line: ");
    RCLCPP_INFO(this->get_logger(), "Hello %s!",
                my_param.c_str());  

    vector<Parameter> all_new_parameters{
        Parameter("my_parameter",
                  "world")};  
    this->set_parameters(
        all_new_parameters);  
  }
};

/**
 * @brief Main function which initialises ROS communication and service
 * @param argc stores the no of arguements to send & receive in command line
 * @param argv stores the actual arguement data to send and receive in command
 * line
 * @return zero
 */
int main(int argc, char **argv) {
  
  init(argc, argv);  

  shared_ptr<Node> node = Node::make_shared(
      "Modify_String_Client");  

  Client<ModifyString>::SharedPtr client = node->create_client<ModifyString>(
      "Modify_String_Server");  

  auto request =
      make_shared<ModifyString::Request>();  

  for (int i = 1; i <= (argc - 1);
       i++) {  
    request->to_modify += argv[i];
  }

  while (!client->wait_for_service(
      500ms)) {   
    if (!ok()) {  
      RCLCPP_FATAL(get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_WARN(get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  auto result = client->async_send_request(
      request);  

  spin_some(
      std::make_shared<MinimalParam>());  

  if (spin_until_future_complete(node, result) ==
      FutureReturnCode::SUCCESS) {  
    RCLCPP_INFO(get_logger("rclcpp"), "Modified_String: %s",
                result.get()->mod_string.c_str());  
  } else {
    RCLCPP_ERROR(get_logger("rclcpp"),
                 "Failed to call service Modify_String_Server");
    RCLCPP_DEBUG(get_logger("rclcpp"),
                 "Check server node if it's running on new terminal & then "
                 "rerun the client");
  }
  shutdown();  
  return 0;
}
