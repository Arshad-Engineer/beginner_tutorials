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
 * @file publisher_member_function.cpp
 * @author Arshad S (arshad22@umd.edu)
 * @brief This is a ROS server node which uses a custom service to modify a
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
 * @brief User defined function to modify incoming string
 * @param request stores the incoming string from command line
 * @param response stores the output string to be sent back to client
 * @return zero
 */
void modify(const shared_ptr<ModifyString::Request> request,
            shared_ptr<ModifyString::Response> response) {
  response->mod_string =
      request->to_modify +
      string(" Hi");  

  RCLCPP_WARN(get_logger("rclcpp"), "Incoming Request:\nString: %s",
              request->to_modify.c_str());  

  RCLCPP_INFO(get_logger("rclcpp"), "sending back response: %s",
              response->mod_string.c_str());  
}

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
      "Modify_String_Server");  
                                
  Service<ModifyString>::SharedPtr service = node->create_service<ModifyString>(
      "Modify_String_Server",
      &modify);  

  RCLCPP_INFO(get_logger("rclcpp"),
              "waiting the string to be changed from user...");

  spin(node);  
  shutdown();  
  return 0;
}
