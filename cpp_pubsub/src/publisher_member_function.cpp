#include "rclcpp/rclcpp.hpp"
#include "cpp_custinterface/srv/modstr.hpp"

using namespace std::chrono_literals;
using namespace std;
using namespace cpp_custinterface::srv;
using namespace rclcpp;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

 void modify(const shared_ptr<Modstr::Request> request,
          shared_ptr<Modstr::Response> response)
          {
            response->output_str = request->input_str + string("modified");
            RCLCPP_INFO(get_logger("rclcpp"), "Incoming Request:\nString: %s", request->input_str.c_str());                                         
            RCLCPP_INFO(get_logger("rclcpp"), "sending back response: %s", response->output_str.c_str());
          }

int main(int argc, char **argv)
{
  init(argc, argv);
  shared_ptr<Node> node = Node::make_shared("mod_str_server");
  Service<Modstr>::SharedPtr service =
    node->create_service<::Modstr>("mod_str", &modify);
  RCLCPP_INFO(get_logger("rclcpp"), "Ready to modify strings.");
  spin(node);
  rclcpp::shutdown();
}
