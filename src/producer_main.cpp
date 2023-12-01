#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <producer.hpp>

int main(int argc, char * argv[])
{
	/// Component container with dedicated single-threaded executors for each components.
	rclcpp::init(argc, argv);

	auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
	std::shared_ptr<Producer> node = std::make_shared<Producer>(exec);
	node->Configure();
	// exec->add_node(node->System::get_node_base_interface());
	exec->add_node(node->PublicComponentManagerIsolated<rclcpp::executors::SingleThreadedExecutor>::get_node_base_interface());
	while (rclcpp::ok()) {
		exec->spin_once();
	}
}
