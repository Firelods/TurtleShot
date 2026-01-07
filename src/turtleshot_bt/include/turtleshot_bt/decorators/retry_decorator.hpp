#ifndef TURTLESHOT_BT__DECORATORS__RETRY_DECORATOR_HPP_
#define TURTLESHOT_BT__DECORATORS__RETRY_DECORATOR_HPP_

#include <behaviortree_cpp_v3/decorator_node.h>

namespace turtleshot_bt
{

/**
 * @brief Retry decorator - retries child node N times
 *
 * Returns SUCCESS if child succeeds within N attempts
 * Returns FAILURE if all N attempts fail
 * Returns RUNNING while retrying
 */
class RetryDecorator : public BT::DecoratorNode
{
public:
  RetryDecorator(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config),
    num_attempts_(0),
    max_attempts_(0),
    current_attempt_(0)
  {
    if (!getInput("num_attempts", max_attempts_)) {
      max_attempts_ = 3;  // default
    }
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("num_attempts", 3, "Number of retry attempts")
    };
  }

private:
  int num_attempts_;
  int max_attempts_;
  int current_attempt_;

  BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    while (current_attempt_ < max_attempts_) {
      const BT::NodeStatus child_status = child_node_->executeTick();

      if (child_status == BT::NodeStatus::SUCCESS) {
        current_attempt_ = 0;  // Reset for next time
        return BT::NodeStatus::SUCCESS;
      }

      if (child_status == BT::NodeStatus::RUNNING) {
        return BT::NodeStatus::RUNNING;
      }

      // FAILURE - try again
      current_attempt_++;
    }

    // All attempts failed
    current_attempt_ = 0;  // Reset
    return BT::NodeStatus::FAILURE;
  }

  void halt() override
  {
    current_attempt_ = 0;
    DecoratorNode::halt();
  }
};

}  // namespace turtleshot_bt

#endif  // TURTLESHOT_BT__DECORATORS__RETRY_DECORATOR_HPP_
