# simple_actionclient_cpp

C++ library that wraps the `rclcpp` action client providing a simpler interface.

## Contents

This package offers the `simple_actionclient::Client<ActionT>` class template, that can be used to create a ROS 2 action client just by passing the node and the action name, and optionally a feedback callback of the appropriate type, *i.e.*, `std::function<void (const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr, const typename ActionT::Feedback::ConstSharedPtr)>`.

The client offers methods to either perform all the major operations pertaining to the lifetime of an action goal individually, or to execute an entire action goal in a single call. Each API method is documented in the code, and has a normal and a `sync`, *i.e.* blocking, variant.

More specifically:

- `send_goal` sends a new goal to the server, and returns a `std::shared_future<typename ActionGoalHandleT::SharedPtr>` that can be used to wait for the result of the goal. The `sync` variant returns the goal handle pointer directly.
- `cancel` cancels a given goal, and returns a `std::shared_future<action_msgs::srv::CancelGoal::Response::SharedPtr>` that can be used to wait for the result of the cancellation. The `sync` variant returns the cancellation result directly.
- `get_result` returns a `std::shared_future<typename ActionGoalHandleT::WrappedResult>` that can be used to wait for the result of a given goal. The `sync` variant returns the `WrappedResult` object directly.
- `call_sync` performs an action call in a single operation. Timeouts can be specified for the goal and the result, and the goal can be canceled if the result is not received in time. The method returns an `std::tuple<bool, rclcpp_action::ResultCode, std::shared_ptr<ActionResultT>>` containing the accepted flag, the result code, and a pointer to the result object; the values of all the different fields can tell whether the goal was accepted, whether the result was received in time, whether the goal was canceled, and whether the server stopped responding.

## Usage

Consider the following short example. Within your node class, you can define a member variable to point to a client, like this:

```cpp
std::shared_ptr<SimpleActionClient::Client<Fibonacci>> fib_client;
```

Then, you can initialize it in the node constructor, or elsewhere, like this:

```cpp
fib_client = std::make_shared<simple_actionclient::Client<Fibonacci>>(
  this,
  "/fibonacci",
  std::bind(
    &TestNode::feedback_callback,
    this,
    std::placeholders::_1,
    std::placeholders::_2));
```

where the `feedback_callback` method is defined as follows:

```cpp
void feedback_callback(
  const rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr goal_handle,
  const Fibonacci::Feedback::ConstSharedPtr feedback)
{
  (void)(goal_handle);
  std::cout << "Feedback: ";
  for (auto & num : feedback->partial_sequence) {
    std::cout << num << " ";
  }
  std::cout << std::endl;
}
```

Finally, you can use the client to send goals to the server using its methods. For example, to perform an entire action goal in one shot and print the result:

```cpp
Fibonacci::Goal goal{};
goal.order = 10;
auto ret = node->fib_client->call_sync(goal, true, 5000, 2000, 2000); // Notice all the timeouts for the different phases of the action goal
auto code = std::get<rclcpp_action::ResultCode>(ret);
if (code == rclcpp_action::ResultCode::SUCCEEDED) {
  auto res = std::get<std::shared_ptr<Fibonacci::Result>>(ret);
  std::cout << "Result: ";
  for (auto & num : res->sequence) {
    std::cout << num << " ";
  }
  std::cout << std::endl;
}
```

See code documentation for more information.

---

## Copyright and License

Copyright 2024 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at <http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the License for the specific language governing permissions and limitations under the License.
