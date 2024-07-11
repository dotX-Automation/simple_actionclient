# simple_actionclient_py

Python library that wraps the `rclpy` action client providing a simpler interface.

## Contents

This package offers the `Client` class, that can be used to create a ROS 2 action client just by passing the node, the action type, the action name, and other optional arguments.

### Features

The different phases of the lifetime of an action goal can be handled either one by one, using dedicated methods, or all together, using the `call` method.

Single methods reflect the different operations that can be performed on an action goal, *i.e.*:

- `send_goal` or `send_goal_sync`
- `cancel` or `cancel_sync`
- `get_result` or `get_result_sync`

The `sync` variants are blocking, and wait for the corresponding operation to complete while spinning the node. Timeouts can be applied to blocking operations.

Each method returns either an operation result, or an error code explaining what happened, or a combination of both.

#### `call` semantics

In order to describe the outcom of each phase, the `call` method returns a tuple containing:

1. the `accepted` flag;
2. the `GoalStatus` code;
3. the `result` object (if any).

If the goal was not accepted for any reason, the `accepted` flag is `None`.

If any operation has timed out without the server responding, the `GoalStatus` code is `STATUS_UNKNOWN`. This is also true when a cancellation request is not accepted by the server.

When a cancellation request is done without waiting for the result, or it times out, the `GoalStatus` code is `STATUS_CANCELING`.

If the goal was completed, the `GoalStatus` code and the `result` object are returned as they are received from the server.

## Usage

See code documentation for a quick overview.

---

## Copyright and License

Copyright 2024 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at <http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the License for the specific language governing permissions and limitations under the License.
