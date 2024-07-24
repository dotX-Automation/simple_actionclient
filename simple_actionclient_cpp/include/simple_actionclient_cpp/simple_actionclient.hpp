/**
 * rclcpp action client wrapper implementation.
 *
 * Roberto Masocco <r.masocco@dotxautomation.com>
 *
 * September 25, 2023
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SIMPLE_ACTIONCLIENT_CPP__SIMPLE_ACTIONCLIENT_HPP_
#define SIMPLE_ACTIONCLIENT_CPP__SIMPLE_ACTIONCLIENT_HPP_

#include "visibility_control.h"

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <action_msgs/srv/cancel_goal.hpp>

namespace simple_actionclient
{

/* Feedback callback type. */
template<typename ActionT>
using FeedbackCallbackT = std::function<void (
      const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
      const typename ActionT::Feedback::ConstSharedPtr)>;

/**
 * Wraps an action client providing fully a/synchronous, transparent operation.
 * Requests are either handled in a single call that directly returns the final
 * result, but internally runs the back-end by spinning the node when
 * necessary, or in an asynchronous way, returning Future objects.
 */
template<typename ActionT>
class SIMPLE_ACTIONCLIENT_PUBLIC Client final
{
public:
  using ActionGoalHandleT = typename rclcpp_action::ClientGoalHandle<ActionT>;
  using ActionResultT = typename ActionT::Result;

  /**
   * @brief Constructor, can wait for the server to become active.
   *
   * @param node The node to be used to manage the client.
   * @param action_name The name of the action to be called.
   * @param feedback_callback The callback to be called when feedback is received.
   * @param wait Whether to wait for the server to become active.
   *
   * @throws RuntimeError if interrupted while waiting.
   */
  explicit Client(
    rclcpp::Node * node,
    const std::string & action_name,
    const FeedbackCallbackT<ActionT> & feedback_callback = nullptr,
    bool wait = true)
  : node_(node),
    action_name_(action_name),
    feedback_callback_(feedback_callback)
  {
    // Create the ROS 2 action client
    client_ = rclcpp_action::create_client<ActionT>(node_, action_name);
    client_opts_.feedback_callback = feedback_callback_;

    // Wait for the server to come up
    while (wait && !client_->wait_for_action_server(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error("Interrupted while waiting for action " + action_name);
      }
      RCLCPP_WARN(
        node_->get_logger(),
        "Action %s not available...",
        action_name.c_str());
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Initialized client for action %s",
      action_name.c_str());
  }

  /**
   * @brief Sends a new goal to the server.
   *
   * @param goal_msg The goal message to be sent.
   * @return The goal handle future.
   */
  std::shared_future<typename ActionGoalHandleT::SharedPtr> send_goal(
    const typename ActionT::Goal & goal_msg)
  {
    return client_->async_send_goal(goal_msg, client_opts_);
  }

  /**
   * @brief Sends a new goal to the server and waits for the response.
   *
   * @param goal_msg The goal message to be sent.
   * @param timeout_msec The timeout to be used when waiting for the response (milliseconds).
   * @return The goal handle.
   */
  typename ActionGoalHandleT::SharedPtr send_goal_sync(
    const typename ActionT::Goal & goal_msg,
    int64_t timeout_msec = 0)
  {
    auto goal_future = send_goal(goal_msg);
    if (timeout_msec <= 0) {
      auto err = rclcpp::spin_until_future_complete(node_->shared_from_this(), goal_future);
      if (err != rclcpp::FutureReturnCode::SUCCESS) {
        return nullptr;
      }
      return goal_future.get();
    } else {
      auto err = rclcpp::spin_until_future_complete(
        node_->shared_from_this(),
        goal_future,
        std::chrono::milliseconds(timeout_msec));
      if (err != rclcpp::FutureReturnCode::SUCCESS) {
        return nullptr;
      }
      return goal_future.get();
    }
  }

  /**
   * @brief Cancels a given goal.
   *
   * @param goal_handle The goal handle to be canceled.
   * @return The cancellation response future.
   */
  std::shared_future<action_msgs::srv::CancelGoal::Response::SharedPtr> cancel(
    const typename ActionGoalHandleT::SharedPtr goal_handle)
  {
    return client_->async_cancel_goal(goal_handle);
  }

  /**
   * @brief Cancels a given goal and waits for the cancellation result.
   *
   * @param goal_handle The goal handle to be canceled.
   * @param timeout_msec The timeout to be used when waiting for the response (milliseconds).
   * @return The cancellation response.
   */
  action_msgs::srv::CancelGoal::Response::SharedPtr cancel_sync(
    const typename ActionGoalHandleT::SharedPtr goal_handle,
    int64_t timeout_msec = 0)
  {
    auto cancel_future = cancel(goal_handle);
    if (timeout_msec <= 0) {
      auto err = rclcpp::spin_until_future_complete(node_->shared_from_this(), cancel_future);
      if (err != rclcpp::FutureReturnCode::SUCCESS) {
        return nullptr;
      }
      return cancel_future.get();
    } else {
      auto err = rclcpp::spin_until_future_complete(
        node_->shared_from_this(),
        cancel_future,
        std::chrono::milliseconds(timeout_msec));
      if (err != rclcpp::FutureReturnCode::SUCCESS) {
        return nullptr;
      }
      return cancel_future.get();
    }
  }

  /**
   * @brief Requests the goal result to the server.
   *
   * @param goal_handle The goal handle to be used.
   * @return Future to the WrappedResult object.
   */
  std::shared_future<typename ActionGoalHandleT::WrappedResult> get_result(
    const typename ActionGoalHandleT::SharedPtr goal_handle)
  {
    return client_->async_get_result(goal_handle);
  }

  /**
   * @brief Requests the goal result to the server and waits for the response.
   *
   * @param goal_handle The goal handle to be used.
   * @param timeout_msec The timeout to be used when waiting for the response (milliseconds).
   * @return The goal WrappedResult object.
   */
  std::shared_ptr<typename ActionGoalHandleT::WrappedResult> get_result_sync(
    const typename ActionGoalHandleT::SharedPtr goal_handle,
    int64_t timeout_msec = 0)
  {
    auto result_future = get_result(goal_handle);
    if (timeout_msec <= 0) {
      auto err = rclcpp::spin_until_future_complete(node_->shared_from_this(), result_future);
      if (err != rclcpp::FutureReturnCode::SUCCESS) {
        return nullptr;
      }
      return std::make_shared<typename ActionGoalHandleT::WrappedResult>(result_future.get());
    } else {
      auto err = rclcpp::spin_until_future_complete(
        node_->shared_from_this(),
        result_future,
        std::chrono::milliseconds(timeout_msec));
      if (err != rclcpp::FutureReturnCode::SUCCESS) {
        return nullptr;
      }
      return std::make_shared<typename ActionGoalHandleT::WrappedResult>(result_future.get());
    }
  }

  /**
   * @brief Calls the action, returns only when it has been completed, or canceled, or timed out.
   *
   * @param goal_msg The goal message to be sent.
   * @param cancel_on_timeout Whether to cancel the goal if it times out.
   * @param send_goal_timeout_msec The timeout to be used when sending the goal (milliseconds).
   * @param get_result_timeout_msec The timeout to be used when waiting for the result (milliseconds).
   * @param cancel_timeout_msec The timeout to be used when canceling the goal (milliseconds).
   *
   * @return Tuple containing accepted flag, result code, and pointer to the result message.
   */
  std::tuple<bool, rclcpp_action::ResultCode, std::shared_ptr<ActionResultT>> call_sync(
    const typename ActionT::Goal & goal_msg,
    bool cancel_on_timeout = false,
    int64_t send_goal_timeout_msec = 0,
    int64_t get_result_timeout_msec = 0,
    int64_t cancel_timeout_msec = 0)
  {
    // Send the goal
    auto goal_handle = send_goal_sync(goal_msg, send_goal_timeout_msec);
    if (!goal_handle) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "%s: server did not respond to/accept goal request",
        action_name_.c_str());
      return std::make_tuple(false, rclcpp_action::ResultCode::UNKNOWN, nullptr);
    }
    RCLCPP_INFO(node_->get_logger(), "%s: goal ACCEPTED", action_name_.c_str());

    // Get the result, using the internal wrapper to deal with the back-end
    // The following can happen now:
    // - The goal is completed before the timeout expires, and we get the result
    // - The timeout expires, but we should not cancel the goal, simply return
    // - The timeout expires, and we should cancel the goal without waiting for the result
    // - The timeout expires, and we should cancel the goal and wait for the result
    // In the last case, the following can happen:
    // - The goal cancellation response arrives before the timeout expires
    // - The timeout expires before a cancellation response is received
    auto goal_result = get_result_sync(goal_handle, get_result_timeout_msec);
    if (goal_result != nullptr) {
      // The goal was completed before the timeout expired
      RCLCPP_INFO(node_->get_logger(), "%s: goal COMPLETED", action_name_.c_str());
      return std::make_tuple(true, goal_result->code, goal_result->result);
    } else {
      if (!cancel_on_timeout) {
        // The timeout expired, but we should not cancel the goal
        RCLCPP_ERROR(node_->get_logger(), "%s: goal timed out", action_name_.c_str());
        return std::make_tuple(true, rclcpp_action::ResultCode::UNKNOWN, nullptr);
      } else {
        RCLCPP_WARN(node_->get_logger(), "%s: goal timed out, CANCELING...", action_name_.c_str());
        if (cancel_timeout_msec == 0) {
          // We should cancel the goal without waiting for the result
          cancel(goal_handle);
          return std::make_tuple(true, rclcpp_action::ResultCode::CANCELED, nullptr);
        } else {
          // We should cancel the goal and wait for the result
          auto cancel_result = cancel_sync(goal_handle, cancel_timeout_msec);
          if (cancel_result != nullptr) {
            // Goal cancellation response arrived before the timeout expired
            RCLCPP_INFO(
              node_->get_logger(),
              "%s: goal cancellation response received",
              action_name_.c_str());
            if (cancel_result->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
              return std::make_tuple(true, rclcpp_action::ResultCode::CANCELED, nullptr);
            } else {
              return std::make_tuple(true, rclcpp_action::ResultCode::UNKNOWN, nullptr);
            }
          } else {
            // Timeout expired before a cancellation response was received
            RCLCPP_WARN(
              node_->get_logger(),
              "%s: goal cancellation response timed out",
              action_name_.c_str());
            return std::make_tuple(true, rclcpp_action::ResultCode::UNKNOWN, nullptr);
          }
        }
      }
    }
  }

private:
  rclcpp::Node * node_;
  std::string action_name_;
  FeedbackCallbackT<ActionT> feedback_callback_;

  typename rclcpp_action::Client<ActionT>::SharedPtr client_;
  typename rclcpp_action::Client<ActionT>::SendGoalOptions client_opts_;
};

} // namespace simple_actionclient

#endif // SIMPLE_ACTIONCLIENT_CPP__SIMPLE_ACTIONCLIENT_HPP_
