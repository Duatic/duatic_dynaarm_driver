/*
 * Copyright 2026 Duatic AG
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <type_traits>
#include <utility>
#include <optional>

#include <duatic_dynaarm_controllers/exceptions.hpp>

namespace duatic_dynaarm_controllers::compat
{

/**
 * @brief Compile-time trait to detect whether a loaned interface provides
 *        get_optional<double>().
 *
 * This is used to distinguish between ROS 2 Rolling (which provides
 * get_optional<T>()) and older distributions such as Jazzy, where interfaces
 * expose get_value() and are always assumed to be valid.
 *
 * @tparam T Interface type to inspect
 */
template <typename T, typename = void>
struct has_get_optional_double : std::false_type
{
};

/**
 * @brief Specialization for interfaces that implement get_optional<double>().
 *
 * This matches the Rolling hardware_interface API.
 */
template <typename T>
struct has_get_optional_double<T, std::void_t<decltype(std::declval<const T&>().template get_optional<double>())>>
  : std::true_type
{
};

/**
 * @brief Retrieve the current value from a loaned state or command interface.
 *
 * This function provides a compatibility layer between ROS 2 Jazzy and Rolling:
 * - **Rolling**: uses `get_optional<double>()` and returns `std::nullopt` if no
 *   value is available.
 * - **Jazzy**: uses `get_value()` and always returns a valid value.
 *
 * @tparam LoanedInterfaceT Type of the loaned interface
 * @param iface Reference to the loaned interface
 * @return std::optional<double> containing the value if available
 */
template <class LoanedInterfaceT>
inline std::optional<double> try_get_value(const LoanedInterfaceT& iface)
{
  if constexpr (has_get_optional_double<LoanedInterfaceT>::value) {
    return iface.template get_optional<double>();  // Rolling
  } else {
    return iface.template get_value();  // Jazzy (always valid)
  }
}

/**
 * @brief Retrieve the current value from a loaned interface or throw on failure.
 *
 * This is a strict variant of @ref try_get_value(). If the interface value is
 * not available (e.g. on ROS 2 Rolling during synchronization), an exception
 * is thrown instead of returning a fallback.
 *
 * Intended for fail-fast code paths where missing interface data must abort
 * the current operation to avoid unsafe behavior.
 *
 * @tparam LoanedInterfaceT Type of the loaned interface
 * @param iface Reference to the loaned interface
 * @return The current interface value
 * @throws std::runtime_error If the value is not available
 */
template <class LoanedInterfaceT>
inline double require_value(const LoanedInterfaceT& iface)
{
  auto opt = try_get_value(iface);
  if (!opt) {
    throw duatic_dynaarm_controllers::exceptions::MissingInterfaceValue("State interface value not available");
  }
  return *opt;
}

/**
 * @brief Compile-time trait to detect the presence of try_publish(msg).
 *
 * This is used to distinguish between newer realtime publisher APIs
 * (Rolling) and older APIs which require trylock/msg_/unlockAndPublish().
 *
 * @tparam PubT Publisher type
 * @tparam MsgT Message type
 */
template <typename PubT, typename MsgT, typename = void>
struct has_try_publish : std::false_type
{
};

/**
 * @brief Specialization for publishers providing try_publish(msg).
 */
template <typename PubT, typename MsgT>
struct has_try_publish<PubT, MsgT,
                       std::void_t<decltype(std::declval<PubT>()->try_publish(std::declval<const MsgT&>()))>>
  : std::true_type
{
};

/**
 * @brief Publish a message using a realtime-safe publisher.
 *
 * This function abstracts over the API differences between ROS 2 distributions:
 * - **Rolling**: uses `try_publish(msg)`
 * - **Jazzy / older**: uses `trylock()`, writes to `msg_`, then calls
 *   `unlockAndPublish()`
 *
 * The return value of `try_publish()` (if any) is intentionally ignored to
 * maintain consistent behavior across distributions.
 *
 * @tparam PubT Realtime publisher type (usually a std::unique_ptr or similar)
 * @tparam MsgT Message type
 * @param pub Realtime publisher instance
 * @param msg Message to publish
 */
template <typename PubT, typename MsgT>
inline void publish_rt(PubT& pub, const MsgT& msg)
{
  if constexpr (has_try_publish<PubT, MsgT>::value) {
    pub->try_publish(msg);  // Rolling
  } else {
    if (pub->trylock()) {  // Jazzy
      pub->msg_ = msg;
      pub->unlockAndPublish();
    }
  }
}

}  // namespace duatic_dynaarm_controllers::compat
