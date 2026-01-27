#pragma once

#include <type_traits>
#include <utility>
#include <optional>

namespace dynaarm_controllers::compat
{

template <typename T, typename = void>
struct has_get_optional_double : std::false_type {};

template <typename T>
struct has_get_optional_double<T, std::void_t<decltype(std::declval<const T&>().template get_optional<double>())>>
  : std::true_type {};

template <class LoanedInterfaceT>
inline std::optional<double> try_get_value(const LoanedInterfaceT & iface)
{
  if constexpr (has_get_optional_double<LoanedInterfaceT>::value) {
    return iface.template get_optional<double>();   // Rolling
  } else {
    return iface.template get_value();              // Jazzy (always valid)
  }
}

template <typename PubT, typename MsgT, typename = void>
struct has_try_publish : std::false_type
{
};

template <typename PubT, typename MsgT>
struct has_try_publish<PubT, MsgT,
                       std::void_t<decltype(std::declval<PubT>()->try_publish(std::declval<const MsgT&>()))>>
  : std::true_type
{
};

template <typename PubT, typename MsgT>
inline void publish_rt(PubT& pub, const MsgT& msg)
{
  if constexpr (has_try_publish<PubT, MsgT>::value) {
    pub->try_publish(msg);  // return type irrelevant
  } else {
    if (pub->trylock()) {
      pub->msg_ = msg;
      pub->unlockAndPublish();
    }
  }
}
}  // namespace dynaarm_controllers::compat
