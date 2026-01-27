#include <stdexcept>
#include <string>

namespace dynaarm_controllers::exceptions
{

/**
 * @brief Exception thrown when a required hardware interface value is unavailable.
 *
 * This typically indicates a transient synchronization issue (Rolling API),
 * not a programming error.
 */
struct MissingInterfaceValue : public std::runtime_error
{
  explicit MissingInterfaceValue(const std::string& msg) : std::runtime_error(msg)
  {
  }
};

}  // namespace dynaarm_controllers::exceptions
