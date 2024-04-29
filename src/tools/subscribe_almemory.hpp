#include <functional>

#include <qi/anyobject.hpp>
#include <qi/session.hpp>

#include <naoqi_driver/tools.hpp>

namespace naoqi
{
/**
 * Interface to subscribe to ALMemory events,
 * so that code remain compatible regardless the implementation details.
 */
class ALMemorySubscriber
{
  public:
  /**
   * Destructor ensures unsubscription occurred, to avoid leakage.
   */
  virtual ~ALMemorySubscriber() { unsubscribe(); }

  /**
   * Unsubscribes from the event.
   * After the function returns, no eventual callback is called anymore.
   * But unsubscription may still be ongoing, and the future might complete
   * later. Must be reentrant.
   */
  virtual qi::Future<void> unsubscribe() { return qi::Future<void>(nullptr); }
};

/**
 * Subscribes to an ALMemory event the right way for the given NAOqi version.
 */
std::unique_ptr<ALMemorySubscriber>
subscribe(const robot::NaoqiVersion& naoqi_version, qi::SessionPtr session,
          const std::string& key, std::function<void(qi::AnyValue)> callback);
}  // namespace naoqi