#include "subscribe_almemory.hpp"
#include "../helpers/driver_helpers.hpp"

namespace naoqi
{
/**
 * Subscribes to an ALMemory event using the `subscriber` method,
 * and directly subscribing to the resulting signal.
 * This implementation works for NAOqi 2.8 and higher only, using a recent
 * libqi.
 */
class QiALMemorySubscriber : public ALMemorySubscriber
{
  public:
  QiALMemorySubscriber(qi::AnyObject subscriber, qi::SignalLink link_id)
      : _subscriber(subscriber), _link_id(link_id)
  {
  }

  virtual qi::Future<void> unsubscribe()
  {
    int link_id = _link_id.swap(0);
    if (link_id == 0)
      {
        return qi::Future<void>(nullptr);
      }

    return _subscriber.disconnect(link_id);
  }

  private:
  qi::AnyObject _subscriber;
  qi::Atomic<qi::SignalLink> _link_id;
};

/**
 * Subscribes to an ALMemory event by registering a service with a callback.
 * It requires the session to listen to an address reachable by the robot,
 * and thus requires opening a server port for the NAOqi driver.
 * This implementation is the only one compatible when connecting to NAOqi 2.5
 * from a recent libqi.
 */
class LegacyALMemorySubscriber : public ALMemorySubscriber
{
  public:
  LegacyALMemorySubscriber(qi::SessionPtr session, int service_id)
      : _session(session), _service_id(service_id)
  {
  }

  virtual qi::Future<void> unsubscribe()
  {
    int service_id = _service_id.swap(0);
    if (service_id == 0)
      {
        return qi::Future<void>(nullptr);
      }

    return _session->unregisterService(service_id);
  }

  private:
  qi::SessionPtr _session;
  qi::Atomic<int> _service_id;
};

/**
 * A service dedicated to receive ALMemory events and forward them to an
 * arbitrary callback.
 */
class ALMemorySubscriberService
{
  public:
  ALMemorySubscriberService(std::function<void(qi::AnyValue)> callback)
      : _callback(std::move(callback))
  {
  }

  void onEvent(const std::string& /*key*/, const qi::AnyValue& value,
               const qi::AnyValue& /* message */)
  {
    _callback(value);
  }

  private:
  std::function<void(qi::AnyValue)> _callback;
};

QI_REGISTER_OBJECT(ALMemorySubscriberService, onEvent)

/**
 * Subscribes to an ALMemory event the right way for the given NAOqi version.
 */
std::unique_ptr<ALMemorySubscriber>
subscribe(const robot::NaoqiVersion& naoqi_version, qi::SessionPtr session,
          const std::string& key, std::function<void(qi::AnyValue)> callback)
{
  auto memory = session->service("ALMemory").value();
  if (helpers::driver::isNaoqiVersionLesser(naoqi_version, 2, 8))
    {
      // Create one service per subscription.
      auto subscriber_service =
          boost::make_shared<ALMemorySubscriberService>(std::move(callback));
      // Yes the service name might get into collision. Let us not bother
      // until it happens.
      auto service_id =
          session
              ->registerService(std::string("ROSDriver_subscriber_") + key,
                                subscriber_service)
              .value();
      // Note that the service is only owned by the session it is
      // registered to. It will be destructed when unregistering (when
      // calling `unsubscribe`).
      return std::make_unique<LegacyALMemorySubscriber>(std::move(session),
                                                        std::move(service_id));
    }
  else
    {
      auto qi_subscriber = memory.call<qi::AnyObject>("subscribe", key);
      auto signal_link =
          qi_subscriber.connect("signal", std::move(callback)).value();
      return std::make_unique<QiALMemorySubscriber>(std::move(qi_subscriber),
                                                    std::move(signal_link));
    }
}

}  // namespace naoqi