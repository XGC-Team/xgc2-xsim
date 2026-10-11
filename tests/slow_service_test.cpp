#include "io/ros/entity.hpp"
#include <cassert>
#include <future>
#include <iostream>
using namespace xsim;
int main(int argc, char **argv) {
  ros::init(argc, argv, "xsim_slow_service_fixture");
  ros::CallbackQueue input, services;
  World world(1700000000000000000LL);
  ros::AsyncSpinner executor(2, &services);
  auto e = std::make_shared<Entity>(
      parse_entity({{"name", "slow_fixture"}, {"kind", "fs150"}}));
  auto ros_entity = std::make_shared<RosEntity>(e, world, input, services, Json::object());
  e->io = std::make_shared<EntityIO>(EntityIO{
    [ros_entity] { ros_entity->reconcile(); },
    [ros_entity](const State &state, const TelemetryRates &rates) { ros_entity->publish(state, rates); },
    [ros_entity] { return ros_entity->publish_sensor(); },
  });
  auto add = std::make_shared<Command>();
  add->op = Op::Add;
  add->prepared =
      std::make_unique<Prepared>(Prepared{e, prepare_model(e->config)});
  world.submit(add);
  world.apply_commands();
  auto start = std::make_shared<Command>();
  start->op=Op::Provider;start->action=1;start->key={e->id,0};
  start->prepared=std::make_unique<Prepared>(Prepared{e,prepare_model(e->config)});
  world.submit(start);world.apply_commands();assert(start->result.success && e->enabled);
  ros_entity->bind();
  ros::NodeHandle n;
  n.setCallbackQueue(&services);
  std::atomic<unsigned> entered{0};
  boost::function<bool(mavros_msgs::CommandBool::Request &,
                       mavros_msgs::CommandBool::Response &)>
      slow = [&](auto &, auto &reply) {
        ++entered;
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        reply.success = true;
        return true;
      };
  auto service = n.advertiseService("/xsim_fixture/slow", slow);
  executor.start();
  world.start();
  auto client = [] {
    ros::NodeHandle nh;
    auto c = nh.serviceClient<mavros_msgs::CommandBool>("/xsim_fixture/slow");
    assert(c.waitForExistence(ros::Duration(5)));
    mavros_msgs::CommandBool req;
    assert(c.call(req) && req.response.success);
  };
  auto one = std::async(std::launch::async, client),
       two = std::async(std::launch::async, client);
  const auto deadline = Clock::now() + std::chrono::seconds(5);
  while (entered < 2 && Clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  assert(entered == 2);
  const auto before = world.metrics.steps.load();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const auto advanced = world.metrics.steps - before;
  assert(advanced > 0);
  auto pause = std::make_shared<Command>();
  pause->op = Op::Pause;
  world.submit(pause);
  assert(World::wait(pause) && pause->result.success);
  one.get();
  two.get();
  world.stop();
  executor.stop();
  ros::shutdown();
  std::cout << Json({{"result", "PASS"},
                     {"blocked_service_threads", 2}, {"enabled_fs150_physics", true},
                     {"blocked_ms", 250},
                     {"steps_during_100ms_block", advanced},
                     {"management_pause_applied", true}})
                   .dump()
            << '\n';
}
