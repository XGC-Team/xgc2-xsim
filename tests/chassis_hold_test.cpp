// Chassis HOLD in the world (the xgc2-chassis-hold contract): roster, the Velocity gate in apply(), the zero
// output and cancellation in the boundary tick, receipt fence, feedback, re-created entities and the wake of
// a paused world. Boundaries are driven by hand unless a test starts the world thread.
#include "core/world.hpp"
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>

using namespace xsim;
using xgc2::chassis_hold::Outcome;
using xgc2::chassis_hold::Stage;

namespace {
constexpr int64_t epoch = 1700000000000000000LL;

Ticket command(Op op, Key key = {}) {
  auto result = std::make_shared<Command>();
  result->op = op;
  result->key = key;
  return result;
}
void execute(World &world, const Ticket &request) {
  world.submit(request);
  world.boundary();
  assert(request->phase == 2);
}
std::shared_ptr<Entity> add(World &world, Kind kind, const std::string &name,
                            const std::string &public_id = {}, bool expect = true) {
  Config config;
  config.name = name;
  config.kind = kind;
  auto entity = std::make_shared<Entity>(config, public_id);
  auto request = command(Op::Add);
  request->prepared = std::make_unique<Prepared>(Prepared{entity, prepare_model(config)});
  execute(world, request);
  assert(request->result.success == expect);
  if (!expect) assert(request->result.reason == 4);
  return entity;
}
void enable(World &world, const std::shared_ptr<Entity> &entity) {
  auto request = command(Op::SetEnabled, {entity->id, entity->generation});
  request->enabled = true;
  execute(world, request);
  assert(request->result.success && entity->enabled);
}
void remove(World &world, const std::shared_ptr<Entity> &entity) {
  auto request = command(Op::Remove, {entity->id, entity->generation});
  execute(world, request);
  assert(request->result.success && !entity->alive);
}
// A forward velocity command received now, in the HOLD domain's time.
Ticket drive(World &world, const std::shared_ptr<Entity> &entity, double forward = 1.0) {
  auto request = command(Op::Velocity, {entity->id, entity->generation});
  request->velocity = {forward, 0, 0};
  request->received_ns = world.hold().now();
  return request;
}
State state_of(World &world, const std::shared_ptr<Entity> &entity) {
  for (const auto &state : world.capture().states)
    if (state.key.id == entity->id) return state;
  assert(false);
  return {};
}
void run(World &world, int steps) {
  for (int i = 0; i < steps; ++i) {
    world.advance();
    world.boundary();
  }
}
xgc2::chassis_hold::RobotState hold_state(World &world, const std::string &id) {
  const auto snapshot = world.hold().state({id});
  assert(snapshot.unknown.empty() && snapshot.robots.size() == 1);
  return snapshot.robots.front();
}
// Boundaries at about the world period for up to two seconds until the stage is reached.
bool reach(World &world, const std::string &id, Stage stage) {
  const auto deadline = Clock::now() + std::chrono::seconds(2);
  while (Clock::now() < deadline) {
    if (hold_state(world, id).stage == stage) return true;
    run(world, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return hold_state(world, id).stage == stage;
}

void roster_and_gate() {
  World world(epoch, 2000000, 8000000, 8, 10000000, "0123456789abcdef0123456789abcdef");
  assert(world.hold().instance_id() == "0123456789abcdef0123456789abcdef");
  auto flight = add(world, Kind::FS150, "uav");
  auto scout = add(world, Kind::Scout, "scout", "ugv-1");
  auto mecanum = add(world, Kind::Mecanum, "mecanum"); // the public ID defaults to the name
  assert(scout->public_id == "ugv-1" && mecanum->public_id == "mecanum" && flight->public_id == "uav");
  // Scout and Mecanum entities are the roster, by public ID; both report feedback.
  const auto roster = world.hold().describe();
  assert(roster.robots.size() == 2);
  assert(roster.robots[0].robot_id == "mecanum" && roster.robots[1].robot_id == "ugv-1");
  assert(roster.robots[0].feedback && roster.robots[1].feedback);
  const auto unknown = world.hold().engage({"uav", "scout"}); // neither is a public ID of the roster
  assert(unknown[0].outcome == Outcome::unknown_robot && unknown[1].outcome == Outcome::unknown_robot);
  // Another entity cannot take over a live public ID.
  add(world, Kind::Mecanum, "other", "ugv-1", false);
  assert(world.hold().describe().robots.size() == 2);
  enable(world, scout);
  enable(world, mecanum);

  // Released entities move.
  execute(world, drive(world, scout));
  execute(world, drive(world, mecanum, 0.5));
  run(world, 100);
  assert(state_of(world, scout).velocity.norm() > 0.9 && state_of(world, mecanum).velocity.norm() > 0.4);

  // Engage is immediate for the state and the gate, the zero is written at the next boundary.
  const auto engaged = world.hold().engage({"ugv-1"});
  assert(engaged[0].outcome == Outcome::engaged);
  assert(engaged[0].state->held && engaged[0].state->revision == 1 && engaged[0].state->stage == Stage::gated);
  assert(world.hold().engage({"ugv-1"})[0].outcome == Outcome::already_held);
  assert(state_of(world, scout).velocity.norm() > 0.9); // no boundary yet: the output has not changed
  const auto before = state_of(world, scout).stamp;
  world.boundary();
  assert(hold_state(world, "ugv-1").stage == Stage::zero_written);
  assert(state_of(world, scout).stamp == before);

  // The other robot is not touched, and the held one is not disabled: sensors and publication keep running.
  assert(state_of(world, scout).enabled && scout->enabled && scout->alive);
  assert(!hold_state(world, "mecanum").held);
  run(world, 100);
  assert(state_of(world, scout).velocity.norm() < 0.02);
  assert(state_of(world, mecanum).velocity.norm() > 0.4);
  const auto mecanum_at = state_of(world, mecanum).position;

  // A command that reaches a held entity is refused where it would reach the model.
  assert(world.metrics.hold_refused == 0);
  const auto refused = drive(world, scout);
  execute(world, refused);
  assert(!refused->result.success && refused->result.reason == 6);
  assert(world.metrics.hold_refused == 1);
  run(world, 50);
  assert(state_of(world, scout).velocity.norm() < 0.02);
  assert((state_of(world, mecanum).position - mecanum_at).norm() > 0.04);
  assert(!world.hold().admit("ugv-1", world.hold().now()) && world.hold().admit("mecanum", world.hold().now()));

  // Held and at rest for the dwell: stopped, from the entity's own twist.
  assert(reach(world, "ugv-1", Stage::stopped));
  assert(hold_state(world, "ugv-1").stopped_at && hold_state(world, "ugv-1").zero_written_at);

  // Release: compare and set on the revision, new commands pass.
  const auto instance = world.hold().instance_id();
  assert(world.hold().release("ffffffffffffffffffffffffffffffff", {{"ugv-1", 1}})[0].outcome == Outcome::conflict);
  assert(world.hold().release(instance, {{"ugv-1", 7}})[0].outcome == Outcome::conflict);
  const auto released = world.hold().release(instance, {{"ugv-1", 1}, {"mecanum", 0}});
  assert(released[0].outcome == Outcome::released && released[0].state->revision == 2);
  assert(released[1].outcome == Outcome::not_held);
  assert(world.hold().release(instance, {{"ugv-1", 1}})[0].outcome == Outcome::not_held); // no replay
  const auto accepted = drive(world, scout);
  execute(world, accepted);
  assert(accepted->result.success);
  run(world, 100);
  assert(state_of(world, scout).velocity.norm() > 0.9);
}

// A command received before the release does not execute after it, however long it waited in the queue.
void commands_queued_before_release() {
  World world(epoch);
  auto scout = add(world, Kind::Scout, "scout", "ugv-1");
  enable(world, scout);
  // The command waits for its simulation time (the same queue that holds not-yet-due commands).
  auto stale = drive(world, scout);
  stale->at = world.metrics.sim_ns + 20000000;
  world.submit(stale);
  world.boundary();
  assert(stale->phase == 0);
  // engage and release without a boundary in between: no tick cancelled the queued command.
  world.hold().engage({"ugv-1"});
  assert(world.hold().release(world.hold().instance_id(), {{"ugv-1", 1}})[0].outcome == Outcome::released);
  // A command received after the release, due at the same time.
  auto fresh = drive(world, scout, 0.5);
  fresh->at = stale->at;
  world.submit(fresh);
  run(world, 15);
  assert(stale->phase == 2 && !stale->result.success && stale->result.reason == 6);
  assert(fresh->phase == 2 && fresh->result.success);
  run(world, 100);
  assert(state_of(world, scout).velocity.norm() > 0.4 && state_of(world, scout).velocity.norm() < 0.6);
}

// A command still waiting for its time when the entity is engaged is cancelled by the zero output.
void pending_controls_are_cancelled() {
  World world(epoch);
  auto scout = add(world, Kind::Scout, "scout", "ugv-1");
  enable(world, scout);
  auto pending = drive(world, scout);
  pending->at = world.metrics.sim_ns + 1000000000;
  world.submit(pending);
  world.boundary();
  assert(pending->phase == 0);
  world.hold().engage({"ugv-1"});
  assert(pending->phase == 0); // the cancellation is output: it happens at the next boundary
  world.boundary();
  assert(pending->phase == 3);
  assert(hold_state(world, "ugv-1").stage == Stage::zero_written);
}

// A removed entity keeps its HOLD state; a re-created one under the same public ID starts held.
void recreated_entity_is_held() {
  World world(epoch);
  auto scout = add(world, Kind::Scout, "scout", "ugv-1");
  enable(world, scout);
  world.hold().engage({"ugv-1"});
  world.boundary();
  assert(hold_state(world, "ugv-1").stage == Stage::zero_written);
  remove(world, scout);
  assert(world.hold().describe().robots.empty());
  const auto gone = world.hold().state({"ugv-1"});
  assert(gone.robots.empty() && gone.unknown.size() == 1);
  assert(world.hold().engage({"ugv-1"})[0].outcome == Outcome::unknown_robot);
  scout.reset();

  auto again = add(world, Kind::Scout, "scout_again", "ugv-1");
  // The boundary that added the entity also wrote the zero to its new model.
  const auto inherited = hold_state(world, "ugv-1");
  assert(inherited.held && inherited.revision == 1 && inherited.stage == Stage::zero_written);
  enable(world, again);
  const auto refused = drive(world, again);
  execute(world, refused);
  assert(!refused->result.success && refused->result.reason == 6);
  run(world, 50);
  assert(state_of(world, again).velocity.norm() < 1e-12);
  assert(world.hold().release(world.hold().instance_id(), {{"ugv-1", 1}})[0].outcome == Outcome::released);
  assert(hold_state(world, "ugv-1").revision == 2);
}

// A Mecanum jumps to zero; only the dwell separates zero_written from stopped.
void mecanum_reaches_stopped() {
  World world(epoch);
  auto mecanum = add(world, Kind::Mecanum, "mecanum", "ugv-m");
  enable(world, mecanum);
  execute(world, drive(world, mecanum));
  run(world, 50);
  assert(state_of(world, mecanum).velocity.norm() > 0.9);
  world.hold().engage({"ugv-m"});
  world.boundary();
  assert(hold_state(world, "ugv-m").stage == Stage::zero_written);
  assert(state_of(world, mecanum).velocity.norm() < 1e-12);
  assert(reach(world, "ugv-m", Stage::stopped));
}

// A paused world sleeps until woken: the state changes at once, the zero is written at the next boundary.
void paused_world_is_woken() {
  World world(epoch);
  auto scout = add(world, Kind::Scout, "scout", "ugv-1");
  enable(world, scout);
  execute(world, drive(world, scout));
  world.metrics.paused = true;
  world.start();
  const auto steps = world.metrics.steps.load();
  const auto engaged = world.hold().engage({"ugv-1"});
  assert(engaged[0].state->held && engaged[0].state->stage == Stage::gated);
  world.wake(); // what the management route does after an engage
  const auto deadline = Clock::now() + std::chrono::seconds(2);
  while (hold_state(world, "ugv-1").stage == Stage::gated && Clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  assert(hold_state(world, "ugv-1").stage == Stage::zero_written);
  assert(world.metrics.steps == steps && world.metrics.paused);
  // A release while paused is immediate too, and a command received after it is applied at the next boundary.
  assert(world.hold().release(world.hold().instance_id(), {{"ugv-1", 1}})[0].outcome == Outcome::released);
  auto fresh = drive(world, scout, 0.5);
  world.submit(fresh);
  assert(World::wait(fresh) && fresh->result.success);
  world.stop();
}
} // namespace

int main() {
  roster_and_gate();
  commands_queued_before_release();
  pending_controls_are_cancelled();
  recreated_entity_is_held();
  mecanum_reaches_stopped();
  paused_world_is_woken();
  std::cout << "chassis HOLD roster, gate, zero output, receipt fence, re-created entities and paused wake passed\n";
}
