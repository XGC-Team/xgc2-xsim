#include "server.hpp"

namespace xsim {
namespace {
int http_status(xgc2::chassis_hold::Status status) {
  using xgc2::chassis_hold::Status;
  switch (status) {
    case Status::ok: return 200;
    case Status::invalid_argument: return 400;
    case Status::not_found: return 404;
    case Status::internal: break;
  }
  return 500;
}
} // namespace

// The whole transport binding of the chassis HOLD service: the capability entry of the describe facts and the
// method calls. Everything else in the server is independent of it, so a library adapter can replace both
// functions. The two host actions around the call stay with the host: running the queued cmd_vel callbacks
// before a Release, and waking the world thread after an Engage.

// The capabilities this world serves and, for each, the entities it acts on (describe facts, xrpc.md method
// addressing). The chassis are the entities of the HOLD roster, so the list follows the world as entities come
// and go.
Json Server::capabilities() {
  Json chassis = Json::array();
  for (const auto &robot : world_.hold().describe().robots) chassis.push_back(robot.robot_id);
  return Json::array({Json{{"name", chassis_hold_capability}, {"entities", std::move(chassis)}}});
}

// POST /v1/call/<service>/<Method>: one method of a capability this world serves, with the JSON request as the
// body. The reply is the JSON result, or the XRPC error envelope around the error object of the service.
// Engage replies when the world thread has written zero, or after the service's reply wait, and never holds the
// input thread.
bool Server::capability_call(const xgc2::xrpc::HttpRequest &request, const std::string &service,
                             const std::string &method, xgc2::xrpc::HttpReply reply) {
  if (service != chassis_hold_capability) return false;
  if (!hold_service_) {
    reply.complete(xgc2::xrpc::http_error(503, "unavailable", "the capability is stopping"));
    return true;
  }
  // A release takes effect after every cmd_vel received so far has been through the gate: callbacks that
  // are still queued for this thread run now, while the robots are still held.
  if (method == "Release" && io_.poll_inputs) io_.poll_inputs();
  hold_service_->call_async(method, request.body, request.deadline, [reply](xgc2::chassis_hold::ServiceReply result) {
    xgc2::xrpc::HttpResponse response;
    response.status = http_status(result.status);
    response.headers.emplace_back("Content-Type", "application/json");
    response.body = result.status == xgc2::chassis_hold::Status::ok ? std::move(result.body)
                                                                    : "{\"error\":" + result.body + "}";
    reply.complete(std::move(response));
  });
  // The gate is closed by now; a paused or idle world thread must still write the zero soon.
  if (method == "Engage") world_.wake();
  return true;
}
} // namespace xsim
