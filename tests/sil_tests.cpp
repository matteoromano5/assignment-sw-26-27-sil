#include "sil.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>

namespace {

using raceup::sil::Frame;
using raceup::sil::OutputId;
using raceup::sil::SensorId;
using raceup::sil::SilSystem;

constexpr auto kWait = std::chrono::milliseconds(300);

struct TestContext {
  int failures = 0;

  void expect(const bool condition, const char* message) {
    if (condition) {
      std::printf("[PASS] %s\n", message);
    } else {
      std::printf("[FAIL] %s\n", message);
      ++failures;
    }
  }
};

template <typename Predicate>
bool wait_for_matching_frame(SilSystem& system, const std::uint32_t id,
                             Predicate predicate, Frame& result) {
  const auto deadline = std::chrono::steady_clock::now() + kWait;
  while (std::chrono::steady_clock::now() < deadline) {
    Frame frame{};
    if (system.receive_frame(frame, std::chrono::milliseconds(20)) &&
        frame.id == id && predicate(frame)) {
      result = frame;
      return true;
    }
  }
  return false;
}

bool wait_for_frame(SilSystem& system, const std::uint32_t id, Frame& result) {
  return wait_for_matching_frame(
      system, id, [](const Frame&) { return true; }, result);
}

Frame vehicle_status(const bool manual, const bool ready_to_drive,
                     const std::uint8_t speed) {
  Frame frame = raceup::sil::make_frame(raceup::sil::can_id::VehicleStatus, 3U);
  frame.data[0] = static_cast<std::uint8_t>(manual);
  frame.data[1] = static_cast<std::uint8_t>(ready_to_drive);
  frame.data[2] = speed;
  return frame;
}

Frame brake_pressure(const std::uint8_t front, const std::uint8_t rear,
                     const std::uint8_t pneumatic_left,
                     const std::uint8_t pneumatic_right) {
  Frame frame = raceup::sil::make_frame(raceup::sil::can_id::BrakePressure, 4U);
  frame.data[0] = front;
  frame.data[1] = rear;
  frame.data[2] = pneumatic_left;
  frame.data[3] = pneumatic_right;
  return frame;
}

Frame ebs_command(const bool brake_request, const bool mission_finished) {
  Frame frame = raceup::sil::make_frame(raceup::sil::can_id::EbsCommand, 2U);
  frame.data[0] = static_cast<std::uint8_t>(brake_request);
  frame.data[1] = static_cast<std::uint8_t>(mission_finished);
  return frame;
}

void configure_nominal_inputs(SilSystem& system) {
  system.set_sensor(SensorId::SmuApps1Raw, 3500U);
  system.set_sensor(SensorId::SmuApps2Raw, 3000U);
  system.set_sensor(SensorId::SmuBrakeRaw, 2500U);
  system.set_sensor(SensorId::EbsValvesPowered, 1U);
  system.set_sensor(SensorId::EbsAsmsActive, 1U);
  system.set_sensor(SensorId::EbsWatchdogHealthy, 1U);
}

void send_nominal_ebs_frames(SilSystem& system) {
  system.send_frame(vehicle_status(false, true, 0U));
  system.send_frame(brake_pressure(40U, 40U, 8U, 8U));
  system.send_frame(ebs_command(false, false));
}

// Lifecycle / Infrastructure Tests
void test_lifecycle(TestContext& context) {
  SilSystem system;
  context.expect(system.start(), "SIL starts");
  context.expect(system.running(), "SIL reports running");
  system.stop();
  context.expect(!system.running(), "SIL stops and joins its threads");
}

void test_receive_frame_timeout(TestContext& context) {
  SilSystem system;
  Frame frame{};

  const auto start = std::chrono::steady_clock::now();

  const bool received =
      system.receive_frame(frame, std::chrono::milliseconds(50));

  const auto elapsed = std::chrono::steady_clock::now() - start;

  context.expect(!received,
                 "receive_frame returns false when no frame is available");

  context.expect(elapsed < std::chrono::milliseconds(200),
                 "receive_frame respects the timeout without hanging");
}

// SMU Tests
void test_smu_nominal(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  Frame frame{};
  const bool received = wait_for_frame(
      system, raceup::sil::can_id::SmuSense, frame);
  context.expect(received, "SMU publishes a nominal sense frame");
  if (received) {
    context.expect(frame.dlc == 3U, "SMU sense frame has the expected DLC");
    context.expect(frame.data[0] > 40U && frame.data[0] < 60U,
                   "SMU computes the expected throttle percentage");
    context.expect(frame.data[2] == 0U, "SMU reports no pedal fault");
  }

  system.stop();
}

void test_smu_fault(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  Frame nominal{};
  const bool nominal_received = wait_for_frame(
      system, raceup::sil::can_id::SmuSense, nominal);
  context.expect(nominal_received,
                 "SMU publishes a frame before a live sensor change");

  system.set_sensor(SensorId::SmuApps2Raw, 3490U);

  Frame frame{};
  const bool received = wait_for_matching_frame(
      system, raceup::sil::can_id::SmuSense,
      [](const Frame& candidate) {
        return candidate.data[0] == 0U && candidate.data[2] != 0U;
      },
      frame);
  context.expect(received, "SMU publishes a fault frame after a live change");
  if (received) {
    context.expect(frame.data[0] == 0U,
                   "SMU inhibits throttle on APP disagreement");
    context.expect(frame.data[2] != 0U, "SMU reports the APP fault");
  }

  system.set_sensor(SensorId::SmuApps2Raw, 3000U);
  Frame recovered{};
  const bool recovered_received = wait_for_matching_frame(
      system, raceup::sil::can_id::SmuSense,
      [](const Frame& candidate) {
        return candidate.data[0] != 0U && candidate.data[2] == 0U;
      },
      recovered);
  context.expect(recovered_received,
                 "SMU publishes a frame after recovering the live input");
  if (recovered_received) {
    context.expect(recovered.data[0] != 0U && recovered.data[2] == 0U,
                   "SMU returns to the nominal state after recovery");
  }

  system.stop();
}

void test_smu_periodic_frames(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  Frame first{};
  Frame second{};

  const bool first_received =
      wait_for_frame(system, raceup::sil::can_id::SmuSense, first);

  const bool second_received =
      wait_for_frame(system, raceup::sil::can_id::SmuSense, second);

  context.expect(first_received,
                 "SMU emits the first periodic sense frame");
  context.expect(second_received,
                 "SMU continues emitting periodic sense frames");

  system.stop();
}

void test_smu_plausibility_fault(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);

  system.set_sensor(SensorId::SmuApps1Raw, 3500U);  // 50%
  system.set_sensor(SensorId::SmuApps2Raw, 3300U);  // 20%

  system.start();

  Frame frame{};
  const bool received = wait_for_matching_frame(
      system, raceup::sil::can_id::SmuSense,
      [](const Frame& candidate) {
        return candidate.data[0] == 0U && candidate.data[2] != 0U;
      },
      frame);

  context.expect(received,
                 "SMU detects APP plausibility disagreement");

  if (received) {
    context.expect(frame.data[0] == 0U,
                   "SMU inhibits throttle on APP plausibility fault");
    context.expect(frame.data[2] != 0U,
                   "SMU reports an APP plausibility fault");
  }

  system.stop();
}

// EBS nominal/comunication Tests
void test_ebs_missing_critical_inputs(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                 "EBS keeps ASNode open when critical inputs are missing");

  context.expect(system.output(OutputId::EbsValve1Engaged) == 1U &&
                     system.output(OutputId::EbsValve2Engaged) == 1U,
                 "EBS engages both valves when critical inputs are missing");

  context.expect(system.output(OutputId::EbsFaultCode) != 0U,
                 "EBS reports a fault when critical inputs are missing");

  system.stop();
}

void test_ebs_nominal_and_stale_input(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  system.send_frame(vehicle_status(false, true, 0U));
  system.send_frame(brake_pressure(40U, 40U, 8U, 8U));
  system.send_frame(ebs_command(false, false));

  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                 "EBS closes ASNode with valid fresh inputs");
  context.expect(system.output(OutputId::EbsValve1Engaged) == 0U &&
                     system.output(OutputId::EbsValve2Engaged) == 0U,
                 "EBS releases both valves in the nominal state");

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                 "EBS opens ASNode after critical inputs become stale");
  context.expect(system.output(OutputId::EbsFaultCode) != 0U,
                 "EBS reports a stale-input fault");

  system.stop();
}

void test_ebs_periodically_refreshed_inputs(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  for (int i = 0; i < 5; ++i) {
    send_nominal_ebs_frames(system);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                 "EBS remains nominal while critical inputs are refreshed");

  context.expect(system.output(OutputId::EbsValve1Engaged) == 0U &&
                     system.output(OutputId::EbsValve2Engaged) == 0U,
                 "EBS keeps both valves released with fresh inputs");

  context.expect(system.output(OutputId::EbsFaultCode) == 0U,
                 "EBS reports no fault while inputs remain fresh");

  system.stop();
}

void test_ebs_requires_ready_to_drive(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  system.send_frame(vehicle_status(true, false, 0U));
  system.send_frame(brake_pressure(40U, 40U, 8U, 8U));
  system.send_frame(ebs_command(false, false));

  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                 "EBS stays safe when ready-to-drive is inactive");
  context.expect(system.output(OutputId::EbsValve1Engaged) == 1U &&
                     system.output(OutputId::EbsValve2Engaged) == 1U,
                 "EBS engages both valves without ready-to-drive");
  context.expect(system.output(OutputId::EbsFaultCode) != 0U,
                 "EBS reports a readiness fault");

  system.send_frame(vehicle_status(true, true, 0U));
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                 "EBS closes ASNode after ready-to-drive becomes active");
  context.expect(system.output(OutputId::EbsValve1Engaged) == 0U &&
                     system.output(OutputId::EbsValve2Engaged) == 0U,
                 "EBS releases both valves after readiness is restored");

  system.stop();
}

// EBS fault/safety Tests
void test_ebs_invalid_pressure(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  system.send_frame(vehicle_status(false, true, 0U));
  system.send_frame(brake_pressure(40U, 40U, 5U, 8U));
  system.send_frame(ebs_command(false, false));

  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                 "EBS opens ASNode with invalid pneumatic pressure");

  context.expect(system.output(OutputId::EbsValve1Engaged) == 1U &&
                     system.output(OutputId::EbsValve2Engaged) == 1U,
                 "EBS engages both valves with invalid pneumatic pressure");

  context.expect(system.output(OutputId::EbsFaultCode) != 0U,
                 "EBS reports a pressure fault");

  system.stop();
}

void test_ebs_watchdog_fault(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  system.send_frame(vehicle_status(false, true, 0U));
  system.send_frame(brake_pressure(40U, 40U, 8U, 8U));
  system.send_frame(ebs_command(false, false));

  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                 "EBS is nominal before watchdog fault");

  system.set_sensor(SensorId::EbsWatchdogHealthy, 0U);

  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                 "EBS opens ASNode after watchdog fault");

  context.expect(system.output(OutputId::EbsValve1Engaged) == 1U &&
                     system.output(OutputId::EbsValve2Engaged) == 1U,
                 "EBS engages both valves after watchdog fault");

  context.expect(system.output(OutputId::EbsFaultCode) != 0U,
                 "EBS reports a watchdog fault");

  system.stop();
}

void test_ebs_asms_fault(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  send_nominal_ebs_frames(system);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                 "EBS is nominal before ASMS fault");

  system.set_sensor(SensorId::EbsAsmsActive, 0U);

  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                 "EBS opens ASNode when ASMS becomes inactive");

  context.expect(system.output(OutputId::EbsValve1Engaged) == 1U &&
                     system.output(OutputId::EbsValve2Engaged) == 1U,
                 "EBS engages both valves when ASMS becomes inactive");

  context.expect(system.output(OutputId::EbsFaultCode) != 0U,
                 "EBS reports an ASMS fault");

  system.stop();
}

void test_ebs_valve_power_fault(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  send_nominal_ebs_frames(system);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                 "EBS is nominal before valve-power fault");

  system.set_sensor(SensorId::EbsValvesPowered, 0U);

  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                 "EBS opens ASNode when valve power is lost");

  context.expect(system.output(OutputId::EbsValve1Engaged) == 1U &&
                     system.output(OutputId::EbsValve2Engaged) == 1U,
                 "EBS enters safe outputs when valve power is lost");

  context.expect(system.output(OutputId::EbsFaultCode) != 0U,
                 "EBS reports a fault when valve power is lost");

  system.stop();
}

void test_ebs_brake_request(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  system.send_frame(vehicle_status(false, true, 0U));
  system.send_frame(brake_pressure(40U, 40U, 8U, 8U));
  system.send_frame(ebs_command(false, false));

  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                 "EBS is nominal before a live brake request");

  system.send_frame(ebs_command(true, false));
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                 "EBS opens ASNode after a live brake request");
  context.expect(system.output(OutputId::EbsValve1Engaged) == 1U &&
                     system.output(OutputId::EbsValve2Engaged) == 1U,
                 "EBS engages both valves after a live brake request");

  system.stop();
}

void test_ebs_finished_mission(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  send_nominal_ebs_frames(system);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  system.send_frame(ebs_command(false, true));

  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                 "EBS opens ASNode when the mission is finished");

  context.expect(system.output(OutputId::EbsValve1Engaged) == 1U &&
                     system.output(OutputId::EbsValve2Engaged) == 1U,
                 "EBS engages both valves when the mission is finished");

  system.stop();
}

void test_ebs_status_frame(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  send_nominal_ebs_frames(system);

  Frame frame{};
  const bool received = wait_for_matching_frame(
      system, raceup::sil::can_id::EbsStatus,
      [](const Frame& candidate) {
        return candidate.dlc == 5U &&
               candidate.data[0] == 1U &&
               candidate.data[1] == 0U &&
               candidate.data[2] == 0U &&
               candidate.data[3] == 0U &&
               candidate.data[4] == 0U;
      },
      frame);

  context.expect(received,
                 "EBS publishes the expected nominal status frame");

  if (received) {
    context.expect(frame.dlc == 5U,
                   "EBS status frame has the expected DLC");
    context.expect(frame.data[0] == 1U,
                   "EBS status reports ASNode closed");
    context.expect(frame.data[1] == 0U && frame.data[2] == 0U,
                   "EBS status reports both valves released");
    context.expect(frame.data[3] == 0U,
                   "EBS status reports no fault");
    context.expect(frame.data[4] == 0U,
                   "EBS status reports the expected vehicle speed");
  }

  system.stop();
}

}  // namespace

int main() {
  TestContext context;

  test_lifecycle(context);
  test_receive_frame_timeout(context);

  test_smu_nominal(context);
  test_smu_fault(context);
  test_smu_plausibility_fault(context);
  test_smu_periodic_frames(context);

  test_ebs_missing_critical_inputs(context);
  test_ebs_nominal_and_stale_input(context);
  test_ebs_periodically_refreshed_inputs(context);
  test_ebs_requires_ready_to_drive(context);
  
  test_ebs_invalid_pressure(context);
  test_ebs_watchdog_fault(context);
  test_ebs_asms_fault(context);
  test_ebs_valve_power_fault(context);
  test_ebs_brake_request(context);
  test_ebs_finished_mission(context);
  test_ebs_status_frame(context);

  if (context.failures != 0) {
    std::printf("%d test assertion(s) failed\n", context.failures);
    return 1;
  }

  std::printf("all SIL tests passed\n");
  return 0;
}
