#include "sil.hpp"

#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <thread>

namespace raceup::sil {
namespace {

SilSystem* active_system = nullptr;

constexpr auto kCyclePeriod = std::chrono::milliseconds(10);
constexpr std::size_t kQueueCapacity = 64U;

constexpr std::size_t sensor_index(const SensorId id) {
  return static_cast<std::size_t>(id);
}

constexpr std::size_t output_index(const OutputId id) {
  return static_cast<std::size_t>(id);
}

}  // namespace

struct SilSystem::Impl {
  struct State {
    // All simulator-owned state is protected by mutex. The two queues have
    // deliberately different meanings: ebs_input is consumed by the EBS;
    // emitted is the monitor queue consumed by receive_frame().
    std::array<std::uint16_t, sensor_index(SensorId::Count)> sensors{};
    std::array<std::uint16_t, output_index(OutputId::Count)> outputs{};
    std::deque<Frame> ebs_input;
    std::deque<Frame> emitted;
    std::mutex mutex;
    std::condition_variable emitted_available;
  } state;

  std::atomic<bool> stop_requested{false};
  std::atomic<bool> running{false};
  std::thread smu_thread;
  std::thread ebs_thread;
};

SilSystem::SilSystem() : impl_(std::make_unique<Impl>()) {}

SilSystem::~SilSystem() { stop(); }

bool SilSystem::start() {
  if (impl_->running.exchange(true)) {
    return true;
  }

  active_system = this;
  impl_->stop_requested.store(false);
  smu::reset();
  ebs::reset();

  impl_->smu_thread = std::thread([this] {
    auto next = std::chrono::steady_clock::now();
    while (!impl_->stop_requested.load()) {
      next += kCyclePeriod;
      smu::run_cycle();
      std::this_thread::sleep_until(next);
    }
  });

  impl_->ebs_thread = std::thread([this] {
    auto next = std::chrono::steady_clock::now();
    while (!impl_->stop_requested.load()) {
      next += kCyclePeriod;
      ebs::run_cycle();
      std::this_thread::sleep_until(next);
    }
  });

  return true;
}

void SilSystem::stop() {
  if (!impl_->running.exchange(false)) {
    return;
  }

  impl_->stop_requested.store(true);
  if (impl_->smu_thread.joinable()) {
    impl_->smu_thread.join();
  }
  if (impl_->ebs_thread.joinable()) {
    impl_->ebs_thread.join();
  }
  if (active_system == this) {
    active_system = nullptr;
  }
}

bool SilSystem::running() const { return impl_->running.load(); }

void SilSystem::set_sensor(const SensorId id, const std::uint16_t value) {
  std::lock_guard<std::mutex> lock(impl_->state.mutex);
  impl_->state.sensors[sensor_index(id)] = value;
}

std::uint16_t SilSystem::output(const OutputId id) const {
  std::lock_guard<std::mutex> lock(impl_->state.mutex);
  return impl_->state.outputs[output_index(id)];
}

void SilSystem::send_frame(const Frame& frame) {
  std::lock_guard<std::mutex> lock(impl_->state.mutex);

  if (impl_->state.ebs_input.size() >= kQueueCapacity) {
    impl_->state.ebs_input.pop_front();
  }

  impl_->state.ebs_input.push_back(frame);
}

bool SilSystem::receive_frame(Frame& frame,
                              const std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(impl_->state.mutex);

  const bool available = impl_->state.emitted_available.wait_for(
      lock,
      timeout,
      [this] {
        return !impl_->state.emitted.empty();
      });

  if (!available) {
    return false;
  }

  frame = impl_->state.emitted.front();
  impl_->state.emitted.pop_front();
  return true;
}

std::uint16_t SilSystem::read_sensor(const SensorId id) const {
  std::lock_guard<std::mutex> lock(impl_->state.mutex);
  return impl_->state.sensors[sensor_index(id)];
}

bool SilSystem::try_read(const Ecu ecu, Frame& frame) {
  if (ecu != Ecu::Ebs) {
    return false;
  }

  std::lock_guard<std::mutex> lock(impl_->state.mutex);

  if (impl_->state.ebs_input.empty()) {
    return false;
  }

  frame = impl_->state.ebs_input.front();
  impl_->state.ebs_input.pop_front();
  return true;
}

void SilSystem::write(const Ecu ecu, const Frame& frame) {
  {
  std::lock_guard<std::mutex> lock(impl_->state.mutex);

    if (impl_->state.emitted.size() >= kQueueCapacity) {
      impl_->state.emitted.pop_front();
    }
    impl_->state.emitted.push_back(frame);

    if (ecu == Ecu::Smu) {
      if (impl_->state.ebs_input.size() >= kQueueCapacity) {
        impl_->state.ebs_input.pop_front();
      }
      impl_->state.ebs_input.push_back(frame);
    }
  }

  impl_->state.emitted_available.notify_one();
}

void SilSystem::write_output(const Ecu /*ecu*/, const OutputId id,
                             const std::uint16_t value) {
  std::lock_guard<std::mutex> lock(impl_->state.mutex);
  impl_->state.outputs[output_index(id)] = value;
}

namespace ecu_io {

std::uint16_t read_sensor(const SensorId id) {
  return active_system == nullptr ? 0U : active_system->read_sensor(id);
}

bool try_read(const Ecu ecu, Frame& frame) {
  return active_system != nullptr && active_system->try_read(ecu, frame);
}

void write(const Ecu ecu, const Frame& frame) {
  if (active_system != nullptr) {
    active_system->write(ecu, frame);
  }
}

void write_output(const Ecu ecu, const OutputId id, const std::uint16_t value) {
  if (active_system != nullptr) {
    active_system->write_output(ecu, id, value);
  }
}

}  // namespace ecu_io

}  // namespace raceup::sil
