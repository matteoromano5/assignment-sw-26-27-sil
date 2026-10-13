# Thought Process

## Synchronization

I used a single `std::mutex` to protect all simulator-owned shared state:
sensor values, actuator outputs and both frame queues.

Simple accesses use `std::lock_guard`, while `receive_frame()` uses
`std::unique_lock` because the condition variable must temporarily release the
mutex while waiting.

I preferred a single mutex over multiple fine-grained locks because the shared
state is small and the critical sections are short. This keeps the locking
strategy simple and reduces the risk of deadlocks or inconsistent lock ordering.

The `running` and `stop_requested` flags remain atomic because they are simple
cross-thread lifecycle flags and do not need to be protected by the simulator
state mutex.

## Frame routing

I implemented the bus using the two queues already provided by the simulator.

Frames injected through `send_frame()` are appended to the EBS input queue.

Frames emitted by the SMU are copied both to the monitor queue and to the EBS
input queue, while frames emitted by the EBS are added only to the monitor
queue.

Both queues remain bounded to the provided capacity. When full, the oldest
frame is discarded before inserting the new one.

This preserves FIFO behavior while preventing unbounded memory growth.

## Waiting for emitted frames

`receive_frame()` waits on the provided condition variable instead of polling
the queue.

The wait is bounded by the caller-provided timeout, so the test code cannot
block indefinitely.

The condition variable is notified only after a frame has been inserted into
the emitted queue and after releasing the mutex, avoiding unnecessary
contention when the waiting thread wakes up.

## Test extensions

I extended the provided test suite with additional black-box tests covering:

- invalid EBS pneumatic pressure;
- a watchdog fault injected while the SIL is running;
- periodic SMU frame emission;
- `receive_frame()` timeout behavior.

The added tests focus only on observable behavior through the public
`SilSystem` interface rather than depending on private implementation details.

In particular, the watchdog test verifies a live transition from the nominal
state to the safe state after changing a sensor while both ECU threads are
already running.

## CI

The GitHub Actions workflow runs the same CMake workflow used locally on a clean
Ubuntu runner.

This verifies that the project can be configured, built and tested without
depending on local build artifacts or machine-specific setup.