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

## Test approach

I extended the provided test suite with additional black-box tests covering
the safety and infrastructure requirements that were not already explicitly
covered.

The additional tests cover:

- invalid EBS pneumatic pressure and the resulting safe state;
- watchdog failure injected while the SIL is already running;
- ASMS becoming inactive while the SIL is running;
- loss of valve power while the SIL is running;
- missing critical EBS input frames;
- periodically refreshed EBS inputs remaining valid beyond the stale timeout;
- the finished-mission condition causing the EBS to enter the safe state;
- APP plausibility disagreement in the SMU;
- periodic `SMU_SENSE` frame emission;
- correct contents of the `EBS_STATUS` frame;
- timeout behavior of `receive_frame()`.

The tests interact only through the public `SilSystem` interface and verify
observable behavior rather than private ECU state.

Runtime fault tests first establish a nominal condition and then modify a
sensor value or inject a new frame while both ECU threads are already running.
This allows the tests to verify the resulting state transition instead of only
checking static initial conditions.

The test file is organized by responsibility: common helpers and test
utilities first, followed by lifecycle/infrastructure tests, SMU tests, EBS
communication tests, EBS safety tests, and finally frame-output tests.

This organization was chosen to keep the suite easy to navigate and to make
the relationship between each test and the corresponding system requirement
clear. The tests were kept in a single source file, matching the small scope of the
assignment. They are grouped logically instead of being split into multiple
files, avoiding unnecessary test infrastructure while keeping the suite
readable.

## CI

The GitHub Actions workflow runs the same CMake workflow used locally on a clean
Ubuntu runner.

This verifies that the project can be configured, built and tested without
depending on local build artifacts or machine-specific setup.