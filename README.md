# Consens

A C++ library for distributed multi-agent task allocation using the Consensus-Based Bundle Algorithm (CBBA).

## Overview

Consens enables multiple agents to autonomously assign tasks among themselves without a central coordinator. Each agent runs the same algorithm independently and reaches consensus through message passing.

**Key Features:**
- Fully distributed - no central coordinator needed
- Algorithm-agnostic design - plug in custom consensus algorithms
- Spatial indexing with R-trees for efficient task queries
- Works with any simulator via callbacks

## Quick Start

```cpp
#include <consens/consens.hpp>

// Configure agent
consens::Consens::Config config;
config.agent_id = "robot_1";
config.max_bundle_size = 20;

// Setup communication (integrate with your simulator)
config.send_message = [](const std::vector<uint8_t>& data) {
    // Send to neighbors
};
config.receive_messages = []() {
    return received_messages;  // Get messages from neighbors
};

// Create agent
consens::Consens agent(config);

// Add tasks
agent.add_task("task_1", consens::Point(10.0, 20.0), 30.0);
agent.add_task("task_2", consens::Point(50.0, 60.0), 45.0);

// Main loop
while (!agent.has_converged()) {
    agent.update_pose(x, y, heading);
    agent.tick(0.1);  // Run one iteration
    
    auto next = agent.get_next_task();
    if (next) {
        // Execute next task
    }
}
```

## Installation

### CMake
```cmake
FetchContent_Declare(
  consens
  GIT_REPOSITORY https://github.com/yourusername/consens.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(consens)
target_link_libraries(your_target PRIVATE consens::consens)
```

**Dependencies:** Boost Geometry, spdlog (optional)

## How CBBA Works

Each agent independently executes three phases:

1. **Build Bundle** - Select best nearby tasks using spatial queries
2. **Communicate** - Share assignments with neighbors
3. **Resolve Conflicts** - Update assignments based on neighbor bids

The algorithm repeats until all agents converge on conflict-free task assignments.

## Configuration

```cpp
consens::cbba::CBBAConfig config;
config.max_bundle_size = 50;              // Max tasks per agent
config.spatial_query_radius = 150.0f;     // Search radius (meters)
config.metric = consens::cbba::Metric::TDR;  // Scoring method
config.bundle_mode = consens::cbba::BundleMode::FULLBUNDLE;
```

**Scoring Metrics:**
- `RPT` - Minimize total time
- `TDR` - Value earlier tasks more (time-discounted)

**Bundle Modes:**
- `ADD` - Add one task per iteration
- `FULLBUNDLE` - Fill entire bundle at once

## Custom Algorithms

Implement the `Algorithm` interface to use your own consensus method:

```cpp
class MyAlgorithm : public consens::Algorithm {
public:
    void tick(float dt) override {
        // Your logic here
    }
};

auto algo = std::make_unique<MyAlgorithm>();
consens::Consens agent(config, std::move(algo));
```

## Testing

```bash
make build
make test  # 43 tests, 209 assertions
```

## Examples

See `examples/` directory:
- `simple_test.cpp` - Basic usage
- `cbba_test.cpp` - CBBA configuration
- `spatial_index_test.cpp` - Spatial queries

## Acknowledgments

See [ACKNOWLEDGMENT](ACKNOWLEDGMENT) file for research papers and references.

## License

MIT License - see [LICENSE](LICENSE) file.
