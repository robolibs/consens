
<img align="right" width="26%" src="./misc/logo.png">

Consens
===

A C++ library for distributed multi-agent task allocation using the Consensus-Based Bundle Algorithm (CBBA).

---

## Overview

Consens provides a clean, algorithm-agnostic framework for multi-agent consensus and task allocation. The library is designed to be:

- **Fully Distributed**: No central coordinator - each agent runs independently
- **Algorithm-Agnostic**: Easy to plug in different consensus algorithms (CBBA, auction-based, greedy, etc.)
- **Simulator-Independent**: Works with any simulator through callback-based communication
- **High Performance**: Optimized spatial indexing and efficient conflict resolution

## Current Implementation: CBBA

The primary implementation is the **Consensus-Based Bundle Algorithm (CBBA)**, a distributed auction algorithm for multi-agent task allocation with temporal/spatial constraints.

### How CBBA Works

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    DISTRIBUTED CBBA LOOP (Per Agent)                     │
└─────────────────────────────────────────────────────────────────────────┘

Each Agent Independently Runs:

1. BUNDLE BUILDING PHASE
   ┌──────────────────────┐
   │  Query Nearby Tasks  │ ← Spatial Index (R-tree)
   │  Compute Scores      │ ← RPT or TDR metric
   │  Select Best Task    │ ← Greedy / Full Bundle mode
   │  Add to Bundle       │
   └──────────────────────┘
                ↓
2. COMMUNICATION PHASE
   ┌──────────────────────┐
   │  Package State       │ ← Bundle, Path, Bids, Winners
   │  Serialize Message   │ ← Binary format
   │  Send to Neighbors   │ ← User callback
   └──────────────────────┘
                ↓
3. CONSENSUS PHASE
   ┌──────────────────────┐
   │  Receive Messages    │ ← User callback
   │  Compare Bids        │ ← Timestamp-based
   │  Apply Rules:        │
   │    • UPDATE          │ ← Accept better bid
   │    • RESET           │ ← Lost task, drop subsequent
   │    • LEAVE           │ ← Keep current assignment
   └──────────────────────┘
                ↓
        Repeat Until Convergence
```

### Key Features

- 🤖 **Distributed Consensus**: Agents converge to conflict-free task assignments
- 🎯 **Spatial Awareness**: R-tree spatial indexing for efficient task queries
- 📊 **Multiple Metrics**: RPT (Reward Per Time) and TDR (Time-Discounted Reward)
- 🔄 **Conflict Resolution**: Three-rule consensus (UPDATE/RESET/LEAVE)
- 📡 **Binary Messaging**: Efficient serialization for network transmission
- 🏗️ **Modular Design**: Clean separation of components (scoring, bundling, consensus)
- ⚡ **Real-Time Ready**: Designed for online replanning in dynamic environments

---

## Architecture

### Strategy Pattern Design

```
Consens (Public API - Algorithm Agnostic)
  │
  └─> std::unique_ptr<Algorithm>  (Strategy Pattern)
       │
       ├─> CBBAAlgorithm (Default Implementation)
       │    ├─> CBBAAgent (Agent State)
       │    ├─> SpatialIndex (R-tree for tasks)
       │    ├─> BundleBuilder (Task selection)
       │    ├─> TaskScorer (Utility computation)
       │    └─> ConsensusResolver (Conflict resolution)
       │
       └─> YourCustomAlgorithm (Plug in your own!)
```

### Core Components

#### 1. **Consens** (`consens::Consens`)
High-level API that manages agent state and delegates to algorithms.

```cpp
consens::Consens::Config config;
config.agent_id = "robot_1";
config.max_bundle_size = 100;
config.send_message = [](const std::vector<uint8_t>& data) { /* send */ };
config.receive_messages = []() { return messages; };

consens::Consens agent(config);  // Uses CBBA by default
```

#### 2. **Algorithm** (`consens::Algorithm`)
Abstract interface for all consensus algorithms - makes it easy to implement custom algorithms.

#### 3. **CBBA Algorithm** (`consens::cbba::CBBAAlgorithm`)
Complete CBBA implementation with:
- Bundle building (greedy or full bundle modes)
- Binary message serialization
- Three-rule consensus resolution
- Spatial task filtering

#### 4. **Spatial Index** (`consens::cbba::SpatialIndex`)
R-tree based spatial indexing for O(log n) task queries:
- Nearest neighbor search
- Radius queries
- Bounding box queries

#### 5. **Task Scorer** (`consens::cbba::TaskScorer`)
Computes task utilities using different metrics:
- **RPT**: Reward Per Time (minimize total time)
- **TDR**: Time-Discounted Reward (earlier tasks valued more)

#### 6. **Bundle Builder** (`consens::cbba::BundleBuilder`)
Greedy task selection with two modes:
- **ADD**: Add one task per tick
- **FULLBUNDLE**: Fill entire bundle capacity

#### 7. **Consensus Resolver** (`consens::cbba::ConsensusResolver`)
Implements the three CBBA consensus rules:
- **UPDATE**: Accept neighbor's better/newer information
- **RESET**: Lost task - remove it and all subsequent tasks
- **LEAVE**: Keep current assignment (we have better info)

---

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

### Dependencies

- **Boost Geometry**: Spatial operations and R-tree indexing
- **spdlog**: Logging (optional)
- **doctest**: Testing framework (tests only)

---

## Quick Start

### Basic Usage

```cpp
#include <consens/consens.hpp>

// Setup agent configuration
consens::Consens::Config config;
config.agent_id = "robot_1";
config.max_bundle_size = 20;
config.spatial_query_radius = 100.0f;  // meters

// Communication callbacks (provided by your simulator)
config.send_message = [](const std::vector<uint8_t>& data) {
    // Send to neighbors via your network/simulator
};

config.receive_messages = []() -> std::vector<std::vector<uint8_t>> {
    // Receive from neighbors
    return received_messages;
};

// Create agent (uses CBBA by default)
consens::Consens agent(config);

// Update agent state
agent.update_pose(x, y, heading);
agent.update_velocity(2.0);  // m/s

// Add tasks
agent.add_task("task_1", consens::Point(10.0, 20.0), 30.0);  // position, duration
agent.add_task("task_2", consens::Point(50.0, 60.0), 45.0);

// Main loop - run algorithm
while (!agent.has_converged()) {
    agent.tick(0.1);  // dt = 100ms
    
    // Query results
    auto path = agent.get_path();  // Ordered execution sequence
    auto next = agent.get_next_task();  // Next task to execute
    
    if (next) {
        std::cout << "Executing: " << *next << std::endl;
    }
}
```

### Custom Algorithm

Implement your own consensus algorithm:

```cpp
#include <consens/algorithm.hpp>

class MyCustomAlgorithm : public consens::Algorithm {
public:
    void tick(float dt) override {
        // Your algorithm here
    }
    
    // Implement other virtual methods...
};

// Use custom algorithm
auto custom_algo = std::make_unique<MyCustomAlgorithm>();
consens::Consens agent(config, std::move(custom_algo));
```

### Working with Tasks

```cpp
// Point task (single location)
agent.add_task("mow_section_1", consens::Point(100, 200), 120.0);

// Geometric task (line segment, e.g., row/swath)
consens::Point head(0, 0);
consens::Point tail(100, 0);
agent.add_task("row_1", head, tail, 60.0);

// Query task details
auto task = agent.get_task("row_1");
if (task) {
    std::cout << "Task position: " << task->get_position().x << std::endl;
    std::cout << "Task duration: " << task->get_duration() << std::endl;
}

// Remove completed tasks
agent.mark_task_completed("row_1");
```

---

## CBBA Configuration

### Custom CBBA Settings

```cpp
#include <consens/cbba/cbba_algorithm.hpp>
#include <consens/cbba/types.hpp>

// Configure CBBA parameters
consens::cbba::CBBAConfig cbba_config;
cbba_config.max_bundle_size = 50;
cbba_config.spatial_query_radius = 150.0f;
cbba_config.metric = consens::cbba::Metric::TDR;  // Time-Discounted Reward
cbba_config.lambda = 0.95;  // Discount factor
cbba_config.bundle_mode = consens::cbba::BundleMode::FULLBUNDLE;

// Create CBBA algorithm with custom config
auto cbba = std::make_unique<consens::cbba::CBBAAlgorithm>(
    "robot_1", cbba_config, send_callback, receive_callback
);

// Use it
consens::Consens agent(config, std::move(cbba));
```

### Bundle Modes

- **ADD**: Add one task per tick (iterative)
- **FULLBUNDLE**: Fill entire bundle capacity in one tick

### Scoring Metrics

- **RPT** (Reward Per Time): Minimize total time
  ```
  Score = -travel_time - task_duration
  ```

- **TDR** (Time-Discounted Reward): Earlier tasks more valuable
  ```
  Score = λ^(time_to_task)
  ```

---

## Message Format

CBBA uses binary serialization for efficient network transmission:

```cpp
struct CBBAMessage {
    AgentID sender_id;
    Timestamp timestamp;
    
    Bundle bundle;              // Tasks claimed
    Path path;                  // Ordered execution
    
    TaskBids winning_bids;      // Best bid for each task
    TaskWinners winners;        // Winner for each task
    AgentTimestamps timestamps; // Multi-hop consensus info
    
    std::vector<uint8_t> serialize() const;
    bool deserialize(const std::vector<uint8_t>& data);
};
```

### Example Message Flow

```cpp
// Agent 1: Create and send message
consens::cbba::CBBAMessage msg = agent1.create_message();
std::vector<uint8_t> data = msg.serialize();
network.send(data);

// Agent 2: Receive and process
std::vector<uint8_t> received = network.receive();
consens::cbba::CBBAMessage msg;
if (msg.deserialize(received)) {
    // Process message in consensus phase
}
```

---

## Integration with Simulators

Consens is designed to integrate with any simulator through callbacks:

### Example: Integration with Custom Simulator

```cpp
class MySimulator {
    std::vector<std::unique_ptr<consens::Consens>> agents_;
    std::map<std::string, std::vector<std::vector<uint8_t>>> message_buffers_;
    
public:
    void setup_agent(const std::string& agent_id) {
        consens::Consens::Config config;
        config.agent_id = agent_id;
        
        // Send callback
        config.send_message = [this, agent_id](const std::vector<uint8_t>& data) {
            // Broadcast to all other agents
            for (auto& [other_id, buffer] : message_buffers_) {
                if (other_id != agent_id) {
                    buffer.push_back(data);
                }
            }
        };
        
        // Receive callback
        config.receive_messages = [this, agent_id]() {
            auto messages = message_buffers_[agent_id];
            message_buffers_[agent_id].clear();
            return messages;
        };
        
        agents_.push_back(std::make_unique<consens::Consens>(config));
        message_buffers_[agent_id] = {};
    }
    
    void tick(float dt) {
        for (auto& agent : agents_) {
            // Update state from simulator
            agent->update_pose(get_pose(agent->get_agent_id()));
            agent->update_velocity(get_velocity(agent->get_agent_id()));
            
            // Run algorithm
            agent->tick(dt);
            
            // Get results
            auto next_task = agent->get_next_task();
            if (next_task) {
                execute_task(agent->get_agent_id(), *next_task);
            }
        }
    }
};
```

---

## Performance Characteristics

| Operation | Time Complexity | Description |
|-----------|----------------|-------------|
| Add Task | O(log n) | Insert into R-tree spatial index |
| Spatial Query | O(log n + k) | Find k nearby tasks |
| Bundle Building | O(m log n) | m = bundle size, n = task count |
| Message Serialize | O(b + w) | b = bundle size, w = winner count |
| Consensus Resolution | O(k × w) | k = neighbors, w = winner count |
| Full Tick | O(m log n + kw) | Complete CBBA iteration |

---

## Testing

Consens includes comprehensive tests:

```bash
# Build and run tests
make build
make test

# Test suites:
# - test_bundle_builder: Task selection logic (8 tests)
# - test_consensus_resolver: Conflict resolution (10 tests)
# - test_messages: Serialization (11 tests)
# - test_scorer: Utility computation (8 tests)
# - test_spatial_index: R-tree queries (6 tests)
```

All 43 tests with 209 assertions currently passing ✅

---

## Examples

See `examples/` directory:
- `simple_test.cpp`: Basic Consens usage
- `cbba_test.cpp`: CBBA-specific features
- `spatial_index_test.cpp`: Spatial queries

---

## Roadmap

- [x] Core CBBA implementation
- [x] Binary message serialization
- [x] Spatial indexing with R-trees
- [x] Consensus resolution
- [x] Strategy pattern for algorithms
- [ ] Dynamic replanning
- [ ] Temporal constraints
- [ ] Heterogeneous agents
- [ ] Additional algorithms (auction, greedy)
- [ ] Visualization tools
- [ ] Python bindings

---

## Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

---

## References

- Choi, H. L., Brunet, L., & How, J. P. (2009). "Consensus-Based Decentralized Auctions for Robust Task Allocation". IEEE Transactions on Robotics.
- Johnson, L., et al. (2017). "Decentralized Task Allocation Using Local Information Consistency Assumptions". AIAA Journal of Aerospace Computing.

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---
