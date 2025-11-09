#pragma once

#include "../types.hpp"

namespace consens::cbba {

    // Re-export common types
    using consens::AgentID;
    using consens::Point;
    using consens::Pose;
    using consens::Score;
    using consens::TaskID;
    using consens::Timestamp;

    /**
     * Scoring metric for CBBA
     */
    enum class Metric {
        RPT, // Reward Per Time (minimize total time) - default
        TDR  // Time-Discounted Reward (maximize discounted reward)
    };

    /**
     * Bundle building mode
     */
    enum class BundleMode {
        ADD,       // Add one task per iteration (for global consensus)
        FULLBUNDLE // Build full bundle in one iteration (baseline CBBA)
    };

    /**
     * CBBA algorithm configuration
     */
    struct CBBAConfig {
        // Bundle capacity
        size_t max_bundle_size = 10;

        // Spatial filtering
        float spatial_query_radius = 100.0f; // meters

        // Algorithm parameters
        BundleMode bundle_mode = BundleMode::ADD;
        size_t consensus_iterations_per_bundle = 1;
        size_t max_iterations = 1000;

        // Scoring
        Metric metric = Metric::RPT;
        double lambda = 0.95; // Discount factor for TDR metric

        // Convergence
        bool enable_convergence_detection = true;

        // Logging
        bool enable_logging = true;

        // Communication
        size_t max_message_hops = 2; // For multi-hop relay (future)
    };

    /**
     * Minimum score value (for unassigned bids)
     */
    constexpr Score MIN_SCORE = -1e20;

    /**
     * Invalid agent ID (for unassigned tasks)
     */
    const AgentID NO_AGENT = "";

} // namespace consens::cbba
