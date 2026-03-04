#pragma once

#include "core/simulation.hpp"

namespace pathlearn {

// 简单仿真：时间步进 + 每步重规划
class SimpleSimulation final : public Simulation {
 public:
  SimpleSimulation() = default;
  ~SimpleSimulation() override = default;

  Status Run(
      const SimulationRequest& request,
      const SimulationConfig& config,
      SimulationResult* result) override;
};

}  // namespace pathlearn
