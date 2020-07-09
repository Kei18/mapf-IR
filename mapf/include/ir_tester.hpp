#pragma once
#include "ir_paths.hpp"

class IR_TESTER : public IR_PATHS {
protected:
  Plan refinePlan(const Config& config_s,
                  const Config& config_g,
                  const Plan& current_plan);

public:
  IR_TESTER(Problem* _P);
  ~IR_TESTER();
};
