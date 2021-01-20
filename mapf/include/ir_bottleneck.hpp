#pragma once
#include "ir_focus_one_agent.hpp"

class IR_Bottleneck : public IR_FOCUS_ONE_AGENT
{
public:
  static const std::string SOLVER_NAME;

protected:
  void updatePlanFocusOneAgent(const int i, Plan& plan);

public:
  IR_Bottleneck(Problem* _P);

  static void printHelp();
};
