#pragma once
#include "ir_focus_one_agent.hpp"

class IR_FocusGoals : public IR_FOCUS_ONE_AGENT
{
public:
  static const std::string SOLVER_NAME;

private:
  void updatePlanFocusOneAgent(const int i, Plan& plan);

public:
  IR_FocusGoals(Problem* _P);

  static void printHelp();
};
