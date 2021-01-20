#pragma once
#include "ir.hpp"

class IR_FOCUS_ONE_AGENT : public IR
{
protected:
  virtual void refinePlan();
  virtual void updatePlanFocusOneAgent(const int i, Plan& plan) {}

public:
  IR_FOCUS_ONE_AGENT(Problem* _P);
};
