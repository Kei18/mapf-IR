#pragma once
#include "ir.hpp"

class IR_FocusGoals : public IR
{
public:
  static const std::string SOLVER_NAME;

private:
  void refinePlan();

public:
  IR_FocusGoals(Problem* _P);

  static void printHelp();
};
