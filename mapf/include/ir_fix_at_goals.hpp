#pragma once
#include "ir.hpp"

class IR_FixAtGoals : public IR
{
public:
  static const std::string SOLVER_NAME;

private:
  void refinePlan();

public:
  IR_FixAtGoals(Problem* _P);

  static void printHelp();
};
