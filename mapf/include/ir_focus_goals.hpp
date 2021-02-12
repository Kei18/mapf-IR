#pragma once
#include "ir.hpp"

class IR_FOCUS_GOALS : public IR
{
public:
  static const std::string SOLVER_NAME;

private:
  void refinePlan();

public:
  IR_FOCUS_GOALS(Problem* _P);

  static void printHelp();
};
