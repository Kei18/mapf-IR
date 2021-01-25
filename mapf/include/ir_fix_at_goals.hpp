#pragma once
#include "ir.hpp"

class IR_FIX_AT_GOALS : public IR
{
public:
  static const std::string SOLVER_NAME;

private:
  void refinePlan();

public:
  IR_FIX_AT_GOALS(Problem* _P);

  static void printHelp();
};
