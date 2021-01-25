#pragma once
#include "ir.hpp"

class IR_BOTTLENECK : public IR
{
public:
  static const std::string SOLVER_NAME;

protected:
  void refinePlan();

public:
  IR_BOTTLENECK(Problem* _P);

  static void printHelp();
};
