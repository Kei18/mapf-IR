#pragma once
#include "ir.hpp"

class IR_Bottleneck : public IR
{
public:
  static const std::string SOLVER_NAME;

protected:
  void refinePlan();

public:
  IR_Bottleneck(Problem* _P);

  static void printHelp();
};
