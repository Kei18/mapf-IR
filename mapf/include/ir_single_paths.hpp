#pragma once
#include "ir.hpp"

class IR_SINGLE_PATHS : public IR
{
public:
  static const std::string SOLVER_NAME;

private:
  void refinePlan();

public:
  IR_SINGLE_PATHS(Problem* _P);

  static void printHelp();
};
