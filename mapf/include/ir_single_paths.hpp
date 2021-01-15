#pragma once
#include "ir.hpp"

class IR_SinglePaths : public IR
{
public:
  static const std::string SOLVER_NAME;

private:
  void refinePlan();

public:
  IR_SinglePaths(Problem* _P);

  static void printHelp();
};
