#pragma once
#include "ir.hpp"

class IR_MDD : public IR
{
public:
  static const std::string SOLVER_NAME;

private:
  void refinePlan();

public:
  IR_MDD(Problem* _P);

  static void printHelp();
};
