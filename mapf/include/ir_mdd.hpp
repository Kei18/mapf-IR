#pragma once
#include "ir.hpp"

class IR_MDD : public IR
{
public:
  static const std::string SOLVER_NAME;

private:
  void refinePlan();

  bool simple_refine;

public:
  IR_MDD(Problem* _P);

  static void printHelp();
  void setParams(int argc, char* argv[]);
};
