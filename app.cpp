#include <getopt.h>

#include <cbs.hpp>
#include <default_params.hpp>
#include <ecbs.hpp>
#include <hca.hpp>
#include <icbs.hpp>
#include <iostream>
#include <ir.hpp>
#include <ir_single_paths.hpp>
#include <ir_fix_at_goals.hpp>
#include <ir_focus_goals.hpp>
#include <ir_mdd.hpp>
#include <ir_bottleneck.hpp>
#include <ir_hybrid.hpp>
#include <pibt.hpp>
#include <pibt_complete.hpp>
#include <problem.hpp>
#include <random>
#include <util.hpp>
#include <vector>
#include <whca.hpp>
#include <revisit_pp.hpp>

void printHelp();
std::unique_ptr<Solver> getSolver
(const std::string solver_name, Problem* P, bool verbose, int argc, char* argv[]);

int main(int argc, char* argv[])
{
  std::string instance_file = "";
  std::string output_file = DEFAULT_OUTPUT_FILE;
  std::string solver_name;
  bool verbose = false;
  char* argv_copy[argc + 1];
  for (int i = 0; i < argc; ++i) argv_copy[i] = argv[i];

  struct option longopts[] = {
      {"instance", required_argument, 0, 'i'},
      {"output", required_argument, 0, 'o'},
      {"solver", required_argument, 0, 's'},
      {"verbose", no_argument, 0, 'v'},
      {"help", no_argument, 0, 'h'},
      {"make-scen", no_argument, 0, 'P'},
      {0, 0, 0, 0},
  };
  bool make_scen = false;

  // command line args
  int opt, longindex;
  opterr = 0;  // ignore getopt error
  while ((opt = getopt_long(argc, argv, "i:o:s:vhP", longopts, &longindex)) !=
         -1) {
    switch (opt) {
      case 'i':
        instance_file = std::string(optarg);
        break;
      case 'o':
        output_file = std::string(optarg);
        break;
      case 's':
        solver_name = std::string(optarg);
        break;
      case 'v':
        verbose = true;
        break;
      case 'h':
        printHelp();
        return 0;
      case 'P':
        make_scen = true;
        break;
      default:
        break;
    }
  }

  if (instance_file.length() == 0) {
    std::cout << "specify instance file using -i [INSTANCE-FILE], e.g.,"
              << std::endl;
    std::cout << "> ./app -i ../instance/sample.txt" << std::endl;
    return 0;
  }

  // set problem
  Problem P = Problem(instance_file);

  // create scenario
  if (make_scen) {
    P.makeScenFile(output_file);
    return 0;
  }

  // solve
  auto solver = getSolver(solver_name, &P, verbose, argc, argv_copy);
  solver->solve();
  if (solver->succeed() && !solver->getSolution().validate(&P)) {
    halt("invalid results");
  }
  solver->printResult();

  // output result
  solver->makeLog(output_file);
  if (verbose) {
    std::cout << "save result as " << output_file << std::endl;
  }

  return 0;
}

std::unique_ptr<Solver> getSolver
(const std::string solver_name, Problem* P, bool verbose, int argc, char* argv[])
{
  std::unique_ptr<Solver> solver;
  if (solver_name == "PIBT") {
    solver = std::make_unique<PIBT>(P);
  } else if (solver_name == "HCA") {
    solver = std::make_unique<HCA>(P);
  } else if (solver_name == "WHCA") {
    solver = std::make_unique<WHCA>(P);
  } else if (solver_name == "CBS") {
    solver = std::make_unique<CBS>(P);
  } else if (solver_name == "ICBS") {
    solver = std::make_unique<ICBS>(P);
  } else if (solver_name == "PIBT_COMPLETE") {
    solver = std::make_unique<PIBT_COMPLETE>(P);
  } else if (solver_name == "ECBS") {
    solver = std::make_unique<ECBS>(P);
  } else if (solver_name == "RevisitPP") {
    solver = std::make_unique<RevisitPP>(P);
  } else if (solver_name == "IR") {
    solver = std::make_unique<IR>(P);
  } else if (solver_name == "IR_SINGLE_PATHS") {
    solver = std::make_unique<IR_SinglePaths>(P);
  } else if (solver_name == "IR_FIX_AT_GOALS") {
    solver = std::make_unique<IR_FixAtGoals>(P);
  } else if (solver_name == "IR_FOCUS_GOALS") {
    solver = std::make_unique<IR_FocusGoals>(P);
  } else if (solver_name == "IR_MDD") {
    solver = std::make_unique<IR_MDD>(P);
  } else if (solver_name == "IR_BOTTLENECK") {
    solver = std::make_unique<IR_Bottleneck>(P);
  } else if (solver_name == "IR_HYBRID") {
    solver = std::make_unique<IR_HYBRID>(P);
  } else {
    warn("unknown solver name, " + solver_name + ", continue by PIBT");
    solver = std::make_unique<PIBT>(P);
  }
  solver->setParams(argc, argv);
  solver->setVerbose(verbose);
  return solver;
}

void printHelp()
{
  std::cout << "\nUsage: ./app [OPTIONS] [SOLVER-OPTIONS]\n"
            << "\n**instance file is necessary to run MAPF simulator**\n\n"
            << "  -i --instance [FILE_PATH]     instance file path\n"
            << "  -o --output [FILE_PATH]       ouptut file path\n"
            << "  -v --verbose                  print additional info\n"
            << "  -h --help                     help\n"
            << "  -s --solver [SOLVER_NAME]     solver, choose from the below\n"
            << "  -P --make-scen                make scenario file using "
               "random starts/goals"
            << "\n\nSolver Options:" << std::endl;
  // each solver
  PIBT::printHelp();
  HCA::printHelp();
  WHCA::printHelp();
  RevisitPP::printHelp();
  CBS::printHelp();
  ECBS::printHelp();
  ICBS::printHelp();
  PIBT_COMPLETE::printHelp();
  IR::printHelp();
  IR_SinglePaths::printHelp();
  IR_FixAtGoals::printHelp();
  IR_FocusGoals::printHelp();
  IR_MDD::printHelp();
  IR_Bottleneck::printHelp();
  IR_HYBRID::printHelp();
}
