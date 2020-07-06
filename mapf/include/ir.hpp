/*
 * Implementation of Iterative Refinement (IR)
 */

#pragma once
#include "solver.hpp"
#include "../include/pibt.hpp"
#include "../include/hca.hpp"
#include "../include/whca.hpp"
#include "../include/cbs_refine.hpp"
#include "../include/ecbs.hpp"
#include "../include/icbs_refine.hpp"

using Ints = std::vector<int>;
using Strings = std::vector<std::string>;


class IR : public Solver {
public:
  static const std::string SOLVER_NAME;

protected:
  enum struct INIT_SOLVER_TYPE
    { PIBT, HCA, WHCA, ECBS };
  INIT_SOLVER_TYPE init_solver;
  Strings option_init_solver;

  enum struct OPTIMAL_SOLVER_TYPE
    { CBS, CBS_NORMAL, ICBS, ICBS_NORMAL };
  OPTIMAL_SOLVER_TYPE refine_solver;
  Strings option_optimal_solver;

  // for log
  std::string output_file;
  bool make_log_every_itr;

  // elapsed timestep, plan
  std::vector<std::tuple<int, Plan>> HIST;

  // early stop
  int timeout_refinement;

  // for debug
  bool verbose_underlying_solver;

  // default params
  static const INIT_SOLVER_TYPE DEFAULT_INIT_SOLVER;
  static const OPTIMAL_SOLVER_TYPE DEFAULT_REFINE_SOLVER;

  void run();
  Plan getInitialPlan();
  // success?, solution
  std::tuple<bool, Plan> getOptimalPlan(Problem* _P,
                                        const Plan& current_plan,
                                        const Ints& sample);

  // should be defined
  virtual bool stopRefinement() { return false; }
  virtual Plan refinePlan(const Config& config_s,
                          const Config& config_g,
                          const Plan& current_plan) { return current_plan; }
  virtual Ints sampling(const Plan& current_plan);

public:
  IR(Problem* _P);
  ~IR();

  void makeLog(const std::string& logfile);

  virtual void setParams(int argc, char *argv[]);
  static void printHelp();
};



static void setSolverOption(Solver* solver, const Strings& option)
{
  if (option.empty()) return;
  int argc = option.size() + 1;
  char *argv[argc+1];
  for (int i = 0; i < argc; ++i) {
    char *tmp = const_cast<char*>(option[i].c_str());
    argv[i+1] = tmp;
  }
  solver->setParams(argc, argv);
}
