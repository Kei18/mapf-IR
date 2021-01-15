/*
 * Implementation of Iterative Refinement (IR)
 */

#pragma once
#include "cbs_refine.hpp"
#include "ecbs.hpp"
#include "hca.hpp"
#include "icbs_refine.hpp"
#include "pibt.hpp"
#include "pibt_complete.hpp"
#include "whca.hpp"
#include "revisit_pp.hpp"
#include "lib_ir.hpp"
#include "solver.hpp"

class IR : public Solver
{
public:
  static const std::string SOLVER_NAME;

protected:
  // init-solver
  enum struct INIT_SOLVER_TYPE { PIBT, HCA, WHCA, ECBS, PIBT_COMPLETE, RevisitPP };
  INIT_SOLVER_TYPE init_solver;
  std::vector<std::string> option_init_solver;

  // refine-solver
  enum struct OPTIMAL_SOLVER_TYPE { CBS, CBS_NORMAL, ICBS, ICBS_NORMAL };
  OPTIMAL_SOLVER_TYPE refine_solver;
  std::vector<std::string> option_optimal_solver;

  // max iteration
  int current_iteration;
  int max_iteration;

  // for log
  std::string output_file;
  bool make_log_every_itr;  // true -> create log for every iteration
  std::vector<std::tuple<int, int, int>> HIST;  // elapsed timestep, soc, makespan

  // early stop
  int timeout_refinement;

  // print underlying solver info
  bool verbose_underlying_solver;

  //  last sum of cost
  int last_soc;
  int last_makespan;

  //  random sampling
  int sampling_num;

  // default params
  static const INIT_SOLVER_TYPE DEFAULT_INIT_SOLVER;
  static const OPTIMAL_SOLVER_TYPE DEFAULT_REFINE_SOLVER;
  static const int DEFAULT_MAX_ITERATION;
  static const int DEFAULT_TIMEOUT_REFINEMENT;
  static const int DEFAULT_SAMPLING_NUM;

  void run();
  void updateSolution(const Plan& plan);
  Plan getInitialPlan();
  // success?, solution
  std::tuple<bool, Plan> getOptimalPlan(Problem* _P, const Plan& current_plan,
                                        const std::vector<int>& sample);

  virtual void refinePlan();

  void printProcessInfo();
  int getRefineTimeLimit() const { return std::min(getRemainedTime(), timeout_refinement); }

public:
  IR(Problem* _P);
  ~IR();

  void makeLog(const std::string& logfile);
  virtual void setParams(int argc, char* argv[]);
  static void printHelp();
};
