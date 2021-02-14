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
#include "push_and_swap.hpp"
#include "revisit_pp.hpp"
#include "solver.hpp"
#include "whca.hpp"

class IR : public Solver
{
public:
  static const std::string SOLVER_NAME;

protected:
  // init-solver
  enum struct INIT_SOLVER_TYPE {
    PIBT,
    HCA,
    WHCA,
    ECBS,
    PIBT_COMPLETE,
    RevisitPP,
    PushAndSwap
  };
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
  static constexpr INIT_SOLVER_TYPE DEFAULT_INIT_SOLVER = IR::INIT_SOLVER_TYPE::PIBT_COMPLETE;
  static constexpr OPTIMAL_SOLVER_TYPE DEFAULT_REFINE_SOLVER = IR::OPTIMAL_SOLVER_TYPE::ICBS;
  static constexpr int DEFAULT_MAX_ITERATION = 100;
  static constexpr int DEFAULT_TIMEOUT_REFINEMENT = 3000;
  static constexpr int DEFAULT_SAMPLING_NUM = 10;

  void run();
  void updateSolution(const Plan& plan);
  Plan getInitialPlan();
  // success?, solution
  std::tuple<bool, Plan> getOptimalPlan(Problem* _P, const Plan& current_plan,
                                        const std::vector<int>& sample);
  virtual void refinePlan();
  void printProcessInfo();

  // ----------------------------
  // define refinement rules
  void updateByRandom();
  void updatePlanFocusOneAgent(std::function<void(const int, Plan&, IR*)> fn);  // work as macro
  static void updateBySinglePaths(const int i, Plan& plan, IR* const solver);
  static void updateByFixAtGoals(const int i, Plan& plan,  IR* const solver);
  static void updateByFocusGoals(const int i, Plan& plan,  IR* const solver);
  static void updateByBottleneck(const int i, Plan& plan,  IR* const solver);
  static void updateByMDD(const int i, Plan& plan, IR* const solver);

  // ----------------------------
  // utilities for refinement rules
public:
  static std::vector<int> identifyInteractingSetByMDD
  (const int i, const Plan& plan, Solver* const solver,
   bool whole_duration = false, const int time_limit = -1, std::mt19937* MT = nullptr);
  static std::vector<int> identifyAgentsAtGoal
  (const int i, const Plan& plan, const Node* g, const int dist);
  static std::tuple<int, std::vector<int>> identifyBottleneckAgentsWithScore
  (const int i, const Paths& original_paths, Solver* const solver, const int time_limit = -1);

public:
  IR(Problem* _P);
  ~IR();

  int getRefineTimeLimit() const
  {
    return std::min(getRemainedTime(), timeout_refinement);
  }

  void makeLog(const std::string& logfile);
  virtual void setParams(int argc, char* argv[]);
  static void printHelp();

  // for tests
  void setInitialPlan(const Plan& plan);
};

// ---------------------------------
// IR_SINGLE_PATHS
// ---------------------------------
class IR_SINGLE_PATHS : public IR
{
public:
  static const std::string SOLVER_NAME;
private:
  void refinePlan() { updatePlanFocusOneAgent(updateBySinglePaths); }
public:
  IR_SINGLE_PATHS(Problem* _P) : IR(_P) { solver_name = SOLVER_NAME; }
  static void printHelp() { printHelpWithoutOption(SOLVER_NAME); }
};

// ---------------------------------
// IR_FIX_AT_GOALS
// ---------------------------------
class IR_FIX_AT_GOALS : public IR
{
public:
  static const std::string SOLVER_NAME;
private:
  void refinePlan() { updatePlanFocusOneAgent(updateByFixAtGoals); }
public:
  IR_FIX_AT_GOALS(Problem* _P) : IR(_P) { solver_name = SOLVER_NAME; }
  static void printHelp() { printHelpWithoutOption(SOLVER_NAME); }
};

// ---------------------------------
// IR_FOCUS_GOALS
// ---------------------------------
class IR_FOCUS_GOALS : public IR
{
public:
  static const std::string SOLVER_NAME;
private:
  void refinePlan() { updatePlanFocusOneAgent(updateByFocusGoals); }
public:
  IR_FOCUS_GOALS(Problem* _P) : IR(_P) { solver_name = SOLVER_NAME; }
  static void printHelp() { printHelpWithoutOption(SOLVER_NAME); }
};

// ---------------------------------
// IR_MDD
// ---------------------------------
class IR_MDD : public IR
{
public:
  static const std::string SOLVER_NAME;
private:
  void refinePlan() { updatePlanFocusOneAgent(updateByMDD); }
public:
  IR_MDD(Problem* _P) : IR(_P) { solver_name = SOLVER_NAME; }
  static void printHelp() { printHelpWithoutOption(SOLVER_NAME); }
};

// ---------------------------------
// IR_BOTTLENECK
// ---------------------------------
class IR_BOTTLENECK : public IR
{
public:
  static const std::string SOLVER_NAME;
protected:
  void refinePlan() { updatePlanFocusOneAgent(updateByBottleneck); }
public:
  IR_BOTTLENECK(Problem* _P) : IR(_P) { solver_name = SOLVER_NAME; }
  static void printHelp() { printHelpWithoutOption(SOLVER_NAME); }
};

// ---------------------------------
// IR_HYBRID
// ---------------------------------
class IR_HYBRID : public IR
{
public:
  static const std::string SOLVER_NAME;
private:
  void refinePlan();
public:
  IR_HYBRID(Problem* _P) : IR(_P) { solver_name = SOLVER_NAME; }
  static void printHelp() { printHelpWithoutOption(SOLVER_NAME); }
};
