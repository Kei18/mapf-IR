/*
 * Implementation of Iterative Refinement (IR)
 */

#pragma once
#include "solver.hpp"

class IR : public Solver {
public:
  static const std::string SOLVER_NAME;

private:
  std::string output_file;

  // used in solvableDirectly
  int threshold_makespan;

  // used in stopRefinement
  int threshold_soc_diff;
  int threshold_nondiff_refine;

  enum struct REFINEMENT_TYPE
    { SIMPLE, QUADRATIC, NUM_ITMES };
  REFINEMENT_TYPE refinement_type;

  enum struct INIT_SOLVER_TYPE
    { PIBT, HCA, WHCA, NUM_ITMES };
  INIT_SOLVER_TYPE init_solver;
  std::vector<std::string> option_init_solver;

  enum struct REFINE_SOLVER_TYPE
    { CBS, CBS_NORMAL, NUM_ITMES };
  REFINE_SOLVER_TYPE refine_solver;
  std::vector<std::string> option_refine_solver;

  // default params
  static const int DEFAULT_THRESHOLD_MAKESPAN;
  static const int DEFAULT_THRESHOLD_SOC_DIFF;
  static const int DEFAULT_THRESHOLD_NONDIFF_REFINE;
  static const REFINEMENT_TYPE DEFAULT_REFINEMENT_TYPE;
  static const INIT_SOLVER_TYPE DEFAULT_INIT_SOLVER;
  static const REFINE_SOLVER_TYPE DEFAULT_REFINE_SOLVER;

  void iterativeRefinement();
  Plan getInitialPlan();
  bool stopRefinement(const Plan& new_plan, const Plans& hist);
  Plan refinePlan(const Config& config_s,
                  const Config& config_g,
                  const Plan& old_plan);
  bool solvableDirectly(const Config& config_s,
                        const Config& config_g,
                        const Plan& old_plan);
  Plan MAPFSolver(const Config& config_s,
                  const Config& config_g,
                  const Plan& old_plan);
  std::tuple<int, Config> subgoalConfig(Plan plan);
  Plan simpleRefine(const Config& config_s,
                    const Config& config_g,
                    const Plan& old_plan);
  Plan quadraticRefine(const Config& config_s,
                       const Config& config_g,
                       const Plan& old_plan);

public:
  IR(Problem* _P);
  ~IR() {};

  void solve();

  void setParams(int argc, char *argv[]);
  static void printHelp();
};
