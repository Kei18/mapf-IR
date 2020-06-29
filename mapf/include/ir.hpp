/*
 * Implementation of Iterative Refinement (IR)
 */

#pragma once
#include "solver.hpp"

class IR : public Solver {
public:
  static const std::string SOLVER_NAME;

protected:
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
    { PIBT, HCA, WHCA, ECBS, NUM_ITMES };
  INIT_SOLVER_TYPE init_solver;
  std::vector<std::string> option_init_solver;

  enum struct REFINE_SOLVER_TYPE
    { CBS, CBS_NORMAL, ICBS, ICBS_NORMAL, NUM_ITMES };
  REFINE_SOLVER_TYPE refine_solver;
  std::vector<std::string> option_refine_solver;

  // default params
  static const int DEFAULT_THRESHOLD_MAKESPAN;
  static const int DEFAULT_THRESHOLD_SOC_DIFF;
  static const int DEFAULT_THRESHOLD_NONDIFF_REFINE;
  static const REFINEMENT_TYPE DEFAULT_REFINEMENT_TYPE;
  static const INIT_SOLVER_TYPE DEFAULT_INIT_SOLVER;
  static const REFINE_SOLVER_TYPE DEFAULT_REFINE_SOLVER;

  // early stop
  int timeout_refinement;

  // sampling rate
  float sampling_rate;

  // cache
  bool cache_on;
  std::unordered_map<std::string, Plan> PLAN_TABLE;

  // verbose for underlying solver
  bool verbose_underlying_solver;

  virtual void iterativeRefinement();
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
  void registerTable(const Plan& plan);
  Plan shrink(const Plan& plan);
  static std::string getPlanTableKey(const Config& config_s,
                                     const Config& config_g);
  static std::string getPlanTableKey(const std::string& config_s_key,
                                     const Config& config_g);
  static std::string getPlanTableKey(const Config& config_s,
                                     const std::string& config_g_key);
  static std::string getPlanTableKey(const std::string& config_s_key,
                                     const std::string& config_g_key);

  void run();

public:
  IR(Problem* _P);
  ~IR() {};

  virtual void setParams(int argc, char *argv[]);
  static void printHelp();
};
