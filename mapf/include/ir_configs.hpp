#pragma once
#include "ir.hpp"

class IR_CONFIGS : public IR {
public:
  static const std::string SOLVER_NAME;

protected:

  // used in solvableDirectly
  int threshold_makespan;

  // used in stopRefinement
  int threshold_soc_diff;
  int threshold_nondiff_refine;

  enum struct REFINEMENT_TYPE
    { SIMPLE, TWOSTAGE };
  REFINEMENT_TYPE refinement_type;

  // default params
  static const int DEFAULT_THRESHOLD_MAKESPAN;
  static const int DEFAULT_THRESHOLD_SOC_DIFF;
  static const int DEFAULT_THRESHOLD_NONDIFF_REFINE;
  static const REFINEMENT_TYPE DEFAULT_REFINEMENT_TYPE;

  // sampling rate
  float sampling_rate;

  // cache
  bool cache_on;
  std::unordered_map<std::string, Plan> PLAN_TABLE;

  bool stopRefinement(const Plans& hist);
  Plan refinePlan(const Config& config_s,
                  const Config& config_g,
                  const Plan& current_plan);
  Plan simpleRefine(const Config& config_s,
                    const Config& config_g,
                    const Plan& old_plan);
  Plan twoStageRefine(const Config& config_s,
                      const Config& config_g,
                      const Plan& old_plan);
  std::tuple<int, Config> subgoalConfig(Plan plan);
  bool solvableDirectly(const Config& config_s,
                        const Config& config_g,
                        const Plan& current_plan);
  std::tuple<bool, Plan> getOptimalPlan(const Config& config_s,
                                        const Config& config_g,
                                        const Plan& current_plan);
  Ints sampling(const Plan& current_plan);
  void registerTable(const Plan& plan);

  static std::string getPlanTableKey(const Config& config_s,
                                     const Config& config_g);
  static std::string getPlanTableKey(const std::string& config_s_key,
                                     const Config& config_g);
  static std::string getPlanTableKey(const Config& config_s,
                                     const std::string& config_g_key);
  static std::string getPlanTableKey(const std::string& config_s_key,
                                     const std::string& config_g_key);

public:
  IR_CONFIGS(Problem* _P);
  ~IR_CONFIGS();

  void setParams(int argc, char *argv[]);
  static void printHelp();
};
