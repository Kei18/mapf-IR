#include "../include/ir_configs.hpp"

const std::string IR_CONFIGS::SOLVER_NAME = "IR_CONFIGS";
const int IR_CONFIGS::DEFAULT_THRESHOLD_MAKESPAN = 8;
const int IR_CONFIGS::DEFAULT_THRESHOLD_SOC_DIFF = 1;
const int IR_CONFIGS::DEFAULT_THRESHOLD_NONDIFF_REFINE = 3;
const IR_CONFIGS::REFINEMENT_TYPE IR_CONFIGS::DEFAULT_REFINEMENT_TYPE =
  IR_CONFIGS::REFINEMENT_TYPE::SIMPLE;


IR_CONFIGS::IR_CONFIGS(Problem* _P)
  : IR(_P)
{
  solver_name = IR_CONFIGS::SOLVER_NAME;

  refinement_type = DEFAULT_REFINEMENT_TYPE;
  threshold_makespan = DEFAULT_THRESHOLD_MAKESPAN;
  threshold_nondiff_refine = DEFAULT_THRESHOLD_NONDIFF_REFINE;
  threshold_soc_diff = DEFAULT_THRESHOLD_SOC_DIFF;
  cache_on = true;
  make_log_every_itr = false;
  sampling_rate = 0;
}

IR_CONFIGS::~IR_CONFIGS()
{
}

bool IR_CONFIGS::stopRefinement(const Plans& hist)
{
  if (hist.size() <= threshold_nondiff_refine) return false;
  auto itr = hist.end() - 2;
  int cnt = 0;
  while (itr != hist.begin()) {
    if (std::abs((*itr).getSOC() - (*(itr+1)).getSOC())
        <= threshold_soc_diff) {
      ++cnt;
      --itr;
      if (threshold_nondiff_refine <= cnt) return true;
    } else {
      return false;
    }
  }
  return false;
}

Plan IR_CONFIGS::refinePlan(const Config& config_s,
                            const Config& config_g,
                            const Plan& current_plan)
{
  if (solvableDirectly(config_s, config_g, current_plan)) {
    auto res = getOptimalPlan(config_s, config_g, current_plan);
    return std::get<1>(res);  // plan
  }

  switch (refinement_type) {
  case REFINEMENT_TYPE::TWOSTAGE:
    return twoStageRefine(config_s, config_g, current_plan);
  case REFINEMENT_TYPE::SIMPLE:
  default:
    return simpleRefine(config_s, config_g, current_plan);
  }
}

Plan IR_CONFIGS::simpleRefine(const Config& config_s,
                              const Config& config_g,
                              const Plan& current_plan)
{
  std::tuple<int, Config> mid_point = subgoalConfig(current_plan);
  Config config_mid = std::get<1>(mid_point);
  int mid_index = std::get<0>(mid_point);
  Plan current_plan_former = current_plan.getPartialPlan(0, mid_index);
  Plan current_plan_latter =
    current_plan.getPartialPlan(mid_index, current_plan.getMakespan());
  Plan plan_former = refinePlan(config_s, config_mid, current_plan_former);
  Plan plan_latter = refinePlan(config_mid, config_g, current_plan_latter);

  return plan_former + plan_latter;
}

Plan IR_CONFIGS::twoStageRefine(const Config& config_s,
                                const Config& config_g,
                                const Plan& current_plan)
{
  std::tuple<int, Config> mid_point = subgoalConfig(current_plan);
  Config config_mid = std::get<1>(mid_point);
  int mid_index = std::get<0>(mid_point);
  Plan current_plan_former = current_plan.getPartialPlan(0, mid_index);
  Plan current_plan_latter
    = current_plan.getPartialPlan(mid_index, current_plan.getMakespan());
  Plan plan_former = refinePlan(config_s, config_mid, current_plan_former);
  Plan plan_latter = refinePlan(config_mid, config_g, current_plan_latter);
  Plan tmp_plan = plan_former + plan_latter;

  // second stage
  std::tuple<int, Config> former_mid_point = subgoalConfig(plan_former);
  std::tuple<int, Config> latter_mid_point = subgoalConfig(plan_latter);
  Config config_former_mid = std::get<1>(former_mid_point);
  Config config_latter_mid = std::get<1>(latter_mid_point);
  Plan tmp_plan_mid = tmp_plan.getPartialPlan(config_former_mid,
                                              config_latter_mid);
  Plan plan_mid = refinePlan(config_former_mid,
                             config_latter_mid,
                             tmp_plan_mid);
  Plan new_plan;
  new_plan += plan_former.getPartialPlan(0, std::get<0>(former_mid_point));
  new_plan += plan_mid;
  new_plan += plan_latter.getPartialPlan(std::get<0>(latter_mid_point),
                                         plan_latter.getMakespan());
  return new_plan;
}

std::tuple<int, Config> IR_CONFIGS::subgoalConfig(Plan plan)
{
  int i = getRandomInt(0, plan.getMakespan(), MT);
  return std::make_tuple(i, plan.get(i));
}

bool IR_CONFIGS::solvableDirectly(const Config& config_s,
                                  const Config& config_g,
                                  const Plan& current_plan)
{
  if (cache_on) {
    std::string key = getPlanTableKey(config_s, config_g);
    if (PLAN_TABLE.find(key) != PLAN_TABLE.end()) return true;
  }

  return current_plan.getMakespan() <= threshold_makespan;
}

std::tuple<bool, Plan> IR_CONFIGS::getOptimalPlan(const Config& config_s,
                                                  const Config& config_g,
                                                  const Plan& current_plan)
{
  if (current_plan.getMakespan() <= 1) {
    return std::make_tuple(true, current_plan);
  }

  // caching
  if (cache_on) {
    std::string key = getPlanTableKey(config_s, config_g);
    auto itr = PLAN_TABLE.find(key);
    if (PLAN_TABLE.find(key) != PLAN_TABLE.end()) {
      return std::make_tuple(true, itr->second);
    }
  }

  // create sample
  Ints sample = sampling(current_plan);

  // create problem
  int comp_time_limit
    = std::min(max_comp_time - (int)getSolverElapsedTime(),
               timeout_refinement);
  if (comp_time_limit <= 0) return std::make_tuple(false, current_plan);
  Problem* _P = new Problem(P,
                            config_s,
                            config_g,
                            comp_time_limit,
                            max_timestep);
  auto res = IR::getOptimalPlan(_P, current_plan, sampling(current_plan));
  if (cache_on && std::get<0>(res)) {
    registerTable(std::get<1>(res));
  }

  delete _P;
  return res;
}

Ints IR_CONFIGS::sampling(const Plan& current_plan)
{
  Ints sample;
  if (sampling_rate > 0) {
    int sample_num = sampling_rate * P->getNum();
    Ints ids (P->getNum());
    std::iota(ids.begin(), ids.end(), 0);
    std::shuffle(ids.begin(), ids.end(), *MT);
    for (int i = 0; i < sample_num; ++i) sample.push_back(ids[i]);
  }
  return sample;
}

void IR_CONFIGS::registerTable(const Plan& plan)
{
  if (!cache_on) return;
  std::string key = getPlanTableKey(plan.get(0), plan.last());
  if (PLAN_TABLE.find(key) != PLAN_TABLE.end()) return;
  for (int i = 0; i < plan.size(); ++i) {
    Config c_i = plan.get(i);
    std::string c_i_name = getConfigName(c_i);
    Plan partial_plan;
    partial_plan.add(c_i);
    for (int j = i + 1; j < plan.size(); ++j) {
      Config c_j = plan.get(j);
      partial_plan.add(c_j);
      key = getPlanTableKey(c_i_name, c_j);
      PLAN_TABLE[key] = partial_plan;
    }
  }
}

std::string IR_CONFIGS::getPlanTableKey(const std::string& config_s_key,
                                        const Config& config_g)
{
  return getPlanTableKey(config_s_key, getConfigName(config_g));
}

std::string IR_CONFIGS::getPlanTableKey(const Config& config_s,
                                        const std::string& config_g_key)
{
  return getPlanTableKey(getConfigName(config_s), config_g_key);
}

std::string IR_CONFIGS::getPlanTableKey(const Config& config_s,
                                        const Config& config_g)
{
  return getPlanTableKey(getConfigName(config_s),
                         getConfigName(config_g));
}

std::string IR_CONFIGS::getPlanTableKey(const std::string& config_s_key,
                                        const std::string& config_g_key)
{
  return config_s_key + "_" + config_g_key;
}

void IR_CONFIGS::setParams(int argc, char *argv[])
{
  struct option longopts[] = {
    { "refinement-method", required_argument, 0, 'r' },
    { "threshold-makespan", required_argument, 0, 'm' },
    { "threshold-soc-diff", required_argument, 0, 'd' },
    { "threshold-nondiff", required_argument, 0, 'n' },
    { "sampling-rate", required_argument, 0, 'S' },
    { 0, 0, 0, 0 },
  };
  optind = 1;  // reset
  int opt, longindex;
  std::string s, s_tmp;

  char *argv_copy[argc+1];
  for (int i = 0; i < argc; ++i) argv_copy[i] = argv[i];
  IR::setParams(argc, argv_copy);

  while ((opt = getopt_long(argc, argv, "r:m:d:n:S:",
                            longopts, &longindex)) != -1) {
    switch (opt) {
    case 'r':
      s = std::string(optarg);
      if (s == "simple") {
        refinement_type = REFINEMENT_TYPE::SIMPLE;
      } else if (s == "quadratic") {
        refinement_type = REFINEMENT_TYPE::TWOSTAGE;
      } else {
        warn("type does not exists, use simple-method");
        refinement_type = REFINEMENT_TYPE::SIMPLE;
      }
      break;
    case 'm':
      threshold_makespan = std::atoi(optarg);
      if (threshold_makespan < 1) {
        warn("threshold must be >= 2, using default value");
        threshold_makespan = DEFAULT_THRESHOLD_MAKESPAN;
      }
      break;
    case 'd':
      threshold_soc_diff = std::atoi(optarg);
      if (threshold_soc_diff < 0) {
        warn("threshold must be >= 0, using default value");
        threshold_soc_diff = DEFAULT_THRESHOLD_SOC_DIFF;
      }
      break;
    case 'n':
      threshold_nondiff_refine = std::atoi(optarg);
      if (threshold_nondiff_refine < 1) {
        warn("threshold must be >= 1, using default value");
        threshold_nondiff_refine = DEFAULT_THRESHOLD_NONDIFF_REFINE;
      }
      break;
    case 'S':
      sampling_rate = std::atof(optarg);
      if (sampling_rate < 0 || 1 < sampling_rate) {
        warn("sampling rate is within 0-1.");
        sampling_rate = 0;
      }
      if (sampling_rate > 0) {
        cache_on = false;
      }
      break;
    default:
      break;
    }
  }
}

void IR_CONFIGS::printHelp()
{
  std::cout << IR_CONFIGS::SOLVER_NAME << "\n"
            << "  -r --refinement-method [TYPE]"
            << " "
            << "{simple, quadratic}\n"

            << "  -m --threshold-makespan [INT]"
            << " "
            << "threshold makespan used to judge apply MAPF solver\n"

            << "  -d --threshold-soc-diff [INT]"
            << " "
            << "threshold soc value used to finish refinement\n"

            << "  -n --threshold-nondiff [INT]"
            << "  "
            << "threshold non-diff value used to finish refinement\n"

            << "  -S --sampling-rate [rate]"
            << "     "
            << "sampling rate for refine-solver\n"

            << "  (other: same as IR)"
            << std::endl;
}
