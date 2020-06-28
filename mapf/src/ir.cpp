#include "../include/ir.hpp"
#include "../include/problem.hpp"
#include "../include/pibt.hpp"
#include "../include/hca.hpp"
#include "../include/whca.hpp"
#include "../include/cbs_refine.hpp"
#include "../include/ecbs.hpp"
#include "../include/icbs_refine.hpp"


const std::string IR::SOLVER_NAME = "IR";
const int IR::DEFAULT_THRESHOLD_MAKESPAN = 8;
const int IR::DEFAULT_THRESHOLD_SOC_DIFF = 1;
const int IR::DEFAULT_THRESHOLD_NONDIFF_REFINE = 3;
const IR::REFINEMENT_TYPE IR::DEFAULT_REFINEMENT_TYPE =
  IR::REFINEMENT_TYPE::SIMPLE;
const IR::INIT_SOLVER_TYPE IR::DEFAULT_INIT_SOLVER =
  IR::INIT_SOLVER_TYPE::PIBT;
const IR::REFINE_SOLVER_TYPE IR::DEFAULT_REFINE_SOLVER =
  IR::REFINE_SOLVER_TYPE::CBS;


IR::IR(Problem* _P) : Solver(_P)
{
  solver_name = IR::SOLVER_NAME;

  refinement_type = DEFAULT_REFINEMENT_TYPE;
  init_solver = DEFAULT_INIT_SOLVER;
  refine_solver = DEFAULT_REFINE_SOLVER;

  threshold_makespan = DEFAULT_THRESHOLD_MAKESPAN;
  threshold_nondiff_refine = DEFAULT_THRESHOLD_NONDIFF_REFINE;
  threshold_soc_diff = DEFAULT_THRESHOLD_SOC_DIFF;

  cache_on = true;
  sampling_rate = 0;
}

void IR::solve()
{
  start();
  iterativeRefinement();
  end();
}

void IR::iterativeRefinement()
{
  int iteration = 0;
  solution = getInitialPlan();
  solved = !solution.empty();
  if (!solved) return;  // failure

  // start refinement
  Plans hist = { solution };
  while (true) {
    makeLog(output_file);  // anytime planning
    if (overCompTime()) break;

    info("  iter: ", iteration++,
         ", comp_time:", getSolverElapsedTime(),
         ", soc:", solution.getSOC(),
         ", makespan:", solution.getMakespan());
    Plan new_plan = refinePlan(P->getConfigStart(),
                               P->getConfigGoal(),
                               solution);
    hist.push_back(new_plan);
    if (stopRefinement(new_plan, hist)) break;
    solution = new_plan;
  }
}

Plan IR::getInitialPlan()
{
  Problem* _P = new Problem(P,
                            P->getConfigStart(),
                            P->getConfigGoal(),
                            max_comp_time,
                            max_timestep);

  Solver* solver;
  switch (init_solver) {
  case INIT_SOLVER_TYPE::HCA:
    solver = new HCA(_P);
    break;
  case INIT_SOLVER_TYPE::WHCA:
    solver = new WHCA(_P);
    break;
  case INIT_SOLVER_TYPE::ECBS:
    solver = new ECBS(_P);
    break;
  case INIT_SOLVER_TYPE::PIBT:
  default:
    solver = new PIBT(_P);
    break;
  }

  // set params
  if (!option_init_solver.empty()) {
    int argc = option_init_solver.size() + 1;
    char *argv[argc+1];
    for (int i = 0; i < argc; ++i) {
      char *tmp = const_cast<char*>(option_init_solver[i].c_str());
      argv[i+1] = tmp;
    }
    solver->setParams(argc, argv);
  }

  // solve
  solver->solve();

  // success
  if (solver->succeed()) return solver->getSolution();

  // failed
  Plan dummy;
  return dummy;
}

bool IR::stopRefinement(const Plan& new_plan, const Plans& hist)
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

Plan IR::refinePlan(const Config& config_s,
                    const Config& config_g,
                    const Plan& current_plan)
{
  if (solvableDirectly(config_s, config_g, current_plan)) {
    return MAPFSolver(config_s, config_g, current_plan);
  }

  switch (refinement_type) {
  case REFINEMENT_TYPE::QUADRATIC:
    return quadraticRefine(config_s, config_g, current_plan);
  case REFINEMENT_TYPE::SIMPLE:
  default:
    return simpleRefine(config_s, config_g, current_plan);
  }
}

Plan IR::simpleRefine(const Config& config_s,
                      const Config& config_g,
                      const Plan& old_plan)
{
  std::tuple<int, Config> mid_point = subgoalConfig(old_plan);
  Config config_mid = std::get<1>(mid_point);
  int mid_index = std::get<0>(mid_point);
  Plan old_plan_former = old_plan.getPartialPlan(0, mid_index);
  Plan old_plan_latter = old_plan.getPartialPlan(mid_index,
                                                 old_plan.getMakespan());
  Plan plan_former = refinePlan(config_s, config_mid, old_plan_former);
  Plan plan_latter = refinePlan(config_mid, config_g, old_plan_latter);

  Plan new_plan = plan_former + plan_latter;
  return new_plan;
}

Plan IR::quadraticRefine(const Config& config_s,
                         const Config& config_g,
                         const Plan& old_plan)
{
  std::tuple<int, Config> mid_point = subgoalConfig(old_plan);
  Config config_mid = std::get<1>(mid_point);
  int mid_index = std::get<0>(mid_point);
  Plan old_plan_former = old_plan.getPartialPlan(0, mid_index);
  Plan old_plan_latter = old_plan.getPartialPlan(mid_index,
                                                 old_plan.getMakespan());
  Plan plan_former = refinePlan(config_s, config_mid, old_plan_former);
  Plan plan_latter = refinePlan(config_mid, config_g, old_plan_latter);
  Plan tmp_plan = plan_former + plan_latter;

  // second step
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

std::tuple<int, Config> IR::subgoalConfig(Plan plan)
{
  int i = getRandomInt(0, plan.getMakespan(), MT);
  return std::make_tuple(i, plan.get(i));
}

bool IR::solvableDirectly(const Config& config_s,
                          const Config& config_g,
                          const Plan& current_plan)
{
  if (cache_on) {
    std::string key = getPlanTableKey(config_s, config_g);
    if (PLAN_TABLE.find(key) != PLAN_TABLE.end()) return true;
  }

  return current_plan.getMakespan() <= threshold_makespan;
}

Plan IR::MAPFSolver(const Config& config_s,
                    const Config& config_g,
                    const Plan& current_plan)
{
  if (current_plan.getMakespan() <= 1) return current_plan;
  int comp_time_limit = max_comp_time - getSolverElapsedTime();
  if (comp_time_limit <= 0) return current_plan;

  // caching
  if (cache_on) {
    std::string key = getPlanTableKey(config_s, config_g);
    auto itr = PLAN_TABLE.find(key);
    if (PLAN_TABLE.find(key) != PLAN_TABLE.end()) {
      return itr->second;
    }
  }

  Problem* _P = new Problem(P,
                            config_s,
                            config_g,
                            comp_time_limit,
                            max_timestep);

  // create sample
  std::vector<int> sample;
  if (sampling_rate > 0) {
    int sample_num = sampling_rate * P->getNum();
    std::vector<int> ids (P->getNum());
    std::iota(ids.begin(), ids.end(), 0);
    std::shuffle(ids.begin(), ids.end(), *MT);
    for (int i = 0; i < sample_num; ++i) sample.push_back(ids[i]);
  }

  // set solver
  Solver* solver;
  switch (refine_solver) {
  case REFINE_SOLVER_TYPE::CBS_NORMAL:
    solver = new CBS(_P);
    break;
  case REFINE_SOLVER_TYPE::ICBS:
    solver = new ICBS_REFINE(_P, current_plan, sample);
    break;
  case REFINE_SOLVER_TYPE::ICBS_NORMAL:
    solver = new ICBS(_P);
    break;
  case REFINE_SOLVER_TYPE::CBS:
  default:
    solver = new CBS_REFINE(_P, current_plan, sample);
    break;
  }

  // set params
  int argc = option_refine_solver.size() + 1;
  char *argv[argc+1];
  for (int i = 0; i < argc; ++i) {
    char *tmp = const_cast<char*>(option_refine_solver[i].c_str());
    argv[i+1] = tmp;
  }
  // solver->setVerbose(true);

  // solve
  solver->solve();

  if (solver->succeed()) {
    Plan plan = solver->getSolution();
    if (!cache_on) return plan;
    if (plan.getSOC() < current_plan.getSOC()) {
      registerTable(plan);
      return plan;
    }
  }
  registerTable(current_plan);
  return current_plan;
}

void IR::registerTable(const Plan& plan)
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

Plan IR::shrink(const Plan& plan)
{
  // try to shrink
  int makespan = plan.getMakespan();
  Paths paths = planToPaths(plan);
  int num_agents = paths.size();
  for (int i = 0; i < num_agents; ++i) {
    Nodes tmp1_path = paths.get(i);
    for (int t = 1; t <= makespan; ++t) {
      if (tmp1_path[t] == paths.get(i, makespan)) continue;  // goal
      if (tmp1_path[t] != tmp1_path[t-1]) continue;  // cannot shrink
      // create new path
      Nodes tmp2_path;
      for (int _t = 0; _t < t; ++_t) {
        tmp2_path.push_back(tmp1_path[_t]);
      }
      for (int _t = t + 1; _t <= makespan; ++_t) {
        tmp2_path.push_back(tmp1_path[_t]);
      }
      tmp2_path.push_back(*(tmp1_path.end()-1));
      // check collisions
      bool conflicted = false;
      for (int j = 0; j < num_agents; ++j) {
        if (j == i) continue;
        for (int _t = t; _t <= makespan; ++_t) {
          // vertex conflict
          if (tmp2_path[_t] == paths.get(j, _t)) {
            conflicted = true;
            break;
          }
          // swap conflict
          if (tmp2_path[_t] == paths.get(j, _t-1) &&
              tmp2_path[_t-1] == paths.get(j, _t)) {
            conflicted = true;
            break;
          }
        }
        if (conflicted) break;
      }
      if (!conflicted) {
        tmp1_path = tmp2_path;
        --t;
      }
    }
    paths.insert(i, tmp1_path);
  }
  return pathsToPlan(paths);
}

std::string IR::getPlanTableKey(const std::string& config_s_key,
                                const Config& config_g)
{
  return getPlanTableKey(config_s_key, getConfigName(config_g));
}

std::string IR::getPlanTableKey(const Config& config_s,
                                const std::string& config_g_key)
{
  return getPlanTableKey(getConfigName(config_s), config_g_key);
}

std::string IR::getPlanTableKey(const Config& config_s,
                                const Config& config_g)
{
  return getPlanTableKey(getConfigName(config_s),
                         getConfigName(config_g));
}

std::string IR::getPlanTableKey(const std::string& config_s_key,
                                const std::string& config_g_key)
{
  return config_s_key + "_" + config_g_key;
}

void IR::setParams(int argc, char *argv[])
{
  struct option longopts[] = {
    { "refinement-method", required_argument, 0, 'r' },
    { "threshold-makespan", required_argument, 0, 'm' },
    { "threshold-soc-diff", required_argument, 0, 'd' },
    { "threshold-nondiff", required_argument, 0, 'n' },
    { "sampling-rate", required_argument, 0, 'S' },
    { "init-solver", required_argument, 0, 'x' },
    { "refine-solver", required_argument, 0, 'y' },
    { "option-init-solver", required_argument, 0, 'X' },
    { "option-refine-solver", required_argument, 0, 'Y' },
    { 0, 0, 0, 0 },
  };
  optind = 1;  // reset
  int opt, longindex;
  std::string s, s_tmp;

  while ((opt = getopt_long(argc, argv, "o:r:m:d:n:S:x:y:X:Y:",
                            longopts, &longindex)) != -1) {
    switch (opt) {
    case 'o':
      output_file = std::string(optarg);
      break;
    case 'r':
      s = std::string(optarg);
      if (s == "simple") {
        refinement_type = REFINEMENT_TYPE::SIMPLE;
      } else if (s == "quadratic") {
        refinement_type = REFINEMENT_TYPE::QUADRATIC;
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
      if (sampling_rate < 0 && 1 < sampling_rate) {
        warn("sampling rate is within 0-1.");
        sampling_rate = 0;
      }
      if (sampling_rate > 0) cache_on = false;
      break;
    case 'x':
      s = std::string(optarg);
      if (s == "PIBT") {
        init_solver = INIT_SOLVER_TYPE::PIBT;
      } else if (s == "HCA") {
        init_solver = INIT_SOLVER_TYPE::HCA;
      } else if (s == "WHCA") {
        init_solver = INIT_SOLVER_TYPE::WHCA;
      } else if (s == "ECBS") {
        init_solver = INIT_SOLVER_TYPE::ECBS;
      } else {
        warn("solver does not exists, use PIBT");
      }
      break;
    case 'X':
      s = std::string(optarg);
      for (int i = 0; i < s.size(); ++i) {
        if (s[i] == ' ') {
          option_init_solver.push_back(s_tmp);
          s_tmp = "";
        } else {
          s_tmp += s[i];
          if (i == s.size() - 1) option_init_solver.push_back(s_tmp);
        }
      }
      break;
    case 'y':
      s = std::string(optarg);
      if (s == "CBS") {
        refine_solver = REFINE_SOLVER_TYPE::CBS;
      } else if (s == "CBS_NORMAL") {
        refine_solver = REFINE_SOLVER_TYPE::CBS_NORMAL;
      } else if (s == "ICBS") {
        refine_solver = REFINE_SOLVER_TYPE::ICBS;
      } else if (s == "ICBS_NORMAL") {
        refine_solver = REFINE_SOLVER_TYPE::ICBS_NORMAL;
      } else {
        warn("solver does not exists, use CBS");
      }
      break;
    case 'Y':
      s = std::string(optarg);
      for (int i = 0; i < s.size(); ++i) {
        if (s[i] == ' ') {
          option_refine_solver.push_back(s_tmp);
          s_tmp = "";
        } else {
          s_tmp += s[i];
          if (i == s.size() - 1) option_refine_solver.push_back(s_tmp);
        }
      }
      break;
    default:
      break;
    }
  }
}

void IR::printHelp()
{
  std::cout << IR::SOLVER_NAME << "\n"
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

            << "  -x --init-solver [SOLVER]"
            << "     "
            << "init solver, { PIBT, HCA, WHCA }\n"

            << "  -X --option-init-solver [\"OPTION\"]\n"
            << "                                "
            << "option for init-solver\n"

            << "  -y --refine-solver [SOLVER]"
            << "   "
            << "refine solver, { CBS, CBS_USUAL }\n"

            << "  -Y --option-refine-solver [\"OPTION\"]\n"
            << "                                "
            << "option for refine-solver\n"

            << "  -S --sampling-rate [rate]"
            << "     "
            << "sampling rate for refine-solver\n"
            << std::endl;
}
