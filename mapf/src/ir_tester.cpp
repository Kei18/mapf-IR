#include "../include/ir_tester.hpp"
#include<set>


const std::string IR_TESTER::SOLVER_NAME = "IR_TESTER";


IR_TESTER::IR_TESTER(Problem* _P)
  : IR(_P)
{
  cache_on = false;
}

IR_TESTER::~IR_TESTER()
{
}

void IR_TESTER::iterativeRefinement()
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
    Plan new_plan = getPickUpRefinement(solution);

    hist.push_back(new_plan);
    if (stopRefinement(new_plan, hist)) break;
    solution = new_plan;
  }

  info("  refinement results, soc:", hist.begin()->getSOC(),
       "->", solution.getSOC(),
       ", makespan:", hist.begin()->getMakespan(),
       "->", solution.getMakespan());
}

Plan IR_TESTER::getPickUpRefinement(const Plan& plan)
{
  std::vector<int> ids(P->getNum());
  std::iota(ids.begin(), ids.end(), 0);
  Paths paths = planToPaths(plan);
  auto getGap =
    [&] (int i) {
      return paths.costOfPath(i)
        - pathDist(P->getStart(i), P->getGoal(i));
    };
  std::sort(ids.begin(), ids.end(),
            [&] (int i, int j) {
              int gap_i = getGap(i);
              int gap_j = getGap(j);
              if (gap_i != gap_j) return gap_i > gap_j;
              return false;
            });
  for (int j = 0; j < ids.size()/20; ++j) {
    int i = ids[j];
    std::cout << i << ": ideal=" << pathDist(i)
              << ", gap=" << getGap(i)
              << std::endl;
  }

  std::set<int> sample;
  std::vector<int> CLOSE;
  int i = ids[0];

  while (true) {
    std::cout << i << ": " << pathDist(i) << ", " << getGap(i) << std::endl;
    CLOSE.push_back(i);
    if (getGap(i) == 0) break;
    int cost_i = paths.costOfPath(i);
    int ideal_i = pathDist(i);
    Node* g = P->getGoal(i);
    for (int t = cost_i - 1; t >= ideal_i; --t) {
      for (int j = 0; j < P->getNum(); ++j) {
        if (i == j) continue;
        if (plan.get(t, j) == g) {
          if (sample.empty()) sample.insert(i);
          if (sample.size() > 2) continue;
          sample.insert(j);
          std::cout << "- " << j << ", " << t << ": "
                    << pathDist(j)
                    << ", " << getGap(j) << std::endl;
        }
      }
    }
    if (sample.empty()) break;
    std::vector<int> next_cands;
    for (auto j : sample) {
      if (inArray(j, CLOSE)) continue;
      next_cands.push_back(j);
    }
    if (next_cands.empty()) break;
    std::sort(next_cands.begin(), next_cands.end(),
              [&] (int i, int j) {
                int gap_i = getGap(i);
                int gap_j = getGap(j);
                if (gap_i != gap_j) return gap_i > gap_j;
                return false;
              });
    i = next_cands[0];
  }
  if (sample.empty()) {
    Path path = paths.get(i);
    Node* g = *(path.end()-1);
    bool found_potential_margin = false;
    for (int t = paths.getMakespan(); t > 0; --t) {
      if (!found_potential_margin) {
        if (path[t] == g) continue;
        found_potential_margin = true;
        continue;
      } else {
        if (path[t] != g) continue;
        // create new path
        Path new_path;
        for (int _t = 0; _t < t; ++_t) new_path.push_back(path[_t]);
        for (int _t = t; _t <= paths.getMakespan(); ++_t) {
          new_path.push_back(g);
        }
        int cnt = paths.countConflict(i, new_path);
        if (cnt == 0) {
          paths.insert(i, new_path);
          std::cout << "hit" << std::endl;
          return pathsToPlan(paths);
        }
        break;
      }
    }
    return plan;
  }


  std::vector<int> sample_vec(sample.begin(), sample.end());
  Problem* _P = new Problem(P,
                            P->getConfigStart(),
                            P->getConfigGoal(),
                            max_comp_time,
                            max_timestep);
  Solver* solver = new ICBS_REFINE(_P, plan, sample_vec);
  solver->setVerbose(true);
  solver->solve();

  if (solver->succeed()) return solver->getSolution();
  return plan;
}

Plan IR_TESTER::MAPFSolver(const Config& config_s,
                    const Config& config_g,
                    const Plan& current_plan)
{
  if (current_plan.getMakespan() <= 1) return current_plan;
  int comp_time_limit = std::min(max_comp_time -
                                 (int)getSolverElapsedTime(),
                                 timeout_refinement);
  if (comp_time_limit <= 0) return current_plan;

  // check lower bound
  int LB = 0;
  for (int i = 0; i < P->getNum(); ++i) {
    LB += pathDist(config_s[i], config_g[i]);
  }
  if (current_plan.getSOC() == LB) return current_plan;

  Problem* _P = new Problem(P,
                            config_s,
                            config_g,
                            comp_time_limit,
                            max_timestep);

  // create sample
  std::set<int> interacting_group;
  std::vector<int> sample(interacting_group.begin(),
                          interacting_group.end());

  Solver* solver = new ICBS_REFINE(_P, current_plan, sample);
  solver->setVerbose(verbose_underlying_solver);

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
