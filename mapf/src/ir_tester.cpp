#include "../include/ir_tester.hpp"
#include <set>
#include <memory>

IR_TESTER::IR_TESTER(Problem* _P)
  : IR_PATHS(_P)
{
}

IR_TESTER::~IR_TESTER()
{
}

Plan IR_TESTER::refinePlan(const Config& config_s,
                           const Config& config_g,
                           const Plan& current_plan)
{
  // ============================================
  // sampling
  Paths current_paths = planToPaths(current_plan);
  auto gap =
    [&] (int i) {
      return current_paths.costOfPath(i) - pathDist(i);
    };

  // find agent with largest gap
  int id_largest_gap = -1;
  int gap_largest = 0;
  for (int i = 0; i < P->getNum(); ++i) {
    if (inArray(i, CLOSE_GAP)) continue;
    int gap_i = gap(i);
    if (gap_i > gap_largest) {
      id_largest_gap = i;
      gap_largest = gap_i;
    }
  }
  if (id_largest_gap == -1) {
    CLOSE_GAP.clear();
    return current_plan;
  }

  // ===================================
  // find interacting agents
  struct AgentNode;
  using AgentNode_p = std::shared_ptr<AgentNode>;
  struct AgentNode {
    int id;
    std::set<AgentNode_p> children;

    AgentNode(int _id): id(_id) {}
  };

  std::unordered_map<int, AgentNode_p> KNOWN;

  AgentNode_p n = AgentNode_p(new AgentNode(id_largest_gap));
  std::queue<AgentNode_p> OPEN;
  KNOWN[n->id] = n;
  OPEN.push(n);
  // create dependencies
  while (!OPEN.empty()) {
    n = OPEN.front();
    OPEN.pop();
    int id = n->id;
    Node* g = P->getGoal(id);
    int cost = current_paths.costOfPath(id);
    int dist = pathDist(id);
    for (int t = cost - 1; t >= dist; --t) {
      for (int i = 0; i < P->getNum(); ++i) {
        if (i == id) continue;
        if (current_paths.get(i, t) != g) continue;
        auto itr = KNOWN.find(i);
        AgentNode_p m;
        if (itr == KNOWN.end()) {  // create new
          m = AgentNode_p(new AgentNode(i));
          KNOWN[i] = m;
          OPEN.push(m);
        } else {
          m = itr->second;
        }
        n->children.insert(m);
      }
    }
  }

  // find small interacting group
  Ints sample = { id_largest_gap };
  for (auto itr = KNOWN.begin(); itr != KNOWN.end(); ++itr) {
    if (itr->first == id_largest_gap) continue;
    n = itr->second;
    sample.push_back(n->id);

    if (inArray(n->id, CLOSE_GAP)) continue;

    // single agent
    if (n->children.empty()) {
      if (gap(n->id) > 0) {
        sample.clear();
        sample.push_back(n->id);
        break;
      }
      continue;
    }

    // case: having children
    if (std::all_of(n->children.begin(), n->children.end(),
                    [] (AgentNode_p m) { return m->children.empty(); })) {
      sample.clear();
      sample.push_back(n->id);
      for (auto m : n->children) sample.push_back(m->id);
      break;
    }
  }

  info(" ", "id=", id_largest_gap,
       " ", "gap=", gap_largest,
       ", sample size=", sample.size(),
       ", sample top=", sample[0],
       ", sample gap=", gap(sample[0]),
       ", direct dependencies=", KNOWN[id_largest_gap]->children.size()+1,
       ", all dependencies=", KNOWN.size());


  // ============================================
  // create problem
  int comp_time_limit
    = std::min(max_comp_time - (int)getSolverElapsedTime(),
               timeout_refinement);
  if (comp_time_limit <= 0) return current_plan;
  Problem* _P = new Problem(P,
                            config_s,
                            config_g,
                            comp_time_limit,
                            max_timestep);

  // solve
  auto res = getOptimalPlan(_P, current_plan, sample);
  Plan plan = std::get<1>(res);
  if (!std::get<0>(res) || plan.getSOC() == current_plan.getSOC()) {
    CLOSE_GAP.push_back(sample[0]);
  }
  return plan;
}
