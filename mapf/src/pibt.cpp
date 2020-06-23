#include "../include/pibt.hpp"

const std::string PIBT::SOLVER_NAME = "PIBT";

PIBT::PIBT(Problem* _P) : Solver(_P)
{
  solver_name = PIBT::SOLVER_NAME;
}

void PIBT::solve()
{
  start();

  Plan plan;
  auto compare = [] (Agent* a, const Agent* b) {
                   if (a->elapsed != b->elapsed)
                     return a->elapsed < b->elapsed;
                   if (a->init_d != b->init_d)
                     return a->init_d < b->init_d;
                   return a->tie_breaker < b->tie_breaker;
                 };
  std::priority_queue<Agent*,
                      std::vector<Agent*>,
                      decltype(compare)> undecided(compare);
  std::vector<Agent*> decided;

  // initialize
  std::unordered_map<int, Agent*> occupied_now;
  std::unordered_map<int, Agent*> occupied_next;

  for (int i = 0; i < P->getNum(); ++i) {
    Node* s = P->getStart(i);
    Node* g = P->getGoal(i);
    int d = disable_dist_init ? 0 : pathDist(s, g);
    Agent* a = new Agent { i,        // id
                           s,        // current location
                           nullptr,  // next location
                           g,        // goal
                           0,        // elapsed
                           d,        // dist from s -> g
                           getRandomFloat(0, 1, MT) };  // tiebreaker
    undecided.push(a);
    occupied_now[s->id] = a;
  }
  plan.add(P->getConfigStart());

  // main loop
  int timestep = 0;
  while (true) {
    info(" ",
         "elapsed:", getSolverElapsedTime(),
         ", timestep:", timestep);

    // planning
    while (!undecided.empty()) {
      Agent* a = undecided.top();
      undecided.pop();
      if (a->v_next == nullptr) {
        funcPIBT(a, occupied_now, occupied_next);
      }
      decided.push_back(a);
    }
    occupied_now.clear();
    occupied_next.clear();

    // acting
    bool check_goal_cond = true;

    Config config(P->getNum(), nullptr);
    for (auto a : decided) {
      // set next location
      config[a->id] = a->v_next;
      occupied_now[a->v_next->id] = a;
      // check goal condition
      check_goal_cond &= (a->v_next == a->g);
      // update priority
      a->elapsed = (a->v_next == a->g) ? 0 : a->elapsed + 1;
      // reset params
      a->v_now = a->v_next;
      a->v_next = nullptr;
      // push to priority queue
      undecided.push(a);
    }
    decided.clear();
    plan.add(config);

    ++timestep;
    if (check_goal_cond) {
      solved = true;
      break;
    }
    if (timestep >= max_timestep || overCompTime()) {
      break;
    }
  }

  solution = plan;
  end();
}

bool PIBT::funcPIBT(Agent* ai,
                    std::unordered_map<int, Agent*>& occupied_now,
                    std::unordered_map<int, Agent*>& occupied_next)
{
  Node* v = planOneStep(ai, occupied_now, occupied_next);
  while (v != nullptr) {
    auto itr = occupied_now.find(v->id);
    if (itr != occupied_now.end()) {
      Agent* aj = itr->second;
      if (aj != ai && aj->v_next == nullptr) {
        if (!funcPIBT(aj, occupied_now, occupied_next)) {
          v = planOneStep(ai, occupied_now, occupied_next);
          continue;
        }
      }
    }
    return true;
  }
  // failed to secure node, cope stuck
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

Node* PIBT::planOneStep(Agent* a,
                        std::unordered_map<int, Agent*>& occupied_now,
                        std::unordered_map<int, Agent*>& occupied_next)
{
  Node* v = chooseNode(a, occupied_now, occupied_next);
  if (v != nullptr) {
    occupied_next[v->id] = a;
    a->v_next = v;
  }
  return v;
}

Node* PIBT::chooseNode(Agent* a,
                       std::unordered_map<int, Agent*>& occupied_now,
                       std::unordered_map<int, Agent*>& occupied_next)
{
  Nodes C;
  Nodes C_pre = a->v_now->neighbor;
  C_pre.push_back(a->v_now);

  for (auto v : C_pre) {
    // avoid vertex conflict
    if (occupied_next.find(v->id) != occupied_next.end()) continue;
    // avoid swap conflict
    if (occupied_now.find(v->id) != occupied_now.end() &&
        occupied_now[v->id]->v_next == a->v_now) continue;
    C.push_back(v);
  }

  if (C.empty()) return nullptr;

  // randomize
  std::shuffle(C.begin(), C.end(), *MT);

  if (inArray(a->g, C)) return a->g;

  // see cost
  Node* v = *std::min_element(C.begin(), C.end(),
                              [&] (Node* v, Node* u) {
                                // path distance
                                int c_v = pathDist(v, a->g);
                                int c_u = pathDist(u, a->g);
                                if (c_v != c_u) return c_v < c_u;
                                // occupancy
                                int o_v = (int)(occupied_now.find(v->id)
                                                != occupied_now.end());
                                int o_u = (int)(occupied_now.find(u->id)
                                                != occupied_now.end());
                                if (o_v != o_u) return o_v < o_u;
                                // degree
                                int d_v = v->getDegree();
                                int d_u = u->getDegree();
                                if (d_v != d_u) return d_v > d_u;
                                // Manhattan dist
                                int m_v = v->manhattanDist(a->g);
                                int m_u = u->manhattanDist(a->g);
                                if (m_v != m_u) return m_v < m_u;
                                // Euclidean dist
                                float e_v = v->euclideanDist(a->g);
                                float e_u = u->euclideanDist(a->g);
                                if (e_v != e_u) return e_v < e_u;
                                // random
                                return getRandomBoolean(MT);
                              });
  return v;
}

void PIBT::setParams(int argc, char *argv[]) {
  struct option longopts[] = {
    { "disable-dist-init", no_argument, 0, 'd' },
    { 0, 0, 0, 0 },
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "d",
                            longopts, &longindex)) != -1) {
    switch (opt) {
    case 'd':
      disable_dist_init = true;
      break;
    default:
      break;
    }
  }
}

void PIBT::printHelp() {
  std::cout << PIBT::SOLVER_NAME << "\n"
            << "  -d --disable-dist-init"
            << "        "
            << "disable initialization of priorities "
            << "using distance from starts to goals"
            << std::endl;
}
