#include "../include/pibt.hpp"

const std::string PIBT::SOLVER_NAME = "PIBT";

PIBT::PIBT(Problem* _P)
  : Solver(_P),
    D(P->getNum(), std::vector<int>(P->getG()->getNodesSize(), 0)),
    occupied_now(Agents(P->getG()->getNodesSize(), nullptr)),
    occupied_next(Agents(P->getG()->getNodesSize(), nullptr))
{
  solver_name = PIBT::SOLVER_NAME;
}

void PIBT::run()
{
  // create distance table
  // preprocessing();

  Plan plan;  // will be solution

  // compare priority of agents
  auto compare = [](Agent* a, const Agent* b) {
    if (a->elapsed != b->elapsed) return a->elapsed < b->elapsed;
    // use initial distance
    if (a->init_d != b->init_d) return a->init_d < b->init_d;
    return a->tie_breaker < b->tie_breaker;
  };

  // agents have not decided their next locations
  std::priority_queue<Agent*, Agents, decltype(compare)> undecided(
      compare);
  // agents have already decided their next locations
  std::vector<Agent*> decided;

  // initialize
  for (int i = 0; i < P->getNum(); ++i) {
    Node* s = P->getStart(i);
    Node* g = P->getGoal(i);
    int d = disable_dist_init ? 0 : pathDist(i);
    Agent* a = new Agent{i,                          // id
                         s,                          // current location
                         nullptr,                    // next location
                         g,                          // goal
                         0,                          // elapsed
                         d,                          // dist from s -> g
                         getRandomFloat(0, 1, MT)};  // tie-breaker
    undecided.push(a);
    occupied_now[s->id] = a;
  }
  plan.add(P->getConfigStart());

  // main loop
  int timestep = 0;
  while (true) {
    info(" ", "elapsed:", getSolverElapsedTime(), ", timestep:", timestep);

    // planning
    while (!undecided.empty()) {
      // pickup the agent with highest priority
      Agent* a = undecided.top();
      undecided.pop();

      // if the agent has next location, then skip
      if (a->v_next == nullptr) {
        // determine its next location
        funcPIBT(a);
      }
      decided.push_back(a);
    }

    // acting
    bool check_goal_cond = true;
    Config config(P->getNum(), nullptr);
    for (auto a : decided) {
      // clear
      if (occupied_now[a->v_now->id] == a) occupied_now[a->v_now->id] = nullptr;
      occupied_next[a->v_next->id] = nullptr;

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

    // update plan
    plan.add(config);

    ++timestep;

    // success
    if (check_goal_cond) {
      solved = true;
      break;
    }

    // failed
    if (timestep >= max_timestep || overCompTime()) {
      break;
    }
  }

  // memory clear
  while (!undecided.empty()) {
    delete undecided.top();
    undecided.pop();
  }

  solution = plan;
}

bool PIBT::funcPIBT(Agent* ai)
{
  // decide next node
  Node* v = planOneStep(ai);
  while (v != nullptr) {
    auto aj = occupied_now[v->id];
    if (aj != nullptr) {  // someone occupies v
      // avoid itself && allow rotations
      if (aj != ai && aj->v_next == nullptr) {
        // do priority inheritance and backtracking
        if (!funcPIBT(aj)) {
          // replan
          v = planOneStep(ai);
          continue;
        }
      }
    }
    // success to plan next one step
    return true;
  }
  // failed to secure node, cope stuck
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

/*
 * no candidate node -> return nullptr
 */
Node* PIBT::planOneStep(Agent* a)
{
  Node* v = chooseNode(a);
  if (v != nullptr) {
    // update reservation
    occupied_next[v->id] = a;
    a->v_next = v;
  }
  return v;
}

// no candidate node -> return nullptr
Node* PIBT::chooseNode(Agent* a)
{
  Nodes C;                           // candidates
  Nodes C_pre = a->v_now->neighbor;  // all possible node
  C_pre.push_back(a->v_now);

  for (auto v : C_pre) {
    // avoid vertex conflict
    if (occupied_next[v->id] != nullptr) continue;
    // avoid swap conflict
    auto a_j = occupied_now[v->id];
    if (a_j != nullptr && a_j->v_next == a->v_now) continue;

    // goal exists -> return immediately
    if (v == a->g) return v;

    C.push_back(v);
  }

  // correspond to stuck
  if (C.empty()) return nullptr;

  // randomize
  std::shuffle(C.begin(), C.end(), *MT);

  // pickup one node
  Node* v = *std::min_element(C.begin(), C.end(), [&](Node* v, Node* u) {
    // path distance
    int c_v = pathDist(a->id, v);
    int c_u = pathDist(a->id, u);
    if (c_v != c_u) return c_v < c_u;
    // occupancy
    int o_v = (int)(occupied_now[v->id] != nullptr);
    int o_u = (int)(occupied_now[u->id] != nullptr);
    if (o_v != o_u) return o_v < o_u;
    return false;
  });
  return v;
}

void PIBT::preprocessing()
{
  for (int i = 0; i < P->getNum(); ++i) {
    // breadth first search
    std::queue<Node*> OPEN;
    Node* n = P->getGoal(i);
    OPEN.push(n);

    while (!OPEN.empty()) {
      n = OPEN.front();
      OPEN.pop();
      const int d_n = D[i][n->id];
      for (auto m : n->neighbor) {
        const int d_m = D[i][m->id];
        if (d_m != 0 && d_n + 1 >= d_m) continue;
        D[i][m->id] = d_n + 1;
        OPEN.push(m);
      }
    }
  }
}

void PIBT::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
      {"disable-dist-init", no_argument, 0, 'd'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "d", longopts, &longindex)) != -1) {
    switch (opt) {
      case 'd':
        disable_dist_init = true;
        break;
      default:
        break;
    }
  }
}

void PIBT::printHelp()
{
  std::cout << PIBT::SOLVER_NAME << "\n"
            << "  -d --disable-dist-init"
            << "        "
            << "disable initialization of priorities "
            << "using distance from starts to goals" << std::endl;
}