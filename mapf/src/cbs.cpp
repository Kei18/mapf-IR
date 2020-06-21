#include "../include/cbs.hpp"

const std::string CBS::SOLVER_NAME = "CBS";

CBS::CBS(Problem* _P) : Solver(_P)
{
  solver_name = CBS::SOLVER_NAME;
  VERVOSE = verbose;
}

void CBS::solve()
{
  start();
  // high-level search

  // set objective function
  CompareHighLevelNodes compare = returnObjectiveFunc();

  // OPEN
  std::priority_queue<HighLevelNode*,
                      std::vector<HighLevelNode*>,
                      decltype(compare)> HighLevelTree(compare);

  HighLevelNode* n;
  n = createInitialHighLevelNode();
  HighLevelTree.push(n);

  int h_node_num = 1;
  int iteration = 0;
  while (!HighLevelTree.empty()) {
    ++iteration;
    // check limitation
    if (overCompTime()) break;

    if (iteration == 10) {
      info(" ",
           "elapsed:", getSolverElapsedTime(),
           ", explored_node_num:", iteration,
           ", nodes_num:", h_node_num);
    }

    n = HighLevelTree.top();
    HighLevelTree.pop();

    // check conflict
    Constraints constraints = getFirstConflict(n);
    if (constraints.empty()) {
      solved = true;
      break;
    }

    // create new nodes
    for (auto c : constraints) {
      Constraints new_constraints = n->constraints;
      new_constraints.push_back(c);
      HighLevelNode* m = new HighLevelNode
        { n->paths, new_constraints, n->makespan, n->soc, n->f, true };
      invoke(m, c->id);
      if (!m->valid) continue;
      HighLevelTree.push(m);
      ++h_node_num;
    }
  }

  if (solved) {  // success
    solution = n->paths.toPlan();
  } else {  // failed
    Config config_s;
    for (int i = 0; i < P->getNum(); ++i) {
      config_s.push_back(P->getStart(i));
    }
    solution.add(config_s);
  }
  end();
}

CBS::CompareHighLevelNodes CBS::returnObjectiveFunc()
{
  CompareHighLevelNodes compare;
  switch (objective_type) {
  case OBJECTIVE::MAKESPAN:
    compare = [] (HighLevelNode* a, HighLevelNode* b)
              { if (a->makespan != b->makespan)
                  return a->makespan > b->makespan;
                if (a->f != b->f) return a->f > b->f;
                return false; };
    break;
  case OBJECTIVE::MAKESPAN_SOC:
    compare = [] (HighLevelNode* a, HighLevelNode* b)
              { if (a->makespan != b->makespan)
                  return a->makespan > b->makespan;
                if (a->soc != b->soc) return a->soc > b->soc;
                if (a->f != b->f) return a->f > b->f;
                return false; };
    break;
  case OBJECTIVE::SOC:
  default:
    compare = [] (HighLevelNode* a, HighLevelNode* b)
              { if (a->soc != b->soc) return a->soc > b->soc;
                if (a->f != b->f) return a->f > b->f;  // tie-breaker
                return false; };
    break;
  }
  return compare;
}

CBS::HighLevelNode* CBS::createInitialHighLevelNode()
{
  Paths paths;
  paths.initialize(P->getNum());
  for (int i = 0; i < P->getNum(); ++i) {
    paths.insert(i, getPath(P->getStart(i), P->getGoal(i)));
  }

  HighLevelNode* s = new HighLevelNode
    { paths,
      {},  // constraints
      paths.getMakespan(),
      paths.getSOC(),
      countConflict(paths),
      true };
  return s;
}

CBS::Constraints CBS::getFirstConflict(const HighLevelNode* n)
{
  Constraints constraints = {};
  for (int t = 1; t <= n->makespan; ++t) {
    for (int i = 0; i < P->getNum(); ++i) {
      for (int j = i + 1; j < P->getNum(); ++j) {
        // vertex conflict
        if (n->paths.get(i, t) == n->paths.get(j, t)) {
          constraints.push_back(new Constraint
                                { i, t, n->paths.get(i, t), nullptr });
          constraints.push_back(new Constraint
                                { j, t, n->paths.get(j, t), nullptr });
          return constraints;
        }
        // swap conflict
        if (n->paths.get(i, t) == n->paths.get(j, t-1) &&
            n->paths.get(j, t) == n->paths.get(i, t-1)) {
          constraints.push_back(new Constraint
                                { i, t,
                                   n->paths.get(i, t),
                                   n->paths.get(i, t-1) });
          constraints.push_back(new Constraint
                                { j, t,
                                   n->paths.get(j, t),
                                   n->paths.get(j, t-1) });
          return constraints;
        }
      }
    }
  }
  return constraints;
}

void CBS::invoke(HighLevelNode* h_node, int id)
{
  Path path = getConstrainedPath(h_node, id);
  if (path.empty()) {
    h_node->valid = false;
    return;
  }
  Paths paths = h_node->paths;
  paths.insert(id, path);
  h_node->paths = paths;
  h_node->makespan = h_node->paths.getMakespan();
  h_node->soc = h_node->paths.getSOC();
  h_node->f = countConflict(h_node->paths);
}

int CBS::countConflict(const Paths& paths)
{
  int cnt = 0;
    for (int i = 0; i < P->getNum(); ++i) {
      for (int j = i + 1; j < P->getNum(); ++j) {
        for (int t = 1; t < paths.getMakespan(); ++t) {
        // vertex conflict
        if (paths.get(i, t) == paths.get(j, t)) ++cnt;
        // swap conflict
        if (paths.get(i, t) == paths.get(j, t-1) &&
            paths.get(j, t) == paths.get(i, t-1)) ++cnt;
      }
    }
  }
  return cnt;
}

Path CBS::getConstrainedPath(HighLevelNode* h_node, int id)
{
  Node* s = P->getStart(id);
  Node* g = P->getGoal(id);

  Constraints constraints;
  int max_constraint_time = 0;
  for (auto c : h_node->constraints) {
    if (c->id == id) {
      constraints.push_back(c);
      if (c->v == g && c->u == nullptr) {
        max_constraint_time = std::max(max_constraint_time, c->t);
      }
    }
  }

  CompareAstarNode compare =
    [&] (AstarNode* a, AstarNode* b) {
      if (a->f != b->f) return a->f > b->f;
      if (a->g != b->g) return a->g < b->g;
      // avoid conflict with others
      for (int i = 0; i < P->getNum(); ++i) {
        if (i == id) continue;
        if (a->g <= h_node->makespan &&
            h_node->paths.get(i, a->g) == a->v) return true;
        if (b->g <= h_node->makespan &&
            h_node->paths.get(i, b->g) == b->v) return false;
      }
      return false;
    };

  CheckAstarFin checkAstarFin =
    [&] (AstarNode* n) {
      return n->v == g && n->g > max_constraint_time;
    };

  CheckInvalidAstarNode checkInvalidAstarNode =
    [&] (AstarNode* m) {
      for (auto c : constraints) {
        if (m->g == c->t && m->v == c->v) {
          // vertex or swap conflict
          if (c->u == nullptr || c->u == m->p->v) return true;
        }
      }
      return false;
    };

  return getTimedPath(s, g,
                      compare,
                      checkAstarFin,
                      checkInvalidAstarNode);
}

void CBS::setParams(int argc, char *argv[]) {
  struct option longopts[] = {
    { "objective-func", required_argument, 0, 'x' },
    { 0, 0, 0, 0 },
  };
  optind = 1;  // reset
  int opt, longindex, tmp;
  int num_items = static_cast<int>(OBJECTIVE::NUM_ITEMS) - 1;
  while ((opt = getopt_long(argc, argv, "x:",
                            longopts, &longindex)) != -1) {
    switch (opt) {
    case 'x':
      tmp = std::atoi(optarg);
      if (tmp < 0 || num_items < tmp) {
        halt("type of objective func is within 0-"
             + std::to_string(num_items));
      }
      objective_type = static_cast<OBJECTIVE>(tmp);
      break;
    default:
      break;
    }
  }
}

void CBS::printHelp() {
  std::cout << CBS::SOLVER_NAME << "\n"
            << "  -x --objective-func [TYPE]"
            << "    "
            << "0: sum-of-cost, "
            << "1: makespan,"
            << "2: makespan using sum-of-cost as secondary"
            << std::endl;
}
