#include "../include/ecbs.hpp"

const std::string ECBS::SOLVER_NAME = "ECBS";
const float ECBS::DEFAULT_SUB_OPTIMALITY = 1.1;

ECBS::ECBS(Problem* _P) : Solver(_P)
{
  sub_optimality = DEFAULT_SUB_OPTIMALITY;
  solver_name = ECBS::SOLVER_NAME + "-" + std::to_string(sub_optimality);
}

void ECBS::run()
{
  // high-level search
  CompareHighLevelNode compareOPEN = getMainObjective();
  CompareHighLevelNode compareFOCAL = getFocalObjective();

  // OPEN, FOCAL
  std::priority_queue<HighLevelNode_p,
                      std::vector<HighLevelNode_p>,
                      CompareHighLevelNode> OPEN(compareOPEN);
  using FocalList = std::priority_queue<HighLevelNode_p,
                                         std::vector<HighLevelNode_p>,
                                         CompareHighLevelNode>;
  FocalList FOCAL(compareFOCAL);

  // initial node
  HighLevelNode_p n(new HighLevelNode);
  setInitialHighLevelNode(n);
  OPEN.push(n);
  FOCAL.push(n);
  int LB_min = n->LB;

  // main loop
  int h_node_num = 1;
  int iteration = 0;
  while (!OPEN.empty()) {
    ++iteration;
    // check limitation
    if (overCompTime()) break;

    // update focal list
    while (!OPEN.empty() && !OPEN.top()->valid) OPEN.pop();
    if (OPEN.empty()) break;  // failed
    if (LB_min != OPEN.top()->LB) {
      LB_min = OPEN.top()->LB;
      float LB_bound = LB_min * sub_optimality;
      std::vector<HighLevelNode_p> tmp;
      FocalList EMPTY(compareFOCAL);
      FOCAL = EMPTY;
      while (!OPEN.empty()) {
        HighLevelNode_p top = OPEN.top();
        OPEN.pop();
        if (!top->valid) continue;  // already searched
        tmp.push_back(top);  // escape
        if ((float)top->LB > LB_bound) break;  // higher than LB_bound
        FOCAL.push(top);  // lower than LB_bound
      }
      for (auto ele: tmp) OPEN.push(ele);   // back
    }

    // pickup one node
    n = FOCAL.top();
    FOCAL.pop();
    n->valid = false;  // closed

    info(" ",
         "elapsed:", getSolverElapsedTime(),
         ", explored_node_num:", iteration,
         ", nodes_num:", h_node_num,
         ", conflicts:", n->f,
         ", constraints:", n->constraints.size(),
         ", soc:", n->soc);

    // check conflict
    LibCBS::Constraints constraints =
      LibCBS::getFirstConstraints(n->paths);
    if (constraints.empty()) {
      solved = true;
      break;
    }

    // create new nodes
    for (auto c : constraints) {
      LibCBS::Constraints new_constraints = n->constraints;
      new_constraints.push_back(c);
      HighLevelNode_p m(new HighLevelNode {
          n->paths,
          new_constraints,
          n->makespan,
          n->soc,
          n->f,
          n->LB,
          n->f_mins,
          true });
      invoke(m, c->id);
      if (!m->valid) continue;
      OPEN.push(m);
      if (m->LB <= LB_min * sub_optimality) FOCAL.push(m);
      ++h_node_num;
    }
  }

  // success
  if (solved) solution = pathsToPlan(n->paths);
}

ECBS::CompareHighLevelNode ECBS::getMainObjective() {
  CompareHighLevelNode compare =
    [&] (HighLevelNode_p a, HighLevelNode_p b) {
      if (a->LB != b->LB) return a->LB > b->LB;
      return false;
    };
  return compare;
}

ECBS::CompareHighLevelNode ECBS::getFocalObjective() {
  CompareHighLevelNode compare =
    [&] (HighLevelNode_p a, HighLevelNode_p b) {
      if (a->f != b->f) return a->f > b->f;
      if (a->soc != b->soc) return a->soc > b->soc;
      return false;
    };
  return compare;
}

void ECBS::setInitialHighLevelNode(HighLevelNode_p n)
{
  Paths paths(P->getNum());
  std::vector<int> f_mins;
  for (int i = 0; i < P->getNum(); ++i) {
    Path path = getInitialPath(i);
    paths.insert(i, path);
    f_mins.push_back(path.size()-1);
  }
  n->paths = paths;
  n->constraints = {};  // constraints
  n->makespan = paths.getMakespan();
  n->soc = paths.getSOC();
  n->f = paths.countConflict();
  n->valid = true;  // valid
  n->f_mins = f_mins;
  n->LB = n->soc;
}

Path ECBS::getInitialPath(int id)
{
  Node* s = P->getStart(id);
  Node* g = P->getGoal(id);
  Nodes config_g = P->getConfigGoal();

  AstarHeuristics fValue =
    [&] (AstarNode* n) { return n->g + pathDist(n->v, g); };

  CompareAstarNode compare =
    [&] (AstarNode* a, AstarNode* b) {
      if (a->f != b->f) return a->f > b->f;
      // [IMPORTANT!] avoid goal locations of others
      if (a->v != g && inArray(a->v, config_g)) return true;
      if (b->v != g && inArray(b->v, config_g)) return false;
      if (a->g != b->g) return a->g < b->g;
      return false;
    };

  CheckAstarFin checkAstarFin = [&] (AstarNode* n) { return n->v == g; };

  CheckInvalidAstarNode checkInvalidAstarNode =
    [&] (AstarNode* m) {  return false; };

  return getTimedPath(s, g,
                      fValue,
                      compare,
                      checkAstarFin,
                      checkInvalidAstarNode);
}

void ECBS::invoke(HighLevelNode_p h_node, int id)
{
  auto res = getFocalPath(h_node, id);
  Path path = std::get<0>(res);
  int f_min = std::get<1>(res); // lower bound

  if (path.empty()) {
    h_node->valid = false;
    return;
  }
  Paths paths = h_node->paths;
  paths.insert(id, path);
  h_node->f = h_node->f
    - h_node->paths.countConflict(id, h_node->paths.get(id))
    + h_node->paths.countConflict(id, paths.get(id));
  h_node->paths = paths;
  h_node->makespan = h_node->paths.getMakespan();
  h_node->soc = h_node->paths.getSOC();
  h_node->LB = h_node->LB - h_node->f_mins[id] + f_min;
  h_node->f_mins[id] = f_min;
}

std::tuple<Path, int> ECBS::getFocalPath(HighLevelNode_p h_node, int id)
{
  Node* s = P->getStart(id);
  Node* g = P->getGoal(id);

  LibCBS::Constraints constraints;
  int max_constraint_time = 0;
  for (auto c : h_node->constraints) {
    if (c->id == id) {
      constraints.push_back(c);
      if (c->v == g && c->u == nullptr) {
        max_constraint_time = std::max(max_constraint_time, c->t);
      }
    }
  }

  FocalHeuristics f1Value;
  if (h_node->paths.costOfPath(id) > max_constraint_time) {
    f1Value = [&] (FocalNode* n) { return n->g + pathDist(n->v, g); };
  } else {
    f1Value =
      [&] (FocalNode* n) {
        return std::max(n->g, max_constraint_time) + pathDist(n->v, g);
      };
  }

  FocalHeuristics f2Value =
    [&] (FocalNode* n) {
      return h_node->paths.countConflict(id, getPathFromFocalNode(n));
    };

  CompareFocalNode compareOPEN =
    [&] (FocalNode* a, FocalNode* b) {
      if (a->f1 != b->f1) return a->f1 > b->f1;
      return false;
    };

  CompareFocalNode compareFOCAL =
    [&] (FocalNode* a, FocalNode* b) {
      if (a->f2 != b->f2) return a->f2 > b->f2;
      if (a->f1 != b->f1) return a->f1 > b->f1;
      if (a->g != b->g) return a->g < b->g;
      return false;
    };

  CheckFocalFin checkFocalFin =
    [&] (FocalNode* n) {
      return n->v == g && n->g > max_constraint_time;
    };

  CheckInvalidFocalNode checkInvalidFocalNode =
    [&] (FocalNode* m) {
      for (auto c : constraints) {
        if (m->g == c->t && m->v == c->v) {
          // vertex or swap conflict
          if (c->u == nullptr || c->u == m->p->v) return true;
        }
      }
      return false;
    };
  return getTimedPathByFocalSearch(s,
                                   g,
                                   sub_optimality,
                                   f1Value,
                                   f2Value,
                                   compareOPEN,
                                   compareFOCAL,
                                   checkFocalFin,
                                   checkInvalidFocalNode);
}

// return path and f_min
std::tuple<Path, int> ECBS::getTimedPathByFocalSearch
(Node* const s,
 Node* const g,
 float w,  // sub-optimality
 FocalHeuristics& f1Value,
 FocalHeuristics& f2Value,
 CompareFocalNode& compareOPEN,
 CompareFocalNode& compareFOCAL,
 CheckFocalFin& checkFocalFin,
 CheckInvalidFocalNode& checkInvalidFocalNode)
{
  auto getNodeName = [] (FocalNode* n)
                     { return std::to_string(n->v->id)
                         + "-" + std::to_string(n->g); };

  std::vector<FocalNode*> GC;  // for memory management
  auto createNewNode =
    [&] (Node* v, int g, int f1, int f2, FocalNode* p)
    {
      FocalNode* new_node = new FocalNode{ v, g, f1, f2, p };
      GC.push_back(new_node);
      return new_node;
    };

  // OPEN, FOCAL, CLOSE
  std::priority_queue<FocalNode*,
                      std::vector<FocalNode*>,
                      CompareFocalNode> OPEN(compareOPEN);
  std::unordered_map<std::string, bool> CLOSE;
  using FocalList = std::priority_queue<FocalNode*,
                                        std::vector<FocalNode*>,
                                        CompareFocalNode>;
  FocalList FOCAL(compareFOCAL);

  // initial node
  FocalNode* n;
  n = createNewNode(s, 0, 0, 0, nullptr);
  n->f1 = f1Value(n);
  n->f2 = f2Value(n);
  OPEN.push(n);
  FOCAL.push(n);
  int f1_min = n->f1;

  // main loop
  bool invalid = true;
  while (!OPEN.empty()) {
    // check time limit
    if (overCompTime()) break;

    // update FOCAL list
    while (!OPEN.empty()
           && CLOSE.find(getNodeName(OPEN.top())) != CLOSE.end())
      OPEN.pop();
    if (OPEN.empty()) break;  // failed
    if (f1_min != OPEN.top()->f1) {
      f1_min = OPEN.top()->f1;
      float f1_bound = f1_min*w;
      std::vector<FocalNode*> tmp;
      FocalList EMPTY(compareFOCAL);
      FOCAL = EMPTY;
      while (!OPEN.empty()) {
        FocalNode* top = OPEN.top();
        OPEN.pop();
        // already searched by focal
        if (CLOSE.find(getNodeName(top)) != CLOSE.end()) continue;
        tmp.push_back(top);  // escape
        if ((float)top->f1 > f1_bound) break;  // higher than f1_bound
        FOCAL.push(top);  // lower than f1_bound
      }
      for (auto ele: tmp) OPEN.push(ele);   // back
    }

    // focal minimum node
    n = FOCAL.top();
    FOCAL.pop();
    if (CLOSE.find(getNodeName(n)) != CLOSE.end()) continue;
    CLOSE[getNodeName(n)] = true;

    // check goal condition
    if (checkFocalFin(n)) {
      invalid = false;
      break;
    }

    // expand
    Nodes C = n->v->neighbor;
    C.push_back(n->v);
    for (auto u : C) {
      int g_cost = n->g+1;
      FocalNode* m = createNewNode(u, g_cost, 0, 0, n);
      // set heuristics
      m->f1 = f1Value(m);
      m->f2 = f2Value(m);
      // already searched?
      if (CLOSE.find(getNodeName(m)) != CLOSE.end()) continue;
      // check constraints
      if (checkInvalidFocalNode(m)) continue;
      // update open list
      OPEN.push(m);
      if (m->f1 <= f1_min * w) FOCAL.push(m);
    }
  }

  Path path;
  // success
  if (!invalid) path = getPathFromFocalNode(n);
  std::tuple<Path, int> ret = std::make_tuple(path, f1_min);

  // free
  while (!OPEN.empty()) OPEN.pop();
  while (!FOCAL.empty()) FOCAL.pop();
  for (auto p : GC) delete p;
  GC.clear();
  CLOSE.clear();

  return ret;
}

Path ECBS::getPathFromFocalNode(FocalNode* _n) {
  Path path;
  FocalNode* n = _n;
  while (n != nullptr) {
    path.push_back(n->v);
    n = n->p;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

void ECBS::setParams(int argc, char *argv[]) {
  struct option longopts[] = {
    { "sub-optimality", required_argument, 0, 'w' },
    { 0, 0, 0, 0 },
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "w:",
                            longopts, &longindex)) != -1) {
    switch (opt) {
    case 'w':
      sub_optimality = std::atof(optarg);
      if (sub_optimality < 1) halt("sub-optimality should be >= 1");
      solver_name = ECBS::SOLVER_NAME
        + "-" + std::to_string(sub_optimality);
      break;
    default:
      break;
    }
  }
}

void ECBS::printHelp() {
  std::cout << ECBS::SOLVER_NAME << "\n"
            << "  -w --sub-optimality [NUMBER]"
            << "  "
            << "sub-optimality >= 1"
            << std::endl;
}
