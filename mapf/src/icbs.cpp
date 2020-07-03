#include "../include/icbs.hpp"

const std::string ICBS::SOLVER_NAME = "ICBS";

ICBS::ICBS(Problem* _P) : CBS(_P)
{
  solver_name = ICBS::SOLVER_NAME;
  LAZY_EVAL_LB_SOC = -1;

  LibCBS::MDD::MT = MT;
}

ICBS::~ICBS() {}

void ICBS::run()
{
  // set objective function
  CompareHighLevelNodes compare = getObjective();

  // OPEN
  std::priority_queue<HighLevelNode_p,
                      std::vector<HighLevelNode_p>,
                      decltype(compare)> HighLevelTree(compare);

  HighLevelNode_p n(new HighLevelNode);
  setInitialHighLevelNode(n);
  if (!n->valid) return;  // failed to plan initial paths
  HighLevelTree.push(n);

  int h_node_num = 1;
  int iteration = 0;
  while (!HighLevelTree.empty()) {
    ++iteration;
    // check limitation
    if (overCompTime()) break;

    n = HighLevelTree.top();

    // check lazy table
    if (LAZY_EVAL_LB_SOC > 0 && n->soc >= LAZY_EVAL_LB_SOC) {
      auto nodes = lazyEval();
      for (auto node : nodes) HighLevelTree.push(node);
      --iteration;
      continue;
    }

    info(" ",
         "elapsed:", getSolverElapsedTime(),
         ", explored_node_num:", iteration,
         ", nodes_num:", h_node_num,
         ", conflicts:", n->f,
         ", constraints:", n->constraints.size(),
         ", soc:", n->soc);

    // check conflict
    LibCBS::Constraints constraints = getPrioritizedConflict(n);
    if (constraints.empty()) {
      solved = true;
      break;
    }

    // find bypass
    if (findBypass(n, constraints)) {
      --iteration;
      continue;
    }

    // failed
    HighLevelTree.pop();

    // create new nodes
    for (auto c : constraints) {
      if (c->id < 0 || P->getNum() <= c->id) halt("invalid id");
      LibCBS::Constraints new_constraints = n->constraints;
      new_constraints.push_back(c);
      HighLevelNode_p m(new HighLevelNode{
          ++h_node_num,
          n->paths,
          new_constraints,
          n->makespan,
          n->soc,
          n->f,
          true });
      MDDTable[m->id] = MDDTable[n->id];  // copy MDD
      invoke(m, c->id);
      if (!m->valid) continue;
      HighLevelTree.push(m);
    }
  }

  if (solved) solution = pathsToPlan(n->paths);
}

void ICBS::setInitialHighLevelNode(HighLevelNode_p n)
{
  CBS::setInitialHighLevelNode(n);
  if (!n->valid) return;

  // register mdds;
  LibCBS::MDDs mdds;
  for (int i = 0; i < P->getNum(); ++i) {
    int c = n->paths.costOfPath(i);
    LibCBS::MDD_p mdd(new LibCBS::MDD(c, i, P, {}));
    mdds.push_back(mdd);
  }
  MDDTable[n->id] = mdds;
}

LibCBS::Constraints ICBS::getPrioritizedConflict(HighLevelNode_p h_node)
{
  return LibCBS::getPrioritizedConflict(h_node->paths,
                                        MDDTable[h_node->id]);
}

// using MDD
Path ICBS::getConstrainedPath(HighLevelNode_p h_node, int id)
{
  LibCBS::MDD mdd = *(MDDTable[h_node->id][id]);
  LibCBS::Constraint_p last_constraint = *(h_node->constraints.end()-1);
  mdd.update({ last_constraint });  // check only last

  if (mdd.valid) {  // use mdd as much as possible
    // update table
    MDDTable[h_node->id][id] = std::make_shared<LibCBS::MDD>(mdd);
    return mdd.getPath();
  } else {
    // lazy evaluation
    if (last_constraint->t > mdd.c) {
      int LB_SOC = h_node->soc - mdd.c + last_constraint->t + 1;
      registerLazyEval(LB_SOC, h_node);
      return {};
    }

    int c = mdd.c;
    while (true) {
      ++c;
      if (overCompTime()) break;
      LibCBS::MDD_p new_mdd(new LibCBS::MDD(c, id, P, h_node->constraints));
      if (new_mdd->valid) {
        MDDTable[h_node->id][id] = new_mdd;
        return new_mdd->getPath();
      }
    }
  }
  return {};
}

void ICBS::registerLazyEval(const int LB_SOC, HighLevelNode_p h_node)
{
  info(" ", "LAZY_TABLE: HighLevelNode", h_node->id,
       ", LB_SOC: ", LB_SOC);
  auto itr = LAZY_EVAL_TABLE.find(LB_SOC);
  if (itr == LAZY_EVAL_TABLE.end()) {
    LAZY_EVAL_TABLE[LB_SOC] = { h_node };
  } else {
    itr->second.push_back(h_node);
  }

  // update lower bound
  if (LAZY_EVAL_LB_SOC == -1) {
    LAZY_EVAL_LB_SOC = LB_SOC;
  } else {
    LAZY_EVAL_LB_SOC = std::min(LB_SOC, LAZY_EVAL_LB_SOC);
  }
}

CBS::HighLevelNodes ICBS::lazyEval()
{
  info(" ", "lazy eval, soc=", LAZY_EVAL_LB_SOC);
  auto itr_lb = LAZY_EVAL_TABLE.find(LAZY_EVAL_LB_SOC);
  HighLevelNodes h_nodes = itr_lb->second;
  for (auto h_node : h_nodes) {
    LibCBS::Constraint_p last_constraint = *(h_node->constraints.end()-1);
    int id = last_constraint->id;
    int c = last_constraint->t;
    while (true) {
      ++c;
      LibCBS::MDD_p new_mdd(new LibCBS::MDD(c, id, P, h_node->constraints));
      if (new_mdd->valid) {
        MDDTable[h_node->id][id] = new_mdd;
        Path path = new_mdd->getPath();
        Paths paths = h_node->paths;
        paths.insert(id, path);
        h_node->f = h_node->f
          - h_node->paths.countConflict(id, h_node->paths.get(id))
          + h_node->paths.countConflict(id, paths.get(id));
        h_node->paths = paths;
        h_node->makespan = h_node->paths.getMakespan();
        h_node->soc = h_node->paths.getSOC();
        break;
      }
    }
  }
  LAZY_EVAL_TABLE.erase(itr_lb);
  if (LAZY_EVAL_TABLE.empty()) {
    LAZY_EVAL_LB_SOC = -1;
  } else {
    LAZY_EVAL_LB_SOC = LAZY_EVAL_TABLE.begin()->first;
    for (auto itr = LAZY_EVAL_TABLE.begin();
         itr != LAZY_EVAL_TABLE.end(); ++itr) {
      LAZY_EVAL_LB_SOC = std::min(itr->first, LAZY_EVAL_LB_SOC);
    }
  }
  return h_nodes;
}

bool ICBS::findBypass(HighLevelNode_p h_node,
                      const LibCBS::Constraints& constraints)
{
  auto itr = MDDTable.find(h_node->id);
  if (itr == MDDTable.end()) halt("MDD is not found.");
  for (auto c : constraints) {
    Path path = itr->second[c->id]->getPath(c);
    if (path.empty()) continue;
    // format
    while (path.size() - 1 < h_node->makespan) {
      path.push_back(*(path.end()-1));
    }
    int cnum_old = h_node->paths.countConflict(c->id,
                                               h_node->paths.get(c->id));
    int cnum_new = h_node->paths.countConflict(c->id, path);
    if (cnum_old <= cnum_new) continue;

    // helpful bypass found
    h_node->paths.insert(c->id, path);
    h_node->f = h_node->f - cnum_old + cnum_new;
    return true;
  }
  return false;
}

void ICBS::printHelp() {
  std::cout << ICBS::SOLVER_NAME << "\n"
            << "  (no option)"
            << std::endl;
}
