#include "../include/icbs.hpp"

const std::string ICBS::SOLVER_NAME = "ICBS";

ICBS::ICBS(Problem* _P) : CBS(_P)
{
  solver_name = ICBS::SOLVER_NAME;
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

    info(" ",
         "elapsed:", getSolverElapsedTime(),
         ", explored_node_num:", iteration,
         ", nodes_num:", h_node_num,
         ", conflicts:", n->f,
         ", constraints:", n->constraints.size(),
         ", soc:", n->soc);

    // check conflict
    LibCBS::Constraints constraints =
      LibCBS::getPrioritizedConflict(n->paths, MDDTable[n->id]);
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
      LibCBS::Constraints new_constraints = n->constraints;
      new_constraints.push_back(c);
      HighLevelNode_p m(new HighLevelNode{
          h_node_num,
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
      ++h_node_num;
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
    int c = std::max(mdd.c, last_constraint->t);
    while (true) {
      ++c;
      LibCBS::MDD_p new_mdd(new LibCBS::MDD(c, id, P, h_node->constraints));
      if (new_mdd->valid) {
        MDDTable[h_node->id][id] = new_mdd;
        return new_mdd->getPath();
      }
    }
  }
  return {};
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
