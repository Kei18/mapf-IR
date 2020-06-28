#include "../include/icbs.hpp"

const std::string ICBS::SOLVER_NAME = "ICBS";
std::unordered_map<std::string, MDD_p> MDD::PURE_MDD_TABLE;


ICBS::ICBS(Problem* _P) : CBS(_P)
{
  solver_name = ICBS::SOLVER_NAME;
}

ICBS::~ICBS() {}

void ICBS::solve()
{
  start();
  // high-level search

  // set objective function
  CompareHighLevelNodes compare = getObjective();

  // OPEN
  std::priority_queue<HighLevelNode*,
                      std::vector<HighLevelNode*>,
                      decltype(compare)> HighLevelTree(compare);

  HighLevelNode* n = new HighLevelNode;
  setInitialHighLevelNode(n);
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
    Conflict::Constraints constraints = getPrioritizedConflict(n);
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
      Conflict::Constraints new_constraints = n->constraints;
      new_constraints.push_back(c);
      HighLevelNode* m = new HighLevelNode
        { h_node_num,
          n->paths,
          new_constraints,
          n->makespan,
          n->soc,
          n->f,
          true };
      MDDTable[m->id] = MDDTable[n->id];  // copy MDD
      invoke(m, c->id);
      if (!m->valid) continue;
      HighLevelTree.push(m);
      ++h_node_num;
    }
  }

  if (solved) solution = pathsToPlan(n->paths);
  end();
}

void ICBS::setInitialHighLevelNode(HighLevelNode* n)
{
  Paths paths(P->getNum());
  MDDs mdds;
  for (int i = 0; i < P->getNum(); ++i) {
    // try to avoid goal locations
    Path path = getInitialPath(i);
    MDD tmp = MDD(path.size() - 1, i, P, {});
    MDD_p mdd = std::make_shared<MDD>(tmp);
    paths.insert(i, path);
    mdds.push_back(mdd);
  }
  n->id = 0;
  n->paths = paths;
  n->constraints = {};  // constraints
  n->makespan = paths.getMakespan();
  n->soc = paths.getSOC();
  n->f = Conflict::countConflict(paths);
  n->valid = true;  // valid

  MDDTable[n->id] = mdds;
}

void ICBS::invoke(HighLevelNode* h_node, int id)
{
  Path path;
  MDD mdd = *(MDDTable[h_node->id][id]);
  Conflict::Constraint* last_constraint = *(h_node->constraints.end()-1);
  mdd.update({ last_constraint });  // check only last
  if (mdd.valid) {  // use mdd as much as possible
    path = mdd.getPath();
    if (path.empty()) halt("failed to Cal MDD");
    MDDTable[h_node->id][id] = std::make_shared<MDD>(mdd);  // update table
  } else {
    int c = std::max(mdd.c, last_constraint->t);
    while (true) {
      ++c;
      MDD tmp_mdd = MDD(c, id, P, h_node->constraints);
      if (tmp_mdd.valid) {
        path = tmp_mdd.getPath();
        MDD_p new_mdd = std::make_shared<MDD>(tmp_mdd);
        MDDTable[h_node->id][id] = new_mdd;
        break;
      }
    }
  }

  Paths paths = h_node->paths;
  paths.insert(id, path);
  // update conflicts counts
  h_node->f = h_node->f
    - Conflict::countConflict(id, h_node->paths.get(id), h_node->paths)
    + Conflict::countConflict(id, paths.get(id), h_node->paths);
  h_node->paths = paths;
  h_node->makespan = h_node->paths.getMakespan();
  h_node->soc = h_node->paths.getSOC();
}

bool ICBS::findBypass(HighLevelNode* h_node,
                      const Conflict::Constraints& constraints)
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
    int conflicts_old = Conflict::countConflict(c->id,
                                                h_node->paths.get(c->id),
                                                h_node->paths);
    int conflicts_new = Conflict::countConflict(c->id, path, h_node->paths);
    if (conflicts_old <= conflicts_new) continue;

    // helpful bypass found
    h_node->paths.insert(c->id, path);
    h_node->f = h_node->f - conflicts_old + conflicts_new;
    return true;
  }
  return false;
}

Conflict::Constraints ICBS::getPrioritizedConflict
(HighLevelNode* const h_node)
{
  Conflict::Constraints semi_cardinal_constraints = {};
  Conflict::Constraints non_cardinal_constraints = {};
  Paths paths = h_node->paths;
  MDDs mdds = MDDTable[h_node->id];
  for (int t = 1; t <= paths.getMakespan(); ++t) {
    for (int i = 0; i < P->getNum(); ++i) {
      for (int j = i + 1; j < P->getNum(); ++j) {
        int c_i = mdds[i]->c;
        int c_j = mdds[j]->c;
        int w_i = (t <= c_i) ? mdds[i]->body[t].size() : 0;
        int w_j = (t <= c_j) ? mdds[j]->body[t].size() : 0;
        // vertex conflict
        if (paths.get(i, t) == paths.get(j, t)) {
          Conflict::Constraint* constraint_i = new Conflict::Constraint
            { i, t, paths.get(i, t), nullptr };
          Conflict::Constraint* constraint_j = new Conflict::Constraint
            { j, t, paths.get(j, t), nullptr };
          // cardinal conflicts
          if ((t <= c_i && w_i == 1 && t <= c_j && w_j == 1) ||
              (t > c_i && w_j == 1) || (t > c_j && w_i == 1)) {
            return { constraint_i, constraint_j };
          }
          // semi-cardinal conflicts
          if (semi_cardinal_constraints.empty() &&
              (t > c_i || t > c_j || w_i == 1 || w_j == 1)) {
            semi_cardinal_constraints.push_back(constraint_i);
            semi_cardinal_constraints.push_back(constraint_j);

          } else if (non_cardinal_constraints.empty()) {
            non_cardinal_constraints.push_back(constraint_i);
            non_cardinal_constraints.push_back(constraint_j);
          }
        }
        // swap conflict
        if (paths.get(i, t) == paths.get(j, t-1) &&
            paths.get(j, t) == paths.get(i, t-1)) {
          Conflict::Constraint* constraint_i = new Conflict::Constraint
            { i, t, paths.get(i, t), paths.get(i, t-1) };
          Conflict::Constraint* constraint_j = new Conflict::Constraint
            { j, t, paths.get(j, t), paths.get(j, t-1) };
          // cardinal conflicts
          if ((t <= c_i && w_i == 1 &&
               mdds[i]->body[t][0]->prev.size() == 1) &&
              (t <= c_j && w_j == 1 &&
               mdds[j]->body[t][0]->prev.size() == 1)) {
            return { constraint_i, constraint_j };
          }
          // semi-cardinal conflicts
          if (semi_cardinal_constraints.empty() &&
              (t > c_i || t > c_j ||
               (w_i == 1 &&
                mdds[i]->body[t][0]->prev.size() == 1) ||
               (w_j == 1 &&
                mdds[j]->body[t][0]->prev.size() == 1))) {
            semi_cardinal_constraints.push_back(constraint_i);
            semi_cardinal_constraints.push_back(constraint_j);
          } else if (non_cardinal_constraints.empty()) {
            non_cardinal_constraints.push_back(constraint_i);
            non_cardinal_constraints.push_back(constraint_j);
          }
        }
      }
    }
  }
  if (!semi_cardinal_constraints.empty()) {
    return semi_cardinal_constraints;
  } else if (!non_cardinal_constraints.empty()) {
    return non_cardinal_constraints;
  }
  return {};
}

void ICBS::printHelp() {
  std::cout << ICBS::SOLVER_NAME << "\n"
            << "  (no option)"
            << std::endl;
}
