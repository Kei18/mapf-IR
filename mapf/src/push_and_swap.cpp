#include "../include/push_and_swap.hpp"

#include <queue>

const std::string PushAndSwap::SOLVER_NAME = "PushAndSwap";

PushAndSwap::PushAndSwap(Problem* _P)
  : Solver(_P)
{
  solver_name = PushAndSwap::SOLVER_NAME;
}

void PushAndSwap::run()
{
  solution.add(P->getConfigStart());

  // occupancy
  std::vector<int> occupied_now(G->getNodesSize(), NIL);
  for (int i = 0; i < P->getNum(); ++i) occupied_now[solution.last(i)->id] = i;

  // pre-processing
  computeNodesWithManyNeighbors();

  Nodes U;

  std::vector<int> ids(P->getNum());
  std::iota(ids.begin(), ids.end(), 0);
  std::sort(ids.begin(), ids.end(),
            [&](int a, int b) { return pathDist(a) > pathDist(b); });

  for (int j = 0; j < P->getNum(); ++j) {
    const int i = ids[j];
    info(" ", "elapsed:", getSolverElapsedTime(),
         ", agent-" + std::to_string(i), "starts planning",
         ", makespan:", solution.getMakespan(),
         ", progress:", j + 1, "/", P->getNum());
    while (solution.last(i) != P->getGoal(i)) {
      if (!push(solution, i, U, occupied_now)) {
        info("   ", "swap required, timestep=", solution.getMakespan());
        if (!swap(solution, i, U, occupied_now)) {
          return;  // failed
        }
      }
    }
    U.push_back(solution.last(i));

    // check limitation
    if (overCompTime() || solution.getMakespan() > max_timestep) return;
  }
  solved = true;
}

bool PushAndSwap::push(Plan& plan, const int id, Nodes& U, std::vector<int>& occupied_now)
{
  if (plan.last(id) == P->getGoal(id)) return true;

  // create shortest path
  Path p_star = getShortestPath(id, plan.last(id), occupied_now);
  p_star.erase(p_star.begin());
  if (p_star.empty()) return true;  // for safety

  Node* v = p_star[0];
  while (plan.last(id) != P->getGoal(id)) {
    while (occupied_now[v->id] == NIL) {
      updatePlan(id, v, plan, occupied_now);
      p_star.erase(p_star.begin());
      if (p_star.empty()) return true;
      v = p_star[0];
    }
    Nodes obs = U;
    obs.push_back(plan.last(id));
    if (!pushTowardEmptyNode(v, plan, occupied_now, obs)) return false;
  }

  return true;
}

bool PushAndSwap::swap(Plan& plan, const int r, Nodes& U, std::vector<int>& occupied_now)
{
  auto p_star = getShortestPath(r, plan.last(r), occupied_now);
  if (p_star.size() <= 1) return true;  // for safety
  const int s = occupied_now[p_star[1]->id];
  if (s == NIL) return true;  // for safety

  const Config c_before = plan.last();

  bool succcess = false;
  Nodes swap_verticies = nodes_with_many_neighbors;
  // sort, make swap operation easy
  {
    Node* v = p_star[0];
    std::sort(swap_verticies.begin(), swap_verticies.end(),
              [v] (Node* a, Node* b) { return v->manhattanDist(a) < v->manhattanDist(b); });
  }

  Plan tmp_plan;
  while (!swap_verticies.empty() && !succcess) {
    Node* v = swap_verticies[0];
    swap_verticies.erase(swap_verticies.begin());
    auto p = G->getPath(plan.last(r), v, false);  // no cache
    tmp_plan.clear();
    auto tmp_occupied_now = occupied_now;
    tmp_plan.add(plan.last());
    if (v == plan.last(r) || multiPush(tmp_plan, r, s, p, tmp_occupied_now)) {
      if (clear(tmp_plan, v, r, s, tmp_occupied_now)) succcess = true;
    }
  }
  if (!succcess) return false;

  // update occupancy
  for (int i = 0; i < P->getNum(); ++i) occupied_now[plan.last(i)->id] = NIL;
  // update plan
  plan += tmp_plan;
  // update occupancy
  for (int i = 0; i < P->getNum(); ++i) occupied_now[plan.last(i)->id] = i;

  executeSwap(plan, r, s, occupied_now);
  Plan reversed_tmp_plan;
  {
    const int makespan = tmp_plan.getMakespan();
    for (int t = makespan; t >=0; --t) {
      auto c = tmp_plan.get(t);
      auto tmp = c[r];
      c[r] = c[s];
      c[s] = tmp;
      reversed_tmp_plan.add(c);
    }
  }
  // update occupancy
  for (int i = 0; i < P->getNum(); ++i) occupied_now[plan.last(i)->id] = NIL;
  // update plan
  plan += reversed_tmp_plan;
  // update occupancy
  for (int i = 0; i < P->getNum(); ++i) occupied_now[plan.last(i)->id] = i;

  // validation
  const Config c_after = plan.last();
  for (int i = 0; i < P->getNum(); ++i) {
    if ((i == s && c_after[s] != c_before[r]) ||
        (i == r && c_after[r] != c_before[s]) ||
        (i != s && i != r && c_after[i] != c_before[i])) {
      halt("invalid swap operation");
    }
  }
  info("   ", "agent-" + std::to_string(r) + ", " + std::to_string(s) + " swap locations "
       + std::to_string(c_before[r]->id) + ", " + std::to_string(c_before[s]->id));

  if (inArray(P->getGoal(s), U)) return resolve(plan, r, s, U, occupied_now);

  return true;
}

bool PushAndSwap::resolve
(Plan& plan, const int r, const int s, Nodes& U, std::vector<int>& occupied_now)
{
  info("      resolve operation for", r);
  // error check
  if (!inArray(plan.last(r), plan.last(s)->neighbor)) halt("invalid resolve operation");

  Node* ideal_loc_s = plan.last(r);

  while (occupied_now[ideal_loc_s->id] != NIL) {
    const int _r = occupied_now[ideal_loc_s->id];
    if (_r == NIL) break;
    // case 1. push
    auto p = getShortestPath(_r, ideal_loc_s, occupied_now);
    // required swap
    if (p.empty()) halt("never-happen situations");
    // _r tries to move p[1]
    if (occupied_now[p[1]->id] != NIL) {
      Nodes obs = U;
      obs.push_back(plan.last(s));
      obs.push_back(plan.last(_r));
      if (!pushTowardEmptyNode(p[1], plan, occupied_now, obs)) {
        // swap required
        info("        recursive swap is called for", r);
        if (!swap(plan, _r, U, occupied_now)) return false;
      } else {
        // success
        // r moves to v
        updatePlan(_r, p[1], plan, occupied_now);
      }
    } else {
      // success
      // r moves to v
      updatePlan(_r, p[1], plan, occupied_now);
    }
  }

  // s moves to its goal
  updatePlan(s, ideal_loc_s, plan, occupied_now);
  return true;
}

bool PushAndSwap::multiPush
(Plan& plan, const int r, const int s, const Path& p, std::vector<int>& occupied_now)
{
  const int p_size = p.size();
  if (p_size == 0) halt("path is empty");

  // case 1
  if (plan.last(s) != p[1]) {
    for (int i = 1; i < p_size; ++i) {
      // r tries to reserve v
      if (occupied_now[p[i]->id] != NIL) {
        if (!pushTowardEmptyNode(p[i], plan, occupied_now, { plan.last(s) })) return false;
      }
      updatePlan(r, p[i], plan, occupied_now);
      // s moves to the last location of r;
      updatePlan(s, p[i-1], plan, occupied_now);
    }

    // case 2
  } else {
    for (int i = 2; i < p_size; ++i) {
      auto v = p[i];
      // s tries to reserve v
      if (occupied_now[v->id] != NIL) {
        if (!pushTowardEmptyNode(v, plan, occupied_now, { plan.last(r) })) return false;
      }
      updatePlan(s, p[i], plan, occupied_now);
      // r moves to the last location of s;
      updatePlan(r, p[i-1], plan, occupied_now);
    }
    // r moves to last loc of p
    if (!pushTowardEmptyNode(p[p_size-1], plan, occupied_now, { plan.last(r) })) return false;
    updatePlan(r, p[p_size-1], plan, occupied_now);
  }

  return true;
}

void PushAndSwap::checkConsistency(Plan& plan, std::vector<int>& occupied_now)
{
  auto c = plan.last();
  for (int i = 0; i < P->getNum(); ++i) {
    if (occupied_now[c[i]->id] != i) halt("check consistency");
  }
}

bool PushAndSwap::clear(Plan& plan, Node* v, const int r, const int s, std::vector<int>& occupied_now)
{
  info("      clear operation for", r, "at v=", v->id);
  auto getUnoccupiedNodes = [&] () {
    Nodes nodes;
    for (auto u : v->neighbor) {
      if (occupied_now[u->id] == NIL) nodes.push_back(u);
    }
    return nodes;
  };

  // trivial case
  Nodes unoccupied_nodes = getUnoccupiedNodes();
  if (unoccupied_nodes.size() >= 2) return true;

  // case 1
  for (auto u : v->neighbor) {
    unoccupied_nodes = getUnoccupiedNodes();
    if (inArray(u, unoccupied_nodes)) continue;
    auto obs = unoccupied_nodes;
    obs.push_back(plan.last(r));
    obs.push_back(plan.last(s));
    checkConsistency(plan, occupied_now);
    if (pushTowardEmptyNode(u, plan, occupied_now, obs)) {
      if (getUnoccupiedNodes().size() >= 2) return true;
    }
  }

  // case 2
  auto last_loc_s = plan.last(s);
  for (auto u : v->neighbor) {
    unoccupied_nodes = getUnoccupiedNodes();
    if (inArray(u, unoccupied_nodes)) continue;
    const int disturbing_agent = occupied_now[u->id];
    for (auto w : unoccupied_nodes) {
      // move s to another loc
      auto obs = getUnoccupiedNodes();
      obs.push_back(u);
      obs.push_back(v);
      obs.push_back(w);
      if (pushTowardEmptyNode(last_loc_s, plan, occupied_now, obs)) {
        // move r to last_loc_s
        updatePlan(r, last_loc_s, plan, occupied_now);
        // move disturbing_agent to v
        updatePlan(disturbing_agent, v, plan, occupied_now);
        // move disturbing_agent to w
        updatePlan(disturbing_agent, w, plan, occupied_now);
        // move r to v
        updatePlan(r, v, plan, occupied_now);
        // move s to last_loc_s
        updatePlan(s, last_loc_s, plan, occupied_now);
        // move disturbing_agent to another loc
        auto obs2 = getUnoccupiedNodes();
        obs2.push_back(v);
        obs2.push_back(last_loc_s);
        if (pushTowardEmptyNode(w, plan, occupied_now, obs2)) {
          if (getUnoccupiedNodes().size() >= 2) return true;
          break;
        }
      }
    }
  }

  return false;
}

void PushAndSwap::executeSwap(Plan& plan, const int r, const int s, std::vector<int>& occupied_now)
{
  // identify empty loc
  Node* empty1 = nullptr;
  Node* empty2 = nullptr;
  Node* v = plan.last(r);
  Node* last_loc_s = plan.last(s);
  for (auto u : v->neighbor) {
    if (occupied_now[u->id] == NIL) {
      if (empty1 == nullptr) {
        empty1 = u;
      } else if (empty2 == nullptr) {
        empty2 = u;
        break;
      }
    }
  }

  // error check
  if (empty2 == nullptr) halt("execute swap, failed to clear");

  updatePlan(r, empty1, plan, occupied_now);
  updatePlan(s, v, plan, occupied_now);
  updatePlan(s, empty2, plan, occupied_now);
  updatePlan(r, v, plan, occupied_now);
  updatePlan(r, last_loc_s, plan, occupied_now);
  updatePlan(s, v, plan, occupied_now);
}

void PushAndSwap::updatePlan(const int id, Node* next_node, Plan& plan, std::vector<int>& occupied_now)
{
  // error check
  if (occupied_now[plan.last(id)->id] != id) halt("invalid update");
  if (occupied_now[next_node->id] != NIL) halt("vertex conflict");

  // update occupancy
  occupied_now[plan.last(id)->id] = NIL;
  occupied_now[next_node->id] = id;
  // update plan
  Config c = plan.last();
  c[id] = next_node;
  plan.add(c);
}

bool PushAndSwap::pushTowardEmptyNode
(Node* v_current, Plan& plan, std::vector<int>& occupied_now, const Nodes& obs)
{
  Node* v_empty = getNearestEmptyNode(v_current, occupied_now, obs);
  if (v_empty == nullptr) return false;
  auto p = G->getPath(v_current, v_empty, obs);
  if (p.empty()) return false;

  for (int i = p.size()-1; i > 0; --i) {
    if (occupied_now[p[i-1]->id] == NIL) halt("node must be occupied");
    updatePlan(occupied_now[p[i-1]->id], p[i], plan, occupied_now);
  }
  return true;
}

Path PushAndSwap::getShortestPath(const int id, Node* s, std::vector<int>& occupied_now)
{
  Nodes p = { s };
  Node* g = P->getGoal(id);
  while (*(p.end()-1) != g) {
    Node* v = *(p.end()-1);
    p.push_back
      (*std::min_element(v->neighbor.begin(), v->neighbor.end(),
                         [&](Node* a, Node* b) {
                           // path distance
                           int c_a = pathDist(id, a);
                           int c_b = pathDist(id, b);
                           if (c_a != c_b) return c_a < c_b;
                           // occupancy
                           int o_a = (int)(occupied_now[a->id] != NIL);
                           int o_b = (int)(occupied_now[b->id] != NIL);
                           if (o_a != o_b) return o_a < o_b;
                           return false; }));
  }
  return p;
}

Node* PushAndSwap::getNearestEmptyNode(Node* v, std::vector<int>& occupied_now, const Nodes& obs)
{
  const int id = occupied_now[v->id];
  Node* v_empty = nullptr;
  std::queue<int> OPEN;
  std::vector<bool> CLOSE(G->getNodesSize(), false);
  for (auto v : obs) CLOSE[v->id] = true;
  OPEN.push(v->id);
  while (!OPEN.empty()) {
    int i = OPEN.front();
    OPEN.pop();
    if (CLOSE[i]) continue;
    CLOSE[i] = true;
    Node* u = G->getNode(i);
    if (occupied_now[i] == NIL) {
      v_empty = u;
      break;
    }
    Nodes C;
    for (auto w : u->neighbor) {
      if (CLOSE[w->id]) continue;
      C.push_back(w);
    }
    std::sort(C.begin(), C.end(),
              [&](Node* a, Node* b) { return pathDist(id, a) < pathDist(id, b); });
    for (auto w : C) OPEN.push(w->id);
  }

  return v_empty;
}

// get all vertices of degree >= 3 on G
void PushAndSwap::computeNodesWithManyNeighbors()
{
  nodes_with_many_neighbors.clear();
  auto V = G->getV();
  for (auto v : V)
    if (v->getDegree() >= 3) nodes_with_many_neighbors.push_back(v);
}

void PushAndSwap::printHelp()
{
  std::cout << PushAndSwap::SOLVER_NAME << "\n"
            << "  (no option)"
            << std::endl;
}
