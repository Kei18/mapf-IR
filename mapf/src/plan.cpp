#include "../include/plan.hpp"

Config Plan::get(int t) const
{
  if (!(0 <= t && t < configs.size())) halt("invalid timestep");
  return configs[t];
}

Node* Plan::get(int t, int i) const
{
  if (empty()) halt("invalid operation");
  if (!(0 <= t && t < configs.size())) halt("invalid timestep");
  if (!(0 <= i && i < configs[0].size())) halt("invalid agent id");
  return configs[t][i];
}

Config Plan::last() const
{
  if (empty()) halt("invalid operation");
  return configs[configs.size()-1];
}

void Plan::add(const Config& c)
{
  if (!configs.empty() && configs.at(0).size() != c.size()) {
    halt("invalid operation");
  }
  configs.push_back(c);
}

bool Plan::empty() const
{
  return configs.empty();
}

int Plan::size() const
{
  return configs.size();
}

int Plan::getMakespan() const
{
  return configs.size() - 1;
}

int Plan::getSOC() const
{
  int makespan = getMakespan();
  if (makespan <= 0) return 0;
  int num_agents = configs[0].size();
  int soc = 0;
  for (int i = 0; i < num_agents; ++i) {
    int c = makespan;
    Node* g = configs[makespan][i];
    while (configs[c-1][i] == g) {
      --c;
      if (c <= 0) break;
    }
    soc += c;
  }
  return soc;
}

Plan Plan::operator+(const Plan& other) const
{
  // check validity
  Config c1 = last();
  Config c2 = other.get(0);
  if (c1.size() != c2.size()) halt("invalid operation");
  for (int i = 0; i < c1.size(); ++i) {
    if (c1[i] != c2[i]) halt("invalid operation.");
  }
  // merge
  Plan new_plan;
  new_plan.configs = configs;
  for (int t = 1; t < other.size(); ++t) new_plan.add(other.get(t));
  return new_plan;
}

void Plan::operator+=(const Plan& other)
{
  if (configs.empty()) {
    configs = other.configs;
    return;
  }
  // check validity
  if (!sameConfig(last(), other.get(0))) halt("invalid operation");
  // merge
  for (int t = 1; t < other.size(); ++t) add(other.get(t));
}

bool Plan::validate(Problem* P) const
{
  if (configs.empty()) return false;

  // start and goal
  if (!sameConfig(P->getConfigStart(), get(0))) return false;
  if (!sameConfig(P->getConfigGoal(), get(getMakespan()))) return false;

  // check conflicts and continuity
  int num_agents = get(0).size();
  for (int t = 1; t < getMakespan(); ++t) {
    if (get(t).size() != num_agents) return false;
    for (int i = 0; i < num_agents; ++i) {
      Node* v_i_t = get(t, i);
      Node* v_i_t_1 = get(t-1, i);
      Nodes cands = v_i_t_1->neighbor;
      cands.push_back(v_i_t_1);
      if (!inArray(v_i_t, cands)) return false;  // invalid move
      // see conflicts
      for (int j = i+1; j < num_agents; ++j) {
        Node* v_j_t = get(t, j);
        Node* v_j_t_1 = get(t-1, j);
        if (v_i_t == v_j_t) return false;
        if (v_i_t == v_j_t_1 && v_i_t_1 == v_j_t) return false;
      }
    }
  }
  return true;
}
