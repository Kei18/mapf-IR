#include "../include/paths.hpp"


Paths::Paths(int num_agents)
{
  std::vector<Path> tmp(num_agents, Path(0));
  paths = tmp;
}

Paths::~Paths()
{
  for (auto p : paths) p.clear();
  paths.clear();
}

Path Paths::get(int i) const
{
  if (!(0 <= i && i < paths.size())) halt("invalid index.");
  return paths[i];
}

Node* Paths::get(int i, int t) const
{
  if (!(0 <= i && i < paths.size()) ||
      !(0 <= t && t <= makespan)) {
    halt("invalid index, i=" + std::to_string(i)
         + ", t=" + std::to_string(t));
  }
  return paths[i][t];
}

bool Paths::empty() const
{
  return paths.empty();
}

void Paths::insert(int i, const Path& path)
{
  if (!(0 <= i && i < paths.size())) halt("invalid index.");
  if (path.empty()) halt("path must not be empty");
  int old_len = paths[i].size();
  paths[i] = path;
  format();
  if (paths[i].size() < old_len) shrink();
  makespan = getMaxLengthPaths();  // update makespan
}

int Paths::size() const
{
  return paths.size();
}

void Paths::operator+=(const Paths& other)
{
  if (other.empty()) return;
  if (paths.size() != other.paths.size()) halt("invalid operation.");
  if (makespan == 0) {// empty
    for (int i = 0; i < paths.size(); ++i) insert(i, other.get(i));
  } else {
    std::vector<Path> new_paths(paths.size());
    for (int i = 0; i < paths.size(); ++i) {
      if (paths[i].empty()) halt("invalid operation");
      if (*(paths[i].end()-1) != other.paths[i][0])
        halt("invalid operation");
      Path tmp;
      for (int t = 0; t <= getMakespan(); ++t)
        tmp.push_back(get(i, t));
      for (int t = 1; t <= other.getMakespan(); ++t) {
        tmp.push_back(other.get(i, t));
      }
      new_paths[i] = tmp;
    }
    for (int i = 0; i < paths.size(); ++i) insert(i, new_paths[i]);
  }
}

int Paths::getMaxLengthPaths() const
{
  int max_val = 0;
  for (auto p : paths) {
    if (p.empty()) continue;
    max_val = (p.size() - 1 > max_val) ? p.size() - 1 : max_val;
  }
  return max_val;
}

int Paths::getMakespan() const
{
  return makespan;
}

int Paths::costOfPath(int i) const
{
  if (!(0 <= i && i < paths.size())) halt("invalid index.");
  int c = paths[i].size();
  auto itr = paths[i].end() - 1;
  Node* g = *itr;
  while (*itr == g) {
    --c;
    if (c <= 0) break;
    --itr;
  }
  return c;
}

int Paths::getSOC() const
{
  int soc = 0;
  for (int i = 0; i < paths.size(); ++i) soc += costOfPath(i);
  return soc;
}

void Paths::format()
{
  int len = getMaxLengthPaths();
  for (int i = 0; i < paths.size(); ++i) {
    if (paths[i].empty()) continue;
    while (paths[i].size()-1 != len) {
      paths[i].push_back(*(paths[i].end()-1));
    }
  }
}

void Paths::shrink()
{
  while (true) {
    bool shrinkable = true;
    for (auto p: paths) {
      if (p.size() <= 1 || *(p.end()-1) != *(p.end()-2)) {
        shrinkable = false;
        break;
      }
    }
    if (!shrinkable) break;
    for (int i = 0; i < paths.size(); ++i) {
      paths[i].resize(paths[i].size()-1);
    }
  }
}

int Paths::countConflict() const
{
  int cnt = 0;
  int num_agents = size();
  int makespan = getMakespan();
  for (int i = 0; i < num_agents; ++i) {
    for (int j = i + 1; j < num_agents; ++j) {
      for (int t = 1; t < makespan; ++t) {
        // vertex conflict
        if (get(i, t) == get(j, t)) ++cnt;
        // swap conflict
        if (get(i, t) == get(j, t-1) &&
            get(j, t) == get(i, t-1)) ++cnt;
      }
    }
  }
  return cnt;
}

int Paths::countConflict(int id, const Path& path) const
{
  int cnt = 0;
  int makespan = getMakespan();
  int num_agents = size();
  for (int i = 0; i < num_agents; ++i) {
    if (i == id) continue;
    for (int t = 1; t < path.size(); ++t) {
      if (t > makespan) {
        if (path[t] == get(i, makespan)) {
          ++cnt;
          break;
        }
        continue;
      }
      // vertex conflict
      if (get(i, t) == path[t]) ++cnt;
      // swap conflict
      if (get(i, t) == path[t-1] && get(i, t-1) == path[t]) ++cnt;
    }
  }
  return cnt;
}
