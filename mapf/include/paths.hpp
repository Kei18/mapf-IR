#pragma once
#include "problem.hpp"

/*
 * array of path
 */

struct Paths {
private:
  std::vector<Path> paths;  // main
  int makespan;

public:
  Paths() {}
  Paths(int num_agents);
  ~Paths() {}

  // agent -> path
  Path get(int i) const;

  // agent, timestep -> location
  Node* get(int i, int t) const;

  // whether paths are empty
  bool empty() const;

  // whether a path of a_i is empty
  bool empty(int i) const;

  // insert new path
  void insert(int i, const Path& path);

  // clear
  void clear(int i);

  // return paths.size
  int size() const;

  // joint with other paths
  void operator+=(const Paths& other);

  // get maximum length
  int getMaxLengthPaths() const;

  // makespan
  int getMakespan() const;

  // cost of paths[i]
  int costOfPath(int i) const;

  // sum of cost
  int getSOC() const;

  // formatting
  void format();
  void shrink();

  // =========================
  // for CBS
  // check conflicted
  bool conflicted(int i, int j, int t) const;

  // count conflicts for all
  int countConflict() const;

  // count conflict within a subset of agents
  int countConflict(const std::vector<int>& sample) const;

  // count conflict with one path
  int countConflict(int id, const Path& path) const;
};
