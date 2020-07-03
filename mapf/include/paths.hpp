#pragma once
#include "problem.hpp"

struct Paths {
private:
  std::vector<Path> paths;
  int makespan;

public:
  Paths() {}
  Paths(int num_agents);
  ~Paths();

  Path get(int i) const;
  Node* get(int i, int t) const;
  bool empty() const;
  void insert(int i, const Path& path);
  int size() const;
  void operator+=(const Paths& other);
  int getMaxLengthPaths() const;
  int getMakespan() const;
  int costOfPath(int i) const;
  int getSOC() const;
  void format();
  void shrink();
  bool conflicted(int i, int j, int t) const;
  int countConflict() const;
  int countConflict(const std::vector<int>& sample) const;
  int countConflict(int id, const Path& path) const;
};
