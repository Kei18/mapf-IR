#pragma once
#include "problem.hpp"

struct Plan {
private:
  Configs configs;

public:
  ~Plan();
  Config get(int t) const;
  Node* get(int t, int i) const;
  Config last() const;
  void add(const Config& c);
  bool empty() const;
  int size() const;
  int getMakespan() const;
  int getSOC() const;
  Plan operator+(const Plan& other) const;
  void operator+=(const Plan& other);
  int getTimestep(const Config& c) const;
  Plan getPartialPlan(const Config& config_i,
                      const Config& config_j) const;
  Plan getPartialPlan(int i, int j) const;
  bool validate(Problem* P) const;
};

using Plans = std::vector<Plan>;
