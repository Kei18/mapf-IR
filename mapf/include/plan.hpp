#pragma once
#include "problem.hpp"

/*
 * array of configurations
 */

struct Plan {
private:
  Configs configs;  // main

public:
  ~Plan() {}

  // timestep -> configuration
  Config get(int t) const;

  // timestep, agent -> location
  Node* get(int t, int i) const;

  // last configuration
  Config last() const;

  // add new configuration to the last
  void add(const Config& c);

  // whether configs are empty
  bool empty() const;

  // configs.size
  int size() const;

  // size - 1
  int getMakespan() const;

  // sum of cost
  int getSOC() const;

  // join with other plan
  Plan operator+(const Plan& other) const;
  void operator+=(const Plan& other);

  // check the plan is valid or not
  bool validate(Problem* P) const;
};

using Plans = std::vector<Plan>;
