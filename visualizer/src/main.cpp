#include "ofMain.h"
#include "../include/ofApp.hpp"
#include <iostream>
#include <fstream>
#include <regex>
#include "../include/graph.hpp"
#include "../include/util.hpp"
#include "../include/mapfplan.hpp"

void readSetResult(const std::string& result_file, MAPFPlan* plan);
void readSetNode(const std::string& s, Config& config, Grid* G);


int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cout << "Put your result file as the first arg." << std::endl;
    return 0;
  }

  MAPFPlan* solution = new MAPFPlan;
  readSetResult(argv[1], solution);
  ofSetupOpenGL(100, 100, OF_WINDOW);
  ofRunApp(new ofApp(solution));

  return 0;
}

void readSetNode(const std::string& s, Config& config, Grid* G)
{
  if (G == nullptr) halt("graph is not read.");
  std::regex r_pos = std::regex(R"(\((\d+),(\d+)\),)");
  std::smatch m;
  auto iter = s.cbegin();
  while (std::regex_search(iter, s.cend(), m, r_pos)) {
    iter = m[0].second;
    int x = std::stoi(m[1].str());
    int y = std::stoi(m[2].str());
    if (!G->existNode(x, y)) halt("node does not exist");
    config.push_back(G->getNode(x, y));
  }
}

void readSetResult(const std::string& result_file, MAPFPlan* plan)
{
  std::ifstream file(result_file);
  if (!file) halt("file " + result_file + " is not found.");

  std::regex r_scen      = std::regex(R"(scen_file=(.+))");
  std::regex r_map       = std::regex(R"(map_file=(.+))");
  std::regex r_agents    = std::regex(R"(agents=(.+))");
  std::regex r_solver    = std::regex(R"(solver=(.+))");
  std::regex r_solved    = std::regex(R"(solved=(\d))");
  std::regex r_soc       = std::regex(R"(soc=(\d+))");
  std::regex r_makespan  = std::regex(R"(makespan=(\d+))");
  std::regex r_comp_time = std::regex(R"(comp_time=(\d+))");
  std::regex r_starts    = std::regex(R"(starts=(.+))");
  std::regex r_goals     = std::regex(R"(goals=(.+))");
  std::regex r_sol       = std::regex(R"(solution=)");
  std::regex r_config    = std::regex(R"(\d+:(.+))");

  std::string line;
  std::smatch results;
  while (getline(file, line)) {
    // read map
    if (std::regex_match(line, results, r_map)) {
      plan->G = new Grid(results[1].str());
      continue;
    }
    // set agent num
    if (std::regex_match(line, results, r_agents)) {
      plan->num_agents = std::stoi(results[1].str());
      continue;
    }
    // solver
    if (std::regex_match(line, results, r_solver)) {
      plan->solver = results[1].str();
      continue;
    }
    // solved?
    if (std::regex_match(line, results, r_solved)) {
      plan->solved = (bool)std::stoi(results[1].str());
      continue;
    }
    // soc
    if (std::regex_match(line, results, r_soc)) {
      plan->soc = std::stoi(results[1].str());
      continue;
    }
    // makespan
    if (std::regex_match(line, results, r_makespan)) {
      plan->makespan = std::stoi(results[1].str());
      continue;
    }
    // comp_time
    if (std::regex_match(line, results, r_comp_time)) {
      plan->comp_time = std::stoi(results[1].str());
      continue;
    }
    // starts
    if (std::regex_match(line, results, r_starts)) {
      readSetNode(results[1].str(), plan->config_s, plan->G);
      continue;
    }
    // goals
    if (std::regex_match(line, results, r_goals)) {
      readSetNode(results[1].str(), plan->config_g, plan->G);
      continue;
    }
    // solution
    if (std::regex_match(line, results, r_sol)) {
      while (getline(file, line)) {
        if (std::regex_match(line, results, r_config)) {
          Config c;
          readSetNode(results[1].str(), c, plan->G);
          plan->transitions.push_back(c);
        }
      }
    }
  }
}
