#include "../include/problem.hpp"
#include <fstream>
#include <regex>
#include "../include/util.hpp"


Problem::Problem(const std::string& _instance) : instance(_instance)
{
  std::ifstream file(instance);
  if (!file) halt("file " + instance + " is not found.");

  std::string line;
  std::smatch results;
  std::regex r_comment = std::regex(R"(#.+)");
  std::regex r_map = std::regex(R"(map_file=(.+))");
  std::regex r_agents = std::regex(R"(agents=(\d+))");
  std::regex r_seed = std::regex(R"(seed=(\d+))");
  std::regex r_random_problem = std::regex(R"(random_problem=(\d+))");
  std::regex r_scen = std::regex(R"(scen_file=(.+))");
  std::regex r_max_timestep = std::regex(R"(max_timestep=(\d+))");
  std::regex r_max_comp_time = std::regex(R"(max_comp_time=(\d+))");
  std::regex r_sg = std::regex(R"((\d+),(\d+),(\d+),(\d+))");

  bool read_scen = true;
  while (getline(file, line)) {
    // comment
    if (std::regex_match(line, results, r_comment)) {
      continue;
    }
    // read map
    if (std::regex_match(line, results, r_map)) {
      G = new Grid(results[1].str());
      continue;
    }
    // set agent num
    if (std::regex_match(line, results, r_agents)) {
      num_agents = std::stoi(results[1].str());
      continue;
    }
    // set random seed
    if (std::regex_match(line, results, r_seed)) {
      MT = new std::mt19937(std::stoi(results[1].str()));
      continue;
    }
    // skip reading initial/goal nodes
    if (std::regex_match(line, results, r_random_problem)) {
      if (std::stoi(results[1].str())) {
        read_scen = false;
        config_s.clear();
        config_g.clear();
      }
      continue;
    }
    if (std::regex_match(line, results, r_scen)
        && read_scen && num_agents > 0) {
      setScenStartsGoals(results[1].str());
      read_scen = false;
      continue;
    }
    // set max timestep
    if (std::regex_match(line, results, r_max_timestep)) {
      max_timestep = std::stoi(results[1].str());
      continue;
    }
    // set max computation time
    if (std::regex_match(line, results, r_max_comp_time)) {
      max_comp_time = std::stoi(results[1].str());
      continue;
    }
    // read initial/goal nodes
    if (std::regex_match(line, results, r_sg) && read_scen) {
      if (config_s.size() >= num_agents) continue;
      int x_s = std::stoi(results[1].str());
      int y_s = std::stoi(results[2].str());
      int x_g = std::stoi(results[3].str());
      int y_g = std::stoi(results[4].str());
      if (!G->nodeExist(x_s, y_s)) {
        halt("start node ("
             + std::to_string(x_s) + ", " + std::to_string(y_s)
             + ") does not exist, invalid scenario file.");
      }
      if (!G->nodeExist(x_g, y_g)) {
        halt("goal node ("
             + std::to_string(x_g) + ", " + std::to_string(y_g)
             + ") does not exist, invalid scenario file.");
      }

      Node* s = G->getNode(x_s, y_s);
      Node* g = G->getNode(x_g, y_g);
      config_s.push_back(s);
      config_g.push_back(g);
    }
  }

  // set default value
  if (MT == nullptr) MT = new std::mt19937(DEFAULT_SEED);
  if (max_timestep == 0) max_timestep = DEFAULT_MAX_TIMESTEP;
  if (max_comp_time == 0) max_comp_time = DEFAULT_MAX_COMP_TIME;

  // check starts/goals
  if (num_agents <= 0) halt("invalid number of agents");
  if (!config_s.empty() && num_agents > config_s.size()) {
    warn("given starts/goals are not sufficient\nrandomly create instances");
  }
  if (num_agents > config_s.size()) {
    setRandomStartsGoals();
  }
}

Problem::Problem(Problem* P,
                 Config _config_s,
                 Config _config_g,
                 int _max_comp_time,
                 int _max_timestep)
  : G(P->getG()),
    MT(P->getMT()),
    num_agents(P->getNum()),
    config_s(_config_s),
    config_g(_config_g),
    max_comp_time(_max_comp_time),
    max_timestep(_max_timestep)
{
}

Problem::~Problem() {
  config_s.clear();
  config_g.clear();
}

Node* Problem::getStart(int i) const {
  if (!(0 <= i && i < config_s.size())) halt("invalid index");
  return config_s[i];
}

Node* Problem::getGoal(int i) const {
  if (!(0 <= i && i < config_g.size())) halt("invalid index");
  return config_g[i];
}

void Problem::setRandomStartsGoals () {
  config_s.clear();
  config_g.clear();
  int n = G->getWidth() * G->getHeight();

  // set starts
  std::vector<int> starts(n);
  std::iota(starts.begin(), starts.end(), 0);
  std::shuffle(starts.begin(), starts.end(), *MT);
  int i = 0;
  while (true) {
    while (G->getNode(starts[i]) == nullptr) {
      ++i;
      if (i >= n) halt("number of agents is too large.");
    }
    config_s.push_back(G->getNode(starts[i]));
    if (config_s.size() == num_agents) break;
    ++i;
  }

  std::vector<int> goals(n);
  std::iota(goals.begin(), goals.end(), 0);
  std::shuffle(goals.begin(), goals.end(), *MT);
  int j = 0;
  while (true) {
    while (G->getNode(goals[j]) == nullptr) {
      ++j;
      if (j >= n) halt("number of agents is too large.");
    }
    if (G->getNode(goals[j]) == config_s[config_g.size()]) {
      // retry
      config_g.clear();
      std::shuffle(goals.begin(), goals.end(), *MT);
      continue;
    }
    config_g.push_back(G->getNode(goals[j]));
    if (config_g.size() == num_agents) break;
    ++j;
  }
}

void Problem::setScenStartsGoals(const std::string& scen_file)
{
  std::ifstream file(scen_file);
  if (!file) halt("file " + scen_file + " is not found.");
  config_s.clear();
  config_g.clear();

  std::regex r_locs = std::regex(R"(\d+\t.+?\t\d+\t\d+\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t.+)");
  std::string line;
  std::smatch results;
  int x_s, y_s, x_g, y_g;

  // read files
  while (getline(file, line)) {
    if (std::regex_match(line, results, r_locs)) {
      x_s = std::stoi(results[1].str());
      y_s = std::stoi(results[2].str());
      x_g = std::stoi(results[3].str());
      y_g = std::stoi(results[4].str());
      if (!G->nodeExist(x_s, y_s) || !G->nodeExist(x_s, y_s)) {
        halt("invalid scenario file.");
      }
      Node* s = G->getNode(x_s, y_s);
      Node* g = G->getNode(x_g, y_g);
      config_s.push_back(s);
      config_g.push_back(g);
    }
  }
}
