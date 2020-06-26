#include "../include/graph.hpp"
#include <iostream>
#include <fstream>
#include <regex>

int Graph::UUID = 0;

Path Graph::getPath(Node* const s, Node* const g)
{
  if (s == g) return {};

  // check cache
  std::string key = getPathTableKey(s, g);
  auto itr = PATH_TABLE.find(key);
  if (itr != PATH_TABLE.end()) return itr->second;

  Path path = AstarSearchWithCache(s, g);
  registerPath(path);
  return path;
}

int Graph::pathDist(Node* const s, Node* const g)
{
  if (s == g) return 0;

  // check cache
  std::string key = getPathTableKey(s, g);
  auto itr = PATH_TABLE.find(key);
  if (itr != PATH_TABLE.end()) return itr->second.size() - 1;

  Path path = AstarSearchWithCache(s, g);
  registerPath(path);
  return path.size() - 1;
}

void Graph::registerPath(const Path& path)
{
  if (path.empty()) return;
  Nodes tmp = path;
  Node* v;
  Node* g = *(path.end() - 1);
  do {
    v = tmp[0];
    PATH_TABLE[getPathTableKey(v, g)] = tmp;
    tmp.erase(tmp.begin());
  } while (tmp.size() > 2);
}

// A* search but using cache as much as possible
Path Graph::AstarSearchWithCache(Node* const s, Node* const g)
{
  struct AstarNode {
    Node* v;
    int g;
    int f;
    AstarNode* p;  // parent
  };

  auto compare = [&] (AstarNode* a, AstarNode* b) {
                   if (a->f != b->f) return a->f > b->f;
                   if (a->g != b->g) return a->g < b->g;
                   return false; };

  // OPEN and CLOSE
  std::priority_queue<AstarNode*,
                      std::vector<AstarNode*>,
                      decltype(compare)> OPEN(compare);
  std::unordered_map<int, AstarNode*> CLOSE;

  // initial node
  AstarNode* n;
  n = new AstarNode { s, 0, dist(s, g), nullptr };
  OPEN.push(n);

  bool invalid = true;
  while (!OPEN.empty()) {
    n = OPEN.top();
    OPEN.pop();

    // check CLOSE list
    if (CLOSE.find(n->v->id) != CLOSE.end()) continue;
    CLOSE[n->v->id] = n;

    // check goal condition
    if (n->v == g) {
      invalid = false;
      break;
    }

    // use cache
    auto itr = PATH_TABLE.find(getPathTableKey(n->v, g));
    if (itr != PATH_TABLE.end()) {
      Path path = itr->second;
      for (int t = 1; t < path.size(); ++t) {
        n = new AstarNode { path[t], 0, 0, n };
        CLOSE[n->v->id] = n;
      }
      invalid = false;
      break;
    }

    // expand
    Nodes C = n->v->neighbor;
    C.push_back(n->v);
    for (auto u : C) {
      // already searched?
      if (CLOSE.find(u->id) != CLOSE.end()) continue;
      int g_value = n->g + 1;
      int h_value = g_value + dist(u, g);
      // use real cost whenever available
      auto itr = PATH_TABLE.find(getPathTableKey(u, g));
      if (itr != PATH_TABLE.end()) {
        h_value = g_value + itr->second.size() - 1;
      }
      AstarNode* m = new AstarNode { u, g_value, h_value, n };
      OPEN.push(m);
    }
  }

  if (invalid) halt("graph contains unreachable nodes.");

  Path path;
  while (n != nullptr) {
    path.push_back(n->v);
    n = n->p;
  }
  std::reverse(path.begin(), path.end());

  // free
  while (!OPEN.empty()) {
    delete OPEN.top();
    OPEN.pop();
  }
  for (auto itr = CLOSE.begin(); itr != CLOSE.end(); ++itr) {
    delete itr->second;
  }

  return path;
}

std::string Graph::getPathTableKey(Node* const s, Node* const g) {
  return std::to_string(s->id) + "-" + std::to_string(g->id);
}

Grid::Grid(const std::string& _map_file): Graph(_map_file)
{
  std::ifstream file(map_file);
  if (!file) halt("file " + map_file + " is not found.");

  std::string line;
  std::smatch results;
  std::regex r_height = std::regex(R"(height\s(\d+))");
  std::regex r_width = std::regex(R"(width\s(\d+))");
  std::regex r_map = std::regex(R"(map)");

  // read fundamental graph params
  while (getline(file, line)) {
    if (std::regex_match(line, results, r_height)) {
      height = std::stoi(results[1].str());
    }
    if (std::regex_match(line, results, r_width)) {
      width = std::stoi(results[1].str());
    }
    if (std::regex_match(line, results, r_map)) break;
  }
  if (!(width > 0 && height > 0)) halt("failed to load width/height.");

  // create nodes
  int y = 0;
  V = Nodes(width*height, nullptr);
  std::vector<std::string> mapstr;
  while (getline(file, line)) {
    mapstr.push_back(line);
    if (line.size() != width) halt("map format is invalid");
    for (int x = 0; x < width; ++x) {
      char s = line[x];
      if (s == 'T' or s == '@') continue;  // object
      int id = width*y+x;
      Pos pos = { x, y };
      Node* v = new Node { id , {}, pos };
      V[id] = v;
    }
    ++y;
  }
  if (y != height) halt("map format is invalid");
  file.close();

  // create edges
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      char s = mapstr[y][x];
      if (s == 'T' || s == '@') continue;
      Node* v = getNode(x, y);
      // left
      if (nodeExist(x-1, y)) v->neighbor.push_back(getNode(x-1, y));
      // right
      if (nodeExist(x+1, y)) v->neighbor.push_back(getNode(x+1, y));
      // up
      if (nodeExist(x, y-1)) v->neighbor.push_back(getNode(x, y-1));
      // down
      if (nodeExist(x, y+1)) v->neighbor.push_back(getNode(x, y+1));
    }
  }
}

bool Grid::nodeExist(int x, int y) const
{
  int id = y * width + x;
  return  0 <= x && x < width
       && 0 <= y && y < height
       && V[id] != nullptr;
}

Node* Grid::getNode(int x, int y) const
{
  return V[y*width+x];
}

Node* Grid::getNode(int id) const
{
  return V[id];
}
