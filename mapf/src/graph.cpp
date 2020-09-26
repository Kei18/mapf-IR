#include "../include/graph.hpp"
#include <iostream>
#include <fstream>
#include <regex>

Graph::~Graph()
{
  for (auto v : V) delete v;
}

Path Graph::getPath(Node* const s, Node* const g)
{
  if (s == g) return {};

  // check cache
  std::string key = getPathTableKey(s, g);
  auto itr = PATH_TABLE.find(key);
  if (itr != PATH_TABLE.end()) return itr->second;

  // failed -> use A* search
  Path path = AstarSearchWithCache(s, g);

  // register new path to the cache
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

  // failed -> use A* search
  Path path = AstarSearchWithCache(s, g);

  // register new path to the cache
  registerPath(path);

  return path.size() - 1;
}

/*
 * Given a path < v_1, ..., v_k >,
 * < v_1, ..., v_k >, < v_2, ..., v_k >, ..., < v_{k-1}, ..., v_k >
 * are registered.
 */
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

/*
 * A* search using cache (already known paths) as much as possible.
 *
 * Note: I tried smart pointer but it was slow.
 */
Path Graph::AstarSearchWithCache(Node* const s, Node* const g)
{
  struct AstarNode {
    Node* v;
    int g;
    int f;
    AstarNode* p;  // parent
  };
  using AstarNodes = std::vector<AstarNode*>;

  auto compare = [&] (AstarNode* a, AstarNode* b) {
                   if (a->f != b->f) return a->f > b->f;
                   if (a->g != b->g) return a->g < b->g;
                   return false; };

  AstarNodes GC;  // garbage collection
  auto createNewNode =
    [&] (Node* v, int g, int f, AstarNode* p)
    {
      AstarNode* new_node = new AstarNode{ v, g, f, p };
      GC.push_back(new_node);
      return new_node;
    };

  // OPEN and CLOSE
  std::priority_queue<AstarNode*, AstarNodes, decltype(compare)> OPEN(compare);
  std::unordered_map<int, bool> CLOSE;

  // initial node
  AstarNode* n = createNewNode(s, 0, dist(s, g), nullptr);
  OPEN.push(n);

  // search start
  bool invalid = true;
  while (!OPEN.empty()) {
    // pop a node with the minimum f-value
    n = OPEN.top();
    OPEN.pop();

    // check CLOSE list
    if (CLOSE.find(n->v->id) != CLOSE.end()) continue;

    // update CLOSE list
    CLOSE[n->v->id] = true;

    // check goal condition
    if (n->v == g) {
      invalid = false;
      break;
    }

    // check whether the remained path has already known
    auto itr = PATH_TABLE.find(getPathTableKey(n->v, g));
    if (itr != PATH_TABLE.end()) {
      // if found then complement the rest
      Path path = itr->second;
      for (auto k = path.begin()+1; k != path.end(); ++k) {
        n = createNewNode(*k, 0, 0, n);
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
      AstarNode* m = createNewNode(u, g_value, h_value, n);
      OPEN.push(m);
    }
  }

  // detect unreachable nodes
  if (invalid) halt("graph contains unreachable nodes.");

  // reconstruct path
  Path path;
  while (n != nullptr) {
    path.push_back(n->v);
    n = n->p;
  }
  std::reverse(path.begin(), path.end());

  // free
  for (auto p : GC) delete p;

  return path;
}

std::string Graph::getPathTableKey(Node* const s, Node* const g)
{
  return std::to_string(s->id) + "-" + std::to_string(g->id);
}

Grid::Grid(const std::string& _map_file): Graph(), map_file(_map_file)
{
  // read map file
#ifdef _MAPDIR_
  std::ifstream file(_MAPDIR_ + map_file);
#else
  std::ifstream file(map_file);
#endif

  if (!file) halt("file " + map_file + " is not found.");

  std::string line;
  std::smatch results;
  std::regex r_height = std::regex(R"(height\s(\d+))");
  std::regex r_width = std::regex(R"(width\s(\d+))");
  std::regex r_map = std::regex(R"(map)");

  // fundamental graph params
  while (getline(file, line)) {
    if (*(line.end()-1) == 0x0d) line.pop_back();  // for CRLF coding
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
  while (getline(file, line)) {
    if (*(line.end()-1) == 0x0d) line.pop_back();  // for CRLF coding
    if (line.size() != width) halt("map format is invalid");
    for (int x = 0; x < width; ++x) {
      char s = line[x];
      if (s == 'T' or s == '@') continue;  // object
      int id = width*y+x;
      Node* v = new Node(id, x, y);
      V[id] = v;
    }
    ++y;
  }
  if (y != height) halt("map format is invalid");
  file.close();

  // create edges
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (!existNode(x, y)) continue;
      Node* v = getNode(x, y);
      // left
      if (existNode(x-1, y)) v->neighbor.push_back(getNode(x-1, y));
      // right
      if (existNode(x+1, y)) v->neighbor.push_back(getNode(x+1, y));
      // up
      if (existNode(x, y-1)) v->neighbor.push_back(getNode(x, y-1));
      // down
      if (existNode(x, y+1)) v->neighbor.push_back(getNode(x, y+1));
    }
  }
}

bool Grid::existNode(int id) const
{
  return 0 <= id && id < width * height && V[id] != nullptr;
}

bool Grid::existNode(int x, int y) const
{
  return  0 <= x && x < width
       && 0 <= y && y < height
       && existNode(y * width + x);
}

Node* Grid::getNode(int id) const
{
  return V[id];
}

Node* Grid::getNode(int x, int y) const
{
  return getNode(y * width + x);
}
