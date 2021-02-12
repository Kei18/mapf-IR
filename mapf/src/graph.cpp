#include "../include/graph.hpp"

#include <fstream>
#include <iostream>
#include <regex>

Graph::~Graph()
{
  for (auto v : V) delete v;
}

Nodes Graph::getV()
{
  Nodes _V;
  for (auto v : V) {
    if (v != nullptr) _V.push_back(v);
  }
  return _V;
}

Path Graph::getPath(Node* const s, Node* const g, const bool cache)
{
  if (s == g) return {};
  if (!cache) return getPathNoCache(s, g);

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

Path Graph::getPath(Node* const s, Node* const g, Nodes prohibited,
                    std::mt19937* MT)
{
  return getPathNoCache(s, g, prohibited, MT);
}

Path Graph::getPathNoCache(Node* const s, Node* const g, Nodes prohibited,
                           std::mt19937* MT)
{
  if (s == g) return {};

  using AstarNode = std::tuple<Node*, int, int>;  // node, g, f
  auto compare = [&](AstarNode a, AstarNode b) {
    // f-value
    if (std::get<2>(a) != std::get<2>(b))
      return std::get<2>(a) > std::get<2>(b);
    // g-value
    if (std::get<1>(a) != std::get<1>(b))
      return std::get<1>(a) < std::get<1>(b);
    return false;
  };

  // OPEN and CLOSE list
  constexpr int NIL = -1;
  std::priority_queue<AstarNode, std::vector<AstarNode>, decltype(compare)>
      OPEN(compare);
  std::vector<int> CLOSE(V.size(), NIL);  // node id, distance

  // initial node
  OPEN.push(std::make_tuple(s, 0, dist(s, g)));

  // main loop
  bool invalid = true;
  while (!OPEN.empty()) {
    // minimum node
    auto n = OPEN.top();
    OPEN.pop();

    // check CLOSE list
    Node* n_v = std::get<0>(n);
    const int n_g = std::get<1>(n);
    if (CLOSE[n_v->id] != NIL) continue;
    CLOSE[n_v->id] = n_g;

    // check goal condition
    if (n_v == g) {
      invalid = false;
      break;
    }

    // expand
    Nodes C = n_v->neighbor;
    if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);
    for (auto u : C) {
      int g_cost = n_g + 1;
      // already searched?
      if (CLOSE[u->id] != NIL) continue;
      // check constraints
      if (inArray(u, prohibited)) continue;
      OPEN.push(std::make_tuple(u, g_cost, g_cost + dist(u, g)));
    }
  }

  if (invalid) return {};

  // success
  Path path = {g};
  auto n = g;
  while (n != s) {
    for (auto m : n->neighbor) {
      if (CLOSE[m->id] == CLOSE[n->id] - 1) {
        n = m;
        path.push_back(n);
        break;
      }
    }
  }
  std::reverse(path.begin(), path.end());

  return path;
}

int Graph::pathDist(Node* const s, Node* const g, const bool cache)
{
  if (s == g) return 0;
  if (!cache) return getPath(s, g, cache).size() - 1;

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

  auto compare = [&](AstarNode* a, AstarNode* b) {
    if (a->f != b->f) return a->f > b->f;
    if (a->g != b->g) return a->g < b->g;
    return false;
  };
  std::function<AstarNode*(Node*, int, int, AstarNode*)> createNewNode;
  std::function<bool(Node*)> isClosed;
  std::function<void(Node*)> setClosed;

  // change data structure by graph size
  constexpr int huge_graph_size = 300000;
  bool is_small_graph = (V.size() <= huge_graph_size);

  // for allocating memory, for small field
  const int MEMORY_SIZE = is_small_graph ? V.size() : 1;
  AstarNode GC_S[MEMORY_SIZE];
  int node_total_cnt = 0;

  // garbage collection, for large field
  AstarNodes GC_L;

  // closed list
  bool CLOSE_S[MEMORY_SIZE];  // for small field
  std::memset(CLOSE_S, false, sizeof(CLOSE_S));
  std::unordered_map<int, bool> CLOSE_L;  // for large field

  if (is_small_graph) {
    createNewNode = [&](Node* v, int g, int f, AstarNode* p) {
      if (node_total_cnt >= MEMORY_SIZE)
        halt("memory over, increase MEMORY_SIZE...");
      auto q = &(GC_S[node_total_cnt++]);
      q->v = v;
      q->g = g;
      q->f = f;
      q->p = p;
      return q;
    };
    isClosed = [&](Node* v) { return CLOSE_S[v->id]; };
    setClosed = [&](Node* v) { CLOSE_S[v->id] = true; };

  } else {
    createNewNode = [&](Node* v, int g, int f, AstarNode* p) {
      AstarNode* new_node = new AstarNode{v, g, f, p};
      GC_L.push_back(new_node);
      return new_node;
    };
    isClosed = [&](Node* v) { return CLOSE_L.find(v->id) != CLOSE_L.end(); };
    setClosed = [&](Node* v) { CLOSE_L[v->id] = true; };
  }

  // OPEN
  std::priority_queue<AstarNode*, AstarNodes, decltype(compare)> OPEN(compare);

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
    if (isClosed(n->v)) continue;

    // update CLOSE list
    setClosed(n->v);

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
      for (auto k = path.begin() + 1; k != path.end(); ++k) {
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
      if (isClosed(u)) continue;
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
  for (auto p : GC_L) delete p;

  return path;
}

std::string Graph::getPathTableKey(Node* const s, Node* const g)
{
  return std::to_string(s->id) + "-" + std::to_string(g->id);
}

Grid::Grid(const std::string& _map_file) : Graph(), map_file(_map_file)
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
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();

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
  V = Nodes(width * height, nullptr);
  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();

    if ((int)line.size() != width) halt("map format is invalid");
    for (int x = 0; x < width; ++x) {
      char s = line[x];
      if (s == 'T' or s == '@') continue;  // object
      int id = width * y + x;
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
      if (existNode(x - 1, y)) v->neighbor.push_back(getNode(x - 1, y));
      // right
      if (existNode(x + 1, y)) v->neighbor.push_back(getNode(x + 1, y));
      // up
      if (existNode(x, y - 1)) v->neighbor.push_back(getNode(x, y - 1));
      // down
      if (existNode(x, y + 1)) v->neighbor.push_back(getNode(x, y + 1));
    }
  }
}

bool Grid::existNode(int id) const
{
  return 0 <= id && id < width * height && V[id] != nullptr;
}

bool Grid::existNode(int x, int y) const
{
  return 0 <= x && x < width && 0 <= y && y < height &&
         existNode(y * width + x);
}

Node* Grid::getNode(int id) const { return V[id]; }

Node* Grid::getNode(int x, int y) const { return getNode(y * width + x); }
