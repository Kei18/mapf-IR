#pragma once
#include <cmath>
#include <iomanip>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>

#include "util.hpp"

struct Node;
using Nodes = std::vector<Node*>;
using Path = std::vector<Node*>;    // < loc_i[0], loc_i[1], ... >
using Config = std::vector<Node*>;  // < loc_0[t], loc_1[t], ... >
using Configs = std::vector<Config>;

// locations of node
struct Pos {
  int x;
  int y;

  Pos(int _x, int _y) : x(_x), y(_y) {}

  void print()
  {
    std::cout << "(" << std::right << std::setw(3) << x << ", " << std::right
              << std::setw(3) << y << ")";
  }
  void println()
  {
    print();
    std::cout << std::endl;
  }

  int manhattanDist(const Pos& pos) const
  {
    return std::abs(x - pos.x) + std::abs(y - pos.y);
  }

  float euclideanDist(const Pos& pos) const
  {
    float dx = x - pos.x;
    float dy = y - pos.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  bool operator==(const Pos& other) const
  {
    return x == other.x && y == other.y;
  }
  Pos operator+(const Pos& other) const
  {
    return Pos(x + other.x, y + other.y);
  }
  Pos operator-(const Pos& other) const
  {
    return Pos(x - other.x, y - other.y);
  }
  Pos operator*(const int i) const { return Pos(x * i, y * i); }
  void operator+=(const Pos& other)
  {
    x = x + other.x;
    y = y + other.y;
  }
  void operator-=(const Pos& other)
  {
    x = x - other.x;
    y = y - other.y;
  }
  void operator*=(const int i)
  {
    x = x * i;
    y = y * i;
  }
};

struct Node {
  int id;
  Nodes neighbor;
  Pos pos;

  Node(int _id, int x, int y) : id(_id), pos(Pos(x, y)) {}

  int getDegree() const { return neighbor.size(); }
  float manhattanDist(const Node& node) const
  {
    return pos.manhattanDist(node.pos);
  }
  float manhattanDist(Node* const node) const
  {
    return pos.manhattanDist(node->pos);
  }
  float euclideanDist(const Node& node) const
  {
    return pos.euclideanDist(node.pos);
  }
  float euclideanDist(Node* const node) const
  {
    return pos.euclideanDist(node->pos);
  }

  void print()
  {
    std::cout << "node[" << std::right << std::setw(6) << id << "]=<pos:";
    pos.print();
    std::cout << ", neigh: d=" << std::setw(1) << getDegree() << ", ";
    for (auto v : neighbor) {
      v->pos.print();
      std::cout << ", ";
    }
    std::cout << ">";
  }
  void println()
  {
    print();
    std::cout << std::endl;
  }

  bool operator==(const Node& v) { return v.id == id; };
  bool operator!=(const Node& v) { return v.id != id; };
  bool operator==(Node* const v) { return v->id == id; };
  bool operator!=(Node* const v) { return v->id != id; };
};

// check two configurations are same or not
[[maybe_unused]] static bool sameConfig(const Config& config_i,
                                        const Config& config_j)
{
  if (config_i.size() != config_j.size()) return false;
  const int size_i = config_i.size();
  for (int k = 0; k < size_i; ++k) {
    if (config_i[k] != config_j[k]) return false;
  }
  return true;
}

[[maybe_unused]] static int getPathCost(const Path& path)
{
  int cost = path.size() - 1;
  auto itr = path.end() - 1;
  while (itr != path.begin() && *itr == *(itr - 1)) {
    --cost;
    --itr;
  }
  return cost;
}

// Pure graph. Base class of Grid class.
class Graph
{
private:
  // cache for finding a path between two nodes
  std::unordered_map<std::string, Path> PATH_TABLE;

  // get key name for cache
  static std::string getPathTableKey(Node* const s, Node* const g);

  // register already searched path to cache
  void registerPath(const Path& path);

  // get path avoiding several nodes, used for creating well-formed instance
  Path getPathNoCache(Node* const s, Node* const g, Nodes prohibited = {},
                      std::mt19937* MT = nullptr);

  // find a path using cache, if failed then return empty
  Path AstarSearchWithCache(Node* const s, Node* const g);

protected:
  // in grid, V[y * width + x] = Node with position (x, y)
  // if (x, y) is occupied then V[y * width + x] = nullptr
  Nodes V;

public:
  Graph(){};
  virtual ~Graph();

  // in grid, id = y * width + x
  virtual bool existNode(int id) const { return false; };
  virtual bool existNode(int x, int y) const { return false; };

  // in grid, id = y * width + x
  virtual Node* getNode(int x, int y) const { return nullptr; };
  virtual Node* getNode(int id) const { return nullptr; };

  // in grid, Manhattan distance
  virtual int dist(Node* const v, Node* const u) { return 0; }

  // get path between two nodes
  Path getPath(Node* const s, Node* const g, const bool cache = true);

  // get path length between two nodes
  int pathDist(Node* const s, Node* const g, const bool cache = true);

  // get path avoiding several nodes, used for creating well-formed instance
  Path getPath(Node* const s, Node* const g, Nodes prohibited,
               std::mt19937* MT = nullptr);

  // get all nodes
  Nodes getV();

  int getNodesSize() const { return V.size(); }
};

class Grid : public Graph
{
private:
  std::string map_file;
  int width;
  int height;

public:
  Grid(){};
  Grid(const std::string& _map_file);
  ~Grid(){};

  bool existNode(int id) const;
  bool existNode(int x, int y) const;
  Node* getNode(int id) const;
  Node* getNode(int x, int y) const;

  int dist(Node* const v, Node* const u) { return v->manhattanDist(u); }

  std::string getMapFileName() { return map_file; };
  int getWidth() { return width; }
  int getHeight() { return height; }
};
