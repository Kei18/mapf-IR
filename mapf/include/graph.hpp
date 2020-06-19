#pragma once
#include <cmath>
#include <vector>
#include <iostream>
#include <iomanip>

struct Pos {
  int x;
  int y;

  void print() {
    std::cout << "("
              << std::right << std::setw(3) << x
              << ", "
              << std::right << std::setw(3) << y
              << ")";
  }
  void println() {
    print();
    std::cout << std::endl;
  }

  int manhattanDist(const Pos& pos) {
    return std::abs(x - pos.x) + std::abs(y - pos.y);
  }

  float euclideanDist(const Pos& pos) {
    float dx = x - pos.x;
    float dy = y - pos.y;
    return std::sqrt(dx*dx + dy*dy);
  }

  Pos operator+(const Pos& other) const {
    return Pos { x + other.x, y + other.y };
  }
  Pos operator-(const Pos& other) const {
    return Pos { x - other.x, y - other.y };
  }
  Pos operator*(const int i) const {
    return Pos { x * i, y * i };
  }
  void operator+=(const Pos& other) {
    x = x + other.x;
    y = y + other.y;
  }
  void operator-=(const Pos& other) {
    x = x - other.x;
    y = y - other.y;
  }
  void operator*=(const int i) {
    x = x * i;
    y = y * i;
  }
};

struct Node {
  int id;
  std::vector<Node*> neighbor;
  Pos pos;

  int getDegree() { return neighbor.size(); }

  float manhattanDist(const Node& node) {
    return pos.manhattanDist(node.pos);
  }
  float manhattanDist(const Node* node) {
    return pos.manhattanDist(node->pos);
  }

  float euclideanDist(const Node& node) {
    return pos.euclideanDist(node.pos);
  }
  float euclideanDist(const Node* node) {
    return pos.euclideanDist(node->pos);
  }

  void print() {
    std::cout << "node["
              << std::right << std::setw(6) << id
              << "]=<pos:";
    pos.print();
    std::cout << ", neigh: d=" << std::setw(1) << getDegree() << ", ";
    for (auto v : neighbor) {
      v->pos.print();
      std::cout << ", ";
    }
    std::cout << ">";
  }
  void println() {
    print();
    std::cout << std::endl;
  }

  bool operator==(const Node& v) { return v.id  == id; };
  bool operator!=(const Node& v) { return v.id  != id; };
  bool operator==(const Node* v) { return v->id == id; };
  bool operator!=(const Node* v) { return v->id != id; };
};

using Nodes   = std::vector<Node*>;
using Path    = std::vector<Node*>;  // < loc_i[0], loc_i[1], ... >
using Config  = std::vector<Node*>;  // < loc_0[t], loc_1[t], ... >
using Paths   = std::vector<Path>;
using Configs = std::vector<Config>;

class Graph {
protected:
  Nodes V;
  std::string map_file;
  int width;
  int height;

public:
  Graph() {};
  Graph(const std::string& _map_file) : map_file(_map_file) {};
  ~Graph() {};

  virtual bool nodeExist(int x, int y) const { return false; };
  virtual Node* getNode(int x, int y) const { return nullptr; };
  virtual Node* getNode(int id) const { return nullptr; };
  virtual int dist(Node* v, Node* u) { return 0; }

  std::string getMapFileName() { return map_file; };
  int getWidth() { return width; }
  int getHeight() { return height; }
};


class Grid : public Graph {
public:
  Grid() {};
  Grid(const std::string& _map_file);
  ~Grid() {};

  bool nodeExist(int x, int y) const;
  Node* getNode(int x, int y) const;
  Node* getNode(int id) const;
  int dist(Node* v, Node* u) { return v->manhattanDist(u); }
};
