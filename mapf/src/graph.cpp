#include "../include/graph.hpp"
#include <iostream>
#include <fstream>
#include <regex>
#include "../include/util.hpp"

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
