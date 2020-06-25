/*
 * Implementation of Improved Conflict-based Search (ICBS)
 *
 * - ref
 * Boyarski, E., Felner, A., Stern, R., Sharon, G., Tolpin, D., Betzalel, O., & Shimony, E. (2015, June).
 * ICBS: improved conflict-based search algorithm for multi-agent pathfinding.
 * In Twenty-Fourth International Joint Conference on Artificial Intelligence.
 */

#pragma once
#include "solver.hpp"
#include "cbs.hpp"
#include <stack>
#include <memory>


struct MDDNode {
  int t;
  Node* v;
  std::vector<MDDNode*> next;
  std::vector<MDDNode*> prev;

  MDDNode(int _t, Node* _v) {
    t = _t;
    v = _v;
  }

  bool operator==(const MDDNode& other) const {
    return t == other.t && v == other.v;
  }
};

using MDDNodes = std::vector<MDDNode*>;

struct MDD {
  int c;  // cost
  int i;  // agent
  Graph* G;  // original graph
  Node* s;  // start
  Node* g;  // goal;
  std::vector<MDDNodes> body;  // t: 0...c
  bool valid;
  std::vector<MDDNode*> generated_nodes;  // for memory management

  MDD(int _c, int _i, Graph* _G, Node* _s, Node* _g, bool _valid) {
    c = _c;
    i = _i;
    G = _G;
    s = _s;
    g = _g;
    valid = _valid;
  }

  MDD(int _c, int _i, Problem* P, CBS::Constraints constraints) {
    c = _c;
    i = _i;
    G = P->getG();
    s = P->getStart(i);
    g = P->getGoal(i);
    valid = G->pathDist(s, g) <= c;
    build();
    update(constraints);
  }

  ~MDD() {
    for (MDDNode* node : generated_nodes) delete node;
  }

  // copy
  MDD(const MDD& other)
    : c(other.c),i(other.i), G(other.G),
      s(other.s), g(other.g), valid(other.valid)
  {
    if (!valid) return;
    // generate body
    MDDNode* new_node = new MDDNode(0, s);
    body.push_back({ new_node });
    generated_nodes.push_back(new_node);
    for (auto nodes : other.body) {
      MDDNodes new_nodes;
      if (!nodes.empty() && nodes[0]->t == 0) continue; // starts
      MDDNodes new_prev_nodes = body[body.size()-1];
      for (auto node : nodes) {
        new_node = new MDDNode(node->t, node->v);
        new_nodes.push_back(new_node);
        generated_nodes.push_back(new_node);
        for (auto prev_node : node->prev) {
          auto itr = std::find_if(new_prev_nodes.begin(),
                                  new_prev_nodes.end(),
                                  [prev_node] (MDDNode* _node)
                                  { return _node->t == prev_node->t &&
                                      _node->v == prev_node->v; });
          MDDNode* new_prev_node = *itr;
          new_prev_node->next.push_back(new_node);
          new_node->prev.push_back(new_prev_node);
        }
      }
      body.push_back(new_nodes);
    }
  }

  void build() {
    // impossible
    if (!valid) return;
    // add start node
    body.push_back({ new MDDNode(0, s) });
    // build
    for (int t = 0; t < c; ++t) {
      MDDNodes nodes_at_t = body[t];
      MDDNodes nodes_at_t_next;
      for (auto node : nodes_at_t) {
        Nodes cands = node->v->neighbor;
        cands.push_back(node->v);
        for (auto v : cands) {
          // valid
          if (G->pathDist(v, g) + t + 1 <= c) {
            MDDNode* next_node = nullptr;
            for (auto _node : nodes_at_t_next) {
              if (_node->v == v) {
                next_node = _node;
                break;
              }
            }
            if (next_node == nullptr) {
              next_node = new MDDNode(t + 1, v);
              nodes_at_t_next.push_back(next_node);
            }
            node->next.push_back(next_node);
            next_node->prev.push_back(node);
          }
        }
      }
      body.push_back(nodes_at_t_next);
    }

    // for memory management
    for (auto nodes : body) {
      for (auto node : nodes) {
        generated_nodes.push_back(node);
      }
    }
  }

  void update(const CBS::Constraints& _constraints) {
    if (!valid || _constraints.empty()) return;
    // format constraints
    CBS::Constraints constraints;
    for (auto constraint : _constraints) {
      if (constraint->id == i) constraints.push_back(constraint);
    }
    if (constraints.empty()) halt("error, constraints never become empty");
    std::sort(constraints.begin(), constraints.end(),
              [] (CBS::Constraint* a, CBS::Constraint* b) {
                if (a->t != b->t) return a->t < b->t;
                if (a->u != nullptr) return true;
                if (b->u != nullptr) return false;
                return false;
              });
    CBS::Constraint* last_constraint = *(constraints.end()-1);
    if ((last_constraint->t > c)
        || (last_constraint->t == c && last_constraint->u == nullptr)) {
      valid = false;
      return;
    }

    // delete nodes
    for (auto constraint : constraints) {
      auto itr_v = std::find_if(body[constraint->t].begin(),
                                body[constraint->t].end(),
                                [constraint] (MDDNode* node)
                                { return node->v == constraint->v; });
      if (itr_v == body[constraint->t].end()) continue;
      MDDNode* node_v = *itr_v;
      if (constraint->u == nullptr) {  // vertex constraints, v
        deleteForward(node_v);
        deleteBackword(node_v);
      } else {  // swap conflict, u->v
        auto itr_vu = std::find_if(node_v->prev.begin(),
                                   node_v->prev.end(),
                                   [constraint] (MDDNode* node)
                                   { return node->v == constraint->u; });
        MDDNode* node_u = *itr_vu;
        if (itr_vu != node_v->prev.end()) {
          auto itr_uv = std::find_if(node_u->next.begin(),
                                     node_u->next.end(),
                                     [node_v] (MDDNode* node)
                                     { return node->v == node_v->v; });
          node_v->prev.erase(itr_vu);
          node_u->next.erase(itr_uv);
          if (node_v->prev.empty()) deleteForward(node_v);
          if (node_u->next.empty()) deleteBackword(node_u);
        }
      }
    }

    if (body[0].empty() || body[c].empty()) valid = false;
  }

  void deleteForward(MDDNode* node) {
    for (auto next_node : node->next) {
      auto itr = std::find_if(next_node->prev.begin(),
                              next_node->prev.end(),
                              [node] (MDDNode* _node)
                              { return _node->v == node->v; });
      next_node->prev.erase(itr);
      if (next_node->prev.empty()) deleteForward(next_node);
    }
    auto itr = std::find_if(body[node->t].begin(),
                            body[node->t].end(),
                            [node] (MDDNode* _node)
                            { return _node->v == node->v; });
    if (itr != body[node->t].end()) body[node->t].erase(itr);
  }

  void deleteBackword(MDDNode* node) {
    for (auto prev_node : node->prev) {
      auto itr = std::find_if(prev_node->next.begin(),
                              prev_node->next.end(),
                              [node] (MDDNode* _node)
                              { return _node->v == node->v; });
      prev_node->next.erase(itr);
      if (prev_node->next.empty()) deleteBackword(prev_node);
    }
    auto itr = std::find_if(body[node->t].begin(),
                            body[node->t].end(),
                            [node] (MDDNode* _node)
                            { return _node->v == node->v; });
    if (itr != body[node->t].end()) body[node->t].erase(itr);
  }

  Path getPathByDFS() const {
    auto getNodeName =
      [] (MDDNode* node) {
        return std::to_string(node->t) + "-" + std::to_string(node->v->id);
      };
    MDDNode* start_node = body[0][0];
    MDDNode* goal_node = body[c][0];

    // depth first search
    std::unordered_map<std::string, bool> CLOSE;
    std::unordered_map<std::string, MDDNode*> PREV;
    std::stack<MDDNode*> OPEN;
    OPEN.push(start_node);
    bool invalid = true;
    while (!OPEN.empty()) {
      MDDNode* node = OPEN.top();
      OPEN.pop();
      CLOSE[getNodeName(node)] = true;
      if (node == goal_node) {
        invalid = false;
        break;
      }
      for (auto next_node : node->next) {
        OPEN.push(next_node);
        PREV[getNodeName(next_node)] = node;
      }
    }
    Path path;
    if (!invalid) {
      MDDNode* node = goal_node;
      while (node != start_node) {
        path.push_back(node->v);
        node = PREV[getNodeName(node)];
      }
      path.push_back(s);
      std::reverse(path.begin(), path.end());
    }
    return path;
  }

  Path getPath(CBS::Constraint* constraint=nullptr) const {
    if (constraint == nullptr) return getPathByDFS();
    MDD mdd = *this;
    mdd.update({ constraint });
    return mdd.getPathByDFS();
  }

  void println() const {
    std::cout << "MDD_" << i << "^" << c << std::endl;
    for (int t = 0; t <= c; ++t) {
      std::cout << "t=" << t << std::endl;
      for (auto node : body[t]) {
        std::cout << "- v=("
                  << node->v->pos.x << ", "
                  << node->v->pos.y << "), "
                  << "t=" << node->t
                  << ", next: ";
        for (auto next_node : node->next) {
          std::cout << "("
                    << next_node->v->pos.x << ", "
                    << next_node->v->pos.y << "), ";
        }
        std::cout << ", prev: ";
        for (auto prev_node : node->prev) {
          std::cout << "("
                    << prev_node->v->pos.x << ", "
                    << prev_node->v->pos.y << "), ";
        }
        std::cout << std::endl;
      }
    }
  }
};
using MDD_p = std::shared_ptr<MDD>;
using MDDs = std::vector<MDD_p>;

class ICBS : public CBS {
public:
  static const std::string SOLVER_NAME;

protected:
  std::unordered_map<int, MDDs> MDDTable;  // store MDD_c^i

  void setInitialHighLevelNode(HighLevelNode* n);
  void invoke(HighLevelNode* h_node, int id);
  bool findBypass(HighLevelNode* h_node,
                  const Constraints& constraints);
  Constraints getPrioritizedConflict(const HighLevelNode* h_node);

public:
  ICBS(Problem* _P);
  ~ICBS();

  void solve();
};
