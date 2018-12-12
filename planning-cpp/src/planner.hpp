//
// Created by danendra on 12/2/18.
//
#ifndef PLANNER_H
#define PLANNER_H

#include <limits>
#include <vector>
#include <math.h>

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include "helper.hpp"
#include "actions.hpp"

using namespace std;

class Planner
{
public:

    priority_queue<Node*,vector<Node*>,NodeCompare> m_aStarPQ;
    unordered_set<Node*, NodeHasher, NodeComparator> m_closedSet;
    unordered_set<Node*, NodeHasher, NodeComparator> m_openSet;
    std::vector<Node*> m_nodeMap;
    Actions *m_actions;
    bool m_debug;

    Planner(Actions *actions, bool debug);
    ~Planner();
    void InitializeAstar();
    pair<vector<vector<double>>, vector<vector<double>>> search(Config start,
                                  double targetX,
                                  double targetTime);
    const double Heuristic (double x1, double y1, double x2, double y2);
    void AddToClosedSet(uset &closedSet, Node* node);
    Node* CheckIfInClosedSet(uset &closedSet, Node* node);
    void UpdateTrajectory(vector<vector<double>> &traj, double x, double y, double s, double t);
    double GetHeuristic(Config config, double targetX, double targetTime);
};




#endif
