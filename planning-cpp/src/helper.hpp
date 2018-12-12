#ifndef HELPER_H
#define HELPER_H

#include <vector>
#include <math.h>
#include <utility>
#include <math.h>
#include <iostream>

#include <queue>
#include <unordered_map>
#include <unordered_set>

using namespace std;

#define DOUBLE_TOL (0.001)

class Config
{
public:
    double m_x;
    double m_y;
    double m_theta;
    double m_s;
    double m_k;
    double m_v;
    double m_t;
    double m_a;
    double m_closeness;  //Defines the lane of object
    double m_timeCost;
    pair<int,int> m_id; //TODO: decide unique ID assignment

    Config( double x, double y, double theta, double s, double k, double v, 
            double t, double a, double closeness, double timeCost, pair<int,int> id):
            m_x(x), m_y(y), m_theta(theta), m_s(s), m_k(k), m_v(v), m_t(t),
            m_a(a), m_closeness(closeness), m_timeCost(timeCost), m_id(id)
    {

    }

    Config& operator=(Config& input)
    {
        m_x = input.m_x;
		m_y = input.m_y;
		m_theta = input.m_theta;
		m_s = input.m_s;
		m_v = input.m_v;
		m_t = input.m_t;
		m_t = input.m_a;
		m_a = input.m_closeness;
		m_closeness = input.m_closeness;
		m_timeCost = input.m_timeCost;
		m_id = input.m_id;

		return *this;
    }

    void Print()
    {
    	cout<<"\nx:"<<m_x;
    	cout<<"\ny:"<<m_y;
    	cout<<"\ntheta:"<<m_theta;
    	cout<<"\ns:"<<m_s;
    	cout<<"\nk:"<<m_k;
    	cout<<"\nv:"<<m_v;
    	cout<<"\nt:"<<m_t;
    	cout<<"\na:"<<m_a;
    }
};


//Defining each node state
struct Node
{
    Node* parentNode;
    Config config;
    double g_val;
    double h_val;

    Node(Node* ptr, Config newConfig, double gval, double hval):
    parentNode(ptr),
    config(newConfig),
    g_val(gval),
    h_val(hval)
    {
    }
};

class NodeComparator
{
public:
    bool operator()(const Node* lhs, const Node* rhs) const
    {
        // if x, y, theta and time are comparable, two configs are same
        if(fabsf(lhs->config.m_x - rhs->config.m_x)<DOUBLE_TOL &&
            fabsf(lhs->config.m_y - rhs->config.m_y)<DOUBLE_TOL &&
            // fabsf(lhs->config.m_theta - rhs->config.m_theta)<DOUBLE_TOL &&
            fabsf(lhs->config.m_v - rhs->config.m_v)<DOUBLE_TOL &&
            fabsf(lhs->config.m_t - rhs->config.m_t)<DOUBLE_TOL)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};

class NodeHasher {
public:
    size_t operator()(const Node* node) const {
        std::hash<double> hasher;
        size_t seed = 0;        // TODO: maybe 4 as size of config 
         seed ^= hasher(node->config.m_x) + 0x9e3779b9 + (seed<<6) + (seed>>2);
         seed ^= hasher(node->config.m_y) + 0x9e3779b9 + (seed<<6) + (seed>>2);
         // seed ^= hasher(node->config.m_theta) + 0x9e3779b9 + (seed<<6) + (seed>>2);
         seed ^= hasher(node->config.m_v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
         seed ^= hasher(node->config.m_t) + 0x9e3779b9 + (seed<<6) + (seed>>2);

        return seed;
    }
};

class NodeCompare
{
public:
    bool operator()(Node* lhs, Node* rhs)
    {
        return (lhs->g_val + lhs->h_val) > (rhs->g_val + rhs->h_val);
    }
};

typedef unordered_set<Node*, NodeHasher, NodeComparator> uset;


#endif