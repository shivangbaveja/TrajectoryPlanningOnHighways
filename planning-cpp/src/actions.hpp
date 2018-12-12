#ifndef ACTIONS_H
#define ACTIONS_H

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <iomanip>
#include <fstream>
#include <string>
#include <list>
#include <map>
#include <vector>
#include <tuple>
#include <cmath>


// Third party libraries
// used for parsing json
#include "nlohmann/json.hpp"

// used to plot for debugging purposes, can be removed in the final implementation
#include "matplotlibcpp.h"
#include "helper.hpp"
#include "CollisionChecker.hpp"


using namespace std;
// for convenience
using json = nlohmann::json;
// // For matplotlib-CPP
namespace plt = matplotlibcpp;

class Actions
{
public:
	Actions(string fileName, double speedRes, CollisionChecker & cc, bool debugActions);
	~Actions();

	void ReadActionsJson(vector<vector<vector<vector<double> > > > &actions,
                                   string filePath,
                                   vector<vector<double> > &actionsCost);
	void PlotPath(std::vector<std::vector<double> >& path);
	void PlotActions(std::vector<std::vector<std::vector<std::vector<double> > > >& actions, int speedId);
	int SpeedToIndex(double speed);
	vector<Config> GetSuccessors(const Config &current);

	vector< vector< vector< vector< double>>>> m_actions;
	vector< vector< double>> m_actionCost;
	string m_fileName;
	double m_speedRes;
	int m_skip;
    CollisionChecker & m_cc;
    bool m_debugActions;
};







#endif
