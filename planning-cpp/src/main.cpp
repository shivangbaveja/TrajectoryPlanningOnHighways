#include <vector>
#include <tuple>
#include <list>
#include <utility>
#include <stdlib.h>     /* srand, rand */
#include <iostream>
#include <sstream> 

// custom includes
#include "CollisionChecker.hpp"
#include "env.hpp"
#include "actions.hpp"
#include "planner.hpp"
#include "Timer.h"

enum TestCases
{
    No_Obstacle,
    VeryFastObstacles,
    Only_Obstacle_Middle_Slow,
    Choose_Lane,
    Wait_And_Change_Lane,
    Multiple_Lane_Change,
    Continuous_Avoid,
};

using namespace std;

string actionFileName = "../actions.json";


class Cars{
    public:
        Cars(int n, Env env): env(env), cars(n){
        }

        vector<std::list<std::tuple<int, double, double>>> getCars(){
            return cars;
        }

        void step(double t){
            int lane = 0;
            for (auto & l: cars){
                if (!l.size()){
                    l.push_back(std::tuple<int, double, double>{lane, rand() % 9 + 1, rand() % 10 + 10});
                }
                lane++;
            }

            tick(t);

        }

        void shiftPosition(double y, double x){
            std::cout << "shift: " << y << std::endl;
            for (auto & l: cars){
                for (auto & c: l){
                    get<2>(c) = get<2>(c) + y;
                }
            }

        }

    private:
        Env env;
        vector<std::list<std::tuple<int, double, double>>> cars;

        void tick(double t){
            std::cout << "tick: " << t << std::endl;
            for (auto & l: cars){
                for (auto & c: l){
                    get<2>(c) = get<2>(c) - get<1>(c) * t;
                }
            }

            for (auto & l: cars){
                while (l.size () && ((get<2>(l.front()) > env.dist_window) || (get<2>(l.front()) <= 0))){
                    l.pop_front();
                }
            }
        }
        
};

bool testCollisionCheck()
{
    const double speedRes = 3.0;  
    const int lanes = 3;
    const double targetX = 90.;
    const double targetTime = 20.;
    Env env{lanes, 4, 0.1, 0.1, targetTime, targetX, 2, 4}; 
    Cars cars(3, env);
    cars.step(0);

    vector<std::list<std::tuple<int, double, double>>> c = cars.getCars();
    std::vector<std::tuple<int, double, double>> car_vector; 
    for (auto & cc: c){
        for (auto & ccc: cc){
            car_vector.push_back(ccc);
        }
    }
    CollisionChecker collisionchecker(env, car_vector);
    Actions *actions = new Actions(actionFileName, speedRes, collisionchecker, false);

    Planner *planner = new Planner(actions, false);
    Config startConfig(0., 0., 0., 0., 0., 9., 0., 0., 0., 0., make_pair(-1,-1));
    while(true){
        vector<std::list<std::tuple<int, double, double>>> c = cars.getCars();
        std::vector<std::tuple<int, double, double>> car_vector; 
        for (auto & cc: c){
            for (auto & ccc: cc){
                car_vector.push_back(ccc);
            }
        }

        Timer t, t2;
        collisionchecker.updateCars(car_vector);
        t2.reset();   
        pair<vector<vector<double>>, vector<vector<double>>> finalPlan= planner->search(startConfig, targetX - 10, 10);
        cout << t2.elapsed() << endl;
        cout<<"\n Final config, x:"<<finalPlan.first.back()[0]
                                    <<"y:"<<finalPlan.first.back()[1]
                                    <<"v:"<<finalPlan.first.back()[5]
                                    <<"t:"<<finalPlan.first.back()[6]
                                    <<"a:"<<finalPlan.first.back()[7];
        //collisionchecker.visualizePath(finalPlan.first);
        collisionchecker.visualizeSegment(finalPlan.first, finalPlan.second);
       
        vector<double> new_position = finalPlan.first[finalPlan.second.size() - 1];
        cars.shiftPosition(new_position[0], new_position[1]);
        cars.step(new_position[6]);
        startConfig.m_y = new_position[1];
        startConfig.m_v = new_position[5];
    }

    delete actions;
    delete planner;
}


std::vector<std::tuple<int, double, double>> GenerateTestCases(enum TestCases test)
{
    std::vector<std::tuple<int, double, double>> cars;
    // No_Obstacle,
    // VeryFastObstacles,
    // Only_Obstacle_Middle_Slow,
    // Choose_Lane,
    // Multiple_Lane_Change,
    // Wait_And_Change_Lane,
    switch(test)
    {
        case No_Obstacle:
            cars.push_back(std::tuple<int, double, double>(2, 0, 0));
        break;
        case VeryFastObstacles:
            cars.push_back(std::tuple<int, double, double>(0, 30, 75));
            cars.push_back(std::tuple<int, double, double>(1, 30, 75));
            cars.push_back(std::tuple<int, double, double>(2, 30, 75));
        break;
        case Only_Obstacle_Middle_Slow:
            cars.push_back(std::tuple<int, double, double>(0, 30, 75));
            cars.push_back(std::tuple<int, double, double>(1, 5, 75));
            cars.push_back(std::tuple<int, double, double>(2, 30, 75));
        break;
        case Choose_Lane:
            cars.push_back(std::tuple<int, double, double>(0, 0, 25));
            cars.push_back(std::tuple<int, double, double>(1, 5, 75));
            cars.push_back(std::tuple<int, double, double>(2, 5, 60));
        break;
        case Wait_And_Change_Lane:
            cars.push_back(std::tuple<int, double, double>(0, 0, 75));
            cars.push_back(std::tuple<int, double, double>(0, 0, 50));
            cars.push_back(std::tuple<int, double, double>(0, 0, 25));
            cars.push_back(std::tuple<int, double, double>(1, 12,90));
            cars.push_back(std::tuple<int, double, double>(2, 9, 90));
        break;
        case Multiple_Lane_Change:
            cars.push_back(std::tuple<int, double, double>(0, 0, 75));
            cars.push_back(std::tuple<int, double, double>(0, 0, 20));
            cars.push_back(std::tuple<int, double, double>(1, 0, 75));
            cars.push_back(std::tuple<int, double, double>(1, 0, 20));
            cars.push_back(std::tuple<int, double, double>(2, 0, 50));
        break;
        case Continuous_Avoid:
        break;
        default:
            cars.push_back(std::tuple<int, double, double>(0, 0, 75));
            cars.push_back(std::tuple<int, double, double>(0, 0, 20));
            cars.push_back(std::tuple<int, double, double>(1, 0, 75));
            cars.push_back(std::tuple<int, double, double>(1, 0, 20));
            cars.push_back(std::tuple<int, double, double>(2, 0, 50));
    }
    return cars;
}



/*
 *  Specifications for the environment and coliision checker
    // lane, absolute speed, x coord  
    // lanes 0: left, 1: center, 2:right, 
    // units: m and m/s
    // numLanes, laneWidth, discretization for distance, discretization for time, 
    // time window (in secs), distance window (in m)   
    // car width, car length
    //
 */

int main(int argc, char *argv[]){
    const double speedRes = 3.0;  
    const int lanes = 3;
    const double targetX = 90.;
    const double targetTime = 90.;
    bool debugPlanner = false;
    bool debugActions = false; 
    bool plotPathFlag = false;
    bool visualizePathFlag = true;

    // time
    double timeToLoadActions = 0.0;
    double timeToSetupEnv = 0.0;
    double timeToPlan = 0.0;
    double timeToVisualize = 0.0;
    double timeToPlotPlan = 0.0;
    double totalTime = 0.0;

    Timer t, t2;
    t2.reset();   

    //testCollisionCheck();

    // default test case, if no argument provided
    TestCases test= Multiple_Lane_Change;
    if(argc==2)
    {
        int x =0;
        string s = argv[1];
        stringstream parse(s); 
        parse >> x;
        test = (TestCases)x;
    }

    // Environment
    t.reset();
    Env env{lanes, 4, 0.1, 0.1, 100, 100, 2, 4}; 
    std::vector<std::tuple<int, double, double>> cars = GenerateTestCases(test);
    CollisionChecker collisionchecker(env, cars);
    timeToSetupEnv = t.elapsed();

    // Actions
    t.reset();
    Actions *actions = new Actions(actionFileName, speedRes, collisionchecker, debugActions);
    timeToLoadActions = t.elapsed();

    // Planner
    t.reset();
    Planner *planner = new Planner(actions, debugPlanner);
    Config startConfig(0., 0., 0., 0., 0., 9., 0., 0., 0., 0., make_pair(-1,-1));
    pair<vector<vector<double>>, vector<vector<double>>> Plan = planner->search(startConfig, targetX, targetTime);
    vector<vector<double>> &finalPlan(Plan.first);
    timeToPlan = t.elapsed();

    t.reset();
    cout<<"\n Plan size:"<<finalPlan.size()<<endl;
    if(plotPathFlag)
    {
        actions->PlotPath(finalPlan);
    }
    timeToPlotPlan = t.elapsed();

    // Visualize
    t.reset();
    if(visualizePathFlag)
    {
        collisionchecker.visualizePath(finalPlan);
    }
    timeToVisualize = t.elapsed();

    totalTime = t2.elapsed();

    cout<<"\n Time To Setup env"<<timeToSetupEnv;
    cout<<"\n Time To load actions"<<timeToLoadActions;
    cout<<"\n Time To Plan"<<timeToPlan;
    cout<<"\n Time To Plot Plan"<<timeToPlotPlan;
    cout<<"\n Time To Visualize"<<timeToVisualize;
    cout<<"\n Total Time"<<totalTime;
    cout<<"\n Final config, x:"<<finalPlan.back()[0]
                                <<"y:"<<finalPlan.back()[1]
                                <<"v:"<<finalPlan.back()[5]
                                <<"t:"<<finalPlan.back()[6]
                                <<"a:"<<finalPlan.back()[7];
}

