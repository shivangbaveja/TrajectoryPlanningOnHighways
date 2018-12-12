//
// Created by danendra on 12/2/18.
//

#include "planner.hpp"
#include <utility>


Planner::Planner(Actions *actions, bool debug):m_actions(actions), m_debug(debug)
{

}

Planner::~Planner()
{
    //  Return the memory allocated for all the nodes
    for(auto it=m_nodeMap.begin();it!=m_nodeMap.end();++it)
    {
        delete (*it);
    } 
}

void Planner::InitializeAstar()
{
    m_aStarPQ = priority_queue<Node*,vector<Node*>,NodeCompare>();
    m_closedSet.clear();
    m_nodeMap.clear();
    m_openSet.clear();
}

//TODO: Choose better Heuristic
const double Planner::Heuristic (double x1, double y1, double x2, double y2)
{
    double euclideanDist = sqrt(pow(x1-x2, 2) + pow(y1 - y2, 2));
    return euclideanDist;
}

void Planner::AddToClosedSet(uset &closedSet, Node* node)
{
    closedSet.insert(node);
}

Node* Planner::CheckIfInClosedSet(uset &closedSet, Node* node)
{
    auto it= closedSet.find(node);

    if(it!=closedSet.end())
    {
        return (*it);
    }
    else
    {
        return nullptr;
    }
}

void Planner::UpdateTrajectory(vector<vector<double>> &traj, double x, double y, double s, double t)
{
    for(int i=0;i<traj.size();i++)
    {
        traj[i][0] += x;
        traj[i][1] += y;
        traj[i][3] += s;
        traj[i][6] += t;
    }
}

double Planner::GetHeuristic(Config config, double targetX, double targetTime)
{
  double time = (targetX - config.m_x)/21.0;
  return time;
}

//vector of trajectory, vector of configs, vector of config values
pair<vector<vector<double>>,
    vector<vector<double>>>
Planner::search(Config start,
                                          double targetX,
                                          double targetTime)
{
    InitializeAstar();
    double heuristic = GetHeuristic(start, targetX, targetTime);
    Node* startNode = new Node(nullptr, start, 0.0, heuristic);
    m_nodeMap.push_back(startNode);
    m_aStarPQ.push(startNode);
    m_openSet.insert(startNode);

    Node *curNode;

    while (!m_aStarPQ.empty())
    {
          curNode = m_aStarPQ.top();
          m_aStarPQ.pop();

          if(m_debug)
          {
            cout<<"\n Exanding node"<<endl;
            curNode->config.Print();
          }

          if(CheckIfInClosedSet(m_closedSet, curNode) != nullptr)
          {
             continue;
          }
          else
          {
             AddToClosedSet(m_closedSet, curNode);
          }

          // Check if Goal
          if(curNode->config.m_x >= (targetX-0.1) || curNode->config.m_t > targetTime)
          {
            cout<<"\n Found goal:"<<curNode->config.m_x<<"At t"<<curNode->config.m_t<<endl;
            break;
          }

          vector<Config> successors = m_actions->GetSuccessors(curNode->config);

          for(int i=0;i<successors.size();i++)
          {
            double heuristic = GetHeuristic(successors[i], targetX, targetTime);
            Node* newNode = new Node(curNode, successors[i], curNode->g_val + successors[i].m_timeCost, heuristic); 

            if(CheckIfInClosedSet(m_closedSet, newNode) != nullptr)
            {           
                delete newNode;
                continue;
            }
            else
            {
                Node *tryNode = CheckIfInClosedSet(m_openSet, newNode);
             
                if(tryNode==nullptr)
                {
                    m_nodeMap.push_back(newNode);
                    m_aStarPQ.push(newNode);
                    m_openSet.insert(newNode);
                }
                else if(tryNode->g_val > curNode->g_val + successors[i].m_timeCost)
                {
                    m_nodeMap.push_back(newNode);
                    m_aStarPQ.push(newNode);
                    m_openSet.insert(newNode);
                }
                else
                {
                    delete newNode;
                }
            }
          } 
    }

    vector<vector<double>> finalPlan;
    vector<vector<double>> firstSegment;
    Node* temp=curNode;
    while(temp!=nullptr)
    {
        if(temp->config.m_id.first==-1)
        {
            break;
        }

        vector<vector<double>> traj = m_actions->m_actions[temp->config.m_id.first][temp->config.m_id.second];
        firstSegment = m_actions->m_actions[temp->config.m_id.first][temp->config.m_id.second];

        double x = temp->config.m_x - traj.back()[0];
        double y = temp->config.m_y - traj.back()[1];
        double s = temp->config.m_s - traj.back()[3];
        double t = temp->config.m_t - traj.back()[6];

        UpdateTrajectory(traj, x, y, s, t);
        UpdateTrajectory(firstSegment, x, y, s, t);
   
        reverse(traj.begin(),traj.end());

        finalPlan.insert(finalPlan.end(), traj.begin(), traj.end());
        temp = temp->parentNode;
    }    

    reverse(finalPlan.begin(),finalPlan.end());
    return pair<vector<vector<double>>, vector<vector<double>>>(finalPlan, firstSegment);
}
