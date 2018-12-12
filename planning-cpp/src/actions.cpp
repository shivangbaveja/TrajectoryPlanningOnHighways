#include "actions.hpp"
#include "CollisionChecker.hpp"

Actions::Actions(string fileName, 
                double speedRes, 
                CollisionChecker & cc, 
                bool debugActions): 
                m_fileName(fileName), m_speedRes(speedRes), m_cc(cc), m_debugActions(debugActions)
{
	ReadActionsJson(m_actions,m_fileName,m_actionCost);
    m_skip = 1;
}

Actions::~Actions()
{

}

void Actions::ReadActionsJson(vector<vector<vector<vector<double> > > >& actions,
                                   string filePath,
                                   vector<vector<double> >& actionsCost)
{
    json j;
    //   PrintFile(fileHandler, 1);
    std::ifstream* fileHandler = new (std::ifstream)(filePath);
    (*fileHandler) >> j;

    actions.clear();
    actionsCost.clear();

    for (json::iterator it = j.begin(); it != j.end(); ++it) {
	
		std::vector<std::vector<std::vector<double> > > speed;
		// std::vector<double> headingCost;
		for (auto it2 = it->begin(); it2 != it->end(); ++it2) {
	    
		    std::vector<std::vector<double> > path;
		    double pathCost = 0;
		    for (auto it3 = it2->begin(); it3 != it2->end(); ++it3) {
		
				std::vector<double> config;
				for (auto it4 = it3->begin(); it4 != it3->end(); ++it4) {
				    config.push_back(*it4);
				}
				path.push_back(config);
		    }
		    // pathCost = GetPathLength(path);
		    // headingCost.push_back(pathCost);
		    speed.push_back(path);
		}
		// actionsCost.push_back(headingCost);
		actions.push_back(speed);
    }

    fileHandler->close();
}

void Actions::PlotPath(std::vector<std::vector<double> >& path)
{
    if (path.size() <= 0)
	return;

    std::vector<double> x, y;

    // cout<<"\n traj starts here:"<<endl;

    // cout<<"\n Final plan";
    for (auto it = path.begin(); it != path.end(); ++it) {
    	std::vector<double> temp = *it;

    	x.push_back(temp.at(0));
    	y.push_back(temp.at(1));
    }

    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(x, y);

    // Set x-axis to interval [0,1000000]
    plt::xlim(0, 120);

    // Set y-axis to interval [0,1000000]
    plt::ylim(-10, 10);

    plt::title("Highway Trajectories for a particular speed");

    // Enable legend.
    //  plt::legend();

    // save figure
    const char* filename = "../finalPlan.png";
    std::cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
}

void Actions::PlotActions(std::vector<std::vector<std::vector<std::vector<double> > > >& actions, int speedId)
{
    if (speedId < 0 || actions.size() <= speedId) {
	std::cout << "cant plot actions which do not exist";
	return;
    }

    std::vector<std::vector<std::vector<double> > > actionsForSpeed = actions[speedId];

    // Set the size of output image = 1200x780 pixels
    plt::figure_size(1200, 780);

    //	Plot all the paths for one heading
    for (auto it = actionsForSpeed.begin(); it != actionsForSpeed.end(); ++it) {
    	cout<<"Plotted one";
	PlotPath(*it);
    }

    // Set x-axis to interval [0,1000000]
    plt::xlim(0, 120);

    // Set y-axis to interval [0,1000000]
    plt::ylim(-10, 10);

    // Add graph title
    plt::title("Highway Trajectories for a particular speed");

    // Enable legend.
    //	plt::legend();

    // save figure
    const char* filename = "../trajectories.png";
    std::cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
}

int Actions::SpeedToIndex(double speed)
{
    double speedSnapped = round(speed);
    int index = ceil(speedSnapped/m_speedRes) -1 ;

    return index;
}

vector<Config> Actions::GetSuccessors(const Config &current)
{
    vector<Config> final;
    bool collision;
    int speedIndex = SpeedToIndex(current.m_v);

    if(m_debugActions)
    {
        cout<<"\n Speed index "<<speedIndex<<"for speed"<<current.m_v; 
    }

    vector< vector< vector< double>>> trajs = m_actions[speedIndex];
    for(int i=0;i<trajs.size();i++)
    {
        vector<vector<double>> traj=trajs[i];

        collision = false;
        double netCloseness = 0.0;
        for(int j=0;j<traj.size();j+=m_skip)
        {

            double closeness = m_cc.checkCollision(traj[j][0] + current.m_x, traj[j][1] + current.m_y, traj[j][6] + current.m_t);
            if(closeness<0)
            {
                collision=true;
                break;
            }
            else
            {
                netCloseness += closeness;
            }
        }

        if(collision)
        {
            continue;
        }
        else
        {
            if(traj.size()>0)
            {
                pair<int,int> indexes=make_pair(speedIndex,i);
                Config newConfig(traj.back()[0] + current.m_x, 
                                 traj.back()[1] + current.m_y, 
                                 traj.back()[2], 
                                 traj.back()[3] + current.m_s, 
                                 traj.back()[4], 
                                 traj.back()[5], 
                                 traj.back()[6] + current.m_t, 
                                 traj.back()[7],
                                 netCloseness,
                                 traj.back()[6],// + fabsf(traj.back()[1])/4.0 + 100.0/netCloseness,          //
                                 indexes);
                if(m_debugActions)
                {
                    newConfig.Print();   
                }
                final.push_back(newConfig);
            }
        }

    }

    return final;
}
