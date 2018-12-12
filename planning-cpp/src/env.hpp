#ifndef ENV_H
#define ENV_H

    // numLanes, laneWidth, discretization for distance, discretization for time, 
    // time window (in secs), distance window (in m)   
struct Env{
    int num_lanes;
    double lane_width;
    double dist_dis;
    double dist_time;      
    double time_window;      
    double dist_window;
    double car_width;
    double car_length;
};

#endif
