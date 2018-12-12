#include <opencv2/core/utility.hpp>
#include "CollisionChecker.hpp"
#include <vector>
#include <iostream>
#include <utility>
using namespace cv;
CollisionChecker::CollisionChecker(const Env & env, std::vector<state> cars):
    env(env), 
    cars(cars){
    int sizes[] = {static_cast<int>(env.dist_window / env.dist_dis), 
                    static_cast<int>(env.num_lanes * env.lane_width / env.dist_dis)};
    distance_matrix = std::vector<Mat>(env.time_window / env.dist_time, Mat(2, sizes, CV_32FC1, Scalar(0))),
    vis_matrix = std::vector<Mat>(env.time_window / env.dist_time, Mat(2, sizes, CV_32FC1, Scalar(0))),
    constructDistanceMatrix();
}

void CollisionChecker::updateCars(std::vector<state> & cars){
    this->cars = cars;
    constructDistanceMatrix();

}


double CollisionChecker::checkCollision(double xd, double yd, double td){
    //assumes 0.1 x and y discretization and 0.5 t discretization
    int x = static_cast<int>(yd / env.dist_dis);
    int y = static_cast<int>(xd / env.dist_dis);
    int t = static_cast<int>(td / env.dist_time);

    y = env.dist_window / env.dist_dis - y - 1;
    x = (env.num_lanes * env.lane_width / env.dist_dis) / 2 + x;

    if ((y < 0) || (y >= env.dist_window / env.dist_dis))
        return -1;
    if ((x < 0) || (x >= env.num_lanes * env.lane_width / env.dist_dis))
        return -1;
    if (t >= env.time_window / env.dist_time - 1)
        return std::numeric_limits<double>::max();

    Scalar i0 = distance_matrix[t].at<float>(y, x);
    Scalar i1 = distance_matrix[t+1].at<float>(y, x);

    double cost = i0.val[0] * (td / env.dist_time - t) + i1.val[0] * (t + 1 - td / env.dist_time);
    
    return cost < 0.1 ? -1 : cost;
}

void CollisionChecker::constructDistanceMatrix(){
     
    for (int i = 0; i < env.time_window / env.dist_time; i++){
        Mat obstacle_map = computeObstacleMap(env, cars, i);
        distance_matrix[i] = computeDistanceTransform(obstacle_map);
        vis_matrix[i] = getVisualizationMap(env, cars, i);
    }

    //initial_map = getVisualizationMap(env, cars, 0);
}

Mat CollisionChecker::getVisualizationMapatTime(const Env& env, const std::vector<state>& cars, double t){

    int sizes[] = {static_cast<int>(env.dist_window / env.dist_dis), 
                    static_cast<int>(env.num_lanes * env.lane_width / env.dist_dis)};
    Mat obstacle_map(2, sizes, CV_8UC1, Scalar(255));

    for (auto c: cars){
        double lane = std::get<0>(c);
        double speed = std::get<1>(c);
        double dist = std::get<2>(c);
        
        int x = (lane + 0.5) * env.lane_width / env.dist_dis;
        int y = dist / env.dist_dis - (speed / env.dist_dis) * t;
        Rect rect(x - env.car_width / env.dist_dis / 2, y - env.car_length / env.dist_dis / 2, env.car_width / env.dist_dis, env.car_length / env.dist_dis);
        rectangle(obstacle_map, rect, Scalar(0), -1); 

    }
    
    //imshow("obstacle", obstacle_map);
    //waitKey(10);
    return obstacle_map;
    

}
Mat CollisionChecker::getVisualizationMap(const Env& env, const std::vector<state>& cars, int i){

    int sizes[] = {static_cast<int>(env.dist_window / env.dist_dis), 
                    static_cast<int>(env.num_lanes * env.lane_width / env.dist_dis)};
    Mat obstacle_map(2, sizes, CV_8UC1, Scalar(255));

    for (auto c: cars){
        double lane = std::get<0>(c);
        double speed = std::get<1>(c);
        double dist = std::get<2>(c);
        
        int x = (lane + 0.5) * env.lane_width / env.dist_dis;
        int y = dist / env.dist_dis - (speed / env.dist_dis) * ((float)i * env.dist_time);
        Rect rect(x - env.car_width / env.dist_dis / 2, y - env.car_length / env.dist_dis / 2, env.car_width / env.dist_dis, env.car_length / env.dist_dis);
        rectangle(obstacle_map, rect, Scalar(0), -1); 

    }
    
    //imshow("obstacle", obstacle_map);
    //waitKey(10);
    return obstacle_map;
    

}

Mat CollisionChecker::computeObstacleMap(const Env& env, const std::vector<state>& cars, int i){
    int sizes[] = {static_cast<int>(env.dist_window / env.dist_dis), 
                    static_cast<int>(env.num_lanes * env.lane_width / env.dist_dis)};
    Mat obstacle_map(2, sizes, CV_8UC1, Scalar(255));

    for (auto c: cars){
        double lane = std::get<0>(c);
        double speed = std::get<1>(c);
        double dist = std::get<2>(c);
        
        int x = (lane + 0.5) * env.lane_width / env.dist_dis;
        int y = dist / env.dist_dis - (speed /  env.dist_dis) * ((float)i * env.dist_time);
        Rect rect(x - env.car_width / env.dist_dis -2, y - env.car_length / env.dist_dis-2, env.car_width / env.dist_dis * 2 + 4, env.car_length / env.dist_dis * 2 + 4);
        rectangle(obstacle_map, rect, Scalar(0), -1); 
    }
    
    //imshow("obstacle", obstacle_map);
    //waitKey(0);
    return obstacle_map;
    

}

Mat CollisionChecker::computeDistanceTransform(const Mat & obstacle_map){
    Mat distance_map, temp;
    distanceTransform(obstacle_map, distance_map, DIST_L1, 5);

    //normalize(distance_map, temp, 0, 1.0, NORM_MINMAX);

  //  imshow("distance", temp);
  //  waitKey(1);
    return distance_map; 
}

bool CollisionChecker::checkCollision(const Env & env, std::vector<std::pair<int, int>> traj){
    double thresh = 10; 

    for (int i = 0; i < traj.size(); i++){
        int x = traj[i].first; 
        int y = traj[i].second; 
        Scalar dist = distance_matrix[i].at<uchar>(y,x);
        if (dist.val[0] < thresh){
            return true;
        }

    }

    return false;
}

void CollisionChecker::showMap(){
    imshow("obstacle", vis_matrix[0]);
    waitKey(0);
}

void CollisionChecker::visualizeTrajectory(std::vector<std::vector<double>> finalPlan){
    
    Mat temp(vis_matrix[0]);
    for (auto p : finalPlan){
        double xd = p[0];
        double yd = p[1];
        double t = p[6];

        int x = static_cast<int>(yd / env.dist_dis);
        int y = static_cast<int>(xd / env.dist_dis);

        y = env.dist_window / env.dist_dis - y - 1;
        x = (env.num_lanes * env.lane_width / env.dist_dis) / 2 + x;

        int i = static_cast<int>(t / env.dist_time);
        RotatedRect rRect = RotatedRect(Point2f(x,y), Size2f(env.car_width / env.dist_dis,env.car_length / env.dist_dis), - p[2] * 180 / 3.14159);
        Point2f vertices[4];
        rRect.points(vertices);
        for (int i = 0; i < 4; i++)
            line(temp, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));

    }

    imshow("Trajectory", temp);
    waitKey(0);

}

void CollisionChecker::visualizePath(std::vector<std::vector<double>> finalPlan){
    int s = 0;
    for (double i = 0; ; i = i + (env.dist_time) / 5){

        if (i > finalPlan[s + 1][6])
            s++;

        double xd = finalPlan[s][0];
        double yd = finalPlan[s][1];
        double t = finalPlan[s][6];
        double vd = finalPlan[s][5];
        int x = static_cast<int>(yd / env.dist_dis);
        int y = static_cast<int>(xd / env.dist_dis);

        y = env.dist_window / env.dist_dis - y - 1;
        x = (env.num_lanes * env.lane_width / env.dist_dis) / 2 + x;

        std::cout << "x: " << yd << " y: " << xd << " t: " << t << "vd:"<<vd<< std::endl;
        Mat temp = getVisualizationMapatTime(env, cars, i);
        RotatedRect rRect = RotatedRect(Point2f(x,y), Size2f(env.car_width / env.dist_dis,env.car_length / env.dist_dis), - finalPlan[s][2] * 180 / 3.14159);
        Point2f vertices[4];
        rRect.points(vertices);
        for (int j = 0; j < 4; j++)
            line(temp, vertices[j], vertices[(j+1)%4], Scalar(0,255,0));

        imshow("obstacle", temp);
        waitKey(50);

        if (s == finalPlan.size() - 1)
            break;

    }

}

void CollisionChecker::visualizeSegment(std::vector<std::vector<double>> finalPlan, std::vector<std::vector<double>> & firstSegment){
    int s = 0;
    for (double i = 0; ; i = i + (env.dist_time) / 5){

        while ((s < firstSegment.size()) && (i > finalPlan[s + 1][6]))
            s++;

        if (s >= firstSegment.size())
            break;
        
        //int i = static_cast<int>(firstSegment[s][6] / env.dist_time);

        Mat temp = getVisualizationMapatTime(env, cars, i);

        for (auto p = finalPlan.begin() + s; p != finalPlan.begin() + firstSegment.size() + s; p++ ){
            double xd = (*p)[0];
            double yd = (*p)[1];
            double t = (*p)[6];

            int x = static_cast<int>(yd / env.dist_dis);
            int y = static_cast<int>(xd / env.dist_dis);

            y = env.dist_window / env.dist_dis - y - 1;
            x = (env.num_lanes * env.lane_width / env.dist_dis) / 2 + x;

            RotatedRect rRect = RotatedRect(Point2f(x,y), Size2f(env.car_width / env.dist_dis,env.car_length / env.dist_dis), - (*p)[2] * 180 / 3.14159);
            Point2f vertices[4];
            rRect.points(vertices);
            for (int i = 0; i < 4; i++)
                line(temp, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));

            break;

        }
        
        Mat transform(2, 3, CV_32FC1);
        transform = 0;
        std::cout << (finalPlan.begin() + s)->operator[](0) << std::endl;
        transform.at<float>(1, 2) = (finalPlan.begin() + s)->operator[](0) / env.dist_dis;
        transform.at<float>(0, 0) = 1;
        transform.at<float>(1, 1) = 1;
        warpAffine(temp, temp, transform, temp.size(), INTER_LINEAR, BORDER_CONSTANT, 255);
        imshow("Trajectory", temp);
        waitKey(250);

    }

}
