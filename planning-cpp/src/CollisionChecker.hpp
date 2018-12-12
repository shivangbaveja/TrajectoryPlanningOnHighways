#ifndef COLLISIONCHECKER_H
#define COLLISIONCHECKER_H
#include <tuple>
#include <vector>
#include <utility>

#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <stdio.h>
#include "env.hpp"

class CollisionChecker{
    using state = std::tuple<int, double, double>; //lane, speed, distance
    public:
        CollisionChecker(const Env & env, std::vector<state> cars); 
        void updateCars(std::vector<state> & cars);
        bool checkCollision(const Env & env, std::vector<std::pair<int, int>> traj);
        double checkCollision(double xd, double yd, double td);
        void showMap();
        void visualizePath(std::vector<std::vector<double>> finalPlan);
        void visualizeTrajectory(std::vector<std::vector<double>> finalPlan);
        void visualizeSegment(std::vector<std::vector<double>> finalPlan, std::vector<std::vector<double>>& firstSegment);

    private:
        Env env;
        std::vector<cv::Mat> distance_matrix;
        std::vector<cv::Mat> vis_matrix;
        std::vector<state> cars;
        cv::Mat initial_map;

        void constructDistanceMatrix();
        cv::Mat computeObstacleMap(const Env& env, const std::vector<state>& cars, int i);
        cv::Mat getVisualizationMap(const Env& env, const std::vector<state>& cars, int i);
        cv::Mat getVisualizationMapatTime(const Env& env, const std::vector<state>& cars, double t);
        cv::Mat computeDistanceTransform(const cv::Mat & obstacle_map);
};

#endif
