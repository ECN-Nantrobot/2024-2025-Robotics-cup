#pragma once
#include <vector>
#include <cmath>
#include "point.h"

namespace ecn
{

    class Robot
    {
    public:
        Robot(float x, float y, float theta, float wheelBase, float speed, float kp, float ki, float kd, float dt);

        void followPath(const std::vector<Point> &path);
        bool turnToGoalOrientation();
        bool turnToPathOrientation(const std::vector<Point> &path);
        int findClosestPointOnPath(const std::vector<Point> &path);
        float calcAngleError(int target_index, const std::vector<Point> &path);

        void setPose(float x, float y, float theta)
        {
            x_ = x;
            y_ = y;
            theta_ = theta;
        }
        void setTargetTheta(float targetTheta) { targetTheta_ = targetTheta * M_PI / 180.0f; }
        float getX() const { return x_; }
        float getY() const { return y_; }

        void setPosition(float x, float y)
        {
            x_ = x;
            y_ = y;
        }

        float getLeftSpeed() const { return leftSpeed_; }
        float getRightSpeed() const { return rightSpeed_; }

    	float getDt() const { return dt_; }


    private:
        double x_;
        double y_;
        float theta_;
        float wheelBase_;
        float robot_diameter_ = wheelBase_;
        float dt_;

        float leftSpeed_;
        float rightSpeed_;
        float speed_;
        float maxSpeed_ = 10.0;
        bool isStarting_ = true;


        float targetTheta_ = 0.0f;

        // PID parameters
        float kp_, ki_, kd_;
        float prevError_;
        float integral_;

        float computePID(float targetAngle);
        void updatePosition();
    };

} // namespace ecn
