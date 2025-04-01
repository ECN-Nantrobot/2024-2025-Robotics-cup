#pragma once
#include <vector>
#include <cmath>
#include "point.h"
#include "pose.h"

namespace ecn
{

    class Robot
    {
    public:
        Robot(float x, float y, float theta, float target_speed, float kp, float ki, float kd);

        void followPath();
        bool turnToGoalOrientation();
        bool turnToPathOrientation();
        int findClosestPointOnPath();
        float calcAngleError(int target_index);

        void setIsStarting(bool isStarting) { isStarting_ = isStarting; }


        float getLeftSpeed() const { return leftSpeed_; }
        float getRightSpeed() const { return rightSpeed_; }

        float getDt() const { return dt_; }
        float getDtInMs() const { return dt_inms_; }

        volatile float getX() const { return x_; }
        volatile float getY() const { return y_; }
        volatile float getTheta() const { return theta_; }


        void setTargetTheta(float targetTheta) { targetTheta_ = targetTheta; }

        void setX(volatile float x) { x_ = x; }
        void setY(volatile float y) { y_ = y; }
        void setTheta(volatile float theta) { theta_ = theta; }

        void setPosition(volatile float x, volatile float y)
        {
            x_ = x;
            y_ = y;
        }
        void setSpeed(float speed)
        {
            target_speed_ = speed;
        }

        void setPID(float P, float I, float D)
        {
            kp_ = P;
            ki_ = I;
            kd_ = D;
        }

        void setPose(volatile float x, volatile float y, volatile float theta)
        {
            x_ = x;
            y_ = y;
            theta_ = theta;
        }

        // void updatePositionFromMotors(volatile float d_center, volatile float thetaMid, volatile float dTheta)
        // {
        //     x_ += d_center * cos(thetaMid);
        //     y_ += d_center * sin(thetaMid);
        //     theta_ += dTheta;
        // }

        void updatePosition();

        std::vector<Point> path_;

        std::vector<Pose> goals;
        int getCurrentGoalindex() const { return current_goal_index; }
        void incrementCurrentGoalIndex() { current_goal_index += 1; }

        float getLeftSpeed() { return leftSpeed_; }
        float getRightSpeed() { return rightSpeed_; }

        float distanceToGoal(const Point &p) const { return std::hypot(p.x - x_, p.y - y_); };

        bool start_turning = true;

    private :
        float turnsignal_limit = 1.0;
        float reducing_factor = 0;
        int current_goal_index = 1;

        volatile float x_;
        volatile float y_;
        volatile float theta_;

        float targetTheta_;

        float target_speed_;

        float wheelBase_ = 18.5;
        float robot_diameter_ = wheelBase_;

        float leftSpeed_;
        float rightSpeed_;

        float dt_ = 0.05; //in seconds
        float dt_inms_ = dt_ * 1000.0;

        bool isStarting_ = true;

        // PID parameters
        float kp_, ki_, kd_;
        float prevError_;
        float integral_;

        float computePID(float targetAngle);
    };

} // namespace ecn
