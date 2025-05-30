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

        Point getPosition() const { return Point{x_, y_}; }

        void generateStraightPath()
        {
            Point start = Point{x_, y_};
            Point end = goals[current_goal_index].point;

            float dx = end.x - start.x;
            float dy = end.y - start.y;
            float distance = std::sqrt(dx * dx + dy * dy);
            int steps = static_cast<int>(distance / path_cm_per_point_);

            for (int i = 0; i <= steps; ++i)
            {
                float t = static_cast<float>(i) / steps;
                float x = start.x + t * dx;
                float y = start.y + t * dy;

                ecn::Point point;
                point.x = x;
                point.y = y;
                path_.push_back(point);
            }
        }

        std::vector<Point> getPath() const
        {
            return path_;
        }

        void turn(float angle_error);

        void setIsStarting(bool value)
        {
            is_starting_ = value;
        }

        void stop()
        {
            leftSpeed_ = 0;
            rightSpeed_ = 0;
            is_starting_ = true;
        }

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

        bool moveStraight(float target_x, float target_y);

        std::vector<Point> path_;

        std::vector<Pose> goals;
        int getCurrentGoalindex() const { return current_goal_index; }
        void incrementCurrentGoalIndex() { current_goal_index += 1; }


        float distanceToGoal(const Point &p) const { return std::hypot(p.x - x_, p.y - y_); };

        bool start_turning = true;

    private:
        float turnsignal_limit = 3.0;
        float reducing_factor = 0;
        float max_control_output_ = 30.0; // (also max wheel speed for turnig)
        float min_control_output = 0.5;   // minimum control output to avoid oscillations

        float path_cm_per_point_ = 2.0;

        int current_goal_index = 1;

        volatile float x_;
        volatile float y_;
        volatile float theta_;

        float targetTheta_;

        float target_speed_;

        float wheelBase_ = 37.6;
        float robot_diameter_ = wheelBase_;

        float leftSpeed_;
        float rightSpeed_;

        float dt_ = 0.05; // in seconds
        float dt_inms_ = dt_ * 1000.0;

        bool is_starting_ = true;
        float starting_speed_ = 0;

        // PID parameters
        float kp_,
            ki_, kd_;
        float prevError_;
        float integral_;

        float computePID(float targetAngle);
    };

} // namespace ecn
