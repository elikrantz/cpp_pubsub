// importing of packages goes here
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
// #include "nav2_behavior_tree/ros_topic_logger.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

// const std::string RVIZ_NAV_GOAL = "move_base_simple/goal";
const std::string RVIZ_NAV_GOAL = "/goal_pose2";

// variables
// geometry_msgs::msg::PoseStamped goals[] = new geometry_msgs::msg::PoseStamped[20];

std::vector<geometry_msgs::msg::PoseStamped> goals = {};
int goalNum = 0;
bool arrivedAtGoal = false;

std::mutex mtx;
std::condition_variable cv;
bool ready = false;

bool needToBreak = false;
std::thread looping_thread_;

// creation of node for proccessing of depth camera data



class goalHandler : public rclcpp::Node {
public:
    goalHandler() : Node("goalHandle") {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            RVIZ_NAV_GOAL, 10, std::bind(&goalHandler::testingReading, this, std::placeholders::_1));
        
        behav_log_sub = this->create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
            "/behavior_tree_log", 10, std::bind(&goalHandler::logCallback, this, std::placeholders::_1));
            
        reset_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/reset_button_topic", 10, std::bind(&goalHandler::resetFunc, this, std::placeholders::_1));

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose",10);


        // runLoop(); // runLoop not cycling


        looping_thread_ = std::thread(&goalHandler::waitingThread, this);


        // std::lock_guard<std::mutex> lock(mtx);

        // std::this_thread::sleep_for(std::chrono::seconds(2)); // Simulate some work
        // {
        //     std::lock_guard<std::mutex> lock(mtx);
        //     ready = true;
        // }
        // cv.notify_one();
        // startThread();
    }
    
private:
    void testingReading(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received pose: x=%f, y=%f, z=%f", 
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        geometry_msgs::msg::PoseStamped goal;
        goal.pose = msg->pose;
        // std::lock_guard<std::mutex> lock(mtx);
        // ready = true;
        // cv.notify_one();
        goals.push_back(goal);
        RCLCPP_INFO(this->get_logger(), "Number of Goals Left: %zu", goals.size());
        runLoop();
        // if button pub msg is pressed, set goals to zero
    }

    // void startThread() {
    //     std::thread worker(worker_thread);

    //     std::this_thread::sleep_for(std::chrono::seconds(2)); // Simulate some work
    //     {
    //         std::lock_guard<std::mutex> lock(mtx);
    //         ready = true;
    //     }
    //     cv.notify_one(); // Notify the waiting thread

    //     worker.join();
    // }

    void worker_thread() {
        std::unique_lock<std::mutex> lock(mtx);
        // cv.wait(lock, []{ return arrivedAtGoal; }); // Wait until ready is true
        cv.wait(lock, []{ return (ready); }); // Wait until ready is true
        std::cout << "Worker thread is processing." << std::endl;
        runLoop();
    }

    void waitingThread() {
        while (true) {
            if (needToBreak) {
                break;
            }

            RCLCPP_INFO(this->get_logger(), "If Arrived At Goal 2: %s", (arrivedAtGoal ? "TRUE" : "FALSE"));
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, []{ return (arrivedAtGoal); });
            RCLCPP_INFO(this->get_logger(), "Arrived at goal");
            if (goalNum % 2 == 0) {
                digSequence();
            } else if (goalNum % 2 == 1) {
                dropSequence();
            }
            
            if (goals.size() > 0) {
                // goal_pub_->publish(goals[0]);
                RCLCPP_INFO(this->get_logger(), "Publish goal");
                arrivedAtGoal = false;
            } else {
                arrivedAtGoal = false;
                RCLCPP_INFO(this->get_logger(), "Out of goals");
            }

            goalNum += 1;
            goals.erase(goals.begin());
            running = false;
            // looping_thread_.join();
        }
    }

    void resetFunc(const std_msgs::msg::String::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Recived Goal");
        // std::lock_guard<std::mutex> lock(mtx);
        arrivedAtGoal = true;
        RCLCPP_INFO(this->get_logger(), "If Arrived At Goal: %s", (arrivedAtGoal ? "TRUE" : "FALSE"));
        
        cv.notify_all();
        runLoop();
    }
    
    bool running = false;
    void runLoop() {
        if (goalNum == 0 && goals.size() > 0) {
            // goal_pub_->publish(goals[0]);
            RCLCPP_INFO(this->get_logger(), "Publish goal");
            goalNum += 1;
            arrivedAtGoal = false;
        }
        bool goalStatus = checkIfAtGoal();
        // RCLCPP_INFO(this->get_logger(), "Running");
        // bool goalStatus = false;
        // if (goalStatus && goals.size() > 0) {
        if (!running && goals.size() > 0) {
            // looping_thread_.join();
            running = true;
            // looping_thread_ = std::thread(&goalHandler::waitingThread, this);
        }
        // runLoop();
    }

    void logCallback(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg)
    {
        for (const auto& event : msg->event_log)
        {
            if (event.node_name == "NavigateRecovery")
            {
                // RCLCPP_INFO(this->get_logger(), "NavigateRecovery Log: %s", event.current_status.c_str());
                
                // You can check for specific statuses as well
                if (event.current_status == "SUCCESS") {
                    RCLCPP_INFO(this->get_logger(), "Recovery succeeded.");

                    std::lock_guard<std::mutex> lock(mtx);
                    arrivedAtGoal = true;
                    cv.notify_one();
                } else if (event.current_status == "RUNNING") {
                    // RCLCPP_INFO(this->get_logger(), "Recovery is running.");
                    // arrivedAtGoal = false;
                } else if (event.current_status == "IDLE") {
                    // RCLCPP_INFO(this->get_logger(), "Recovery is idle.");
                    // arrivedAtGoal = false;
                }
                runLoop();
            }
        }
    }

    bool checkIfAtGoal() {
        // TODO: create function that checks if robot reached the goal
        // RCLCPP_INFO(this->get_logger(), "If Arrived At Goal: %s", (arrivedAtGoal ? std::string("true") : std::string("false")));
        if (arrivedAtGoal == true) {
            // RCLCPP_INFO(this->get_logger(), "At Goal.");
        }
        // RCLCPP_INFO(this->get_logger(), "If Arrived At Goal: %s", (arrivedAtGoal ? "TRUE" : "FALSE"));
        // arrivedAtGoal = false;
        return arrivedAtGoal;
    }

    void digSequence() {
        // TODO: create a dig sequence (motor movements and actuator movements)

        RCLCPP_INFO(this->get_logger(), "Running Temp Dig Sequence");

        
    }

    void dropSequence() {
        // TODO: create a dig sequence (motor movements and actuator movements)
        RCLCPP_INFO(this->get_logger(), "Running Temp Dropping Sequence");
    }

    // std::thread looping_thread_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr behav_log_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reset_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

};

int main (int argc, char *argv[])
{
    // std::thread worker(worker_thread);

    // std::this_thread::sleep_for(std::chrono::seconds(2)); // Simulate some work
    // {
    //     std::lock_guard<std::mutex> lock(mtx);
    //     ready = true;
    // }
    // cv.notify_one(); // Notify the waiting thread

    // worker.join();

    rclcpp::init(argc, argv);
    auto node = std::make_shared<goalHandler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    needToBreak = true;
    arrivedAtGoal = true;
    cv.notify_all();
    looping_thread_.join();
    return 0;
}