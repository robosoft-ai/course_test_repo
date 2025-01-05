#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include <chrono>
#include <iostream>
#include <string>

using std::placeholders::_1;
using namespace std;
using namespace std::chrono_literals;

#define MAX_LINEAR_SPEED 0.8
#define MAX_ANGULAR_SPEED 0.5
#define IMAGE_WIDTH 640
#define CONFIDENCE 0.85

//typedef enum {stopped, moving} robot_state_t;

class Control : public rclcpp::Node
{
public:
  Control(): Node("control") {
    subscription_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      "detections_output", 10, std::bind(&Control::subscriber_callback, this, _1));

    //publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    //current_state = stopped;
    move = false;
  }

private:
  void subscriber_callback(const vision_msgs::msg::Detection2DArray::SharedPtr detection_p) {
    uint detection_count = detection_p ->detections.size();
    for (uint i = 0; i < detection_count; i++) {
      vision_msgs::msg::ObjectHypothesisWithPose *results =detection_p->detections[i].results.data();
      id = results->hypothesis.class_id;
      score = results->hypothesis.score;
      bbox_center_x = detection_p->detections[i].bbox.center.position.x;
      bbox_center_y = detection_p->detections[i].bbox.center.position.y;
      bbox_width = detection_p->detections[i].bbox.size_x;
      bbox_height = detection_p->detections[i].bbox.size_y;
      cout << "Detected Object, id = " << id << ", score = ";
      printf("%4.2f", score);
      cout << ", bbox(" 
           << bbox_center_x << "," << bbox_center_y << "), bbox size: w = " 
           << bbox_width << ", h = " << bbox_height << endl;
    }
    cout << "*** end of detection frame ***" << endl << endl;

    auto motors_message = geometry_msgs::msg::Twist();
    if ((id == "2") && (score > CONFIDENCE)) {
      // object detected with high confidence. 
      move = true;
      // The bbox_center_y controls the linear.x speed, 
      // Transform the bbox_center_y to factor between 0 and 1 and multiply by MAX_LINEAR_SPEED
      motors_message.linear.x = MAX_LINEAR_SPEED * ((-bbox_center_y / 640.0) + 1);
      // The bbox_center_x controls the angular.z speed
      // Transform the bbox_center_x to factor between -1 and +1 and multiple by MAX_ANGULAR SPEED:
      motors_message.angular.z = (((2.0 * bbox_center_x) / IMAGE_WIDTH) - 1.0) * MAX_ANGULAR_SPEED;
    } else {
      // stop
      move = false;
      motors_message.linear.x = 0;
      motors_message.angular.z = 0;
    }
    // publish the motors_message;
    publisher_->publish(motors_message);

  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  //robot_state_t current_state, next_state;
  bool move;
  string id;
  double score, bbox_center_x, bbox_center_y, bbox_width, bbox_height;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Control>());
  rclcpp::shutdown();
  return 0;
}
