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

#define MAX_LINEAR_SPEED 1.0
#define MAX_ANGULAR_SPEED 0.5
#define IMAGE_WIDTH 640
#define CONFIDENCE 0.5
#define BANANA "46"
#define MOUSE "64"
#define SLOW1_FRAC 0.1
#define SLOW2_FRAC 0.2
#define SLOW3_FRAC 0.3
#define SLOW4_FRAC 0.4
#define SLOW5_FRAC 0.5
#define SLOW6_FRAC 0.6
#define SLOW7_FRAC 0.7
#define SLOW8_FRAC 0.8
#define SLOW9_FRAC 0.9
#define FULL_SPEED_FRAC 1.0

typedef enum {stopped, slow_1, slow_2, slow_3, slow_4, slow_5, slow_6, slow_7, slow_8, slow_9, moving} robot_state_t;

class Control : public rclcpp::Node
{
public:
  Control(): Node("control") {
    subscription_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      "detections_output", 10, std::bind(&Control::subscriber_callback, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    current_state = stopped;
    detect = false;
  }

private:
  void subscriber_callback(const vision_msgs::msg::Detection2DArray::SharedPtr detection_p) {
    uint detection_count = detection_p ->detections.size();
    detect = false;
    for (uint i = 0; i < detection_count; i++) {
      vision_msgs::msg::ObjectHypothesisWithPose *results =detection_p->detections[i].results.data();
      id = results->hypothesis.class_id;
      score = results->hypothesis.score;
      if ((id == MOUSE) && (score > CONFIDENCE)) {
        bbox_center_x = detection_p->detections[i].bbox.center.position.x;
        bbox_center_y = detection_p->detections[i].bbox.center.position.y;
        bbox_width = detection_p->detections[i].bbox.size_x;
        bbox_height = detection_p->detections[i].bbox.size_y;
        cout << "Detected Object, id = " << id << ", score = ";
        printf("%4.2f\n", score);
        // cout << ", bbox(" 
        //     << bbox_center_x << "," << bbox_center_y << "), bbox size: w = " 
        //     << bbox_width << ", h = " << bbox_height << endl;
        detect = true;
        break;
      }
    }
   
    auto motors_message = geometry_msgs::msg::Twist();

    switch (current_state) {
      case (stopped):
        if (detect) {
          next_state = slow_1;
          speed_fraction = SLOW1_FRAC;
        } else {
          next_state = stopped;
        }
        break;
      case (slow_1):
        if (detect) {
          next_state = slow_2;
          speed_fraction = SLOW2_FRAC;
        } else {
          next_state = stopped;
        }
        break;
      case (slow_2):
        if (detect) {
          next_state = slow_3;
          speed_fraction = SLOW3_FRAC;
        } else {
          next_state = slow_1;
          speed_fraction = SLOW1_FRAC;
        }
        break;
      case (slow_3):
        if (detect) {
          next_state = slow_4;
          speed_fraction = SLOW4_FRAC;
        } else {
          next_state = slow_2;
          speed_fraction = SLOW2_FRAC;
        }
        break;
      case (slow_4):
        if (detect) {
          next_state = slow_5;
          speed_fraction = SLOW5_FRAC;
        } else {
          next_state = slow_3;
          speed_fraction = SLOW3_FRAC;
        }
        break;
      case (slow_5):
        if (detect) {
          next_state = slow_6;
          speed_fraction = SLOW6_FRAC;
        } else {
          next_state = slow_4;
          speed_fraction = SLOW4_FRAC;
        }
        break;
      case (slow_6):
        if (detect) {
          next_state = slow_7;
          speed_fraction = SLOW7_FRAC;
        } else {
          next_state = slow_5;
          speed_fraction = SLOW5_FRAC;
        }
        break;
      case (slow_7):
        if (detect) {
          next_state = slow_8;
          speed_fraction = SLOW8_FRAC;
        } else {
          next_state = slow_6;
          speed_fraction = SLOW6_FRAC;
        }
        break;
      case (slow_8):
        if (detect) {
          next_state = slow_9;
          speed_fraction = SLOW9_FRAC;
        } else {
          next_state = slow_7;
          speed_fraction = SLOW7_FRAC;
        }
        break;
      case (slow_9):
        if (detect) {
          next_state = moving;
          speed_fraction = FULL_SPEED_FRAC;
        } else {
          next_state = slow_8;
          speed_fraction = SLOW8_FRAC;
        }
        break;
      case (moving):
        if (detect) {
          next_state = moving;
          speed_fraction = FULL_SPEED_FRAC;
        } else {
          next_state = slow_9;
          speed_fraction = SLOW9_FRAC;
        }
        break;
      default:
        next_state = stopped;
    }

    if (next_state != stopped) {
      // object detected with high confidence. 
      // The bbox_center_y controls the linear.x speed, 
      // Transform the bbox_center_y to a factor between 0 and 1 and multiply by MAX_LINEAR_SPEED
      motors_message.linear.x = speed_fraction * ((-bbox_center_y / 640.0) + 1);
      // The bbox_center_x controls the angular.z speed
      // Transform the bbox_center_x to a factor between -1 and +1 and multiple by MAX_ANGULAR SPEED:
      motors_message.angular.z = (((2.0 * bbox_center_x) / IMAGE_WIDTH) - 1.0) * speed_fraction * MAX_ANGULAR_SPEED;
    } else {
      // stop
      motors_message.linear.x = 0;
      motors_message.angular.z = 0;
    }
    // publish the motors_message;
    publisher_->publish(motors_message);

    cout << "current_state = " << current_state << endl;
    cout << "next_state = " << next_state << endl;
    
    // change state
    current_state = next_state;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  robot_state_t current_state, next_state;
  bool detect, move;
  string id;
  double score, bbox_center_x, bbox_center_y, bbox_width, bbox_height;
  //geometry_msgs::msg::Twist motors_message;
  double speed_fraction;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Control>());
  rclcpp::shutdown();
  return 0;
}
