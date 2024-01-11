#include "dynamixel_sdk/dynamixel_sdk.h"
#include <cstdio>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/detail/int16__struct.hpp>
#include <std_msgs/msg/int16.hpp>

class motor_controll : public rclcpp::Node {
public:
  motor_controll(std::string name) : rclcpp::Node(name) {
    this->declare_parameter("port", "/dev/ttyACM0");
    this->declare_parameter("BAUDRATE", 1000000);
    this->declare_parameter("arm-motor-id", 1);
    this->declare_parameter("tool-motor-id", 0);

    std::string port = this->get_parameter("port").as_string();
    arm_motor_id_ = this->get_parameter("arm-motor-id").as_int();
    tool_motor_id_ = this->get_parameter("tool-motor-id").as_int();
    BAUDRATE_ = this->get_parameter("BAUDRATE").as_int();

    portHandler_ = dynamixel::PortHandler::getPortHandler(port.c_str());
    packetHandler_ =
        dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION_);

    tool_pos_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        "tool_angle", 10,
        std::bind(&motor_controll::tool_motor_callback, this,
                  std::placeholders::_1));
    arm_pos_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        "arm_angle", 10,
        std::bind(&motor_controll::arm_motor_callback, this,
                  std::placeholders::_1));

    // Open Serial Port
    dxl_comm_result_ = portHandler_->openPort();
    if (dxl_comm_result_ == false) {
      RCLCPP_ERROR(rclcpp::get_logger("read_write_node"),
                   "Failed to open the port!");
      return;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("read_write_node"),
                  "Succeeded to open the port.");
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    dxl_comm_result_ = portHandler_->setBaudRate(BAUDRATE_);
    if (dxl_comm_result_ == false) {
      RCLCPP_ERROR(rclcpp::get_logger("read_write_node"),
                   "Failed to set the baudrate!");
      return;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("read_write_node"),
                  "Succeeded to set the baudrate.");
    }

    setupDynamixel(tool_motor_id_,0);
    setupDynamixel(arm_motor_id_,10);
  }

  bool tool_motor_callback(std_msgs::msg::Int16::ConstSharedPtr msg) {

    float scale =  0.29297;


    dxl_comm_result_ = packetHandler_->write4ByteTxRx(
        portHandler_, (uint8_t)tool_motor_id_, ADDR_GOAL_POSITION_, (int)(msg->data/scale),
        &dxl_error_);

    if (dxl_comm_result_ != COMM_SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "%s",
                  packetHandler_->getTxRxResult(dxl_comm_result_));
    } else if (dxl_error_ != 0) {
      RCLCPP_INFO(this->get_logger(), "%s",
                  packetHandler_->getRxPacketError(dxl_error_));
    } else {
      RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]",
                  tool_motor_id_, msg->data);
    }
  };

  bool arm_motor_callback(std_msgs::msg::Int16::ConstSharedPtr msg) {
    // dxl_comm_result_ = packetHandler_->write4ByteTxRx(
    //     portHandler_, (uint8_t)arm_motor_id_, ADDR_speed_, 10, &dxl_error_);

    // if (dxl_comm_result_ != COMM_SUCCESS) {
    //   RCLCPP_INFO(this->get_logger(), "%s",
    //               packetHandler_->getTxRxResult(dxl_comm_result_));
    // } else if (dxl_error_ != 0) {
    //   RCLCPP_INFO(this->get_logger(), "%s",
    //               packetHandler_->getRxPacketError(dxl_error_));
    // }

    float scale =  0.29297;

    
    if(msg->data < 70 || msg->data > 200){
      RCLCPP_INFO(this->get_logger(), "requested pose in out of bounds");
      return false;
    }

    dxl_comm_result_ = packetHandler_->write4ByteTxRx(
        portHandler_, (uint8_t)arm_motor_id_, ADDR_GOAL_POSITION_, (int)(msg->data/scale),
        &dxl_error_);

    if (dxl_comm_result_ != COMM_SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "%s",
                  packetHandler_->getTxRxResult(dxl_comm_result_));
    } else if (dxl_error_ != 0) {
      RCLCPP_INFO(this->get_logger(), "%s",
                  packetHandler_->getRxPacketError(dxl_error_));
    } else {
      RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]",
                  tool_motor_id_, msg->data);
    }
    return true;
  };

  void setupDynamixel(uint8_t dxl_id,uint8_t speed) {
    // Use Position Control Mode
    dxl_comm_result_ = packetHandler_->write1ByteTxRx(
        portHandler_, dxl_id, ADDR_OPERATING_MODE_, 3, &dxl_error_);

    if (dxl_comm_result_ != COMM_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("read_write_node"),
                   "Failed to set Position Control Mode.");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("read_write_node"),
                  "Succeeded to set Position Control Mode.");
    }
    // Enable Torque of DYNAMIXEL
    dxl_comm_result_ = packetHandler_->write1ByteTxRx(
        portHandler_, dxl_id, ADDR_TORQUE_ENABLE_, 1, &dxl_error_);

    if (dxl_comm_result_ != COMM_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("read_write_node"),
                   "Failed to enable torque.");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("read_write_node"),
                  "Succeeded to enable torque.");
    }
    // Enable set speed of arm
    dxl_comm_result_ = packetHandler_->write4ByteTxRx(
        portHandler_, dxl_id, ADDR_speed_, speed, &dxl_error_);

    if (dxl_comm_result_ != COMM_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("read_write_node"),
                   "Failed to set speed.");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("read_write_node"),
                  "Succeeded to set speed to %i" ,speed);
    }
  }

private:
  int arm_motor_id_;
  int tool_motor_id_;

  int dxl_comm_result_;
  uint8_t dxl_error_ = 0;

  int ADDR_OPERATING_MODE_ = 8;
  int ADDR_TORQUE_ENABLE_ = 24;
  int ADDR_GOAL_POSITION_ = 30;
  int ADDR_speed_ = 32;
  int ADDR_PRESENT_POSITION_ = 36;

  float PROTOCOL_VERSION_ = 1.0;

  int BAUDRATE_ = 1000000;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr tool_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr arm_pos_sub_;
};

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);
  auto motor = std::make_shared<motor_controll>("motor_controller");
  rclcpp::spin(motor);

  return 0;
}
