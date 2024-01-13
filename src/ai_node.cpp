#include "axidma.h"
#include "cv_bridge/cv_bridge.h"
#include "iostream"
#include "opencv4/opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "xnn_inference.h"
#include <cstdint>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#define UIO_DMA_N 0

#define DEVICE_FILENAME "/dev/reservedmemLKM"
#define MAX_LENGTH 0x01000000 // Bytes

class ai : public rclcpp::Node {
public:
  ai(std::string name) : rclcpp::Node(name), dma_(UIO_DMA_N, 0x1000) {
    setupDma();
    setupAi();

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10,
        std::bind(&ai::image_callback, this, std::placeholders::_1));
  }

  bool image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::Mat image = cv_ptr->image;
    const int cropSize[2] = {400, 350};
    const int offsetW = (image.cols - cropSize[0]) / 2;
    const int offsetH = (image.rows - cropSize[1]) / 2;
    const cv::Rect roi(offsetW, offsetH, cropSize[0], cropSize[1]);
    cv::Mat cropped_image = image(roi).clone();
    cv::Mat Small_image,greyMat;
    cv::resize(cropped_image, Small_image, cv::Size(10, 10));
    cv::cvtColor(Small_image, greyMat, CV_BGR2GRAY);

    memcpy(&shared_data_,&greyMat.data,sizeof(uint32_t)*100);
    dma_.SimpleTransfer(0, sizeof(uint32_t)*100, DMA_DIR_DMA_TO_DEVICE);
    dma_.SimpleTransfer(sizeof(uint32_t)*200, sizeof(uint32_t), DMA_DIR_DEVICE_TO_DMA);
    XNn_inference_Start(&ai_);
    while (!XNn_inference_IsDone(&ai_));
    RCLCPP_INFO(get_logger(),"predect %i", shared_data_[200]);
  }

private:
  void setupSharedMemory() {

    int memory_file;

    if ((memory_file = open(DEVICE_FILENAME, O_RDWR | O_SYNC)) < 0) {
      std::stringstream ss;
      ss << DEVICE_FILENAME << " could not be opened";
      throw ss.str();
    }

    shared_data_ = (uint32_t *)mmap(NULL, MAX_LENGTH/4, PROT_READ | PROT_WRITE,
                                MAP_SHARED, memory_file, 0);
  }

  void setupDma() {
    dma_.Reset();
    dma_.IntrDisable(IRQ_ALL, DMA_DIR_DEVICE_TO_DMA);
    dma_.IntrDisable(IRQ_ALL, DMA_DIR_DMA_TO_DEVICE);
  }

  void setupAi() {
    int rc;
    if ((rc = XNn_inference_Initialize(&ai_, "nn_ingerence")) != XST_SUCCESS) {
      fprintf(stderr, "Initialization failed. Return code: %d\n", rc);
      exit(EXIT_FAILURE);
    }
  }

  XNn_inference ai_;
  axiDma dma_;
  cv_bridge::CvImagePtr cv_ptr;
  uint32_t *shared_data_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);
  auto motor = std::make_shared<ai>("ai_node");
  rclcpp::spin(motor);

  return 0;
}
