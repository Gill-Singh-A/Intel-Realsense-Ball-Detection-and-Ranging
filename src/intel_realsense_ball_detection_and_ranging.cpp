#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace ros;
using namespace sensor_msgs;
using namespace std_msgs;
using namespace cv_bridge;
using namespace cv;

void getImage(Image ros_image) {
    CvImagePtr image_pointer;
    try {
        image_pointer = toCvCopy(ros_image, image_encodings::BGR8);
    } catch (cv_bridge::Exception error) {
        ROS_ERROR("CV Bride Encountered Error : %s", error.what());
        return;
    }
    imshow("Realsense Camera", image_pointer->image);
    waitKey(1);
}

int main(int argc, char *argv[]) {
    init(argc, argv, "ball_detection_and_ranging");
    NodeHandle node("~");

    Subscriber image_subscriber = node.subscribe("/camera/color/image_raw", 10, getImage);

    Rate loop_rate(10);
    
    while(ok())
        spinOnce();

    return 0;
}