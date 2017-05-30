#include "ros/ros.h"
#include <ros/package.h>
#include <sstream>
#include <fstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <sl/Camera.hpp>
#include <memory>
#include <string>
std::unique_ptr<sl::Camera> zed;

/* \brief Convert an sl:Mat to a cv::Mat
 * \param mat : the sl::Mat to convert
 */
cv::Mat toCVMat(sl::Mat &mat)
{
  if (mat.getMemoryType() == sl::MEM_GPU)
    mat.updateCPUfromGPU();

  int cvType;
  switch (mat.getDataType())
  {
  case sl::MAT_TYPE_32F_C1:
    cvType = CV_32FC1;
    break;
  case sl::MAT_TYPE_32F_C2:
    cvType = CV_32FC2;
    break;
  case sl::MAT_TYPE_32F_C3:
    cvType = CV_32FC3;
    break;
  case sl::MAT_TYPE_32F_C4:
    cvType = CV_32FC4;
    break;
  case sl::MAT_TYPE_8U_C1:
    cvType = CV_8UC1;
    break;
  case sl::MAT_TYPE_8U_C2:
    cvType = CV_8UC2;
    break;
  case sl::MAT_TYPE_8U_C3:
    cvType = CV_8UC3;
    break;
  case sl::MAT_TYPE_8U_C4:
    cvType = CV_8UC4;
    break;
  }
  return cv::Mat((int)mat.getHeight(), (int)mat.getWidth(), cvType, mat.getPtr<sl::uchar1>(sl::MEM_CPU), mat.getStepBytes(sl::MEM_CPU));
}

/* \brief Image to ros message conversion
 * \param img : the image to publish
 * \param encodingType : the sensor_msgs::image_encodings encoding type
 * \param frameId : the id of the reference frame of the image
 * \param t : the ros::Time to stamp the image
 */
sensor_msgs::ImagePtr imageToROSmsg(cv::Mat img, const std::string encodingType, std::string frameId, ros::Time t)
{
  sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
  sensor_msgs::Image &imgMessage = *ptr;
  imgMessage.header.stamp = t;
  imgMessage.header.frame_id = frameId;
  imgMessage.height = img.rows;
  imgMessage.width = img.cols;
  imgMessage.encoding = encodingType;
  int num = 1; //for endianness detection
  imgMessage.is_bigendian = !(*(char *)&num == 1);
  imgMessage.step = img.cols * img.elemSize();
  size_t size = imgMessage.step * img.rows;
  imgMessage.data.resize(size);

  if (img.isContinuous())
    memcpy((char *)(&imgMessage.data[0]), img.data, size);
  else
  {
    uchar *opencvData = img.data;
    uchar *rosData = (uchar *)(&imgMessage.data[0]);
    for (unsigned int i = 0; i < img.rows; i++)
    {
      memcpy(rosData, opencvData, imgMessage.step);
      rosData += imgMessage.step;
      opencvData += img.step;
    }
  }
  return ptr;
}
void fillCamInfo(sl::Camera *zed, sensor_msgs::CameraInfoPtr left_cam_info_msg, sensor_msgs::CameraInfoPtr right_cam_info_msg, std::string left_frame_id, std::string right_frame_id)
{

  int width = zed->getResolution().width;
  int height = zed->getResolution().height;

  sl::CameraInformation zedParam = zed->getCameraInformation();

  float baseline = zedParam.calibration_parameters.T[0] * 0.001; // baseline converted in meters

  float fx = zedParam.calibration_parameters.left_cam.fx;
  float fy = zedParam.calibration_parameters.left_cam.fy;
  float cx = zedParam.calibration_parameters.left_cam.cx;
  float cy = zedParam.calibration_parameters.left_cam.cy;

  // There is no distorsions since the images are rectified
  double k1 = 0;
  double k2 = 0;
  double k3 = 0;
  double p1 = 0;
  double p2 = 0;

  left_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  right_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  left_cam_info_msg->D.resize(5);
  right_cam_info_msg->D.resize(5);
  left_cam_info_msg->D[0] = right_cam_info_msg->D[0] = k1;
  left_cam_info_msg->D[1] = right_cam_info_msg->D[1] = k2;
  left_cam_info_msg->D[2] = right_cam_info_msg->D[2] = k3;
  left_cam_info_msg->D[3] = right_cam_info_msg->D[3] = p1;
  left_cam_info_msg->D[4] = right_cam_info_msg->D[4] = p2;

  left_cam_info_msg->K.fill(0.0);
  right_cam_info_msg->K.fill(0.0);
  left_cam_info_msg->K[0] = right_cam_info_msg->K[0] = fx;
  left_cam_info_msg->K[2] = right_cam_info_msg->K[2] = cx;
  left_cam_info_msg->K[4] = right_cam_info_msg->K[4] = fy;
  left_cam_info_msg->K[5] = right_cam_info_msg->K[5] = cy;
  left_cam_info_msg->K[8] = right_cam_info_msg->K[8] = 1.0;

  left_cam_info_msg->R.fill(0.0);
  right_cam_info_msg->R.fill(0.0);

  left_cam_info_msg->P.fill(0.0);
  right_cam_info_msg->P.fill(0.0);
  left_cam_info_msg->P[0] = right_cam_info_msg->P[0] = fx;
  left_cam_info_msg->P[2] = right_cam_info_msg->P[2] = cx;
  left_cam_info_msg->P[5] = right_cam_info_msg->P[5] = fy;
  left_cam_info_msg->P[6] = right_cam_info_msg->P[6] = cy;
  left_cam_info_msg->P[10] = right_cam_info_msg->P[10] = 1.0;
  right_cam_info_msg->P[3] = (-1 * fx * baseline);

  left_cam_info_msg->width = right_cam_info_msg->width = width;
  left_cam_info_msg->height = right_cam_info_msg->height = height;

  left_cam_info_msg->header.frame_id = left_frame_id;
  right_cam_info_msg->header.frame_id = right_frame_id;
}
void publishCamInfo(sensor_msgs::CameraInfoPtr cam_info_msg, ros::Publisher pub_cam_info, ros::Time t)
{
  static int seq = 0;
  cam_info_msg->header.stamp = t;
  cam_info_msg->header.seq = seq;
  pub_cam_info.publish(cam_info_msg);
  seq++;
}
///////////////////////
//Collision detecotor
///////////////////////
void depthHook(cv::Mat &tmp)
{
  cv::Size s = tmp.size();
  double min = 1e+10, max = -1e+10;
  const double thresh = 0.5;
  int shortCount = 0;
  for (int y = 0; y < s.height; y++)
    for (int x = 0; x < s.width; x++)
    {
      float v = tmp.at<float>(y, x);
      if (0.1 < v && v < 20)
      {
        min = fmin(v, min);
        max = fmax(v, max);
        if (v < thresh)
          shortCount++;
      }
    }
  if (min < 0.5)
  {
    if(shortCount>10){ //Danger Stop

    }
    ROS_INFO_STREAM("depth " << min << "-" << max);
  }
}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "zed_sensor_node");
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(30);
  ros::Publisher pub_rgb_cam_info, pub_depth_cam_info;
  image_transport::Publisher pub_rgb, pub_depth;
  image_transport::ImageTransport it_zed(node_handle);
  pub_rgb = it_zed.advertise("bus_cam/rgb/img", 1);     //rgb
  pub_depth = it_zed.advertise("bus_cam/depth/img", 1); //depth
  pub_rgb_cam_info = node_handle.advertise<sensor_msgs::CameraInfo>("bus_cam/rgb/camera_info", 1);
  pub_depth_cam_info = node_handle.advertise<sensor_msgs::CameraInfo>("bus_cam/depth/camera_info", 1);

  sensor_msgs::CameraInfoPtr depth_cam_info_msg(new sensor_msgs::CameraInfo());
  sensor_msgs::CameraInfoPtr rgb_cam_info_msg(new sensor_msgs::CameraInfo());
  sensor_msgs::CameraInfoPtr left_cam_info_msg(new sensor_msgs::CameraInfo());
  sensor_msgs::CameraInfoPtr right_cam_info_msg(new sensor_msgs::CameraInfo());

  static tf2_ros::TransformBroadcaster br;
  // Create the ZED object
  zed.reset(new sl::Camera());
  // Try to initialize the ZED
  sl::InitParameters param;
  param.camera_fps = 0; //30
  param.camera_resolution = sl::RESOLUTION_HD720;
  param.camera_linux_id = 0;
  param.coordinate_units = sl::UNIT_METER;
  param.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP;     // ROS coordinates system
  param.depth_mode = static_cast<sl::DEPTH_MODE>(sl::DEPTH_MODE_MEDIUM); // sl::DEPTH_MODE_PERFORMANCE);
  param.depth_minimum_distance = 0.3;
  param.sdk_verbose = true;
  param.sdk_gpu_id = -1;

  sl::ERROR_CODE err = sl::ERROR_CODE_CAMERA_NOT_DETECTED;
  while (err != sl::SUCCESS)
  {
    err = zed->open(param);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  sl::TrackingParameters trackParams;
  std::string area_path = ros::package::getPath("st_pub");

  area_path += "/data/now.area";
  trackParams.area_file_path = area_path.c_str();
  trackParams.area_file_path = "";

  int width = zed->getResolution().width;
  int height = zed->getResolution().height;

  cv::Size cvSize(width, height);
  cv::Mat leftImRGB(cvSize, CV_8UC3);

  fillCamInfo(zed.get(), left_cam_info_msg, right_cam_info_msg, "bus_cam" /*left_frame_id*/, "bus_cam" /*right_frame_id*/);
  rgb_cam_info_msg = depth_cam_info_msg = left_cam_info_msg; // the reference camera is the Left one (next t

  zed->enableTracking(trackParams);
  zed->setConfidenceThreshold(80);
  sl::RuntimeParameters runParams;
  runParams.sensing_mode = static_cast<sl::SENSING_MODE>(sl::SENSING_MODE_STANDARD);

  int count = 0;
  sl::Pose pose;
  sl::Mat leftZEDMat, depthZEDMat;
  while (ros::ok())
  {
    int rgb_SubNumber = pub_rgb.getNumSubscribers();
    int depth_SubNumber = pub_rgb.getNumSubscribers();

    zed->grab(runParams); // Ask to not compute the depth

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
    if (sl::TRACKING_STATE_OK == zed->getPosition(pose))
    {
      geometry_msgs::TransformStamped brtf;
      brtf.header.stamp = ros::Time::now();
      brtf.header.frame_id = "world";
      brtf.child_frame_id = "bus_cam";
      sl::Translation t = pose.getTranslation();
      sl::Orientation o = pose.getOrientation();

      brtf.transform.translation.x = t.x;
      brtf.transform.translation.y = t.y;
      brtf.transform.translation.z = t.z;
      brtf.transform.rotation.x = o.x;
      brtf.transform.rotation.y = o.y;
      brtf.transform.rotation.z = o.z;
      brtf.transform.rotation.w = o.w;
      br.sendTransform(brtf);
    }

    zed->retrieveImage(leftZEDMat, sl::VIEW_LEFT);
    cv::cvtColor(toCVMat(leftZEDMat), leftImRGB, CV_RGBA2RGB);
    if (rgb_SubNumber > 0)
    {
      pub_rgb.publish(imageToROSmsg(leftImRGB, sensor_msgs::image_encodings::BGR8, "bus_cam" /*frame id*/, ros::Time::now()));
      publishCamInfo(rgb_cam_info_msg, pub_rgb_cam_info, ros::Time::now());
    }
    if (depth_SubNumber > 0)
    {
      zed->retrieveMeasure(depthZEDMat, sl::MEASURE_DEPTH);
      cv::Mat tmp = toCVMat(depthZEDMat);
      depthHook(tmp);

      pub_depth.publish(imageToROSmsg(tmp, sensor_msgs::image_encodings::TYPE_32FC1, "bus_cam_opt" /*frame id*/, ros::Time::now()));
      publishCamInfo(depth_cam_info_msg, pub_depth_cam_info, ros::Time::now());
    }
  }
  zed->disableTracking();
  zed.reset();

  return 0;
}
