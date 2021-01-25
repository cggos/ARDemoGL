#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <queue>
#include <thread>
#include <vector>

#include "ar_demo/ViewerAR.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/PinholeCamera.h"

using namespace std;
using namespace Eigen;

ViewerAR view_ar;
std::vector<cv::Point3f> mvMPs;

ros::Publisher object_pub;
image_transport::Publisher pub_ARimage;
queue<sensor_msgs::ImageConstPtr> img_buf;

Vector3d Axis[6];
Vector3d Cube_center[3];
vector<Vector3d> Cube_corner[3];
double Cube_center_depth[3];

vector<Vector3d> output_Axis[6];
vector<Vector3d> output_Cube[3];
vector<double> output_corner_dis[3];

std_msgs::ColorRGBA line_color_r;
std_msgs::ColorRGBA line_color_g;
std_msgs::ColorRGBA line_color_b;

bool look_ground = 0;
bool pose_init = false;
int img_cnt = 0;

const int axis_num = 0;
const int cube_num = 1;
const double box_length = 0.5;

std::string fixed_frame_id = "global";

camodocal::CameraPtr cam_ptr;

// int ROW = 640;
// int COL = 480;
// float fx = 611.67431640625f;
// float fy = 611.8240966796875f;
// float cx = 315.9566650390625f;
// float cy = 236.20654296875f;
// void spaceToPlane(const Eigen::Vector3d &P, Eigen::Vector2d &p) {
//     Eigen::Vector2d p_u, p_d;
//     p_u << P(0) / P(2), P(1) / P(2);
//     p_d = p_u;
//     p << fx * p_d(0) + cx, fy * p_d(1) + cy;
// }

void axis_generate(visualization_msgs::Marker &line_list, const Vector3d &origin, int id) {
    line_list.id = id;
    line_list.header.frame_id = fixed_frame_id;
    line_list.header.stamp = ros::Time::now();
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    line_list.color.a = 1.0;
    line_list.lifetime = ros::Duration();
    line_list.pose.orientation.w = 1.0;
    line_list.color.b = 1.0;

    geometry_msgs::Point p;
    p.x = origin.x();
    p.y = origin.y();
    p.z = origin.z();
    // x
    line_list.points.push_back(p);
    line_list.colors.push_back(line_color_r);
    p.x += 1.0;
    line_list.points.push_back(p);
    line_list.colors.push_back(line_color_r);
    p.x -= 1.0;
    // y
    line_list.points.push_back(p);
    line_list.colors.push_back(line_color_g);
    p.y += 1.0;
    line_list.points.push_back(p);
    line_list.colors.push_back(line_color_g);
    p.y -= 1.0;
    // z
    line_list.points.push_back(p);
    line_list.colors.push_back(line_color_b);
    p.z += 1.0;
    line_list.points.push_back(p);
    line_list.colors.push_back(line_color_b);
}

void cube_generate(visualization_msgs::Marker &marker, const Vector3d &origin, int id) {
    marker.header.frame_id = fixed_frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    /*
    marker.pose.position.x = origin.x();
    marker.pose.position.y = origin.y();
    marker.pose.position.z = origin.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    */
    marker.scale.x = box_length;
    marker.scale.y = box_length;
    marker.scale.z = box_length;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    geometry_msgs::Point p;
    p.x = origin.x();
    p.y = origin.y();
    p.z = origin.z();
    marker.points.push_back(p);
    marker.colors.push_back(line_color_r);

    Cube_corner[id].clear();
    Cube_corner[id].push_back(Vector3d(origin.x() - box_length / 2, origin.y() - box_length / 2, origin.z() - box_length / 2));
    Cube_corner[id].push_back(Vector3d(origin.x() + box_length / 2, origin.y() - box_length / 2, origin.z() - box_length / 2));
    Cube_corner[id].push_back(Vector3d(origin.x() - box_length / 2, origin.y() + box_length / 2, origin.z() - box_length / 2));
    Cube_corner[id].push_back(Vector3d(origin.x() + box_length / 2, origin.y() + box_length / 2, origin.z() - box_length / 2));
    Cube_corner[id].push_back(Vector3d(origin.x() - box_length / 2, origin.y() - box_length / 2, origin.z() + box_length / 2));
    Cube_corner[id].push_back(Vector3d(origin.x() + box_length / 2, origin.y() - box_length / 2, origin.z() + box_length / 2));
    Cube_corner[id].push_back(Vector3d(origin.x() - box_length / 2, origin.y() + box_length / 2, origin.z() + box_length / 2));
    Cube_corner[id].push_back(Vector3d(origin.x() + box_length / 2, origin.y() + box_length / 2, origin.z() + box_length / 2));
}

void add_object() {
    visualization_msgs::MarkerArray markerArray_msg;

    visualization_msgs::Marker line_list;
    visualization_msgs::Marker cube_list;

    for (int i = 0; i < axis_num; i++) {
        axis_generate(line_list, Axis[i], i);
        markerArray_msg.markers.push_back(line_list);
    }

    for (int i = 0; i < cube_num; i++) {
        cube_generate(cube_list, Cube_center[i], i);
    }
    markerArray_msg.markers.push_back(cube_list);

    object_pub.publish(markerArray_msg);
}

void project_object(Vector3d camera_p, Quaterniond camera_q) {
    for (int i = 0; i < axis_num; i++) {
        output_Axis[i].clear();
        Vector3d local_point;
        Vector2d local_uv;
        local_point = camera_q.inverse() * (Axis[i] - camera_p);
        cam_ptr->spaceToPlane(local_point, local_uv);

        if (local_point.z() > 0)
        //&& 0 <= local_uv.x() && local_uv.x() <= COL - 1 && 0 <= local_uv.y() && local_uv.y() <= ROW -1)
        {
            output_Axis[i].push_back(Vector3d(local_uv.x(), local_uv.y(), 1));

            local_point = camera_q.inverse() * (Axis[i] + Vector3d(1, 0, 0) - camera_p);
            cam_ptr->spaceToPlane(local_point, local_uv);
            output_Axis[i].push_back(Vector3d(local_uv.x(), local_uv.y(), 1));

            local_point = camera_q.inverse() * (Axis[i] + Vector3d(0, 1, 0) - camera_p);
            cam_ptr->spaceToPlane(local_point, local_uv);
            output_Axis[i].push_back(Vector3d(local_uv.x(), local_uv.y(), 1));

            local_point = camera_q.inverse() * (Axis[i] + Vector3d(0, 0, 1) - camera_p);
            cam_ptr->spaceToPlane(local_point, local_uv);
            output_Axis[i].push_back(Vector3d(local_uv.x(), local_uv.y(), 1));
        }
    }

    for (int i = 0; i < cube_num; i++) {
        output_Cube[i].clear();
        output_corner_dis[i].clear();
        Vector3d local_point;
        Vector2d local_uv;
        local_point = camera_q.inverse() * (Cube_center[i] - camera_p);
        cam_ptr->spaceToPlane(local_point, local_uv);
        if (local_point.z() > box_length / 2)
        //&& 0 <= local_uv.x() && local_uv.x() <= COL - 1 && 0 <= local_uv.y() && local_uv.y() <= ROW -1)
        {
            Cube_center_depth[i] = local_point.z();
            for (int j = 0; j < 8; j++) {
                local_point = camera_q.inverse() * (Cube_corner[i][j] - camera_p);
                output_corner_dis[i].push_back(local_point.norm());
                {
                    cam_ptr->spaceToPlane(local_point, local_uv);
                    local_uv.x() = std::min(std::max(-5000.0, local_uv.x()), 5000.0);
                    local_uv.y() = std::min(std::max(-5000.0, local_uv.y()), 5000.0);
                }
                output_Cube[i].push_back(Vector3d(local_uv.x(), local_uv.y(), 1));
            }
        } else {
            Cube_center_depth[i] = -1;
        }
    }
}

void draw_object(cv::Mat &AR_image) {
    for (int i = 0; i < axis_num; i++) {
        if (output_Axis[i].empty())
            continue;
        cv::Point2d origin(output_Axis[i][0].x(), output_Axis[i][0].y());
        cv::Point2d axis_x(output_Axis[i][1].x(), output_Axis[i][1].y());
        cv::Point2d axis_y(output_Axis[i][2].x(), output_Axis[i][2].y());
        cv::Point2d axis_z(output_Axis[i][3].x(), output_Axis[i][3].y());
        cv::line(AR_image, origin, axis_x, cv::Scalar(0, 0, 255), 2, 8, 0);
        cv::line(AR_image, origin, axis_y, cv::Scalar(0, 255, 0), 2, 8, 0);
        cv::line(AR_image, origin, axis_z, cv::Scalar(255, 0, 0), 2, 8, 0);
    }

    //depth sort  big---->small
    int index[cube_num];
    for (int i = 0; i < cube_num; i++) {
        index[i] = i;
    }
    for (int i = 0; i < cube_num; i++)
        for (int j = 0; j < cube_num - i - 1; j++) {
            if (Cube_center_depth[j] < Cube_center_depth[j + 1]) {
                double tmp = Cube_center_depth[j];
                Cube_center_depth[j] = Cube_center_depth[j + 1];
                Cube_center_depth[j + 1] = tmp;
                int tmp_index = index[j];
                index[j] = index[j + 1];
                index[j + 1] = tmp_index;
            }
        }

    for (int k = 0; k < cube_num; k++) {
        int i = index[k];
        // cout << "draw " << i << " depth " << Cube_center_depth[i] << endl;
        if (output_Cube[i].empty())
            continue;
        //draw color
        cv::Point *p = new cv::Point[8];
        p[0] = cv::Point(output_Cube[i][0].x(), output_Cube[i][0].y());
        p[1] = cv::Point(output_Cube[i][1].x(), output_Cube[i][1].y());
        p[2] = cv::Point(output_Cube[i][2].x(), output_Cube[i][2].y());
        p[3] = cv::Point(output_Cube[i][3].x(), output_Cube[i][3].y());
        p[4] = cv::Point(output_Cube[i][4].x(), output_Cube[i][4].y());
        p[5] = cv::Point(output_Cube[i][5].x(), output_Cube[i][5].y());
        p[6] = cv::Point(output_Cube[i][6].x(), output_Cube[i][6].y());
        p[7] = cv::Point(output_Cube[i][7].x(), output_Cube[i][7].y());

        int npts[1] = {4};
        float min_depth = 100000;
        int min_index = 5;
        for (int j = 0; j < (int)output_corner_dis[i].size(); j++) {
            if (output_corner_dis[i][j] < min_depth) {
                min_depth = output_corner_dis[i][j];
                min_index = j;
            }
        }

        cv::Point plain[1][4];
        const cv::Point *ppt[1] = {plain[0]};
        //first draw large depth plane
        int point_group[8][12] = {{0, 1, 5, 4, 0, 4, 6, 2, 0, 1, 3, 2},
                                  {0, 1, 5, 4, 1, 5, 7, 3, 0, 1, 3, 2},
                                  {2, 3, 7, 6, 0, 4, 6, 2, 0, 1, 3, 2},
                                  {2, 3, 7, 6, 1, 5, 7, 3, 0, 1, 3, 2},
                                  {0, 1, 5, 4, 0, 4, 6, 2, 4, 5, 7, 6},
                                  {0, 1, 5, 4, 1, 5, 7, 3, 4, 5, 7, 6},
                                  {2, 3, 7, 6, 0, 4, 6, 2, 4, 5, 7, 6},
                                  {2, 3, 7, 6, 1, 5, 7, 3, 4, 5, 7, 6}};

        plain[0][0] = p[point_group[min_index][4]];
        plain[0][1] = p[point_group[min_index][5]];
        plain[0][2] = p[point_group[min_index][6]];
        plain[0][3] = p[point_group[min_index][7]];
        cv::fillPoly(AR_image, ppt, npts, 1, cv::Scalar(0, 200, 0));

        plain[0][0] = p[point_group[min_index][0]];
        plain[0][1] = p[point_group[min_index][1]];
        plain[0][2] = p[point_group[min_index][2]];
        plain[0][3] = p[point_group[min_index][3]];
        cv::fillPoly(AR_image, ppt, npts, 1, cv::Scalar(200, 0, 0));

        if (output_corner_dis[i][point_group[min_index][2]] + output_corner_dis[i][point_group[min_index][3]] >
            output_corner_dis[i][point_group[min_index][5]] + output_corner_dis[i][point_group[min_index][6]]) {
            plain[0][0] = p[point_group[min_index][4]];
            plain[0][1] = p[point_group[min_index][5]];
            plain[0][2] = p[point_group[min_index][6]];
            plain[0][3] = p[point_group[min_index][7]];
            cv::fillPoly(AR_image, ppt, npts, 1, cv::Scalar(0, 200, 0));
        }
        plain[0][0] = p[point_group[min_index][8]];
        plain[0][1] = p[point_group[min_index][9]];
        plain[0][2] = p[point_group[min_index][10]];
        plain[0][3] = p[point_group[min_index][11]];
        cv::fillPoly(AR_image, ppt, npts, 1, cv::Scalar(0, 0, 200));
        delete p;
    }
}

void callback(const sensor_msgs::ImageConstPtr &img_msg, const nav_msgs::Odometry::ConstPtr pose_msg) {
    //throw the first few unstable pose
    if (img_cnt < 50) {
        img_cnt++;
        return;
    }

    Vector3d camera_p(pose_msg->pose.pose.position.x,
                      pose_msg->pose.pose.position.y,
                      pose_msg->pose.pose.position.z);
    Quaterniond camera_q(pose_msg->pose.pose.orientation.w,
                         pose_msg->pose.pose.orientation.x,
                         pose_msg->pose.pose.orientation.y,
                         pose_msg->pose.pose.orientation.z);

    //test plane
    Vector3d cam_z(0, 0, -1);
    Vector3d w_cam_z = camera_q * cam_z;
    //cout << "angle " << acos(w_cam_z.dot(Vector3d(0, 0, 1))) * 180.0 / M_PI << endl;
    if (acos(w_cam_z.dot(Vector3d(0, 0, 1))) * 180.0 / M_PI < 90) {
        // ROS_WARN(" look down");
        look_ground = 1;
    } else
        look_ground = 0;

    project_object(camera_p, camera_q);

    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1") {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    } else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat AR_image;
    AR_image = ptr->image.clone();
    cv::cvtColor(AR_image, AR_image, cv::COLOR_GRAY2RGB);

    // Pangolin Viewer
    {
        Eigen::Matrix3f Rwc = camera_q.toRotationMatrix().cast<float>();
        Eigen::Vector3f twc = camera_p.cast<float>();
        Eigen::Isometry3f Tcw;
        Tcw.linear() = Rwc.transpose();
        Tcw.translation() = -Rwc.transpose() * twc;
        cv::Mat mat_Tcw = cv::Mat(3, 4, CV_32F);
        // cv::eigen2cv(Tcw.matrix(), mat_Tcw);
        mat_Tcw.at<float>(0, 0) = Tcw.linear()(0, 0);
        mat_Tcw.at<float>(0, 1) = Tcw.linear()(0, 1);
        mat_Tcw.at<float>(0, 2) = Tcw.linear()(0, 2);
        mat_Tcw.at<float>(1, 0) = Tcw.linear()(1, 0);
        mat_Tcw.at<float>(1, 1) = Tcw.linear()(1, 1);
        mat_Tcw.at<float>(1, 2) = Tcw.linear()(1, 2);
        mat_Tcw.at<float>(2, 0) = Tcw.linear()(2, 0);
        mat_Tcw.at<float>(2, 1) = Tcw.linear()(2, 1);
        mat_Tcw.at<float>(2, 2) = Tcw.linear()(2, 2);
        mat_Tcw.at<float>(0, 3) = Tcw.translation()(0);
        mat_Tcw.at<float>(1, 3) = Tcw.translation()(1);
        mat_Tcw.at<float>(2, 3) = Tcw.translation()(2);
        view_ar.SetImagePose(AR_image, mat_Tcw);
    }

    draw_object(AR_image);

    sensor_msgs::ImagePtr AR_msg = cv_bridge::CvImage(img_msg->header, "bgr8", AR_image).toImageMsg();
    pub_ARimage.publish(AR_msg);
}

void point_callback(const sensor_msgs::PointCloud2ConstPtr &point_msg) {
    if (!look_ground)
        return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*point_msg, *points_ptr);

    int height_range[30];
    double height_sum[30];
    for (int i = 0; i < 30; i++) {
        height_range[i] = 0;
        height_sum[i] = 0;
    }

    for (unsigned int i = 0; i < points_ptr->points.size(); i++) {
        //double x = point_msg->points[i].x;
        //double y = point_msg->points[i].y;
        double z = points_ptr->points[i].z;
        int index = (z + 1.0) / 0.1;
        if (0 <= index && index < 30) {
            height_range[index]++;
            height_sum[index] += z;
        }
        mvMPs.push_back(cv::Point3f(points_ptr->points[i].x, points_ptr->points[i].y, points_ptr->points[i].z));
    }
    view_ar.SetPoints(mvMPs);
    mvMPs.clear();

    int max_num = 0;
    int max_index = -1;
    for (int i = 1; i < 29; i++) {
        if (max_num < height_range[i]) {
            max_num = height_range[i];
            max_index = i;
        }
    }
    if (max_index == -1)
        return;
    int tmp_num = height_range[max_index - 1] + height_range[max_index] + height_range[max_index + 1];
    double new_height = (height_sum[max_index - 1] + height_sum[max_index] + height_sum[max_index + 1]) / tmp_num;
    //ROS_WARN("detect ground plain, height %f", new_height);
    if (tmp_num < (int)points_ptr->points.size() / 2) {
        //ROS_INFO("points not enough");
        return;
    }
    //update height
    for (int i = 0; i < cube_num; i++) {
        Cube_center[i].z() = new_height + box_length / 2.0;
    }
    add_object();
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg) {
    if (pose_init) {
        img_buf.push(img_msg);
    } else
        return;
}

void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) {
    if (!pose_init) {
        pose_init = true;
        return;
    }

    if (img_buf.empty()) {
        ROS_WARN("image coming late");
        return;
    }

    while (img_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec() && !img_buf.empty()) {
        img_buf.pop();
    }

    if (!img_buf.empty()) {
        callback(img_buf.front(), pose_msg);
        img_buf.pop();
    }
    //else
    //    ROS_WARN("image coming late");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "points_and_lines");

    ros::NodeHandle n("~");

    ros::Subscriber sub_img = n.subscribe("image_raw", 100, img_callback);
    ros::Subscriber pose_img = n.subscribe("camera_pose", 100, pose_callback);
    ros::Subscriber sub_point = n.subscribe("pointcloud", 2000, point_callback);

    image_transport::ImageTransport it(n);
    pub_ARimage = it.advertise("AR_image", 1000);
    object_pub = n.advertise<visualization_msgs::MarkerArray>("AR_object", 10);

    string calib_file;
    n.getParam("calib_file", calib_file);
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    cam_ptr = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);

    Axis[0] = Vector3d(0, 1.5, -1.2);
    Axis[1] = Vector3d(-10, 5, 0);
    Axis[2] = Vector3d(3, 3, 3);
    Axis[3] = Vector3d(-2, 2, 0);
    Axis[4] = Vector3d(5, 10, -5);
    Axis[5] = Vector3d(0, 10, -1);

    Cube_center[0] = Vector3d(0, 1.5, -1.0 + box_length / 2.0);
    // Cube_center[0] = Vector3d(0,  3, -1.2 + box_length / 2.0);
    // Cube_center[1] = Vector3d(4, -2, -1.2 + box_length / 2.0);
    // Cube_center[2] = Vector3d(0, -2, -1.2 + box_length / 2.0);

    line_color_r.r = 1.0;
    line_color_r.a = 1.0;
    line_color_g.g = 1.0;
    line_color_g.a = 1.0;
    line_color_b.b = 1.0;
    line_color_b.a = 1.0;

    std::vector<double> parameterVec;
    cam_ptr->writeParameters(parameterVec);
    float fx = parameterVec[4];
    float fy = parameterVec[5];
    float cx = parameterVec[6];
    float cy = parameterVec[7];
    view_ar.SetCameraCalibration(fx, fy, cx, cy, cam_ptr->imageWidth(), cam_ptr->imageHeight());
    std::thread t_viewer = std::thread(&ViewerAR::run, &view_ar);

    ros::Rate r(100);
    ros::Duration(1).sleep();
    add_object();
    add_object();

    ros::spin();
}
