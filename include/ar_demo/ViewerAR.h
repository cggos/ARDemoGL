#pragma once

#include <pangolin/pangolin.h>

#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Plane {
   public:
    Plane(const std::vector<cv::Point3f> &vMPs, const cv::Mat &Tcw);
    Plane(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz);
    void Recompute();
    //normal
    cv::Mat n;
    //origin
    cv::Mat o;
    //arbitrary orientation along normal
    float rang;
    //transformation from world to the plane
    cv::Mat Tpw;
    pangolin::OpenGlMatrix glTpw;
    //MapPoints that define the plane
    std::vector<cv::Point3f> mvMPs;
    //camera pose when the plane was first observed (to compute normal direction)
    cv::Mat mTcw, XC;
};

class ViewerAR {
   public:
    ViewerAR() {}

    void run();

    Plane *DetectPlane(const cv::Mat Tcw, const std::vector<cv::Point3f> &vMPs, const int iterations);

    void SetCameraCalibration(const float &fx_, const float &fy_, const float &cx_, const float &cy_, int w, int h) {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;

        img_w = w;
        img_h = h;
    }

    void SetImagePose(const cv::Mat &im, const cv::Mat &Tcw) {
        std::unique_lock<std::mutex> lock(mMutexPoseImage);
        mImage = im.clone();
        mTcw = Tcw.clone();
    }

    void GetImagePose(cv::Mat &im, cv::Mat &Tcw) {
        std::unique_lock<std::mutex> lock(mMutexPoseImage);
        im = mImage.clone();
        Tcw = mTcw.clone();
    }

    void SetPoints(const std::vector<cv::Point3f> &vMPs) {
        std::unique_lock<std::mutex> lock(mMutexPoints);
        mvMPs = vMPs;
    }

    void GetPoints(std::vector<cv::Point3f> &vMPs) {
        std::unique_lock<std::mutex> lock(mMutexPoints);
        vMPs = mvMPs;
    }

    void AddTextToImage(const std::string &s, cv::Mat &im, const int r, const int g, const int b) {
        int l = 10;
        //imText.rowRange(im.rows-imText.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
        cv::putText(im, s, cv::Point(l, im.rows - l), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 255), 2, 8);
        cv::putText(im, s, cv::Point(l - 1, im.rows - l), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 255), 2, 8);
        cv::putText(im, s, cv::Point(l + 1, im.rows - l), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 255), 2, 8);
        cv::putText(im, s, cv::Point(l - 1, im.rows - (l - 1)), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 255), 2, 8);
        cv::putText(im, s, cv::Point(l, im.rows - (l - 1)), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 255), 2, 8);
        cv::putText(im, s, cv::Point(l + 1, im.rows - (l - 1)), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 255), 2, 8);
        cv::putText(im, s, cv::Point(l - 1, im.rows - (l + 1)), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 255), 2, 8);
        cv::putText(im, s, cv::Point(l, im.rows - (l + 1)), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 255), 2, 8);
        cv::putText(im, s, cv::Point(l + 1, im.rows - (l + 1)), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 255), 2, 8);

        cv::putText(im, s, cv::Point(l, im.rows - l), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(r, g, b), 2, 8);
    }

    void DrawImageTexture(pangolin::GlTexture &imageTexture, cv::Mat &im) {
        if (!im.empty()) {
            imageTexture.Upload(im.data, GL_RGB, GL_UNSIGNED_BYTE);
            imageTexture.RenderToViewportFlipY();
        }
    }

    void DrawCube(const float &size, const float x = 0.f, const float y = 0.f, const float z = 0.f) {
        pangolin::OpenGlMatrix M = pangolin::OpenGlMatrix::Translate(-x, -size - y, -z);
        glPushMatrix();
        M.Multiply();
        pangolin::glDrawColouredCube(-size, size);
        glPopMatrix();
    }

    void DrawPoints3D(const std::vector<cv::Point3f> &vMPs) {
        glPointSize(5);
        glColor3f(0.f, 1.f, 1.f);
        glBegin(GL_POINTS);
        for (const auto &pt : vMPs) {
            glVertex3f(pt.x, pt.y, pt.z);
        }
        glEnd();
    }

    void DrawPlane(int ndivs, float ndivsize) {
        // Plane parallel to x-z at origin with normal -y
        const float minx = -ndivs * ndivsize;
        const float minz = -ndivs * ndivsize;
        const float maxx = ndivs * ndivsize;
        const float maxz = ndivs * ndivsize;

        glLineWidth(2);
        glColor3f(0.7f, 0.7f, 1.0f);
        glBegin(GL_LINES);
        for (int n = 0; n <= 2 * ndivs; n++) {
            glVertex3f(minx + ndivsize * n, 0, minz);
            glVertex3f(minx + ndivsize * n, 0, maxz);
            glVertex3f(minx, 0, minz + ndivsize * n);
            glVertex3f(maxx, 0, minz + ndivsize * n);
        }
        glEnd();
    }

    void LoadCameraPose(const cv::Mat &Tcw) {
        if (!Tcw.empty()) {
            pangolin::OpenGlMatrix M;
            M.m[0] = Tcw.at<float>(0, 0);
            M.m[1] = Tcw.at<float>(1, 0);
            M.m[2] = Tcw.at<float>(2, 0);
            M.m[3] = 0.0;
            M.m[4] = Tcw.at<float>(0, 1);
            M.m[5] = Tcw.at<float>(1, 1);
            M.m[6] = Tcw.at<float>(2, 1);
            M.m[7] = 0.0;
            M.m[8] = Tcw.at<float>(0, 2);
            M.m[9] = Tcw.at<float>(1, 2);
            M.m[10] = Tcw.at<float>(2, 2);
            M.m[11] = 0.0;
            M.m[12] = Tcw.at<float>(0, 3);
            M.m[13] = Tcw.at<float>(1, 3);
            M.m[14] = Tcw.at<float>(2, 3);
            M.m[15] = 1.0;
            M.Load();
        }
    }

   private:
    int img_w, img_h;
    float fx, fy, cx, cy;

    std::mutex mMutexPoseImage;
    std::mutex mMutexPoints;
    cv::Mat mTcw;
    cv::Mat mImage;
    std::vector<cv::Point3f> mvMPs;
};