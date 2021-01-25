#include "ar_demo/ViewerAR.h"

#include <unistd.h>

using namespace std;

void p3f_to_mat(const cv::Point3f &P, cv::Mat &mat) {
    mat = cv::Mat(3, 1, CV_32F);
    mat.at<float>(0, 0) = P.x;
    mat.at<float>(1, 0) = P.y;
    mat.at<float>(2, 0) = P.z;
}

cv::Mat ExpSO3(const float &x, const float &y, const float &z) {
    cv::Mat I = cv::Mat::eye(3, 3, CV_32F);
    const float d2 = x * x + y * y + z * z;
    const float d = sqrt(d2);
    cv::Mat W = (cv::Mat_<float>(3, 3) << 0, -z, y, z, 0, -x, -y, x, 0);
    const float eps = 1e-4;
    if (d < eps)
        return (I + W + 0.5f * W * W);
    else
        return (I + W * sin(d) / d + W * W * (1.0f - cos(d)) / d2);
}

cv::Mat ExpSO3(const cv::Mat &v) {
    return ExpSO3(v.at<float>(0), v.at<float>(1), v.at<float>(2));
}

void ViewerAR::run() {
    int w = img_w;
    int h = img_h;
    int wui = 200;
    int text_h = 30;

    std::string str_status = "";
    cv::Mat im, Tcw;
    vector<Plane *> vpPlane;
    vector<cv::Point3f> vMPs;

    pangolin::CreateWindowAndBind("AR Demo Viewer", w + wui, h + text_h);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(wui));
    pangolin::Var<bool> menu_drawim("menu.Draw Image", true, true);
    pangolin::Var<bool> menu_drawcube("menu.Draw Cube", true, true);
    pangolin::Var<float> menu_cubesize("menu. Cube Size", 0.05, 0.01, 0.3);
    pangolin::Var<bool> menu_drawgrid("menu.Draw Grid", true, true);
    pangolin::Var<int> menu_ngrid("menu. Grid Elements", 3, 1, 10);
    pangolin::Var<float> menu_sizegrid("menu. Element Size", 0.05, 0.01, 0.3);
    pangolin::Var<bool> menu_drawpoints("menu.Draw 3D Points", true, true);
    pangolin::Var<bool> menu_detectplane("menu.Insert Cube", false, false);
    pangolin::Var<bool> menu_clear("menu.Clear All", false, false);

    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'a', pangolin::SetVarFunctor<bool>("menu.Insert Cube", true));
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'c', pangolin::SetVarFunctor<bool>("menu.Clear All", true));

    pangolin::CreatePanel("status").SetBounds(0.0, pangolin::Attach::Pix(text_h), 0.0f, 1.0f);
    pangolin::Var<std::string> var_status("status.status", "Hello Pangolin");
    var_status = "window created!";

    pangolin::View &d_image = pangolin::Display("image")
                                  .SetBounds(pangolin::Attach::Pix(text_h), 1.0f, pangolin::Attach::Pix(wui), 1.0f, (float)w / h)
                                  .SetLock(pangolin::LockLeft, pangolin::LockTop);

    pangolin::GlTexture imageTexture(w, h, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);

    pangolin::OpenGlMatrixSpec P = pangolin::ProjectionMatrixRDF_TopLeft(w, h, fx, fy, cx, cy, 0.001, 1000);

    while (true) {
        GetImagePose(im, Tcw);
        GetPoints(vMPs);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_image.Activate();
        glColor3f(1.0, 1.0, 1.0);

        if (menu_drawim) {
            int sz = vMPs.size();
            std::stringstream ss;
            ss << "vMPs: " << sz;
            std::string s = ss.str();
            AddTextToImage(s, im, 0, 0, 255);
            DrawImageTexture(imageTexture, im);
        }
        glClear(GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        P.Load();  // Load camera projection
        glMatrixMode(GL_MODELVIEW);
        LoadCameraPose(Tcw);

        // draw
        {
            if (menu_clear) {
                if (!vpPlane.empty()) {
                    for (size_t i = 0; i < vpPlane.size(); i++) {
                        delete vpPlane[i];
                    }
                    vpPlane.clear();
                    str_status = "All cubes erased!";
                    var_status = str_status;
                    cout << str_status << endl;
                }
                menu_clear = false;
            }
            if (menu_drawpoints) {
                DrawPoints3D(vMPs);
            }
            if (menu_detectplane) {
                Plane *pPlane = DetectPlane(Tcw, vMPs, 50);
                if (pPlane) {
                    str_status = "New virtual cube inserted!";
                    vpPlane.push_back(pPlane);
                } else {
                    str_status = "No plane detected. Point the camera to a planar region.";
                }
                var_status = str_status;
                cout << str_status << endl;
                menu_detectplane = false;
            }
            if (!vpPlane.empty()) {
                for (size_t i = 0; i < vpPlane.size(); i++) {
                    Plane *pPlane = vpPlane[i];
                    if (pPlane) {
                        glPushMatrix();
                        pPlane->glTpw.Multiply();
                        if (menu_drawcube) {
                            DrawCube(menu_cubesize);
                        }
                        if (menu_drawgrid) {
                            DrawPlane(menu_ngrid, menu_sizegrid);
                        }
                        glPopMatrix();
                    }
                }
            }
        }
        pangolin::FinishFrame();
        usleep(30 * 1000);
    }
}

Plane *ViewerAR::DetectPlane(const cv::Mat Tcw, const std::vector<cv::Point3f> &vMPs, const int iterations) {
    // Retrieve 3D points
    vector<cv::Mat> vPoints;
    vPoints.reserve(vMPs.size());

    for (size_t i = 0; i < vMPs.size(); i++) {
        cv::Mat matP;
        p3f_to_mat(vMPs[i], matP);
        vPoints.push_back(matP);
    }

    const int N = vPoints.size();

    if (N < 30)  // 50
        return NULL;

    // Indices for minimum set selection
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for (int i = 0; i < N; i++) {
        vAllIndices.push_back(i);
    }

    float bestDist = 1e10;
    vector<float> bestvDist;

    //RANSAC
    for (int n = 0; n < iterations; n++) {
        vAvailableIndices = vAllIndices;

        cv::Mat A(3, 4, CV_32F);
        A.col(3) = cv::Mat::ones(3, 1, CV_32F);

        // Get min set of points
        for (short i = 0; i < 3; ++i) {
            // int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);
            int randi = std::rand() % (vAvailableIndices.size() - 1);

            int idx = vAvailableIndices[randi];

            A.row(i).colRange(0, 3) = vPoints[idx].t();

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        cv::Mat u, w, vt;
        cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        const float a = vt.at<float>(3, 0);
        const float b = vt.at<float>(3, 1);
        const float c = vt.at<float>(3, 2);
        const float d = vt.at<float>(3, 3);

        vector<float> vDistances(N, 0);

        const float f = 1.0f / sqrt(a * a + b * b + c * c + d * d);

        for (int i = 0; i < N; i++) {
            vDistances[i] = fabs(vPoints[i].at<float>(0) * a + vPoints[i].at<float>(1) * b + vPoints[i].at<float>(2) * c + d) * f;
        }

        vector<float> vSorted = vDistances;
        sort(vSorted.begin(), vSorted.end());

        int nth = max((int)(0.2 * N), 20);
        const float medianDist = vSorted[nth];

        if (medianDist < bestDist) {
            bestDist = medianDist;
            bestvDist = vDistances;
        }
    }

    // Compute threshold inlier/outlier
    const float th = 1.4 * bestDist;
    vector<bool> vbInliers(N, false);
    int nInliers = 0;
    for (int i = 0; i < N; i++) {
        if (bestvDist[i] < th) {
            nInliers++;
            vbInliers[i] = true;
        }
    }

    if (nInliers == 0) return NULL;

    vector<cv::Point3f> vInlierMPs(nInliers, cv::Point3f(0.f, 0.f, 0.f));
    int nin = 0;
    for (int i = 0; i < N; i++) {
        if (vbInliers[i]) {
            vInlierMPs[nin] = vMPs[i];
            nin++;
        }
    }

    return new Plane(vInlierMPs, Tcw);
}

Plane::Plane(const std::vector<cv::Point3f> &vMPs, const cv::Mat &Tcw) : mvMPs(vMPs), mTcw(Tcw.clone()) {
    rang = -3.14f / 2 + ((float)rand() / RAND_MAX) * 3.14f;
    Recompute();
}

void Plane::Recompute() {
    const int N = mvMPs.size();

    // Recompute plane with all points
    cv::Mat A = cv::Mat(N, 4, CV_32F);
    A.col(3) = cv::Mat::ones(N, 1, CV_32F);

    o = cv::Mat::zeros(3, 1, CV_32F);

    int nPoints = 0;
    for (int i = 0; i < N; i++) {
        cv::Point3f pMP = mvMPs[i];
        cv::Mat Xw;
        p3f_to_mat(pMP, Xw);
        o += Xw;
        A.row(nPoints).colRange(0, 3) = Xw.t();
        nPoints++;
    }
    A.resize(nPoints);

    cv::Mat u, w, vt;
    cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    float a = vt.at<float>(3, 0);
    float b = vt.at<float>(3, 1);
    float c = vt.at<float>(3, 2);

    o = o * (1.0f / nPoints);
    const float f = 1.0f / sqrt(a * a + b * b + c * c);

    // Compute XC just the first time
    if (XC.empty()) {
        cv::Mat Oc = -mTcw.colRange(0, 3).rowRange(0, 3).t() * mTcw.rowRange(0, 3).col(3);
        XC = Oc - o;
    }

    if ((XC.at<float>(0) * a + XC.at<float>(1) * b + XC.at<float>(2) * c) > 0) {
        a = -a;
        b = -b;
        c = -c;
    }

    const float nx = a * f;
    const float ny = b * f;
    const float nz = c * f;

    n = (cv::Mat_<float>(3, 1) << nx, ny, nz);

    cv::Mat up = (cv::Mat_<float>(3, 1) << 0.0f, 1.0f, 0.0f);

    cv::Mat v = up.cross(n);
    const float sa = cv::norm(v);
    const float ca = up.dot(n);
    const float ang = atan2(sa, ca);
    Tpw = cv::Mat::eye(4, 4, CV_32F);

    Tpw.rowRange(0, 3).colRange(0, 3) = ExpSO3(v * ang / sa) * ExpSO3(up * rang);
    o.copyTo(Tpw.col(3).rowRange(0, 3));

    glTpw.m[0] = Tpw.at<float>(0, 0);
    glTpw.m[1] = Tpw.at<float>(1, 0);
    glTpw.m[2] = Tpw.at<float>(2, 0);
    glTpw.m[3] = 0.0;

    glTpw.m[4] = Tpw.at<float>(0, 1);
    glTpw.m[5] = Tpw.at<float>(1, 1);
    glTpw.m[6] = Tpw.at<float>(2, 1);
    glTpw.m[7] = 0.0;

    glTpw.m[8] = Tpw.at<float>(0, 2);
    glTpw.m[9] = Tpw.at<float>(1, 2);
    glTpw.m[10] = Tpw.at<float>(2, 2);
    glTpw.m[11] = 0.0;

    glTpw.m[12] = Tpw.at<float>(0, 3);
    glTpw.m[13] = Tpw.at<float>(1, 3);
    glTpw.m[14] = Tpw.at<float>(2, 3);
    glTpw.m[15] = 1.0;
}

Plane::Plane(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz) {
    n = (cv::Mat_<float>(3, 1) << nx, ny, nz);
    o = (cv::Mat_<float>(3, 1) << ox, oy, oz);

    cv::Mat up = (cv::Mat_<float>(3, 1) << 0.0f, 1.0f, 0.0f);

    cv::Mat v = up.cross(n);
    const float s = cv::norm(v);
    const float c = up.dot(n);
    const float a = atan2(s, c);
    Tpw = cv::Mat::eye(4, 4, CV_32F);
    const float rang = -3.14f / 2 + ((float)rand() / RAND_MAX) * 3.14f;
    cout << rang;
    Tpw.rowRange(0, 3).colRange(0, 3) = ExpSO3(v * a / s) * ExpSO3(up * rang);
    o.copyTo(Tpw.col(3).rowRange(0, 3));

    glTpw.m[0] = Tpw.at<float>(0, 0);
    glTpw.m[1] = Tpw.at<float>(1, 0);
    glTpw.m[2] = Tpw.at<float>(2, 0);
    glTpw.m[3] = 0.0;

    glTpw.m[4] = Tpw.at<float>(0, 1);
    glTpw.m[5] = Tpw.at<float>(1, 1);
    glTpw.m[6] = Tpw.at<float>(2, 1);
    glTpw.m[7] = 0.0;

    glTpw.m[8] = Tpw.at<float>(0, 2);
    glTpw.m[9] = Tpw.at<float>(1, 2);
    glTpw.m[10] = Tpw.at<float>(2, 2);
    glTpw.m[11] = 0.0;

    glTpw.m[12] = Tpw.at<float>(0, 3);
    glTpw.m[13] = Tpw.at<float>(1, 3);
    glTpw.m[14] = Tpw.at<float>(2, 3);
    glTpw.m[15] = 1.0;
}