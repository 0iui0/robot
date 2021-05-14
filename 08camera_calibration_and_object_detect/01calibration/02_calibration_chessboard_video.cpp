//
// Created by zcb on 5/14/21.
//
/*
 *  相机标定 使用 棋盘
 *  1. 打开摄像头,读取每一帧
 *  2. 获取每个图片的角点(像素坐标uv)
 *  3. 通过亚像素优化角点的位置
 *  4. 创建对应的世界坐标(XYZw)
 *  5. 执行标定操作
 *  6. 保存标定的结果到文件中
 */
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;

void findCorners(cv::Mat &src, cv::Size &patternSize, bool &isFound, vector<cv::Point2f> &corners) {
    // src,patternSize 传入参数
    // isFound,corners 传出参数

    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    isFound = cv::findChessboardCorners(gray, patternSize, corners,
                                        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
                                        + cv::CALIB_CB_FAST_CHECK);
    if (isFound) {
        // 3 优化角点位置（只用于棋盘格）
        // 提高角点精度
        cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1);
        cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
    }

}

void getPoints(cv::Size &patternSize, float square_size, vector<cv::Point3f> &points) {
    /*
     * [0,0,0] [1,0,0] [2,0,0] * 20
     * [0,1,0] [1,1,0] [2,1,0] * 20
     */
    for (int i = 0; i < patternSize.height; ++i) {
        for (int j = 0; j < patternSize.width; ++j) {
            points.push_back(cv::Point3f(j * square_size, i * square_size, 0));
        }
    }
}

void saveToFile(cv::String filename, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, double r_mse) {
    // 获取当前时间
    time_t tm;
    time(&tm);
    struct tm *t2 = localtime(&tm);
    char buf[1024];
    strftime(buf, sizeof(buf), "%c", t2);

    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs << "calibration_time" << buf;// key -val
    fs << "camera_matrix" << cameraMatrix;
    fs << "dist_coeffs" << distCoeffs;
    fs << "r_mse" << r_mse;
    fs.release();

}

int main() {
    cv::Size patternSize(6, 9); // 棋盘的角点 size
    float square_size = 20.0; // 每个 棋盘格子的  ! 大小  20mm

    vector<vector<cv::Point2f>> imagePoints; // 所有 像素坐标系下的点
    vector<vector<cv::Point3f>> objectPoints; //所有 世界坐标系下的点,取标定板的平面为xy,原点设置在第一个角点


    cv::Size imgSize;

    //1 打开摄像头
    cv::VideoCapture capture(0);

    if (!capture.isOpened()) {
        cerr << "摄像头打开失败" << endl;
        return -1;
    } else {
        cv::Mat frame, frame_bak;
        int idx = 0;
        while (true) {
            bool isOk = capture.read(frame);
            if (isOk) {
                cv::flip(frame, frame, 1); // 左右 反转
                frame.copyTo(frame_bak); // frame_bak 为原图
            } else {
                cout << "未读到帧" << endl;
                continue;
            }


            bool isFound;
            vector<cv::Point2f> corners;
            findCorners(frame, patternSize, isFound, corners);
            if (isFound) { // 找打了角点
                cv::drawChessboardCorners(frame, patternSize, corners, isFound);
            }

            int key = cv::waitKey(30) & 0xFF;
            if (key == 27) { // ESC 键 // 直接退出
                return -1;
            } else if (key == 'a') {// a 键 // 收集 图像完毕，开始计算
                imgSize = frame.size();

                cout << "图像采集完毕,开始相机标定" << endl;
                cout << imagePoints.size() << endl;
                cout << objectPoints.size() << endl;

                break;
            } else if (key == 's') { // s键 使用该帧 图像
                vector<cv::Point3f> points; // 得到 该图片的 世界真实坐标
                getPoints(patternSize, square_size, points);
                if (isFound) {
                    imagePoints.push_back(corners);
                    objectPoints.push_back(points);

                    // 保存图片
                    cv::imwrite("/home/zcb/camera_calibration/01calibration/output_images/image_" + to_string(idx++) +
                                ".jpg", frame_bak);

                    // 进行闪屏 操作
                    cv::bitwise_not(frame, frame);  // 所有像素点 取反

                }
            }

            cv::imshow("frame", frame);
        }
        // 收集图像 完毕
        // 5 相机标定
        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
        vector<cv::Mat> rvecs;
        vector<cv::Mat> tvecs;

        double r_mse = cv::calibrateCamera(objectPoints, imagePoints, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs);

        cout << "rvecs";
        for (auto a:rvecs) {
            cout << a << endl;
        }
        cout << endl;

        cout << "tvecs";
        for (auto a:tvecs) {
            cout << a << endl;
        }
        cout << endl;



        // mse : 均方差  r_mse 均方差的根
        cout << "r_mse :" << r_mse;

        cout << "camera matrix";
        cout << cameraMatrix << endl;

        cout << "dist coeffs";
        cout << distCoeffs << endl;

        //6 保存标定数据到 文件
        saveToFile("/home/zcb/camera_calibration/01calibration/02calibration_chessboard_video.xml", cameraMatrix,
                   distCoeffs, r_mse);
        saveToFile("/home/zcb/camera_calibration/01calibration/02calibration_chessboard_video.yml", cameraMatrix,
                   distCoeffs, r_mse);

        cv::destroyAllWindows();
    }

    return 0;


}

