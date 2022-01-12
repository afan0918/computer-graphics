#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

// 計算巴斯卡三角形
int factorial(int x) {  
    if (x == 0 || x == 1)
        return 1;
    else
        return factorial(x - 1) * x;
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm

    // 點只會有四個
    // 公式 b = b0(1-t)^3+b1*3t(1-t)^2+b2*3t^2(1-t)+b3*t^3
    // cv::Point2f point = cv::Point2f(
    //     control_points[0].x*pow((1-t), 3)+
    //     control_points[1].x*3*t*pow((1-t), 2)+
    //     control_points[2].x*3*pow(t, 2)*(1-t)+
    //     control_points[3].x*pow(t, 3),
    //     control_points[0].y*pow((1-t), 3)+
    //     control_points[1].y*3*t*pow((1-t), 2)+
    //     control_points[2].y*3*pow(t, 2)*(1-t)+
    //     control_points[3].y*pow(t, 3));

    // 不只四個的情況
    cv::Point2f point = cv::Point2f(0.0f, 0.0f);
    for(int i = 0; i < control_points.size(); i++) {
        int c = factorial(control_points.size() - 1) / (factorial(i) * factorial(control_points.size() - 1 - i));  
        point += control_points[i] * c * pow(1-t, control_points.size()-i-1) * pow(t, i);
    }
    
    return point;
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.

    // for(float t = 0.0; t < 1.0; t += 0.001){
    //     cv::Point2f point = recursive_bezier(control_points, t);
    //     window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    // }

    for (double t = 0.0; t <= 1.0; t += 0.0001) {  
        cv::Point2f point = recursive_bezier(control_points, t);  
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;  
  
        // anti-aliasing  
        float x = point.x - std::floor(point.x);  
        float y = point.y - std::floor(point.y);  
        int x_flag = x < 0.5f ? -1 : 1;  
        int y_flag = y < 0.5f ? -1 : 1;  
  
        // 和採樣點最近的四個座標點
        cv::Point2f p00 = cv::Point2f(std::floor(point.x) + 0.5f, std::floor(point.y) + 0.5f);  
        cv::Point2f p01 = cv::Point2f(std::floor(point.x + x_flag * 1.0f) + 0.5f, std::floor(point.y) + 0.5f);  
        cv::Point2f p10 = cv::Point2f(std::floor(point.x) + 0.5f, std::floor(point.y + y_flag * 1.0f) + 0.5f);  
        cv::Point2f p11 = cv::Point2f(std::floor(point.x + x_flag * 1.0f) + 0.5f, std::floor(point.y + y_flag * 1.0f) + 0.5f);  
  
        std::vector<cv::Point2f> vec;  
        vec.push_back(p01);  
        vec.push_back(p10);  
        vec.push_back(p11);  
  
        // 計算座標點與採樣點距離 
        cv::Point2f distance = p00 - point;  
        float len = sqrt(distance.x * distance.x + distance.y * distance.y);  
  
        // 對邊緣點著色 
        for(auto p:vec) {  
            // 根據距離算要反鋸齒的程度
            cv::Point2f d = p - point;  
            float l = sqrt(d.x * d.x + d.y * d.y);  
            float percnet = len / l;  
  
            cv::Vec3d color = window.at<cv::Vec3b>(p.y, p.x);  
            // 暴力取極大值，其他情況不能亂這樣做  
            color[1] = std::max(color[1], (double)255 * percnet);  
            window.at<cv::Vec3b>(p.y, p.x) = color;  
        }  
    } 
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() >= 4) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(1);
        }
        cv::imshow("Bezier Curve", window);  
        key = cv::waitKey(20); 
    }

    return 0;
}
