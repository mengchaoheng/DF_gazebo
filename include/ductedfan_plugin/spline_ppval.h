#pragma once

#include <string>
#include <vector>

namespace gazebo
{

struct SplineSegment
{
  double x_left;
  double x_right;
  double a;
  double b;
  double c;
  double d;
};

// 从 CSV 文件读取 MATLAB 导出的 pp 系数
bool LoadSplineFromCsv(const std::string& file_path,
                       std::vector<SplineSegment>& spline_data);

// 实现 MATLAB ppval(pp, x) 的等价计算
double PpvalSpline(const std::vector<SplineSegment>& spline_data,
                   double x,
                   bool clamp_input = false);

}