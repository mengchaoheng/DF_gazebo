#include "ductedfan_plugin/spline_ppval.h"

#include <gazebo/common/Console.hh>

#include <algorithm>
#include <fstream>
#include <sstream>

namespace gazebo
{

bool LoadSplineFromCsv(const std::string& file_path,
                       std::vector<SplineSegment>& spline_data)
{
  std::ifstream file(file_path.c_str());

  if (!file.is_open())
  {
    gzerr << "[SplinePPVal] 无法打开样条系数文件: "
          << file_path << "\n";
    return false;
  }

  spline_data.clear();

  std::string line;
  int line_index = 0;

  while (std::getline(file, line))
  {
    line_index++;

    if (line.empty())
    {
      continue;
    }

    std::replace(line.begin(), line.end(), ',', ' ');

    std::stringstream ss(line);

    SplineSegment segment;

    if (!(ss >> segment.x_left
             >> segment.x_right
             >> segment.a
             >> segment.b
             >> segment.c
             >> segment.d))
    {
      gzerr << "[SplinePPVal] 第 " << line_index
            << " 行无法解析: " << line << "\n";
      return false;
    }

    spline_data.push_back(segment);
  }

  if (spline_data.empty())
  {
    gzerr << "[SplinePPVal] 样条系数为空: "
          << file_path << "\n";
    return false;
  }

  std::sort(spline_data.begin(), spline_data.end(),
            [](const SplineSegment& lhs, const SplineSegment& rhs)
            {
              return lhs.x_left < rhs.x_left;
            });

  gzmsg << "[SplinePPVal] 成功读取样条区间数量: "
        << spline_data.size() << "\n";

  return true;
}

double PpvalSpline(const std::vector<SplineSegment>& spline_data,
                   double x,
                   bool clamp_input)
{
  if (spline_data.empty())
  {
    gzerr << "[SplinePPVal] 样条数据为空，返回 0\n";
    return 0.0;
  }

  double x_eval = x;

  if (clamp_input)
  {
    if (x_eval < spline_data.front().x_left)
    {
      x_eval = spline_data.front().x_left;
    }

    if (x_eval > spline_data.back().x_right)
    {
      x_eval = spline_data.back().x_right;
    }
  }

  const SplineSegment* selected_segment = nullptr;

  for (const auto& segment : spline_data)
  {
    if (x_eval >= segment.x_left && x_eval <= segment.x_right)
    {
      selected_segment = &segment;
      break;
    }
  }

  if (selected_segment == nullptr)
  {
    if (x_eval < spline_data.front().x_left)
    {
      selected_segment = &spline_data.front();
    }
    else
    {
      selected_segment = &spline_data.back();
    }
  }

  const double dx = x_eval - selected_segment->x_left;

  const double y =
      selected_segment->a * dx * dx * dx
    + selected_segment->b * dx * dx
    + selected_segment->c * dx
    + selected_segment->d;

  return y;
}

}
