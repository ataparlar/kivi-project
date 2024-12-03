#include <kivi_project/correspondence_matching.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace kivi_project
{
std::vector<std::vector<double>> CorrespondenceMatching::read_csv(const std::string & file_name)
{
  std::ifstream file(file_name);
  std::vector<std::vector<double>> matrix;

  std::string line, value;

  while (std::getline(file, line)) {
    std::vector<double> row;
    std::stringstream ss(line);
    while (std::getline(ss, value, ',')) {
      row.push_back(std::stod(value));
    }
    matrix.push_back(row);
  }
  return matrix;
}

cv::Mat CorrespondenceMatching::convert_to_mat(std::vector<std::vector<double>> & csv)
{
  size_t rows = csv.size();
  size_t cols = csv.empty() ? 0 : csv[0].size();

  cv::Mat image(rows, cols, CV_64F);

  for (size_t i = 0; i < rows; ++i) {
    for (size_t j = 0; j < cols; ++j) {
      image.at<double>(i, j) = csv[i][j];
    }
  }

  return image;
}

double CorrespondenceMatching::computeSSD(double left_val, double right_val) {
  return std::pow(left_val - right_val, 2);
}

void CorrespondenceMatching::sum_of_squared_distances(
  std::vector<std::vector<double>>& left_image,
  std::vector<std::vector<double>>& right_image,
  std::vector<std::vector<double>>& disparity_map)
{
  int window_size = 15;

  for (int y=0; y<height; y++) {
    for (int x_l=0; x_l<width; x_l++) {

      double min_ssd = std::numeric_limits<double>::infinity();
      int best_match_x_r = x_l;

      int width_int = width;
      for (
        int x_r = std::max(0, x_l - window_size);
            x_r < std::min(width_int, x_l + window_size);
            x_r++) {
        double ssd = computeSSD(left_image[y][x_l], right_image[y][x_r]);
        if (ssd < min_ssd) {
          min_ssd = ssd;
          best_match_x_r = x_r;
        }
      }

      disparity_map[y][x_l] = x_l - best_match_x_r;
    }
  }
}

}  // namespace kivi_project