
#include <opencv2/opencv.hpp>

#include <iostream>
#include <memory>
#include <vector>

namespace kivi_project
{

class CorrespondenceMatching
{
public:
  using SharedPtr = std::shared_ptr<CorrespondenceMatching>;
  using ConstSharedPtr = const std::shared_ptr<CorrespondenceMatching>;

  std::vector<std::vector<double>> read_csv(const std::string & file_path);
  cv::Mat convert_to_mat(std::vector<std::vector<double>>& csv);
  void sum_of_squared_distances(
    std::vector<std::vector<double>>& left_image,
    std::vector<std::vector<double>>& right_image,
    std::vector<std::vector<double>>& disparity_map);
  double computeSSD(double left_val, double right_val);

  double height = 720.0;
  double width = 1280.0;

private:
};

}  // namespace kivi_project
