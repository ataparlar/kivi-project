#include "kivi_project/main.hpp"

namespace kivi_project
{

KiviProject::KiviProject()
{
  phase_map_generator_ = std::make_shared<PhaseMapGenerator>();
  bilateral_filter_3d_ = std::make_shared<BilateralFiltering3D>();
  correspondence_matching_ = std::make_shared<CorrespondenceMatching>();
}

void KiviProject::double_three_step_phase_shift()
{
  cv::Mat pattern1 = phase_map_generator_->generate_pattern(0);
  cv::Mat pattern2 = phase_map_generator_->generate_pattern(60);
  cv::Mat pattern3 = phase_map_generator_->generate_pattern(120);

  cv::Mat phase_map1 = phase_map_generator_->compute_phase_map(pattern1, pattern2, pattern3);

  cv::Mat pattern4 = phase_map_generator_->generate_pattern(180);
  cv::Mat pattern5 = phase_map_generator_->generate_pattern(240);
  cv::Mat pattern6 = phase_map_generator_->generate_pattern(300);

  cv::Mat phase_map2 = phase_map_generator_->compute_phase_map(pattern4, pattern5, pattern6);

  cv::Mat average_phase_map = phase_map_generator_->average_of_phase_maps(phase_map1, phase_map2);

  cv::imshow("Display window", average_phase_map);
  cv::waitKey(0);

  cv::Mat normalized_phase_map;
  cv::normalize(average_phase_map, normalized_phase_map, 0, 255, cv::NORM_MINMAX);
  cv::imwrite(
    "/home/ataparlarl/projects/kivi_assignment/kivi_project/export/average_phase_map.png",
    normalized_phase_map);
}

void KiviProject::bilateral_filter_3d()
{
  std::vector<std::vector<std::vector<float>>> cloud = bilateral_filter_3d_->readPLYFileWithNormals(
    "/home/ataparlarl/projects/kivi_assignment/KiviTechnologies_CV_Task/Tasks/3D-bilateral/"
    "OriginalMesh.ply",
    2592, 1944);

  BilateralFiltering3D::CloudXYZ cloud_;
  for (auto row : cloud) {
    for (auto col : row) {
      BilateralFiltering3D::TriangleXYZ triangle;
      triangle.point.x = col.at(0);
      triangle.point.y = col.at(1);
      triangle.point.z = col.at(2);
      triangle.normal.nx = col.at(3);
      triangle.normal.ny = col.at(4);
      triangle.normal.nz = col.at(5);
      cloud_.push_back(triangle);
    }
  }

  std::cout << "cloud.size: " << cloud.size() << std::endl;
  std::vector<BilateralFiltering3D::PointXYZ> filtered_cloud_;
  for (auto triangle : cloud_) {
    auto k_nearest_points = bilateral_filter_3d_->getKNearestNeighbors(triangle.point, cloud_, 4);

    auto filtered_point = bilateral_filter_3d_->denoisePoint(
      triangle.point, triangle.normal, k_nearest_points, 0.5, 0.5);

    filtered_cloud_.push_back(filtered_point);
    std::cout << "filtering" << std::endl;
  }

  std::vector<std::vector<std::vector<float>>> cloud_to_save;

  for (int i = 0; i < 2592; i++) {
    std::vector<std::vector<float>> col_vector;

    for (int j = 0; j < 1944; j++) {
      std::vector<float> point_vector = {
        filtered_cloud_.at(i * 2592 + j).x, filtered_cloud_.at(i * 2592 + j).y,
        filtered_cloud_.at(i * 2592 + j).z};
      col_vector.push_back(point_vector);
    }
    cloud_to_save.push_back(col_vector);
    // std::cout << "continue" << std::endl;
  }

  std::cout << "BEFORE SAVE" << std::endl;

  bilateral_filter_3d_->writePLYFile(
    "/home/ataparlarl/projects/kivi_assignment/kivi_project/export/filtered_cloud.ply",
    cloud_to_save, false);

  std::cout << "saved" << std::endl;
}

void KiviProject::correspondence_matching()
{
  std::vector<std::vector<double>> left_image = correspondence_matching_->read_csv(
    "/home/ataparlarl/projects/kivi_assignment/KiviTechnologies_CV_Task/Tasks/"
    "correspondence-matching/left_uwp_map.csv");
  // cv::Mat left_image = correspondence_matching_->convert_to_mat(left_csv);

  std::vector<std::vector<double>> right_image = correspondence_matching_->read_csv(
    "/home/ataparlarl/projects/kivi_assignment/KiviTechnologies_CV_Task/Tasks/"
    "correspondence-matching/right_uwp_map.csv");
  // cv::Mat right_image = correspondence_matching_->convert_to_mat(left_csv);


  std::vector<std::vector<double>> disparity_map(
    correspondence_matching_->height,
    std::vector<double>(correspondence_matching_->width, 0));


  correspondence_matching_->sum_of_squared_distances(
    left_image, right_image, disparity_map);
  cv::Mat disp_map_image = correspondence_matching_->convert_to_mat(disparity_map);


  // cv::Mat normalized_image_left;
  // cv::normalize(left_image, normalized_image_left, 0.0, 1.0, cv::NORM_MINMAX);
  // cv::imshow("Display window", normalized_image_left);
  // cv::waitKey(0);
  //
  // cv::Mat normalized_image_right;
  // cv::normalize(right_image, normalized_image_right, 0.0, 1.0, cv::NORM_MINMAX);
  // cv::imshow("Display window", normalized_image_right);
  // cv::waitKey(0);


  cv::Mat normalized_disparity_map;
  cv::normalize(disp_map_image, normalized_disparity_map, 0.0, 1.0, cv::NORM_MINMAX);
  cv::imshow("Display window", normalized_disparity_map);
  cv::waitKey(0);

  for (int row=0; row<720; row++) {
    for (int col=0; col<1280; col++) {
      std::cout << disparity_map.at(row).at(col) << ", ";
    }
    std::cout << std::endl;
  }

}

}  // namespace kivi_project

int main(int, char **)
{
  kivi_project::KiviProject kivi;

  // kivi.double_three_step_phase_shift();

  // kivi.bilateral_filter_3d();

  kivi.correspondence_matching();
}
