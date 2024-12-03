#include <kivi_project/3d_bilateral_filtering.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace kivi_project
{

float BilateralFiltering3D::euclideanDistance(
  const BilateralFiltering3D::PointXYZ & a, const BilateralFiltering3D::PointXYZ & b)
{
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

float BilateralFiltering3D::dotProduct(
  const BilateralFiltering3D::NormalXYZ & a, const BilateralFiltering3D::NormalXYZ & b)
{
  return a.nx * b.nx + a.ny * b.ny + a.nz * b.nz;
}

BilateralFiltering3D::NormalXYZ BilateralFiltering3D::normalize(
  const BilateralFiltering3D::NormalXYZ & n)
{
  float length = std::sqrt(n.nx * n.nx + n.ny * n.ny + n.nz * n.nz);
  NormalXYZ normal;
  normal.nx = n.nx / length;
  normal.ny = n.ny / length;
  normal.nz = n.nz / length;
  return normal;
}

std::vector<BilateralFiltering3D::PointXYZ> BilateralFiltering3D::getKNearestNeighbors(
  const PointXYZ & v, const CloudXYZ & allVertices, int k)
{
  std::vector<std::pair<float, PointXYZ>> distances;  // Pair of distance and vertex

  for (const auto & qi : allVertices) {
    float dist = euclideanDistance(v, qi.point);
    distances.push_back({dist, qi.point});
  }

  // Sort by distance
  std::sort(distances.begin(), distances.end(), [](const auto & a, const auto & b) {
    return a.first < b.first;
  });

  // Select the top k neighbors
  std::vector<PointXYZ> neighborhood;
  for (int i = 0; i < std::min(k, static_cast<int>(distances.size())); ++i) {
    neighborhood.push_back(distances[i].second);
  }

  return neighborhood;
}

BilateralFiltering3D::PointXYZ BilateralFiltering3D::denoisePoint(
  const PointXYZ & v, const NormalXYZ & n, const std::vector<PointXYZ> & neighborhood,
  float sigma_c, float sigma_s)
{
  size_t K = neighborhood.size();
  float sum = 0.0f;
  float normalizer = 0.0f;

  for (size_t i = 0; i < K; ++i) {
    const PointXYZ & qi = neighborhood[i];

    float t = euclideanDistance(v, qi);

    NormalXYZ diff;
    diff.nx = qi.x - v.x;
    diff.ny = qi.y - v.y;
    diff.nz = qi.z - v.z;
    float h = dotProduct(n, normalize(diff));

    // Compute the weight for spatial and range components
    float wc = std::exp(-std::pow(t, 2) / (2.0f * std::pow(sigma_c, 2)));
    float ws = std::exp(-std::pow(h, 2) / (2.0f * std::pow(sigma_s, 2)));

    // Update the sum and normalizer
    sum += wc * ws * h;
    normalizer += wc * ws;
  }

  float scale = sum / normalizer;

  PointXYZ v_hat;
  v_hat.x = v.x + (n.nx * scale);
  v_hat.y = v.y + (n.ny * scale);
  v_hat.z = v.z + (n.nz * scale);

  return v_hat;
}

std::vector<std::vector<std::vector<float>>> BilateralFiltering3D::readPLYFileWithNormals(
  const std::string & filename, int width, int height)
{
  std::ifstream inFile(filename);
  std::vector<std::vector<std::vector<float>>> organized_point_cloud(
    width, std::vector<std::vector<float>>(
             height, std::vector<float>(
                       6, std::numeric_limits<float>::quiet_NaN())));  // Initialize with NaN values

  std::string line;

  if (!inFile) {
    std::cerr << "Unable to open file\n";
    return organized_point_cloud;  // return empty opc
  }

  // Read and skip header lines until we reach end_header
  while (std::getline(inFile, line)) {
    if (line == "end_header") {
      break;
    }
  }

  int nan = 0;
  int no_nan = 0;
  int current_row = 0, current_col = 0;

  // Read data lines
  while (std::getline(inFile, line) && current_row < width) {
    std::istringstream lineStream(line);
    float x, y, z, nx, ny, nz;

    // If parsing succeeds and none of the values are NaN
    if ((lineStream >> x >> y >> z >> nx >> ny >> nz)) {
      organized_point_cloud[current_row][current_col] = {x, y, z, nx, ny, nz};
      no_nan++;
    } else {
      nan++;
    }

    current_col++;
    if (current_col >= height) {
      current_col = 0;
      current_row++;
    }
  }

  std::cout << "Nan: " << nan << " Not Nan: " << no_nan << std::endl;
  return organized_point_cloud;
}

void BilateralFiltering3D::writePLYFile(
  const std::string & filename,
  const std::vector<std::vector<std::vector<float>>> & organized_point_cloud, bool organized)
{
  std::ofstream plyFile(filename);

  if (!plyFile) {
    std::cout << "Unable to open file";
    return;
  }

  // Calculate total vertices
  size_t totalVertices = 0;
  if (!organized) {
    for (const auto & row : organized_point_cloud) {
      for (const auto & pt : row) {
        if (!std::isnan(pt[0])) {  // assuming if x is NaN, the entire point is invalid
          totalVertices++;
        }
      }
    }
  } else {
    totalVertices = organized_point_cloud.size() * organized_point_cloud[0].size();
  }

  plyFile << "ply\n";
  plyFile << "format ascii 1.0\n";
  plyFile << "element vertex " << totalVertices << "\n";
  plyFile << "property float x\n";
  plyFile << "property float y\n";
  plyFile << "property float z\n";
  plyFile << "property float nx\n";
  plyFile << "property float ny\n";
  plyFile << "property float nz\n";
  plyFile << "element face " << 0 << "\n";
  plyFile << "property list uchar int vertex_indices\n";
  plyFile << "end_header\n";

  for (const auto & row : organized_point_cloud) {
    for (const auto & pt : row) {
      if (organized || !std::isnan(pt[0])) {  // assuming if x is NaN, the entire point is invalid
        plyFile << pt[0] << " " << pt[1] << " " << pt[2] << " " << pt[3] << " " << pt[4] << " "
                << pt[5] << "\n";
      }
    }
  }
}

}  // namespace kivi_project