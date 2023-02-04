#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <sys/stat.h>

#include "../include/yaml-cpp/yaml.h"

#define PI 3.1415926

using namespace std;

void GuidPrint(void) {
  // std::cout << "Please enter your selection." << std::endl;
  cout << "请输入你需要的转换:" << endl;
  cout << "\t11:旋转向量转旋转矩阵"
       << "\texample: ./TfToolBox 11 alpha x y z" << endl;
  cout << "\t12:旋转向量转欧拉角"
       << "\texample: ./TfToolBox 12 alpha x y z" << endl;
  cout << "\t13:旋转向量转四元数"
       << "\texample: ./TfToolBox 13 alpha x y z" << endl;

  cout << "\t21:旋转矩阵转旋转向量" << endl;
  cout << "\t22:旋转矩阵转欧拉角(xyz,即RPY)" << endl;
  cout << "\t23:旋转矩阵转四元数" << endl;

  cout << "\t31:欧拉角转旋转向量"
       << "\texample: ./TfToolBox 31 roll pitch yaw" << endl;
  cout << "\t32:欧拉角转旋转矩阵"
       << "\texample: ./TfToolBox 32 roll pitch yaw" << endl;
  cout << "\t33:欧拉角转四元数"
       << "\texample: ./TfToolBox 33 roll pitch yaw" << endl;

  cout << "\t41:四元数转旋转向量"
       << "\texample: ./TfToolBox 41 w x y z" << endl;
  cout << "\t42:四元数转旋转矩阵"
       << "\texample: ./TfToolBox 42 w x y z" << endl;
  cout << "\t43:四元数转欧拉角(xyz,即RPY)"
       << "\texample: ./TfToolBox 43 w x y z" << endl;
}

void RotationVectorTransform(
    int select, std::shared_ptr<Eigen::AngleAxisd> &rotation_vector) {
  switch (select) { // 旋转向量转旋转矩阵
  case 1: {
    // 旋转向量转旋转矩阵
    Eigen::Matrix3d rotation_matrix = rotation_vector->matrix();
    cout << "旋转矩阵:" << endl;
    cout << rotation_matrix.transpose() << endl;
    break;
  }
  case 2: {
    Eigen::Vector3d eulerAngle = rotation_vector->matrix().eulerAngles(0, 1, 2);
    cout << "欧拉角:" << endl;
    cout << eulerAngle.transpose() << endl;
    break;
  }
  case 3: {
    Eigen::Quaterniond quaternion(*rotation_vector);
    cout << "四元数: x, y, z, w" << endl;
    cout << quaternion.x() << " " << quaternion.y() << " " << quaternion.z()
         << " " << quaternion.w() << endl;
    break;
  }
  default:
    cout << "Param error!!!" << endl;
    break;
  }
}

void RotationMatrixTransform(
    int select, std::shared_ptr<Eigen::Matrix3d> &rotation_matrix_ptr) {
  switch (select) {
  case 1: {
    Eigen::AngleAxisd rotation_vector(*rotation_matrix_ptr);
    cout << "旋转向量:" << endl;
    cout << rotation_vector.matrix() << endl;
    break;
  }
  case 2: {
    Eigen::Vector3d eulerAngle = rotation_matrix_ptr->eulerAngles(0, 1, 2);
    cout << "欧拉角:" << endl;
    cout << eulerAngle.transpose() << endl;
    break;
  }
  case 3: {
    Eigen::Quaterniond quaternion(*rotation_matrix_ptr);
    cout << "四元数: x, y, z, w" << endl;
    cout << quaternion.x() << " " << quaternion.y() << " " << quaternion.z()
         << " " << quaternion.w() << endl;
    break;
  }
  default:
    break;
  }
}

void EulerAngleTransform(int select,
                         std::shared_ptr<Eigen::Vector3d> &eulerAngle_ptr) {
  Eigen::AngleAxisd rollAngle(
      Eigen::AngleAxisd((*eulerAngle_ptr)(0), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(
      Eigen::AngleAxisd((*eulerAngle_ptr)(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(
      Eigen::AngleAxisd((*eulerAngle_ptr)(2), Eigen::Vector3d::UnitZ()));

  switch (select) {
  case 1: {
    Eigen::AngleAxisd rotation_vector;
    rotation_vector = yawAngle * pitchAngle * rollAngle;
    cout << "旋转向量" << endl;
    cout << rotation_vector.matrix() << endl;
    break;
  }
  case 2: {
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = yawAngle * pitchAngle * rollAngle;
    cout << "旋转矩阵" << endl;
    cout << rotation_matrix.transpose() << endl;
    break;
  }
  case 3: {
    Eigen::Quaterniond quaternion;
    quaternion = yawAngle * pitchAngle * rollAngle;
    cout << "四元数: x, y, z, w" << endl;
    cout << quaternion.x() << " " << quaternion.y() << " " << quaternion.z()
         << " " << quaternion.w() << endl;
    break;
  }
  default:
    break;
  }
}

void ToEulerAngle(const Eigen::Quaterniond &q, double &roll, double &pitch,
                  double &yaw) {
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yaw = atan2(siny_cosp, cosy_cosp);
}

void QuaterniondTransform(int select,
                          std::shared_ptr<Eigen::Quaterniond> &quaternion_ptr) {
  switch (select) {
  case 1: {
    Eigen::AngleAxisd rotation_vector(*quaternion_ptr);
    cout << "旋转向量" << endl;
    cout << rotation_vector.matrix() << endl;
    break;
  }
  case 2: {
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = quaternion_ptr->matrix();
    cout << "旋转矩阵" << endl;
    cout << rotation_matrix.transpose() << endl;
    break;
  }
  case 3: {
    double roll;
    double pitch;
    double yaw;
    ToEulerAngle(*quaternion_ptr, roll, pitch, yaw);
    cout << "欧拉角" << endl;
    cout << "roll: " << roll * (180.0 / PI) << endl
         << "pitch: " << pitch * (180.0 / PI) << endl
         << "yaw: " << yaw * (180.0 / PI) << endl;
    break;
  }
  default:
    break;
  }
}

bool ParseYamlFileForRotationVector(
    const std::string &filename,
    std::shared_ptr<Eigen::AngleAxisd> &rotation_vector_ptr) {
  YAML::Node tf = YAML::LoadFile(filename);
  try {
    vector<long double> rotation_data =
        tf["rotation_vector"]["data"].as<vector<long double>>();
    long double alpha = rotation_data[0];
    long double x = rotation_data[1];
    long double y = rotation_data[2];
    long double z = rotation_data[3];
    rotation_vector_ptr = std::make_shared<Eigen::AngleAxisd>(
        Eigen::AngleAxisd(alpha, Eigen::Vector3d(x, y, z)));
  } catch (...) {
    cout << "Extrinsic yaml file parse failed: " << filename;
    return false;
  }
  return true;
}

bool ParseYamlFileForRotationMatrix(
    const std::string &filename,
    std::shared_ptr<Eigen::Matrix3d> &rotation_matrix_ptr) {
  YAML::Node tf = YAML::LoadFile(filename);

  try {
    int row = tf["rotation_matrix"]["row"].as<int>();
    int col = tf["rotation_matrix"]["col"].as<int>();
    Eigen::Matrix3d rotation_matrix;
    std::vector<long double> matrix3d_temp =
        tf["rotation_matrix"]["data"].as<vector<long double>>();
    assert((int)matrix3d_temp.size() == 9);
    for (int i = 0; i < row; ++i) {
      for (int j = 0; j < col; ++j) {
        rotation_matrix(i, j) = matrix3d_temp[i + j * col];
      }
    }
    rotation_matrix_ptr = std::make_shared<Eigen::Matrix3d>(rotation_matrix);
  } catch (...) {
    cout << "Extrinsic yaml file parse failed: " << filename;
    return false;
  }

  return true;
}

bool ParseYamlFileForEulerAngle(
    const std::string &filename,
    std::shared_ptr<Eigen::Vector3d> &eulerAngle_ptr) {
  YAML::Node tf = YAML::LoadFile(filename);

  try {
    vector<long double> euler_angle_data =
        tf["euler_angle"]["data"].as<vector<long double>>();
    auto roll = euler_angle_data[0];
    auto pitch = euler_angle_data[1];
    auto yaw = euler_angle_data[2];
    eulerAngle_ptr =
        std::make_shared<Eigen::Vector3d>(Eigen::Vector3d(roll, pitch, yaw));
  } catch (...) {
    cout << "Extrinsic yaml file parse failed: " << filename;
    return false;
  }
  return true;
}

bool ParseYamlFileForQuaterniond(
    const std::string &filename,
    std::shared_ptr<Eigen::Quaterniond> &quaternion_ptr) {
  YAML::Node tf = YAML::LoadFile(filename);

  try {
    vector<long double> quaternion_data =
        tf["quaternion"]["data"].as<vector<long double>>();
    auto x = quaternion_data[0];
    auto y = quaternion_data[1];
    auto z = quaternion_data[2];
    auto w = quaternion_data[3];
    quaternion_ptr =
        std::make_shared<Eigen::Quaterniond>(Eigen::Quaterniond(x, y, z, w));
  } catch (...) {
    cout << "Extrinsic yaml file parse failed: " << filename;
    return false;
  }
  return true;
}

int main(int argc, char **argv) {
  if (argc != 2) {
    cout << "Parameter error" << endl;
    cout << "Example: ./TfToolBox path/file.yaml" << endl;
    return 0;
  }

  // int select = atoi(argv[1]);
  string filename = argv[1];
  struct stat buffer;
  if (stat(filename.c_str(), &buffer) != 0) {
    cout << "Error:" << filename << " file does not exist" << endl;
    return 1;
  }
  YAML::Node tf = YAML::LoadFile(filename);
  int select = tf["select"].as<int>();
  switch (select / 10) {
  case 1: // 旋转向量相关变换
  {
    std::shared_ptr<Eigen::AngleAxisd> rotation_vector_ptr;
    if (true != ParseYamlFileForRotationVector(filename, rotation_vector_ptr)) {
      return 1;
    }
    RotationVectorTransform(select % 10, rotation_vector_ptr);
    break;
  }
  case 2: // 旋转矩阵
  {
    std::shared_ptr<Eigen::Matrix3d> rotation_matrix_ptr;
    if (true != ParseYamlFileForRotationMatrix(filename, rotation_matrix_ptr)) {
      return 1;
    }
    RotationMatrixTransform(select % 10, rotation_matrix_ptr);
    break;
  }
  case 3: {
    std::shared_ptr<Eigen::Vector3d> eulerAngle_ptr;
    if (true != ParseYamlFileForEulerAngle(filename, eulerAngle_ptr)) {
      return 1;
    }
    EulerAngleTransform(select % 10, eulerAngle_ptr);
    break;
  }
  case 4: {
    std::shared_ptr<Eigen::Quaterniond> quaternion_ptr;
    if (true != ParseYamlFileForQuaterniond(filename, quaternion_ptr)) {
      return 1;
    }
    QuaterniondTransform(select % 10, quaternion_ptr);
    break;
  }
  default:
    break;
  }

  return 0;
}