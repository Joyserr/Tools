#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

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

void RotationVectorTransform(int select, Eigen::AngleAxisd &rotation_vector) {
  switch (select) { // 旋转向量转旋转矩阵
  case 1: {
    // 旋转向量转旋转矩阵
    Eigen::Matrix3d rotation_matrix = rotation_vector.matrix();
    cout << "旋转矩阵:" << endl;
    cout << rotation_matrix.transpose() << endl;
    break;
  }
  case 2: {
    Eigen::Vector3d eulerAngle = rotation_vector.matrix().eulerAngles(0, 1, 2);
    cout << "欧拉角:" << endl;
    cout << eulerAngle.transpose() << endl;
    break;
  }
  case 3: {
    Eigen::Quaterniond quaternion(rotation_vector);
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

void RotationMatrixTransform(int select, Eigen::Matrix3d &rotation_matrix) {
  switch (select) {
  case 1: {
    Eigen::AngleAxisd rotation_vector(rotation_matrix);
    cout << "旋转向量:" << endl;
    cout << rotation_vector.matrix() << endl;
    break;
  }
  case 2: {
    Eigen::Vector3d eulerAngle = rotation_matrix.eulerAngles(0, 1, 2);
    cout << "欧拉角:" << endl;
    cout << eulerAngle.transpose() << endl;
    break;
  }
  case 3: {
    Eigen::Quaterniond quaternion(rotation_matrix);
    cout << "四元数: x, y, z, w" << endl;
    cout << quaternion.x() << " " << quaternion.y() << " " << quaternion.z()
         << " " << quaternion.w() << endl;
    break;
  }
  default:
    break;
  }
}

void EulerAngleTransform(int select, Eigen::Vector3d &eulerAngle) {
  Eigen::AngleAxisd rollAngle(
      Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(
      Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(
      Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));

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


void toEulerAngle(const Eigen::Quaterniond &q, double &roll, double &pitch,
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

void QuaterniondTransform(int select, Eigen::Quaterniond &quaternion) {
  switch (select) {
  case 1: {
    Eigen::AngleAxisd rotation_vector(quaternion);
    cout << "旋转向量" << endl;
    cout << rotation_vector.matrix() << endl;
    break;
  }
  case 2: {
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = quaternion.matrix();
    cout << "旋转矩阵" << endl;
    cout << rotation_matrix.transpose() << endl;
    break;
  }
  case 3: {
    // Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(0, 1, 2);
    // cout << "欧拉角" << endl;
    // cout << eulerAngle.transpose() << endl;
    double roll;
    double pitch;
    double yaw;
    toEulerAngle(quaternion, roll, pitch, yaw);
    cout << "欧拉角" << endl;
    cout << roll * (180.0 / PI) << " " << pitch * (180.0 / PI) << " " << yaw * (180.0 / PI) << endl;
    break;
  }
  default:
    break;
  }
}

bool ParseYamlFileForRotationVector(const std::string &filename, int select, std::shared_ptr<Eigen::AngleAxisd> &rotation_vector_ptr)
{
  YAML::Node tf = YAML::LoadFile(filename);
  try {
    int select = tf["select"].as<int>();
    long double alpha = tf["rotation_vector"]["alpha"].as<long double>();
    long double x = tf["rotation_vector"]["x"].as<long double>();
    long double y = tf["rotation_vector"]["y"].as<long double>();
    long double z = tf["rotation_vector"]["z"].as<long double>();
    rotation_vector_ptr = std::make_shared<Eigen::AngleAxisd>(Eigen::AngleAxisd(alpha, Eigen::Vector3d(x, y, z)));
  } catch (...) {
    cout << "Extrinsic yaml file parse failed: " << filename;
    return false;
  }
  return true;
}

bool ParseYamlFileForRotationMatrix(const std::string &filename, int select, std::shared_ptr<Eigen::Matrix3d> &rotation_matrix_ptr)
{
  YAML::Node tf = YAML::LoadFile(filename);
  try {
    int select = tf["select"].as<int>();
    long double v_00 = tf["rotation_matrix"]["v_00"].as<long double>();
    long double v_01 = tf["rotation_matrix"]["v_01"].as<long double>();
    long double v_02 = tf["rotation_matrix"]["v_02"].as<long double>();
    long double v_10 = tf["rotation_matrix"]["v_10"].as<long double>();
    long double v_11 = tf["rotation_matrix"]["v_11"].as<long double>();
    long double v_12 = tf["rotation_matrix"]["v_12"].as<long double>();
    long double v_20 = tf["rotation_matrix"]["v_20"].as<long double>();
    long double v_21 = tf["rotation_matrix"]["v_21"].as<long double>();
    long double v_22 = tf["rotation_matrix"]["v_22"].as<long double>();
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << v_00, v_01, v_02, v_10, v_11, v_12, v_20, v_21, v_22;
    rotation_matrix_ptr = std::make_shared<Eigen::Matrix3d>(rotation_matrix);
  } catch (...) {
    cout << "Extrinsic yaml file parse failed: " << filename;
    return false;
  }
  return true;
}

bool ParseYamlFileForEulerAngle(const std::string &filename, int select, std::shared_ptr<Eigen::Vector3d> &eulerAngle_ptr)
{
  YAML::Node tf = YAML::LoadFile(filename);
  try {
    int select = tf["select"].as<int>();
    long double roll = tf["euler_angle"]["roll"].as<long double>();
    long double pitch = tf["euler_angle"]["pitch"].as<long double>();
    long double yaw = tf["euler_angle"]["yaw"].as<long double>();
    eulerAngle_ptr = std::make_shared<Eigen::Vector3d>(Eigen::Vector3d(roll, pitch, yaw));
  } catch (...) {
    cout << "Extrinsic yaml file parse failed: " << filename;
    return false;
  }
  return true;
}

bool ParseYamlFileForQuaterniond(const std::string &filename, int select, std::shared_ptr<Eigen::Quaterniond> &quaternion_ptr)
{
  YAML::Node tf = YAML::LoadFile(filename);
  try {
    int select = tf["select"].as<int>();
    long double x = tf["quaternion"]["x"].as<long double>();
    long double y = tf["quaternion"]["y"].as<long double>();
    long double z = tf["quaternion"]["z"].as<long double>();
    long double w = tf["quaternion"]["w"].as<long double>();
    quaternion_ptr = std::make_shared<Eigen::Quaterniond>(Eigen::Quaterniond(w, x, y, z));
  } catch (...) {
    cout << "Extrinsic yaml file parse failed: " << filename;
    return false;
  }
  return true;
}

int main(int argc, char **argv) {
  GuidPrint();
  if (argc < 2) {
    cout << "Parameter error" << endl;
    exit(EXIT_SUCCESS);
  }

  if (0 == strcmp(argv[2], "help")) {
    GuidPrint();
  }

  int select = atoi(argv[1]);

  switch (select / 10) {
  case 1: //旋转向量相关变换
  {
    long double alpha = stold(argv[2]);
    long double x = stold(argv[3]);
    long double y = stold(argv[4]);
    long double z = stold(argv[5]);
    cout << "alpha = " << alpha << " x = " << x << " y = " << y << " z = " << z
         << endl;
    // 初始化旋转向量
    Eigen::AngleAxisd rotation_vector(alpha, Eigen::Vector3d(x, y, z));
    RotationVectorTransform(select % 10, rotation_vector);
    break;
  }
  case 2: // 旋转矩阵
  {
    long double v_00 = stold(argv[2]);
    long double v_01 = stold(argv[3]);
    long double v_02 = stold(argv[4]);
    long double v_10 = stold(argv[5]);
    long double v_11 = stold(argv[6]);
    long double v_12 = stold(argv[7]);
    long double v_20 = stold(argv[8]);
    long double v_21 = stold(argv[9]);
    long double v_22 = stold(argv[10]);
    cout << "v_00 " << v_00 << " v_01 " << v_01 << " v_02 " << v_02 << endl;
    cout << "v_10 " << v_10 << " v_11 " << v_11 << " v_12 " << v_12 << endl;
    cout << "v_20 " << v_20 << " v_21 " << v_21 << " v_22 " << v_22 << endl;
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << v_00, v_01, v_02, v_10, v_11, v_12, v_20, v_21, v_22;
    RotationMatrixTransform(select % 10, rotation_matrix);
    break;
  }
  case 3: {
    long double roll = stold(argv[2]);
    long double pitch = stold(argv[3]);
    long double yaw = stold(argv[4]);
    cout << "roll = " << roll << " pitch = " << pitch << " yaw = " << yaw
         << endl;
    Eigen::Vector3d eulerAngle(roll, pitch, yaw);
    EulerAngleTransform(select % 10, eulerAngle);
    break;
  }
  case 4: {
    long double w = stold(argv[2]);
    long double x = stold(argv[3]);
    long double y = stold(argv[4]);
    long double z = stold(argv[5]);
    cout << "w = " << w << " x = " << x << " y = " << y << " z = " << z << endl;
    Eigen::Quaterniond quaternion(w, x, y, z);
    QuaterniondTransform(select % 10, quaternion);
    break;
  }
  default:
    break;
  }

  return 0;
}