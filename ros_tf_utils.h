namespace ros_tf_utils {
const double kINF = 1e18;
const double kPI = 3.1415926535897932384626433832795028;

/// Compute 2D plane (only x,y) L2 distance between two pose msgs
/// \param p1
/// \param p2
/// \return
inline double PoseL2Norm2D(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2) {
  double x = p1.position.x - p2.position.x;
  double y = p1.position.y - p2.position.y;
  return std::sqrt(x * x + y * y);
}

/// Eigen 4*4 matrix to ros tf transform msg
/// \param e
/// \param t
inline void matrix4fToTransform(const Eigen::Matrix4f &e, tf::Transform &t) {
  t.setOrigin(tf::Vector3(e(0, 3), e(1, 3), e(2, 3)));
  t.setBasis(tf::Matrix3x3(e(0, 0), e(0, 1), e(0, 2),
                           e(1, 0), e(1, 1), e(1, 2),
                           e(2, 0), e(2, 1), e(2, 2)));
}

/// Eigen 4*4 matrix to pose msg
/// \param e
/// \param p
inline void matrix4fToPose(const Eigen::Matrix4f &e, geometry_msgs::Pose &p) {
  Eigen::Matrix3f m;
  // can not initialize Matrix3f directly in constructor
  m << e(0, 0), e(0, 1), e(0, 2),
      e(1, 0), e(1, 1), e(1, 2),
      e(2, 0), e(2, 1), e(2, 2);
  Eigen::Quaternionf q(m);
  p.orientation.w = q.w();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();

  p.position.x = e(0, 3);
  p.position.y = e(1, 3);
  p.position.z = e(2, 3);
}

/// Pose msg to roll, pitch, yaw
/// \param p
/// \param r
/// \param p
/// \param y
/// \return
inline float poseToEuler(const geometry_msgs::Pose &pose, double &r, double &p, double &y) {
  tf::Matrix3x3(tf::Quaternion(pose.orientation.x,
                               pose.orientation.y,
                               pose.orientation.z,
                               pose.orientation.w)).getRPY(r, p, y);
}

inline void xyzToMatrix4f(float x, float y, float z, Eigen::Matrix4f &e) {
  e << 1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1;
}

/// pose msg to Eigen 4*4 matrix
/// \param r
/// \param p
/// \param y
/// \param e
inline void eulerToMatrix4f(float r, float p, float y, Eigen::Matrix4f &e) {
  Eigen::AngleAxisf rot_x(r, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rot_y(p, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z(y, Eigen::Vector3f::UnitZ());
  Eigen::Matrix3f rot_mat = (rot_z * rot_y * rot_x).matrix();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      e(i,j) = rot_mat(i,j);
    }
  }
  e(0,3) = 0;
  e(1,3) = 0;
  e(2,3) = 0;
  e(3,3) = 1;
}

inline double radToDeg(double rad) {
  return rad / kPI * 180.0;
}

inline double degToRad(double deg) {
  return deg / 180.0 * kPI;
}
}


