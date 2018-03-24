namespace ros_tf_utils {
const double kINF = 1e18;
const double kPI = 3.1415926535897932384626433832795028;

inline double PoseL2Norm2D(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2) {
  double x = p1.position.x - p2.position.x;
  double y = p1.position.y - p2.position.y;
  return std::sqrt(x * x + y * y);
}

inline void matrix4fToTransform(const Eigen::Matrix4f &e, tf::Transform &t) {
  t.setOrigin(tf::Vector3(e(0, 3), e(1, 3), e(2, 3)));
  t.setBasis(tf::Matrix3x3(e(0, 0), e(0, 1), e(0, 2),
                           e(1, 0), e(1, 1), e(1, 2),
                           e(2, 0), e(2, 1), e(2, 2)));
}

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

inline void poseToMatrix4f(const geometry_msgs::Pose &p, Eigen::Matrix4f &e) {
  Eigen::Quaternionf q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  Eigen::Matrix3f m = q.normalized().toRotationMatrix();

  e(0, 3) = p.position.x;
  e(1, 3) = p.position.y;
  e(2, 3) = p.position.z;
  e(3, 0) = 0;
  e(3, 1) = 0;
  e(3, 2) = 0;
  e(3, 3) = 1;

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      e(i, j) = m(i, j);
    }
  }
}

inline float poseToPitch(const geometry_msgs::Pose &p) {
  double r, pitch, y;
  tf::Matrix3x3(tf::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)).getRPY(r, pitch, y);

  return pitch;
}

inline void eulerToMatrix4f(float roll, float pitch, float yaw, Eigen::Matrix4f &e) {
  tf::Matrix3x3 m;
  m.setEulerYPR(yaw, pitch, roll);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      e(i, j) = m[i][j];
    }
  }
  e(3,0) = 0;
  e(3,1) = 0;
  e(3,2) = 0;
  e(3,3) = 1;

  e(0,3) = 0;
  e(1,3) = 0;
  e(2,3) = 0;
}

inline float radToDegree(float rad) {
  return rad / kPI * 180.0;
}
}


