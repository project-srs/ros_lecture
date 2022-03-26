#include <geometry_msgs/Pose.h>

struct EulerRad {
  float roll{0};
  float pitch{0};
  float yaw{0};
};

EulerRad getRpy(const geometry_msgs::Quaternion& q) {
  EulerRad euler;
  float q0q0 = q.w * q.w;
  float q0q1 = q.w * q.x;
  float q0q2 = q.w * q.y;
  float q0q3 = q.w * q.z;
  float q1q1 = q.x * q.x;
  float q1q2 = q.x * q.y;
  float q1q3 = q.x * q.z;
  float q2q2 = q.y * q.y;
  float q2q3 = q.y * q.z;
  float q3q3 = q.z * q.z;
  euler.roll = atan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3);
  euler.pitch = asin(2.0 * (q0q2 - q1q3));
  euler.yaw = atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
  return euler;
}

geometry_msgs::Quaternion mul(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2) {
  geometry_msgs::Quaternion out;
  out.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  out.x = q1.x * q2.w + q1.w * q2.x - q1.z * q2.y + q1.y * q2.z;
  out.y = q1.y * q2.w + q1.z * q2.x + q1.w * q2.y - q1.x * q2.z;
  out.z = q1.z * q2.w - q1.y * q2.x + q1.x * q2.y + q1.w * q2.z;
  return out;
}

geometry_msgs::Point rotate(geometry_msgs::Point p, geometry_msgs::Quaternion q) {
  geometry_msgs::Quaternion pos;
  pos.w = 0;
  pos.x = p.x;
  pos.y = p.y;
  pos.z = p.z;
  geometry_msgs::Quaternion con;
  con.w = q.w;
  con.x = -q.x;
  con.y = -q.y;
  con.z = -q.z;
  geometry_msgs::Quaternion post = mul(mul(q,pos),con);

  geometry_msgs::Point out;
  out.x = post.x;
  out.y = post.y;
  out.z = post.z;
  return out;
}

geometry_msgs::Point add(geometry_msgs::Point a, geometry_msgs::Point b) {
  geometry_msgs::Point out;
  out.x = a.x + b.x;
  out.y = a.y + b.y;
  out.z = a.z + b.z;
  return out;
}

geometry_msgs::Point invert(geometry_msgs::Point p) {
  geometry_msgs::Point out;
  out.x = -p.x;
  out.y = -p.y;
  out.z = -p.z;
  return out;
}

geometry_msgs::Quaternion invert(geometry_msgs::Quaternion q) {
  float size = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  geometry_msgs::Quaternion out;
  out.w = q.w / size;
  out.x = -q.x / size;
  out.y = -q.y / size;
  out.z = -q.z / size;
  return out;
}

geometry_msgs::Pose invert(geometry_msgs::Pose origin) {
  geometry_msgs::Pose out;
  out.position = rotate(invert(origin.position), invert(origin.orientation));
  out.orientation = invert(origin.orientation);
  return out;
}

geometry_msgs::Pose relay(geometry_msgs::Pose origin, geometry_msgs::Pose tip) {
  geometry_msgs::Pose out;
  out.position = add(origin.position, rotate(tip.position, origin.orientation));
  out.orientation = mul(origin.orientation, tip.orientation);
  return out;
}

// print func
std::string toString(EulerRad eluer) {
  char buf[25]="";
  sprintf(buf, "%+6.2f %+6.2f %+6.2f", eluer.roll, eluer.pitch, eluer.yaw);
  return std::string(buf);
}

std::string toString(geometry_msgs::Point p) {
  char buf[30]="";
  sprintf(buf, "%+7.2f %+7.2f %+7.2f", p.x, p.y, p.z);
  return std::string(buf);
}

std::string toString(geometry_msgs::Pose p) {
  return "(" + toString(p.position) + ") (" + toString(getRpy(p.orientation)) + ")";
}
