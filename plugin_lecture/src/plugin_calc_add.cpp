#include <pluginlib/class_list_macros.h>
#include <ros_lecture_base/adv_calc_base.h>

namespace plugin_lecture
{
class add : public ros_lecture_base::calc_base
{
public:
  int op(int a, int b)
  {
    return a + b;
  }
};
}  // namespace plugin_lecture
PLUGINLIB_EXPORT_CLASS(plugin_lecture::add, ros_lecture_base::calc_base)
