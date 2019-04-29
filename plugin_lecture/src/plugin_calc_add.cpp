#include <pluginlib/class_list_macros.h>
#include "adv_lecture/adv_calc_base.h"

namespace plugin_lecture{
class add : public adv_lecture::calc_base{
public:
  int op(int a, int b){
    return a+b;
  }
};
}
PLUGINLIB_EXPORT_CLASS(plugin_lecture::add, adv_lecture::calc_base)
