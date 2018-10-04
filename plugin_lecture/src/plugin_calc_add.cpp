#include <pluginlib/class_list_macros.h>
//#include "../../adv_lecture/include/adv_lecture/adv_calc_base.h"
#include "adv_lecture/adv_calc_base.h"

class add : public calc_base
{
  public:
    int op(int a, int b)
    {
      return a+b;
    }
};
PLUGINLIB_EXPORT_CLASS(add, calc_base)
