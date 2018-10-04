#ifndef PLUGIN_LECTURE_CALC_BASE_H_
#define PLUGIN_LECTURE_CALC_BASE_H_

namespace plugin_namespace
{
  class calc
  {
    public:
      virtual int op(int a, int b)=0;
  };
};
#endif
