#include <stdio.h>

#include <ros/static_assert.h>
#include <json_transport/json_transport.hpp>

int main(int argc, char** argv)
{
  json_transport::json_t data;
  data["bool_field"] = true;
  data["pi"] = 3.14;
  data["list"] = { 1, 0, 2 };

  json_transport::json_t sub;
  sub["item"] = 11;
  data["sub"] = sub;

  std::string str = data.dump();
  std::cout << str << std::endl;

  if (data["bool_field"].is_boolean()){
    std::cout << "bool_field is bool" << std::endl;
  }

  if (data["other"].is_null()){
    std::cout << "other is bool" << std::endl;
  }

  return 0;
}