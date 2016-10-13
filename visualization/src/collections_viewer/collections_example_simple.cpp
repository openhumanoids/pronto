#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/visualization.hpp"
#include "iostream"
int main(int argc, char** argv)
{

  lcm::LCM* lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  vs::object_t obj;
  obj.id = 0;
  obj.x=0;
  obj.y=0;
  obj.z=0;

  obj.qw=1;
  obj.qx=0;
  obj.qy=0;
  obj.qz=0;

  vs::object_t obj2;
  obj2.id = 1;
  obj2.x=0.1;
  obj2.y=0;
  obj2.z=0;

  obj2.qw=1;
  obj2.qx=0;
  obj2.qy=0;
  obj2.qz=0;

  vs::object_collection_t col;
  col.id = 3;
  col.name = "test";
  col.type = 5;
  col.reset = true;
  col.objects.push_back(obj);
  col.objects.push_back(obj2);
  col.nobjects = 2;

  std::cout << "Send Collections example" << std::endl;
  lcm->publish("OBJECT_COLLECTION",&col);
}