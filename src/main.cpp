#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/footprint.h>
// STL
#include <vector>
#include <string>
#include <iostream>
#include <time.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
// msgs
#include <weighted_region_layer/LoadWeightedRegionFile.h>
#include <weighted_region_layer/SaveWeightedRegionFile.h>
#include <map_msgs/OccupancyGridUpdate.h>
// Boost 
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp>


/*****************************************************************************/
void WriteToFile(const std::string& filename)
/*****************************************************************************/
{
  unsigned char MAP_10_BY_10_CHAR[] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 200, 200, 200,
  0, 0, 0, 0, 100, 0, 0, 200, 200, 200,
  0, 0, 0, 0, 100, 0, 0, 200, 200, 200,
  70, 70, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 200, 200, 200, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 255, 255, 255,
  0, 0, 0, 0, 0, 0, 0, 255, 255, 255
  };

  std::string name(filename + ".wrl"); //TODO de/serialization
  std::ofstream file{name.c_str()};
  boost::archive::text_oarchive oa{file};

  std::string s( MAP_10_BY_10_CHAR ) ;
  oa << s;
  return;
}

/*****************************************************************************/
void ReadFromFile(const std::string& filename)
/*****************************************************************************/
{

  std::string name(filename + ".wrl");
  std::ifstream ifs(name.c_str());
  boost::archive::text_iarchive ia(ifs);
  std::string costmap_out;
  if (ifs.good())
  {
    ia >> costmap_out;
    unsigned char* costmap_ = reinterpret_cast<unsigned char*>(const_cast<char*>(costmap_out.c_str()));
  }

  return ;

}

int main(int argc, char const *argv[])
{


  WriteToFile("test");
  ReadFromFile("test");



  return 0;
}