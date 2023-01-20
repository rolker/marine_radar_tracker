#ifndef MARINE_RADAR_TRACKER_TARGET_H
#define MARINE_RADAR_TRACKER_TARGET_H

#include <grid_map_ros/grid_map_ros.hpp>

namespace marine_radar_tracker
{

struct BoundingBox
{
  grid_map::Position min;
  grid_map::Position max;

  BoundingBox(const grid_map::Position &p):min(p),max(p)
  {
  }

  BoundingBox(const BoundingBox &other):min(other.min),max(other.max)
  {

  }

  void expand(const grid_map::Position &p)
  {
    min.x() = std::min(min.x(), p.x());
    min.y() = std::min(min.y(), p.y());
    max.x() = std::max(max.x(), p.x());
    max.y() = std::max(max.y(), p.y());
  }

  void expand(const BoundingBox &bb)
  {
    expand(bb.min);
    expand(bb.max);
  }
};

// sector blobs that connect into a blob
struct Blob
{
  std::vector<grid_map::Position> points;
  ros::Time stamp;
};

struct Target
{
  std::map<ros::Time, std::vector<std::shared_ptr<Blob> > > blobs;
  uint16_t id;

  Target(std::shared_ptr<Blob> blob)
  {
    blobs[blob->stamp].push_back(blob);
  }

  // clears older blobs and returns true if blobs remaining
  bool update(ros::Time current_time)
  {
    // detect restart of sim or data replay
    if(!blobs.empty() && blobs.begin()->first > current_time)
      blobs.clear();

    auto expired = current_time - ros::Duration(10.0);
    while(!blobs.empty() && blobs.begin()->first < expired)
      blobs.erase(blobs.begin());
      
    return !blobs.empty();
  }

};

} // namespace marine_radar_tracker

#endif
