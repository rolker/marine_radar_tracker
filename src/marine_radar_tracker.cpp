#include <ros/ros.h>
#include <marine_sensor_msgs/RadarSector.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <marine_radar_tracker/target.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <tf2/utils.h>

namespace marine_radar_tracker
{

class MarineRadarTracker
{
public:
  MarineRadarTracker():tf_listener_(tf_buffer_),grid_map_({"intensity","latest","latest_age","previous","previous_age","target_id"})
  {
    ros::NodeHandle nh, pnh("~");

    minimum_range_ = pnh.param("minimum_range", minimum_range_);
    detection_threshold_ = pnh.param("detection_threshold", detection_threshold_);
    maximum_cumulative_intensity_ = pnh.param("maximum_cumulative_intensity", maximum_cumulative_intensity_);
    map_frame_ = pnh.param("map_frame", map_frame_);

    grid_map_.setFrameId(map_frame_);
    grid_resolution_factor_ = pnh.param("grid_resolution_factor", grid_resolution_factor_);
    grid_length_factor_ = pnh.param("grid_length_factor", grid_length_factor_);

    radar_subscriber_ = nh.subscribe("radar_data", 100, &MarineRadarTracker::radarSectorCallback, this);

    markers_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("radar_markers", 10);
    grid_map_publisher_ = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  }

private:
  ros::Subscriber radar_subscriber_;
  ros::Publisher markers_publisher_;
  ros::Publisher grid_map_publisher_;
 
  float detection_threshold_ = 0.0;
  float minimum_range_ = 0.0;
  double maximum_cumulative_intensity_ = 0.0;

  uint16_t next_id_ = 0;

  std::list<std::shared_ptr<Target> > targets_;

  std::string map_frame_ = "map";
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  grid_map::GridMap grid_map_;
  float grid_resolution_factor_ = 8.0;
  float grid_length_factor_ = 0.6;

  ros::Time last_time_;
  double last_range_ = 0.0;

  ros::Time last_target_scan_time_;

  void radarSectorCallback(const marine_sensor_msgs::RadarSectorConstPtr &msg)
  {
    if(ros::Time::isSimTime() && msg->header.stamp < last_time_)
    {
      targets_.clear();
      grid_map_.clearAll();
      last_target_scan_time_ = ros::Time();
    }

    if(msg->range_max != last_range_)
    {
      float grid_size = msg->range_max*2.0*grid_length_factor_;
      float resolution = grid_resolution_factor_* (msg->range_max-msg->range_min)/float(msg->intensities.front().echoes.size());
      grid_map_.setGeometry(grid_map::Length(grid_size, grid_size), resolution);
      last_range_ = msg->range_max;
    }

    double dt = std::max(0.0, (msg->header.stamp - last_time_).toSec());

    for(auto target=targets_.begin(); target != targets_.end(); target++)
    {
      if(!(*target)->update(msg->header.stamp))
        target = targets_.erase(target);
    }

    // skip messages from older buggy halo driver
    if(msg->angle_increment > 0)
      return;

    geometry_msgs::TransformStamped transform;
    try{
      transform = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.1));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN_STREAM( ex.what());
      return;
    }

    geometry_msgs::PoseStamped p;
    p.header = msg->header;
    p.pose.orientation.w = 1.0;

    geometry_msgs::PoseStamped radar_in_map_frame;
    tf2::doTransform(p, radar_in_map_frame, transform);

    auto yaw = tf2::getYaw(radar_in_map_frame.pose.orientation);

    grid_map::Position grid_center(radar_in_map_frame.pose.position.x, radar_in_map_frame.pose.position.y);
    grid_map_.move(grid_center);

    for(grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it)
    {
      if(!isnan(grid_map_.at("latest_age",*it)))
      {
        double new_age = grid_map_.at("latest_age",*it) + dt;
        if(new_age > 5.0)
        {
          grid_map_.at("latest_age",*it) = std::nan("");
          grid_map_.at("latest",*it) = std::nan("");
        }
        else
          grid_map_.at("latest_age",*it) = new_age;
      }
      if(!isnan(grid_map_.at("previous_age",*it)))
      {
        double new_age = grid_map_.at("previous_age",*it) + dt;
        if(new_age > 5.0)
        {
          grid_map_.at("previous_age",*it) = std::nan("");
          grid_map_.at("previous",*it) = std::nan("");
        }
        else
          grid_map_.at("previous_age",*it) = new_age;
      }

      grid_map::Position position;
      grid_map_.getPosition(*it, position);
      double dx = position.x() - radar_in_map_frame.pose.position.x;
      double dy = position.y() - radar_in_map_frame.pose.position.y;
      double r = sqrt(dx*dx+dy*dy);
      if(r >= minimum_range_ && r <= msg->range_max && r >= msg->range_min)
      {
        double theta = atan2(dy, dx);
        if (theta < 0.0)
          theta += 2.0*M_PI;
        theta -= yaw;
        if(theta < 0.0)
          theta += 2.0*M_PI;
        if (theta > 2.0*M_PI)
          theta -= 2.0*M_PI;
        int i = (theta-(msg->angle_start-msg->angle_increment/2.0))/msg->angle_increment;
        if(i >= 0 && i < msg->intensities.size())
        {
          int j = (msg->intensities[i].echoes.size()-1)*(r-msg->range_min)/(msg->range_max-msg->range_min);
          float intensity = msg->intensities[i].echoes[j];
          if(!isnan(grid_map_.at("latest_age", *it)) && grid_map_.at("latest_age", *it) > 0.1)
          {
            grid_map_.at("previous", *it) = grid_map_.at("latest", *it);
            grid_map_.at("previous_age", *it) = grid_map_.at("latest_age", *it);
          }
          grid_map_.at("latest", *it) = intensity;
          grid_map_.at("latest_age", *it) = 0.0;
        }
      }
      if(isnan(grid_map_.at("latest", *it)))
        grid_map_.at("intensity", *it) = std::nan("");
      else
        if(isnan(grid_map_.at("previous", *it)))
          grid_map_.at("intensity", *it) = grid_map_.at("latest", *it)*.5;
        else
          grid_map_.at("intensity", *it) = std::min(grid_map_.at("latest", *it), grid_map_.at("previous", *it));
    }

    grid_map_.setTimestamp(msg->header.stamp.toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(grid_map_, {"intensity"}, message);
    grid_map_publisher_.publish(message);

    last_time_ = msg->header.stamp;

    if(last_target_scan_time_.is_zero())
      last_target_scan_time_ = last_time_;

    if((last_time_ - last_target_scan_time_).toSec() < 1.0)
      return;

    grid_map_.clear("target_id");

    std::vector<std::shared_ptr<Blob> > blobs;
    double resolution = grid_map_.getResolution();

    for(grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it)
    {
      if(grid_map_.at("intensity", *it) > 0.0 && isnan(grid_map_.at("target_id", *it)))
      {
        std::list<grid_map::Position> to_check;
        grid_map::Position p;
        auto blob = std::make_shared<Blob>();
        blob->stamp.fromNSec(grid_map_.getTimestamp());
        if(grid_map_.getPosition(*it, p))
          to_check.push_back(p);
        while(!to_check.empty())
        {
          p = to_check.front();
          if(grid_map_.isInside(p))
            if(!isnan(grid_map_.atPosition("intensity", p)))
              if(isnan(grid_map_.atPosition("target_id", p)))
              {
                blob->points.push_back(p);
                grid_map_.atPosition("target_id", p) = 0.0;
                if(grid_map_.atPosition("intensity", p) > 0.0)
                {
                  to_check.push_back(grid_map::Position(p.x()+resolution, p.y()));
                  to_check.push_back(grid_map::Position(p.x()-resolution, p.y()));
                  to_check.push_back(grid_map::Position(p.x(), p.y()+resolution));
                  to_check.push_back(grid_map::Position(p.x(), p.y()-resolution));
                }
              }
          to_check.pop_front();
        }
        if(!blob->points.empty() && blob->points.size() < 50)
          blobs.push_back(blob);
      }
    }

    last_target_scan_time_ = last_time_;

    targets_.clear();

    for(auto b: blobs)
    {
      targets_.push_back(std::make_shared<Target>(b));
      targets_.back()->id = targets_.size();
    }

    visualization_msgs::MarkerArray markers;

    for (auto target: targets_)
    {
      visualization_msgs::Marker marker;
      marker.header.stamp = target->blobs.rbegin()->first;
      marker.header.frame_id = map_frame_;

      auto age = msg->header.stamp - marker.header.stamp;

      marker.ns = "radar_objects";
      marker.id = target->id;
      marker.type = marker.LINE_STRIP;
      marker.pose.orientation.w = 1.0;
      marker.color.a = 1.0;
      marker.lifetime.fromSec(10.0-age.toSec());

      auto score = (10.0-age.toSec())/10.0;

      if(score > 1.0)
        continue;

      marker.color.r = 0.5-score*0.5;
      marker.color.b = 0.5-score*0.5;
      marker.color.g = 0.5+score*0.5;

      marker.scale.x = resolution/10.0;

      geometry_msgs::Point p;

      for(auto bt: target->blobs)
        if(bt.first > msg->header.stamp - ros::Duration(1.0))
          for(auto b: bt.second)
            for(auto bp: b->points)
            {
              p.x = bp.x();
              p.y = bp.y();
              marker.points.push_back(p);
            }

      markers.markers.push_back(marker);
    }
    markers_publisher_.publish(markers);
  }
};

} // namespace marine_radar_tracker

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "marine_radar_tracker");

  marine_radar_tracker::MarineRadarTracker mrt;
    
  ros::spin();
  return 0;
}    
