//
// Created by tony on 2022/10/19.
//

#include <climits>
#include <pluginlib/class_list_macros.h>

#include "costmap_2d/keep_out_layer.h"

PLUGINLIB_EXPORT_CLASS(costmap_2d::KeepOutLayer, costmap_2d::Layer)

namespace costmap_2d {
void KeepOutLayer::onInitialize() {
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  nh.param("enabled", enabled_, false);
  nh.param("fill_zones", fill_zones_, true);
  int inflation_option_temp;
  nh.param("inflation_option", inflation_option_temp, static_cast<int>(costmap_2d::AKM_OPTION_NORM));
  inflation_option_ = static_cast<uint8_t>(inflation_option_temp);

  map_received_ = false;
  rolling_window_ = layered_costmap_->isRolling();
  setDefaultValue(toOri(AKM_COST_NO_INFORMATION, inflation_option_));
  zone_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("/record_zone", 1);
  map_sub_ = nh.subscribe("/map", 1, &KeepOutLayer::incomingMap, this);
  point_sub_ = nh.subscribe("/clicked_point", 1, &KeepOutLayer::incomingPoint, this);

  keep_out_zone_srv_ = nh.advertiseService("akm_zone", &KeepOutLayer::keepOutZoneSrv, this);
}

void KeepOutLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map) {
  map_received_ = false;
  while (updating_) {
    ROS_WARN_THROTTLE(1.0, "[KeepOutLayer] incomingMap while updatecost, wait...");
  }

  resizeMap(new_map->info.width, new_map->info.height, new_map->info.resolution,
            new_map->info.origin.position.x, new_map->info.origin.position.y);
  ROS_INFO("[KeepOutLayer] Received a %d X %d map at %f m/pix origin (%.2f, %.2f)",
           size_x_, size_y_, resolution_, origin_x_, origin_y_);
  setAllZonesCost();
  map_received_ = true;
}

void KeepOutLayer::incomingPoint(const geometry_msgs::PointStampedConstPtr& point) {
  if (!recording_) return;

  record_zone_.emplace_back(*point);
  geometry_msgs::PolygonStamped polygon;
  polygon.header.frame_id = "map";
  polygon.header.stamp = ros::Time::now();
  geometry_msgs::Point32 point32;
  for (auto const& p : record_zone_) {
    point32.x = p.point.x;
    point32.y = p.point.y;
    point32.z = 0;
    polygon.polygon.points.emplace_back(point32);
  }
  zone_pub_.publish(polygon);
}

void KeepOutLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                double* min_x, double* min_y, double* max_x, double* max_y) {
  // never update bounds of keep out layer
  if (!map_received_ || !enabled_) return;
  useExtraBounds(min_x, min_y, max_x, max_y);
}

void KeepOutLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
  if (!map_received_ || !enabled_) return;

  updating_ = true;
  if (!rolling_window_) {
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    updating_ = false;
    return;
  }

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();
  geometry_msgs::PointStamped p;
  // i j master_it master_grid: local
  // mx my it costmap_: global
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy); // local frame
      p.header.frame_id = layered_costmap_->getGlobalFrameID();
      p.point.x = wx;
      p.point.y = wy;
      try {
        tf_->transform(p, p, "map");
      } catch (tf2::TransformException& e) {
        ROS_WARN("Keep out layer local transform error: %s", e.what());
        return;
      }
      wx = p.point.x;
      wy = p.point.y;
      unsigned int mx, my;
      if (!worldToMap(wx, wy, mx, my)) continue; // global frame

      auto it = my * size_x_ + mx;
      auto grid_value = toAKMgrid(costmap_[it]);
      if (grid_value.cost == AKM_COST_NO_INFORMATION) continue;

      auto master_it = j * span + i;
      auto old_grid_value = toAKMgrid(master_array[master_it]);

      if (old_grid_value.cost == AKM_COST_NO_INFORMATION || old_grid_value.cost < grid_value.cost) {
        setAKMcost(master_array[master_it], grid_value.cost);
      }
      if (grid_value.cost == AKM_COST_LETHAL_OBSTACLE && old_grid_value.option < grid_value.option) {
        setAKMoption(master_array[master_it], grid_value.option);
      }
    }
  }

  updating_ = false;
}

void KeepOutLayer::reset() {
  if (map_received_) {
    resetMaps();
    setAllZonesCost();
  }
}

void KeepOutLayer::matchSize() {
  if (!rolling_window_) {
    Costmap2D* master = layered_costmap_->getCostmap();
    if (master->getSizeInCellsX() != size_x_ || master->getSizeInCellsY() != size_y_) {
      ROS_ERROR("[KeepOutLayer] matchSize !rolling_window_ diff size, need wait new map.");
      map_received_ = false;
    }
  }
}

inline bool KeepOutLayer::findAvailableId(uint32_t& id, ZoneConstIter& iter) const {
  if (keep_out_zones_.empty()) {
    id = 0;
    return true;
  }

  id = keep_out_zones_.back().id + 1;
  iter = id_search_start_iter_;
  if (0 == id) {
    iter = keep_out_zones_.begin();
  } else if (iter == keep_out_zones_.end()) {
    return true;
  } else {
    id = (*iter).id + 1;
    ++iter;
  }

  while (iter != keep_out_zones_.end()) {
    if (id < (*iter).id) {
      return true;
    }

    id = (*iter).id + 1;
    ++iter;
  }

  return (0 != id);
}

bool KeepOutLayer::addZone(const KeepOutLayer::PointVector& edges, uint32_t& id, uint8_t cost, uint8_t option) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  bool result = true;
  if (keep_out_zones_.empty()) {
    keep_out_zones_.emplace_back(0, toAKMgrid(cost, option), edges);
    id_search_start_iter_ = keep_out_zones_.end();
    id = 0;
  } else {
    ZoneConstIter iter;
    result = findAvailableId(id, iter);
    if (result) {
      auto inserted_iter = keep_out_zones_.emplace(iter, id, toAKMgrid(cost, option), edges);
      if (iter != keep_out_zones_.end()) {
        id_search_start_iter_ = inserted_iter;
      } else {
        id_search_start_iter_ = keep_out_zones_.end();
      }
    }
  }

  if (map_received_) {
    setZoneValue(costmap_, edges, toAKMgrid(cost, option), fill_zones_);
    for (auto const& e : edges) {
      addExtraBounds(e.x, e.y, e.x, e.y);
    }
  }
  return result;
}

void KeepOutLayer::setZoneValue(unsigned char* grid, const PointVector& zone, const akm_grid_t& value, bool fill_zone) {
  std::vector<PointInt> map_zone(zone.size());
  PointInt loc{0, 0};
  for (unsigned int i = 0; i < zone.size(); ++i) {
    worldToMapNoBounds(zone[i].x, zone[i].y, loc.x, loc.y);
    map_zone[i] = loc;
  }

  std::vector<PointInt> zone_cells;

  // get the cells that fill the zone
  rasterizeZone(map_zone, zone_cells, fill_zone);

  // set the cost of those cells
  for (auto const& p : zone_cells) {
    // check if point is outside bounds
    if (p.x < 0 || p.x >= size_x_) continue;
    if (p.y < 0 || p.y >= size_y_) continue;

    auto index = p.x + size_x_ * p.y;
    auto old_grid_value = toAKMgrid(grid[index]);
    if (old_grid_value.cost == AKM_COST_NO_INFORMATION || old_grid_value.cost < value.cost) {
      setAKMcost(grid[index], value.cost);
    }
    if (value.cost == AKM_COST_LETHAL_OBSTACLE && old_grid_value.option < value.option) {
      setAKMoption(grid[index], value.option);
    }
  }
}

void KeepOutLayer::zoneOutlineCells(const std::vector<PointInt>& zone,
                                    std::vector<PointInt>& zone_cells) {
  for (unsigned int i = 0; i < zone.size() - 1; ++i) {
    raytrace(zone[i].x, zone[i].y, zone[i + 1].x, zone[i + 1].y, zone_cells);
  }
  if (!zone.empty()) {
    auto last_index = static_cast<unsigned int>(zone.size() - 1);
    raytrace(zone[last_index].x, zone[last_index].y, zone[0].x, zone[0].y, zone_cells);
  }
}

void KeepOutLayer::raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt>& cells) {
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  PointInt pt{x0, y0};
  int n = 1 + dx + dy;
  int x_inc = (x1 > x0) ? 1 : -1;
  int y_inc = (y1 > y0) ? 1 : -1;
  int error = dx - dy;
  dx *= 2;
  dy *= 2;

  for (; n > 0; --n) {
    cells.emplace_back(pt);

    if (error > 0) {
      pt.x += x_inc;
      error -= dy;
    } else {
      pt.y += y_inc;
      error += dx;
    }
  }
}

void KeepOutLayer::rasterizeZone(const std::vector<PointInt>& zone,
                                 std::vector<PointInt>& zone_cells, bool fill) {
  // we need a minimum zone of a traingle
  if (zone.size() < 3) return;

  // first get the cells that make up the outline of the zone
  zoneOutlineCells(zone, zone_cells);

  if (!fill) return;

  int max_x = zone.front().x;
  int max_y = zone.front().y;
  int min_x = zone.front().x;
  int min_y = zone.front().y;
  for (int i = 1; i < zone.size(); i++) {
    if (zone.at(i).x > max_x) max_x = zone.at(i).x;
    if (zone.at(i).y > max_y) max_y = zone.at(i).y;
    if (zone.at(i).x < min_x) min_x = zone.at(i).x;
    if (zone.at(i).y < min_y) min_y = zone.at(i).y;
  }

  PointInt pt{};
  for (int i = min_x + 1; i < max_x; i++) {
    for (int j = min_y + 1; j < max_y; j++) {
      pt.x = i;
      pt.y = j;
      if (inZone(zone, pt)) {
        zone_cells.emplace_back(pt);
      }
    }
  }
}

bool KeepOutLayer::keepOutZoneSrv(costmap_2d::keepOutZone::Request& req,
                                  costmap_2d::keepOutZone::Response& res) {
  ROS_INFO("[KeepOutLayer] keep out zone service called!");
  const auto add = [&](bool use_record) {
    if (use_record) req.zone = record_zone_;
    if (req.cost == 0) req.cost = AKM_COST_LETHAL_OBSTACLE;
    auto size = req.zone.size();
    if (size < 2) return false;

    PointVector points;
    points.reserve(4);

    if (size == 2) {
      geometry_msgs::Point point_A;
      geometry_msgs::Point point_B;
      point_A.x = req.zone[0].point.x;
      point_A.y = req.zone[0].point.y;
      point_A.z = 0.0;
      points.emplace_back(point_A);
      point_B.x = req.zone[1].point.x;
      point_B.y = req.zone[1].point.y;
      point_B.z = 0.0;
      points.emplace_back(point_B);

      // calculate the normal vector for AB
      geometry_msgs::Point point_N;
      point_N.x = point_B.y - point_A.y;
      point_N.y = point_A.x - point_B.x;

      // get the absolute value of N to normalize and get
      // it to the length of the costmap resolution
      double abs_N = sqrt(pow(point_N.x, 2) + pow(point_N.y, 2));
      point_N.x = point_N.x / abs_N * resolution_;
      point_N.y = point_N.y / abs_N * resolution_;

      // calculate the new points to get a zone which can be filled
      geometry_msgs::Point point;
      point.x = point_A.x + point_N.x;
      point.y = point_A.y + point_N.y;
      points.emplace_back(point);

      point.x = point_B.x + point_N.x;
      point.y = point_B.y + point_N.y;
      points.emplace_back(point);
    } else {
      geometry_msgs::Point p;
      for (unsigned int i = 0; i < req.zone.size(); i++) {
        p.x = req.zone[i].point.x;
        p.y = req.zone[i].point.y;
        p.z = 0.0;
        points.emplace_back(p);
      }
    }
    return addZone(points, res.id, toAKMcost(req.cost), toAKMoption(req.cost)); // todo 对使用srv的用户要求较高，考虑外部仅保持cost
  };

  switch (req.command) {
    case 0: {
      ROS_INFO("[KeepOutLayer] Add zone!");
      return add(false);
    }
    case 1: {
      ROS_WARN("[KeepOutLayer] Remove zone %d!", res.id);
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (keep_out_zones_.empty()) {
        ROS_WARN("The keep out zone list is empty and remove none!");
        return true;
      }
      auto iter = keep_out_zones_.begin();
      while (iter != keep_out_zones_.end()) {
        if (req.id == (*iter).id) {
          if (id_search_start_iter_ == keep_out_zones_.begin()) {
            keep_out_zones_.pop_front();
            id_search_start_iter_ = keep_out_zones_.begin();
            reset();
            return true;
          }
          // If the id_search_start_iter_ will be deleted, move it to the front
          if (id_search_start_iter_ == iter) {
            --id_search_start_iter_;
          }
          keep_out_zones_.erase(iter);
          reset();
          return true;
        }
        // In ascending list, id < (*iter).id means there is no matched id in the list
        if (req.id < (*iter).id) {
          ROS_WARN("Cannot find match id in the keep out zone list!");
          return true;
        }
        ++iter;
      }
      ROS_WARN("Cannot find match id in the keep out zone list!");
      return true;
    }
    case 2: {
      ROS_WARN("[KeepOutLayer] Clear zones!");
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!keep_out_zones_.empty()) {
        keep_out_zones_.clear();
      }
      reset();
      return true;
    }
    case 3: {
      ROS_INFO("[KeepOutLayer] Record start!");
      record_zone_.clear();
      recording_ = true;
      return true;
    }
    case 4: {
      ROS_INFO("[KeepOutLayer] Record stop, add zone!");
      auto result = add(true);
      record_zone_.clear();
      recording_ = false;
      return result;
    }
    default:
      return false;
  }
}

inline void KeepOutLayer::setAllZonesCost() {
  for (auto const& zone : keep_out_zones_) {
    setZoneValue(costmap_, zone.edges, zone.grid, fill_zones_);
  }
  addExtraBounds(origin_x_, origin_y_, origin_x_ + resolution_ * size_x_, origin_y_ + resolution_ * size_y_);
}

inline bool KeepOutLayer::inZone(const std::vector<PointInt>& zone, PointInt& point) {
  uint32_t size = zone.size();
  if (size < 3) {
    ROS_ERROR("Got a zone zone with less than 3 points");
    return false;
  }
  // Count how many time a ray start at (p_x, p_y) point to x dir intersects with the zone
  // Even->outside  Odd->inside
  // Robot pose
  double p_x = point.x;
  double p_y = point.y;
  // Counter and other variable
  int counter = 0;
  double xinters;
  // Initial zone point
  double p1_x, p1_y, p2_x, p2_y;
  p1_x = zone.back().x;
  p1_y = zone.back().y;

  for (int i = 0; i < size; i++) {
    p2_x = zone[i].x;
    p2_y = zone[i].y;
    if (p1_y == p2_y) {
      p1_x = p2_x;  // Update p1
      continue;
    }
    if (p_y > std::min(p1_y, p2_y) && p_y <= std::max(p1_y, p2_y) && p_x <= std::max(p1_x, p2_x)) {
      xinters = (p_y - p1_y) * (p2_x - p1_x) / (p2_y - p1_y) + p1_x;
      if (p1_x == p2_x || p_x <= xinters) {
        counter++;
      }
    }
    // update p1
    p1_x = p2_x;
    p1_y = p2_y;
  }
  return counter % 2 != 0;
}

}  // namespace costmap_2d