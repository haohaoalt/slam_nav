//
// Created by tony on 2022/9/22.
//

#include "akm_bridge.h"

int main (int argc, char** argv) {
  ros::init(argc, argv, akm::simu::NODE_NAME);
  akm::simu::AkmBridge _bridge_node;
  _bridge_node.init();

  ros::spin();
  return 0;
}
