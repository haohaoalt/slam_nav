//
// Created by tony on 2022/10/11.
//

#include <csignal>
#include "state_machine.h"

std::shared_ptr<akm::pnc::StateMachine> state_machine_ptr;

void sigintHandler(int sig) {
  if (state_machine_ptr) {
    state_machine_ptr->stop();
    state_machine_ptr.reset();
  }

  ROS_INFO("akm state machine shutting down!");
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, akm::pnc::NODE_NAME);
  state_machine_ptr = std::make_shared<akm::pnc::StateMachine>();
  signal(SIGINT, sigintHandler);
  state_machine_ptr->init();
  state_machine_ptr->run();
  return 0;
}
