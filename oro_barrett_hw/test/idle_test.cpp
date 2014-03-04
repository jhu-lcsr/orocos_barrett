/** Copyright (c) 2013, Jonathan Bohren, all rights reserved. 
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository. 
 */

#include <string>
#include <vector>
#include <iterator>

#include <rtt/os/main.h>

#include <ocl/DeploymentComponent.hpp>
#include <ocl/TaskBrowser.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>

int ORO_main(int argc, char** argv) {

  RTT::Logger::log().setStdStream(std::cerr);
  RTT::Logger::log().mayLogStdOut(true);
  RTT::Logger::log().setLogLevel(RTT::Logger::Debug);

  RTT::Logger::In in("Prototype");

  {
    OCL::DeploymentComponent deployer("Deployer");
    deployer.import("oro_barrett_hw");
    deployer.loadComponent("barrett_manager","oro_barrett_hw::BarrettHWManager");

    OCL::TaskBrowser task_browser(&deployer);

    task_browser.loop();
  }

  return 0;
}
