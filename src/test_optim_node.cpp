/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017.
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

//#include <teb_local_planner/teb_local_planner_ros.h>
#include <teb_local_planner/homotopy_class_planner.h>
#include <teb_local_planner/optimal_planner.h>

//#include <interactive_markers/interactive_marker_server.h>
//#include <visualization_msgs/Marker.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>


using namespace teb_local_planner; // it is ok here to import everything for testing purposes

// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
PlannerInterfacePtr planner;
//TebVisualizationPtr visual;
std::vector<ObstaclePtr> obst_vector;
ViaPointContainer via_points;
TebConfig config;
//boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg;
//ros::Subscriber custom_obst_sub;
//ros::Subscriber via_points_sub;
//ros::Subscriber clicked_points_sub;
//std::vector<ros::Subscriber> obst_vel_subs;
unsigned int no_fixed_obstacles;

// =========== Function declarations =============
void CB_mainCycle();
//void CB_publishCycle();
//void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level);
//void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);
//void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb);
//void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
//void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg);
//void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg);
//void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr& twist_msg, const unsigned int id);


// =============== Main function =================
int main( int argc, char** argv )
{
  // setup dynamic reconfigure
  
  // setup callback for custom obstacles
  obst_vector.push_back( boost::make_shared<PointObstacle>(-3,1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(6,2) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
//  obst_vector.push_back( boost::make_shared<LineObstacle>(1,1.5,1,-1.5) ); //90 deg
//  obst_vector.push_back( boost::make_shared<LineObstacle>(1,0,-1,0) ); //180 deg
//  obst_vector.push_back( boost::make_shared<PointObstacle>(-1.5,-0.5) );

  // Dynamic obstacles
  Eigen::Vector2d vel (0.1, -0.3);
  obst_vector.at(0)->setCentroidVelocity(vel);
  vel = Eigen::Vector2d(-0.3, -0.2);
  obst_vector.at(1)->setCentroidVelocity(vel);

  /*
  PolygonObstacle* polyobst = new PolygonObstacle;
  polyobst->pushBackVertex(1, -1);
  polyobst->pushBackVertex(0, 1);
  polyobst->pushBackVertex(1, 1);
  polyobst->pushBackVertex(2, 1);
 
  polyobst->finalizePolygon();
  obst_vector.emplace_back(polyobst);
  */
  
  //for (unsigned int i=0; i<obst_vector.size(); ++i)
  //{
  //
  //  //CreateInteractiveMarker(obst_vector.at(i)[0],obst_vector.at(i)[1],i,&marker_server, &CB_obstacle_marker);  
  //  // Add interactive markers for all point obstacles
  //  boost::shared_ptr<PointObstacle> pobst = boost::dynamic_pointer_cast<PointObstacle>(obst_vector.at(i));
  //  if (pobst)
  //  {
  //    CreateInteractiveMarker(pobst->x(),pobst->y(),i, config.map_frame, &marker_server, &CB_obstacle_marker);  
  //  }
  //}
  //marker_server.applyChanges();
  
  // Setup visualization
  //visual = TebVisualizationPtr(new TebVisualization(n, config));
  
  // Setup robot shape model
  teb_local_planner::Point2dContainer robot = { {-0.5f, -0.5f},{-0.5f, 0.5f},{ 0.5f, -0.5f},{0.5f, 0.5f} };
  teb_local_planner::RobotFootprintModelPtr robot_model = boost::make_shared<PolygonRobotFootprint>(robot);
  
  // Setup planner (homotopy class planning or just the local teb planner)
  if (config.hcp.enable_homotopy_class_planning)
    planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, robot_model, &via_points));
  else
    planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, robot_model, &via_points));
  

  no_fixed_obstacles = obst_vector.size();
   
  int num_cylc = 100;
  for (int i = 0; i < num_cylc; ++i)
  {
      CB_mainCycle();
  }

  return 0;
}

// Planning loop
void CB_mainCycle()
{

    auto t = clock();

  planner->plan(PoseSE2(-4,0,0), PoseSE2(4,0,0)); // hardcoded start and goal for testing purposes
  double vx, vy, omega;
  planner->getVelocityCommand(vx, vy, omega, 20);
  std::cout << vx << "  " << vy << "  " << omega << std::endl;
  std::cout << "planner time " << clock() - t << std::endl;
}


//void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
//{
//  // resize such that the vector contains only the fixed obstacles specified inside the main function
//  obst_vector.resize(no_fixed_obstacles);
//  
//  // Add custom obstacles obtained via message (assume that all obstacles coordiantes are specified in the default planning frame)  
//  for (size_t i = 0; i < obst_msg->obstacles.size(); ++i)
//  {
//    if (obst_msg->obstacles.at(i).polygon.points.size() == 1 )
//    {
//      if (obst_msg->obstacles.at(i).radius == 0) 
//      {
//        obst_vector.push_back(ObstaclePtr(new PointObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
//                                                           obst_msg->obstacles.at(i).polygon.points.front().y )));
//      }
//      else
//      {
//        obst_vector.push_back(ObstaclePtr(new CircularObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
//                                                            obst_msg->obstacles.at(i).polygon.points.front().y,
//                                                            obst_msg->obstacles.at(i).radius )));
//      }
//    }
//    else if (obst_msg->obstacles.at(i).polygon.points.empty())
//    {
//      ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
//      continue;
//    }
//    else
//    {
//      PolygonObstacle* polyobst = new PolygonObstacle;
//      for (size_t j=0; j<obst_msg->obstacles.at(i).polygon.points.size(); ++j)
//      {
//        polyobst->pushBackVertex( obst_msg->obstacles.at(i).polygon.points[j].x,
//                                  obst_msg->obstacles.at(i).polygon.points[j].y );
//      }
//      polyobst->finalizePolygon();
//      obst_vector.push_back(ObstaclePtr(polyobst));
//    }
//
//    if(!obst_vector.empty())
//      obst_vector.back()->setCentroidVelocity(obst_msg->obstacles.at(i).velocities, obst_msg->obstacles.at(i).orientation);
//  }
//}


//void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg)
//{
//  // we assume for simplicity that the fixed frame is already the map/planning frame
//  // consider clicked points as via-points
//  via_points.push_back( Eigen::Vector2d(point_msg->point.x, point_msg->point.y) );
//  ROS_INFO_STREAM("Via-point (" << point_msg->point.x << "," << point_msg->point.y << ") added.");
//  if (config.optim.weight_viapoint<=0)
//    ROS_WARN("Note, via-points are deactivated, since 'weight_via_point' <= 0");
//}

//void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg)
//{
//  ROS_INFO_ONCE("Via-points received. This message is printed once.");
//  via_points.clear();
//  for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
//  {
//    via_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
//  }
//}

//void CB_setObstacleVelocity(const Twist * twist_msg, const unsigned int id)
//{
//  if (id >= obst_vector.size())
//  {
//    ROS_WARN("Cannot set velocity: unknown obstacle id.");
//    return;
//  }
//
//  Eigen::Vector2d vel (twist_msg->linear.x(), twist_msg->linear.y());
//  obst_vector.at(id)->setCentroidVelocity(vel);
//}
