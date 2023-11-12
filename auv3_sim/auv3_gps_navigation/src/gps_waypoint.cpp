#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <math.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>  // action library used for implementing simple action clients
#include <robot_localization/navsat_conversions.h>

// ------ initialize variables -------
typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> MoveBaseClient; // create a type definition for a client called MoveBaseClient
std::vector <std::pair<double, double>> waypointVect;                                  // global vector that contains the GPS waypoints

// represents a 3D point in space with a timestamp, to represent locations in a global coordinate system
geometry_msgs::PointStamped UTM_point, map_point, UTM_next, map_next;
int count = 0, waypointCount = 0, wait_count = 0;
double numWaypoints = 0;
double latiGoal, longiGoal, latiNext, longiNext;
std::string utm_zone;
std::string path_local, path_abs;

// This function counts the number of waypoints given in the file
// Argument = pkg relative path to waypoints file
// Return Value = int, number of waypoints
int countWaypointsInFile(std::string path_local){
    path_abs = ros::package::getPath("auv3_gps_navigation") + path_local; // calculating absolute path of waypoints file
    std::ifstream fileCount(path_abs.c_str());      // initializing an input file stream object and opening file with specified path
    if(fileCount.is_open()){                        // wrapper to check whether the file is open
        double lati = 0;
        while(!fileCount.eof()){                    // each line of the file is read using the >> operator, 
            fileCount >> lati;                      // which extracts the next value from the input stream and stores it in the lati variable.
            ++count;                                // increments with each value read
        }
        --count;                                    // decrementing EOF, end-of-file
        numWaypoints = count / 2;                   // since each waypoint is a pair, calculating half of waypoints
        ROS_INFO("%.0f GPS waypoints were read", numWaypoints);
        fileCount.close();                          // closing file
    }
    else{
        ROS_ERROR("Unable to open waypoint file");  // raise error incase of unsuccessful open
    }
    return numWaypoints;                            // return number of wapoints read
}

// This function extracts the waypoints (both latitude and logitude) from the waypoint file
// Argument = pkg relative path to waypoints file
// Return Value = std vector pair of type double, all waypoints
std::vector <std::pair<double, double>> getWaypoints(std::string path_local){
    double lati = 0, longi = 0;
    path_abs = ros::package::getPath("auv3_gps_navigation") + path_local;
    std::ifstream fileRead(path_abs.c_str());
    for(int i = 0; i < numWaypoints; i++){
        fileRead >> lati;                                   // read both latitude and longitude from waypoints file
        fileRead >> longi;                                  // and store in a vector of pairs
        waypointVect.push_back(std::make_pair(lati, longi));
    }
    fileRead.close();                                       // closing file

    //Output vector
    ROS_INFO("The following GPS Waypoints have been set:");
    // print all extracted waypoints
    for(auto iterDisp = waypointVect.begin(); iterDisp != waypointVect.end(); iterDisp++){
        ROS_INFO("%.9g %.9g", iterDisp->first, iterDisp->second);
    }
    return waypointVect;    // return vector of pair of waypoints
}

// This function converts the waypoints in lat,long to UTM (Universal Transverse Mercator) coordinate system
// Argument = waypoint, lat, long points
// Return Value = utm converted point
geometry_msgs::PointStamped latLongtoUTM(double lati_input, double longi_input){
    double utm_x = 0, utm_y = 0;
    geometry_msgs::PointStamped UTM_point_output;   // variable to store UTM converted point

    // convert lat/long to utm
    // calling the LLtoUTM function from the NavsatConversions class of the RobotLocalization
    // lati_input and longi_input as the input latitude and longitude values, and utm_y, utm_x, and utm_zone as output variables.
    // UTM easting, UTM northing, and UTM zone.
    RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);

    //Construct and add info to UTM_point
    UTM_point_output.header.frame_id = "utm";
    UTM_point_output.header.stamp = ros::Time(0);
    UTM_point_output.point.x = utm_x;
    UTM_point_output.point.y = utm_y;
    UTM_point_output.point.z = 0;

    return UTM_point_output;    // utm converted point
}

// This function converts the waypoints in UTM (Universal Transverse Mercator) coordinate system to relative odom frame, map point
// Argument = utm point
// Return Value = specified frame transformed point
geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input){
    geometry_msgs::PointStamped map_point_output;       // variable to store frame transformed point
    std::string conversion_frame = "odom";
    bool notDone = true;
    tf::TransformListener listener; //create transformlistener object called listener
    ros::Time time_now = ros::Time::now();
    while(notDone){
        try{
            UTM_point.header.stamp = ros::Time::now();
            listener.waitForTransform(conversion_frame.c_str(), "utm", time_now, ros::Duration(3.0));   // wait for transform
            listener.transformPoint(conversion_frame.c_str(), UTM_input, map_point_output);             // transform point
            notDone = false;
        }
        catch (tf::TransformException& ex){     // to catch error or exception
            ROS_WARN("%s", ex.what());
            ros::Duration(0.01).sleep();
        }
    }
    return map_point_output;        // return frame converted point
}

// This function converts the waypoints in UTM (Universal Transverse Mercator) coordinate system to relative odom frame, map point
// Argument = utm point
// Return Value = specified frame transformed point
move_base_msgs::MoveBaseGoal buildGoal(geometry_msgs::PointStamped map_point, geometry_msgs::PointStamped map_next, bool last_point){
    move_base_msgs::MoveBaseGoal goal;      // variable to store move base goal

    //Specify what frame we want the goal to be published in
    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = ros::Time::now();

    // Specify x and y goal
    goal.target_pose.pose.position.x = map_point.point.x; // specify x goal
    goal.target_pose.pose.position.y = map_point.point.y; // specify y goal

    // Specify heading goal using current goal and next goal (point robot towards its next goal once it has achieved its current goal)
    if(last_point == false){
        tf::Matrix3x3 rot_euler;    // variable to store eular orientation data
        tf::Quaternion rot_quat;    // variable to store quaternion orientation data

        // Calculate quaternion
        float x_curr = map_point.point.x, y_curr = map_point.point.y;   // set current coords.
        float x_next = map_next.point.x, y_next = map_next.point.y;     // set coords. of next waypoint
        float delta_x = x_next - x_curr, delta_y = y_next - y_curr;     // change in coords.
        float yaw_curr = 0, pitch_curr = 0, roll_curr = 0;
        yaw_curr = atan2(delta_y, delta_x);

        // Specify quaternions
        rot_euler.setEulerYPR(yaw_curr, pitch_curr, roll_curr);     // convert  roll, pitch, yaw to eular
        rot_euler.getRotation(rot_quat);                            // get converted orientation values in rot_quat variable

        // set goal orientation
        goal.target_pose.pose.orientation.x = rot_quat.getX();
        goal.target_pose.pose.orientation.y = rot_quat.getY();
        goal.target_pose.pose.orientation.z = rot_quat.getZ();
        goal.target_pose.pose.orientation.w = rot_quat.getW();
    }
    else{
        goal.target_pose.pose.orientation.w = 1.0;  // fixed orientation icase of final goal
    }

    return goal;    // return move base goal
}

int main(int argc, char** argv){
    std::string node_name = "gps_waypoint";
    ros::init(argc, argv, node_name);       //initiate node called gps_waypoint
    ros::NodeHandle n;                      //creating object "n" of class type ros::NodeHandle
    ROS_INFO("Initiated %s node", node_name.c_str());

    MoveBaseClient ac("/move_base", true);
    //construct an action client that we use to communication with the action named move_base.
    //Setting true is telling the constructor to start ros::spin()

    // Initiate publisher to send end of node message
    ros::Publisher pubWaypointNodeEnded = n.advertise<std_msgs::Bool>("waypoint_following_status", 100);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        wait_count++;
        if(wait_count > 3){
            ROS_ERROR("move_base action server did not come up, killing gps_waypoint node...");
            // Notify joy_launch_control that waypoint following is complete
            std_msgs::Bool node_ended;
            node_ended.data = true;
            pubWaypointNodeEnded.publish(node_ended);
            ros::shutdown();
        }
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    //Get Longitude and Latitude goals from text file
    ros::param::get("/coordinates_file", path_local); 
    std::cout<<"File Path: "<<path_local<<"\n";
    numWaypoints = countWaypointsInFile(path_local);    //Count number of waypoints
    waypointVect = getWaypoints(path_local);            //Reading waypoints from text file and output results


    // Iterate through vector of waypoints for setting goals
    for(auto iter = waypointVect.begin(); iter < waypointVect.end(); iter++){
        //Setting goal
        latiGoal = iter->first;
        longiGoal = iter->second;
        bool final_point = false;

        //set next goal point if not at last waypoint
        if(iter < (waypointVect.end() - 1)){
            iter++;
            latiNext = iter->first;
            longiNext = iter->second;
            iter--;
        }
        else{ //set to current
            latiNext = iter->first;
            longiNext = iter->second;
            final_point = true;     // condition that received point is the last and final goal
        }

        ROS_INFO("Received Latitude goal:%.6f", latiGoal);
        ROS_INFO("Received longitude goal:%6f", longiGoal);

        //Convert lat/long to utm:
        UTM_point = latLongtoUTM(latiGoal, longiGoal);
        UTM_next = latLongtoUTM(latiNext, longiNext);

        // Transform UTM to map point in odom frame
        map_point = UTMtoMapPoint(UTM_point);
        map_next = UTMtoMapPoint(UTM_next);

        // Build goal to send to move_base
        move_base_msgs::MoveBaseGoal goal = buildGoal(map_point, map_next, final_point); // initiate a move_base_msg called goal

        ROS_INFO("x:%.6f", goal.target_pose.pose.position.x);
        ROS_INFO("y:%.6f", goal.target_pose.pose.position.y );

        // Send Goal
        ROS_INFO("Sending goal");
        ac.sendGoal(goal); // push goal to move_base node

        // Wait for result
        ac.waitForResult(); // waiting to see if move_base was able to reach goal

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){   // if goal is reached, success
            ROS_INFO("successfully reached the goal!");
            // switch to next waypoint and repeat
        }
        else{                                                               // if goal not reached, failure
            ROS_ERROR("Unable to reach goal. GPS Waypoint unreachable.");
            ROS_INFO("Exiting node...");
            // Notify joy_launch_control that waypoint following is complete
            std_msgs::Bool node_ended;
            node_ended.data = true;
            pubWaypointNodeEnded.publish(node_ended);
            ros::shutdown();
        }
    } // End for loop iterating through waypoint vector

    ROS_INFO("Ending node...");

    // Notify joy_launch_control that waypoint following is complete
    std_msgs::Bool node_ended;
    node_ended.data = true;
    pubWaypointNodeEnded.publish(node_ended);

    ros::shutdown();
    ros::spin();
    return 0;
}