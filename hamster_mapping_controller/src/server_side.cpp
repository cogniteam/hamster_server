#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <stdio.h>      /* printf */
#include <stdlib.h>     /* system, NULL, EXIT_FAILURE */
#include <boost/thread.hpp>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

using namespace std;
#define MAP_SAVER_COMMAND_TOPIC "/map_saver_command"
#define MAP_SERVER_KILL_COMMAND_TOPIC "/map_server_kill_command"
#define MAP_SERVER_RUN_COMMAND_TOPIC "/map_server_run_command"
#define MAP_OUTDOOR_SERVER_RUN_COMMAND_TOPIC "/map_server_outdoor_run_command"

//params
string map_file_path, save_map_launch_package, save_map_launch_file;
string robot_position_topic; //the position estimation topic
string robot_initial_position_topic;
//commands
string saveMapCommand;
string killMapServerCommand, startMapServerInBackgroundCommand, startMapOutdoorServerInBackgroundCommand;

//data
geometry_msgs::Pose lastPose;
ros::Publisher initialPositionPublisher;

tf::TransformListener* tfListener;

/**
 * Runs the process that saves the map after the mapping is done.
 */
void saveMap() {
	//The process should be "short", so don't run on another thread.
	ROS_INFO("running map saving process");
	int i = system(saveMapCommand.c_str());
	cout << "map saving process finished with code : " << i << endl;
}

void startMapServer() {
	//The process should be "short", so don't run on another thread.
	ROS_INFO("restarting map server in background");
	system(startMapServerInBackgroundCommand.c_str());
}

void startMapOutdoorServer() {
	//The process should be "short", so don't run on another thread.
	ROS_INFO("restarting map outdoor server in background");
	system(startMapOutdoorServerInBackgroundCommand.c_str());
}

void publishPositionFromFile(){
	ROS_INFO("reading position file before publishing");
	string filename = map_file_path + "/indoor.pstn";
	ifstream positionFile;
	positionFile.open(filename.c_str());

	if(positionFile.fail()){
		ROS_INFO("position file doesn't exist");
		return;
	}

	// geometry_msgs::Pose position;
	// positionFile>>position.position.x;
	// positionFile>>position.position.y;
	// positionFile>>position.orientation.w;
	// positionFile>>position.orientation.x;
	// positionFile>>position.orientation.y;
	// positionFile>>position.orientation.z;

	geometry_msgs::Pose position;
	position.position.x = 0;
	position.position.y = 0;
	position.orientation.w = 1;
	position.orientation.x = 0;
	position.orientation.y = 0;
	position.orientation.z = 0;


	geometry_msgs::PoseWithCovarianceStamped msg;
	msg.pose.pose = position;
	msg.header.frame_id = "map";
	double cov[] = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};
	for(int i=0; i<msg.pose.covariance.size(); ++i)
		msg.pose.covariance.at(i) = cov[i];
	msg.header.stamp = ros::Time::now();
	initialPositionPublisher.publish(msg);
	ROS_INFO("published position ");
}

void killMapServer() {
	//The process should be "short", so don't run on another thread.
	ROS_INFO("killing map server");
	system(killMapServerCommand.c_str());
}

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
	time_t now = time(0);
	struct tm tstruct;
	char buf[120];
	tstruct = *localtime(&now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

	return buf;
}

/**
 * Renames the map file (given in arguments) to "map_<date>.backup".
 * Does the equivalent for the yaml file and position file.
 */
void renameCurrentFiles() {
	//rename the files according to current time.
	std::stringstream fmtPgm;
	string currentTime = currentDateTime();
	//move pgm file
	fmtPgm << "mv " << map_file_path << "/indoor.pgm " << map_file_path << "/indoor_" << currentTime << ".backup.pgm";
	string renamePgmCommand = fmtPgm.str();
	ROS_INFO("running pgm renaming process process");
	int i = system(renamePgmCommand.c_str());
	ROS_INFO("renaming pgm renaming process finished with code : %d", i);

	//move yaml file
	std::stringstream fmtYaml;
	fmtYaml << "mv " << map_file_path << "/indoor.yaml " << map_file_path << "/indoor_" << currentTime << ".backup.yaml";
	string renameYamlCommand = fmtYaml.str();
	ROS_INFO("running yaml renaming process process");
	i = system(renameYamlCommand.c_str());
	ROS_INFO("renaming yaml process finished with code : %d",i);

	//rename location file.
	string renamePosCommand ="mv " + map_file_path + "/indoor.pstn " + map_file_path + "/indoor_" + currentTime + ".backup.pstn";
	ROS_INFO("running pstn renaming process process");
	i = system(renamePosCommand.c_str());
	ROS_INFO("renaming pstn process finished with code : %d",i);
}

/**
 * Saves the current location to a file, so localization will start with these locations.
 */
void saveCurrentPositionFile(){
	//open
	string filename = map_file_path + "indoor.pstn";
	ofstream positionFile;
	positionFile.open(filename.c_str());

	//save
	positionFile << lastPose.position.x << " " <<
					lastPose.position.y << " " <<
					lastPose.orientation.w << " " <<
					lastPose.orientation.x << " " <<
					lastPose.orientation.y << " " <<
					lastPose.orientation.z << "\n";

	positionFile.close();
}

bool onMappingSaveRequest(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
	renameCurrentFiles();
	saveMap();
	saveCurrentPositionFile();
	return true;
}


bool onMapServerKillRequest(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
	killMapServer();
	return true;
}

bool onMapOutdoorServerRunRequest(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
	startMapOutdoorServer();
	publishPositionFromFile();
	return true;
}

bool onMapServerRunRequest(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
	startMapServer();
	publishPositionFromFile();
	return true;
}

void onRobotPosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	lastPose = msg->pose.pose;
}

void readParameters(ros::NodeHandle pnh){
	pnh.param("save_map_launch_package", save_map_launch_package , string("mapping_controller"));
	pnh.param("save_map_launch_file", save_map_launch_file , string("save_map"));
	pnh.param("map_file_path", map_file_path , string("/home/pi"));
	pnh.param("robot_position_topic", robot_position_topic, string("/agent1/amcl_pose"));
	pnh.param("robot_initial_position_topic", robot_initial_position_topic, string("/agent1/initialpose"));
}

void constructCommands(){
	std::stringstream fmt;
	fmt << "roslaunch " << save_map_launch_package << " " << save_map_launch_file << ".launch map_path:=" << map_file_path << "/indoor";
	saveMapCommand = fmt.str();

	killMapServerCommand = "rosnode kill /map_server";
	startMapServerInBackgroundCommand = "rosrun map_server map_server " + map_file_path + "/indoor.yaml __name:=map_server &";
	startMapOutdoorServerInBackgroundCommand = "rosrun map_server map_server " + map_file_path + "/outdoor.yaml __name:=map_server &";
}

int main(int a, char** aa) {
	ros::init(a, aa, "mapping_controller_server_side");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	readParameters(pnh);
	constructCommands();

	ros::ServiceServer mapSavingService = nh.advertiseService(MAP_SAVER_COMMAND_TOPIC, onMappingSaveRequest);
	ros::ServiceServer mapServerKillService = nh.advertiseService(MAP_SERVER_KILL_COMMAND_TOPIC, onMapServerKillRequest);
	ros::ServiceServer mapServerRunService = nh.advertiseService(MAP_SERVER_RUN_COMMAND_TOPIC, onMapServerRunRequest);
	ros::ServiceServer mapOutdoorServerRunService = nh.advertiseService(MAP_OUTDOOR_SERVER_RUN_COMMAND_TOPIC, onMapOutdoorServerRunRequest);

	ros::Subscriber robotPositionSubsriber = nh.subscribe(robot_position_topic, 10, onRobotPosition);
	initialPositionPublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(robot_initial_position_topic, 10 ,true);

	//publish the position from file, if exists from previous runs
	publishPositionFromFile();

	ros::spin();

	return 0;
}
