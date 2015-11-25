#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <stdio.h>      /* printf */
#include <stdlib.h>     /* system, NULL, EXIT_FAILURE */
#include <boost/thread.hpp>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

using namespace std;
#define MAPPING_COMMAND_TOPIC "/mapping_command"
#define MODE_COMMAND_SLAM "slam"
#define MODE_COMMAND_LOCALIZATION "localization"

#define MAP_SAVER_COMMAND_TOPIC "/map_saver_command"
#define MAP_SERVER_KILL_COMMAND_TOPIC "/map_server_kill_command"
#define MAP_SERVER_RUN_COMMAND_TOPIC "/map_server_run_command"

string mapping_launch_package, mapping_launch_file;
string localization_launch_package, localization_launch_file;
string starting_map_mode;

string mappingLaunchCommand, killMappingLaunchCommand;
string localizationLaunchCommand, killLocalizationLaunchCommand;

enum MODE{
	NONE, SLAM, LOCALIZATION, WAITING
};

std::string str(MODE mode){
	switch(mode){
	case NONE: return "None";
	case SLAM: return "SLAM";
	case LOCALIZATION: return "Localization";
	case WAITING: return "Waiting";
	default: return "Unknown";
	}
}

MODE mode = NONE;

ros::Subscriber mappingCommandSubscriber;
ros::ServiceClient mapSaveCommandPublisher;
ros::ServiceClient mapServerKillCommandPublisher;
ros::ServiceClient mapServerRunCommandPublisher;

boost::thread mappingProcessThread;

/**
 * Launches the process for mapping.
 */
void launchMapping() {
	mode = SLAM;
	int i = system(mappingLaunchCommand.c_str());
	cout << "mapping process finished with code : " << i << endl;
}

/**
 * Kills the mapping process.
 */
void killMapping() {
	mode = WAITING;
	cout << "about to kill mapping " << endl;
	system(killMappingLaunchCommand.c_str());
}


/**
 * Launches the process for mapping.
 */
void launchLocalization() {
	mode = LOCALIZATION;
	int i = system(localizationLaunchCommand.c_str());
	cout << "localization process finished with code : " << i << endl;
}

/**
 * Kills the mapping process.
 */
void killLocalization() {
	mode = WAITING;
	cout << "about to kill localization " << endl;
	system(killLocalizationLaunchCommand.c_str());
}

/**
 * Publishes the command to save the current map (from hector), on the server.
 */
void saveMap(){
	std_srvs::Empty srv;
	if(mapSaveCommandPublisher.call(srv)){
		cout << "SUCCESSFUL save response got from server. "<< endl;
	} else {
		cout << "UNSUCCESSFUL save response got from server. "<< endl;
	}
}

void killMapServer(){
	std_srvs::Empty srv;
	cout << "about to kill map server. "<< endl;
	if(mapServerKillCommandPublisher.call(srv)){
		cout << "SUCCESSFUL map server kill response got from server. "<< endl;
	} else {
		cout << "UNSUCCESSFUL map server kill response got from server. "<< endl;
	}
}

void runMapServer(){
	std_srvs::Empty srv;
	cout << "about to run map server. "<< endl;
	if(mapServerRunCommandPublisher.call(srv)){
		cout << "SUCCESSFUL map server run response got from server. "<< endl;
	} else {
		cout << "UNSUCCESSFUL map server run response got from server. "<< endl;
	}
}

void startMapping() {
	if (mode==SLAM or mode == WAITING) return;
	if (mode==LOCALIZATION){
		killLocalization();
		killMapServer();
	}

	if(mode != NONE) mappingProcessThread.join();
	mappingProcessThread = boost::thread(launchMapping);
}

void startLocalization() {
	if (mode == LOCALIZATION or mode == WAITING) return;
	if (mode == SLAM){
		saveMap();
		killMapping();
		runMapServer();
	}

	if(mode != NONE) mappingProcessThread.join();
	mappingProcessThread = boost::thread(launchLocalization);
}

//callback
void onMappingSavingCommand(const std_msgs::String::ConstPtr& msg) {
	if (msg->data == MODE_COMMAND_SLAM) {
		startMapping();
	}

	if (msg->data == MODE_COMMAND_LOCALIZATION) {
		startLocalization();
	}
}


void finishProcess() {
	while (mode == WAITING)
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	if (mode == LOCALIZATION)
		killLocalization();

	if (mode == SLAM)
		killMapping();

	mappingProcessThread.join();
}

void readParameters(ros::NodeHandle pnh){
	pnh.getParam("mapping_launch_package", mapping_launch_package);
	pnh.getParam("mapping_launch_file", mapping_launch_file);
	pnh.getParam("localization_launch_package", localization_launch_package);
	pnh.getParam("localization_launch_file", localization_launch_file);
	pnh.getParam("starting_map_mode", starting_map_mode);
}

void constructCommands(){
	static const std::string launch_command = "roslaunch ";
	static const std::string kill_command = "pkill -2 -f 'roslaunch.*";

	std::stringstream fmt_mappingLaunchCommand;
	fmt_mappingLaunchCommand << launch_command << mapping_launch_package << " " << mapping_launch_file << ".launch";
	mappingLaunchCommand = fmt_mappingLaunchCommand.str();

	std::stringstream fmt_killMappingLaunchCommand;
	fmt_killMappingLaunchCommand<< kill_command <<mapping_launch_file<<"'";
	killMappingLaunchCommand = fmt_killMappingLaunchCommand.str();

	std::stringstream fmt_localizationLaunchCommand;
	fmt_localizationLaunchCommand << launch_command << localization_launch_package << " " << localization_launch_file << ".launch";
	localizationLaunchCommand = fmt_localizationLaunchCommand.str();

	std::stringstream fmt_killLocalizationLaunchCommand;
	fmt_killLocalizationLaunchCommand << kill_command << localization_launch_file<<"'";
	killLocalizationLaunchCommand = fmt_killLocalizationLaunchCommand.str();
}

void handleStartingMode() {
	if (starting_map_mode == "slam") {
		startMapping();
		mode = SLAM;
	}
	if (starting_map_mode == "localization") {
		startLocalization();
		mode = LOCALIZATION;
	}
}

int main(int a, char** aa) {
	ros::init(a, aa, "mapping_controller_client_side");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	readParameters(pnh);
	constructCommands();
	handleStartingMode();

	mappingCommandSubscriber = nh.subscribe(MAPPING_COMMAND_TOPIC, 1, onMappingSavingCommand);
	mapSaveCommandPublisher = nh.serviceClient<std_srvs::Empty>(MAP_SAVER_COMMAND_TOPIC);
	mapServerKillCommandPublisher= nh.serviceClient<std_srvs::Empty>(MAP_SERVER_KILL_COMMAND_TOPIC);
	mapServerRunCommandPublisher= nh.serviceClient<std_srvs::Empty>(MAP_SERVER_RUN_COMMAND_TOPIC);


	ros::spin();

	finishProcess();
	std::cout << "stopped" << std::endl;
	return 0;
}
