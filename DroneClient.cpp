#include <netdb.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <map>
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <fstream>
#include <sstream>
using namespace std;

//Program Constants
#define	AT_PORT	5556

//Enums for command type
enum CommandType { AT_REF, AT_PCMD, AT_FTRIM };
// Value definitions for string values
enum CommandName {
	NOT_DEFINED,
	TAKEOFF,
	LAND,
	RESET,
	YAW,
	PITCH,
	ROLL,
	GAZ,
	WAIT
};

// Initial Drone values
float roll = 0, pitch = 0, gaz = 0, yaw = 0;
int seq = 1, secWait = 0;

// Networking vars
int socketId, serverPort, bufferSize, size;
sockaddr_in serverAddr, clientAddr;
char buffer[200];

//Map for command to code mappings
static map<string, int> at_ref_ints;		//AT*REF (takeoff, land, reset)
static map<string, CommandName> cmdNames;	//Mappings for command names

// Initialization Method
void *send_command(void *ptr);
void issueCommand(string cmdName, string val);
void runScript(string name);
void orientDrone(CommandName cmd, float val);
static void setupMaps();

int main( int argc, char *argv[] ) {
	
	// Initialize the string map
	setupMaps();
	
	// Initialize variables
	string szInput = "", hostName = "localhost";
	socketId = socket(AF_INET,SOCK_DGRAM,0);
	serverPort = AT_PORT;
	pthread_t command_thread;

	sockaddr &serverAddrCast = (sockaddr &)serverAddr;

	//specify server address, port
	serverAddr.sin_family=AF_INET;
	serverAddr.sin_port=htons(serverPort);
	struct hostent*hp=gethostbyname(hostName.c_str());
	
	// Immediately reset the trim of the drone
	bufferSize = sprintf(buffer, "AT*FTRIM=%d,\r", seq);
	sendto(socketId, &buffer, bufferSize, 0, &serverAddrCast, size);
	seq++;
	bufferSize = 0;

	// Start a thread that sends a command every 3 seconds
	pthread_create(&command_thread, NULL, send_command, NULL);
	
	runScript(argv[1]);
	
	//Close the connection
	close(socketId);
	
	return 0;
}

// Thread method that sends commands every 100ms or so
void *send_command( void * ptr) {
	sockaddr &serverAddrCast = (sockaddr &)serverAddr;
	// Sends a command every 1.5 seconds (right now it just sets yaz, pitch, etc as they are)
	while(1) {
		bufferSize = sprintf(buffer, "AT*PCMD=%d,%d,%d,%d,%d,%d\r",
								seq,
								1,
								*(int*)(&roll),
								*(int*)(&pitch),
								*(int*)(&gaz),
								*(int*)(&yaw)	);
		sendto(socketId, &buffer, bufferSize, 0, &serverAddrCast, size);
		seq++;
		bufferSize = sprintf(buffer, "AT*COMWDG=%d\r", seq);
		sendto(socketId, &buffer, bufferSize, 0, &serverAddrCast, size);
		seq++;
		usleep(100000);
	}
}

void issueCommand(string cmdName, string val) {
	switch(cmdNames[cmdName]) {
		//These first 3 are only 1 argument (the command name) and are all AT*REF commands
		case RESET:
			roll = 0;
			pitch = 0;
			gaz = 0;
			yaw = 0;
		case LAND:
		case TAKEOFF:
			bufferSize = sprintf(buffer, "AT*REF=%d,%d\r", seq, at_ref_ints[cmdName]);
			break;
		//The wait command takes an additional argument (wait time)
		case WAIT:
			secWait = strtol(val.c_str(), NULL, 10);
			char sleepCommand[50];
			// Uses /bin/sleep system command because there were problems
			// with the C++ sleep() command
			sprintf(sleepCommand, "/bin/sleep %d", secWait);
			int result;
			result = system(sleepCommand);
			break;
		//The next 4 commands control drone movement
		case YAW:
		case PITCH:
		case ROLL:
		case GAZ: 
			orientDrone(cmdNames[cmdName], strtod(val.c_str(), NULL));
			break;
		default:
			bufferSize = 0;
			cout << "Error in input file. Command " << cmdName << " not found." << endl;
			break;
	}
}

void runScript(string name) {
	string line;
	ifstream script_file (name.c_str());
	// Work through the script line by line
	if(script_file.is_open()) {
		while(script_file.good()) {
			getline(script_file, line);
			cout << endl << "Read line: " << line << endl;
			if(line != "") {
				string sub1, sub2;
				int numSubs = 0;
				istringstream iss(line);
				do {
					if(numSubs == 0)
						iss >> sub1;
					if(numSubs == 1)
						iss >> sub2;
					numSubs++;
				} while(iss && numSubs < 2);

				if(numSubs == 2) {
					issueCommand(sub1, sub2);
				} else {
					issueCommand(line, "");
				}
				
				//Now that the command has been handled, send it to the drone
				if(bufferSize != 0) {
					sockaddr &serverAddrCast = (sockaddr &)serverAddr;
					cout << "Sending command to localhost:" << serverPort << " ";
					for(unsigned int i = 0; i < sizeof(buffer); i++) {
						cout << buffer[i];
					}
					cout << endl;

					sendto(socketId, &buffer, bufferSize, 0, &serverAddrCast, size);
				}
				seq++;
				bufferSize = 0;
			}
		}
		script_file.close();
	} else {
		cout << "Unable to open file";
	}
}

void orientDrone(CommandName cmd, float val) {
	//only value that changes is the one passed in
	// all other values remain the same
	switch(cmd) {
		case YAW:
			yaw = val;
			break;
		case PITCH:
			pitch = val;
			break;
		case ROLL:
			roll = val;
			break;
		case GAZ:
			gaz = val;
			break;
		default:
			cout << "Invalid movement command." << endl;
			break;
	}
	bufferSize = sprintf(buffer, "AT*PCMD=%d,%d,%d,%d,%d,%d\r",
							seq,
							1,
							*(int*)(&roll),
							*(int*)(&pitch),
							*(int*)(&gaz),
							*(int*)(&yaw)	);
}

void setupMaps() {
	
	//AT REF Maps
    at_ref_ints["takeoff"]  = 290718208;	//10001010101000000001000000000
    at_ref_ints["land"]     = 290717696;	//10001010101000000000000000000
    at_ref_ints["reset"]    = 290717952;	//10001010101000000000100000000	
    
    //Command name mappings
    cmdNames["takeoff"] = TAKEOFF;
    cmdNames["land"] 	= LAND;
    cmdNames["yaw"] 	= YAW;
    cmdNames["pitch"] 	= PITCH;
    cmdNames["roll"] 	= ROLL;
    cmdNames["gaz"] 	= GAZ;
    cmdNames["reset"] 	= RESET;
    cmdNames["wait"] 	= WAIT;
    
}
