#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define UNUSED __attribute__((unused))

#include <servo.h>

void display_menu(){
	printf("Welcome to servo controller\n"); 
	printf("Commands\n");
	printf("\tenable <ch>\n");
	printf("\tdisable <ch>\n");
	printf("\tset <ch> <angle>\n");
	printf("\texit\n");
	printf("\n>");
}

int main(UNUSED int argc, UNUSED char const *argv[]){
	char input[100];	
	int angle, channel;
	char command[100];
	while(1) {
		display_menu();
		if(read(0, input, 100) == -1){
			printf("Error reading the input. Try again\n");
		}
		sscanf(input, "%s %d %d", command, &channel, &angle);
		if(strcmp(command, "enable") == 0) {
			if(channel != 1 && channel != 2) {
				printf("Bad channel number. Enter again\n");
				continue;
			}
			if(servo_enable(channel, 1) == 0)
				printf("Enabling the servo at channel %d\n", channel);
		} else if(strcmp(command, "disable") == 0) {
			if(channel != 1 && channel != 2) {
				printf("Bad channel number. Enter again\n");
				continue;
			}
			if(servo_enable(channel, 0) == 0)
				printf("Disabling the servo at channel %d\n", channel);
		} else if(strcmp(command, "set") == 0) {
			if(channel != 1 && channel != 2) {
				printf("Bad channel number. Exiting with error\n");
				return -1;
			}
			if(angle < 0 || angle > 180) {
				printf("Bad angle input. Enter again\n");
				continue;
			}
			if (servo_set(channel, angle) == 0)
				printf("Setting the servo at channel %d at angle %d\n", channel, angle);
		} else if(strcmp(command, "exit") == 0) {
			// simply break and exit out of main
			break;
		} else {
			printf("Bad command. Enter again\n");
		}
	}
	return 0;
}
