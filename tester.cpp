/*
automator
This is a program based on the automator which you may implement to modify your simulation and run many tests (e.g. for a Monte Carlo simulation). 

Compiled with G++ ("g++ tester.cpp -o tester.out")
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string>
#include <string.h>
#include <sys/wait.h>

//normal time + 30 as buffer time
#define SIMULATION_TIME 150	// TODO make sure this is the same as in the evaluator
#define BUFFER_TIME 5
#define SLEEP_TIME (SIMULATION_TIME+16)
#define OUTPUT_ADDITION "_test\n" //the '\n' is CRITICAL
#define LENGTH_OF_EXTENSION 7 //".course" has 7 characters
#define NUM_COURSES 9 //number of different courses you wish to run this for

/*
isBlankLine(...)
simple function for parsing to determine is a string is a "blank" line
NOTE: Copied from elsewhere
*/
bool isBlankLine(char str[])
{
	for(int i = 0; i < strlen(str); i++)
	{
		switch(str[i])
		{
			case ' ':
			case '\n':
			case '\t':
			{
				//keep checking
				break;
			}
			default:
			{
				//not a blank line character
				return false;
				break;
			}
		}
	}
	
	//we made it here, must be blank
	return true;
}

int main()
{
	//get the filename
	char filename[256];
	printf("Enter the file with all the courses in it:");
	scanf("%s", filename);
	
	
	//try to create our pipe for use later
	int pfds[2];
	if(pipe(pfds) == -1)
	{
		perror("Pipe");
		exit(1);
	}
	
	char buffer[256];
	char buffer_2[256];
	char str_value[50];


	FILE *results, *ripna, *fp, *scorefile;
	results = fopen( "/home/eric/ros_workspace/sim/scores/Results.txt", "w");
	fprintf(results, "Conflicts\tCollisions\tEfficiency\n");

	/* This loops through all the values you specify for your parameter of choice. Each time through
	it runs all of the courses with the given value. */
	for ( double value = 0.5; value <= 2.5; value += 0.2 ) {
		//open the file
		
		fp = fopen(filename, "r");
		
		//check for a good open
		if(fp != NULL)
		{
			sprintf(str_value, "%f\n", value);
			fprintf(results, str_value);
			
			// Change the value of the chosen parameter here
			ripna = fopen("/home/eric/ros_workspace/sim/src/ripna.cpp", "r+");
			fseek (ripna, 366, SEEK_SET );
			fputs (str_value, ripna);
			fclose (ripna);	

			system("rosmake sim");

			//while we have something in the file
			while(fgets(buffer, sizeof(buffer), fp))
			{	
				//check for useless lines
				if(buffer[0] == '#' || isBlankLine(buffer))
				{
					//this line is a comment
					continue;
				}
			
				//construct our strings to send
				std::string myStr = std::string(buffer);
				myStr = myStr.substr(0, myStr.size() - LENGTH_OF_EXTENSION - 1);
				myStr = myStr + OUTPUT_ADDITION;
			
				//fork our process
				int pid;
				pid = fork();

				if(pid == 0)
				{
					//we're redirecting STDIN such that it comes from the pipe
					//close standard in
					close(STDIN_FILENO);
		
					//duplicate our stdin as the pipe output
					dup2(pfds[0], STDIN_FILENO);
				
					//child process
					system("roslaunch sim evaluation.launch");
				}
				else
				{
					//send out output over that there pipe
					printf("Writing to the pipe! %s\n", buffer);
					write(pfds[1], buffer, strlen(buffer));
					printf("Writing to the pipe! %s\n", myStr.c_str());
					write(pfds[1], myStr.c_str(), strlen(myStr.c_str()));
		
					//parent waits some time, then kills before starting new one
					sleep(SLEEP_TIME);
					printf("Killing Process ID #%d\n", pid);
					kill(pid, SIGTERM);
					waitpid(pid, NULL, 0);
				
					//give the SIGTERM time to work
					sleep(BUFFER_TIME);
				}
			}	
		}
	
		else {
			printf("ERROR: Bad file name\n");
		}

		fclose(fp);

		// Writing data to a file
		char buffer_3[256];
		char name[256];
		int j;
		for (int i = 1; i <= NUM_COURSES; i++) {
			// Open ith score file
			sprintf(name, "/home/eric/ros_workspace/sim/scores/final_32_500m_%d_test.score", i);
			scorefile = fopen(name, "r");
			
			while(fgets(buffer_3, sizeof(buffer_3), scorefile)) {
				
				for (j = 0; j < sizeof(buffer_3); j++) {				
					if (buffer_3[j] == '\n') {
						break;
					}
				}
				char output[j-1];
				for (int k = 0; k < j; k++) {
					output[k] = buffer_3[k];
				}
				output[j] = '\0';
				
				// Print data to a different kind of score file
				fprintf(results, "%s\t", output);			
			}
			
			fprintf(results, "\n");
			fclose(scorefile);
		}
		fprintf(results, "\n\n");

	}


	fclose(results);
	
	
	return 0;
}
