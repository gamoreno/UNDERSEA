/* A simple server in the internet domain using TCP
   The port number is passed as an argument */
#include <cstdio>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <cstring>
#include <string>

#include "serverLoop.h"

using namespace std;


//store the values returned by the socket system call and the accept system call.
int sockfd, newsockfd;

//store the size of the address of the client
socklen_t clientLen;

//the server reads characters from the socket connection into this buffer.
char buffer[256];

//server and client adresses
struct sockaddr_in serverAddress, clientAddress;



void error(const char *msg)
{
    perror(msg);
    exit(1);
}


void initialiseServer (int portNo)
{
    if (portNo < 2000) {
        fprintf(stderr,"ERROR, no port provided\n");
        exit(1);
    }

    //create a new socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
       error("ERROR opening socket");

    //set all values in a buffer to zero
    bzero((char *) &serverAddress, sizeof(serverAddress));

    //populate serverAddress structure
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = INADDR_ANY;
    serverAddress.sin_port = htons(portNo);

    //bind a socket to an address
    if (bind(sockfd, (struct sockaddr *) &serverAddress, sizeof(serverAddress)) < 0)
   	 error("ERROR on binding");

    //listen on the socket for connections
    listen(sockfd,5);

    clientLen = sizeof(clientAddress);
}


void *runServer (void *dummyPt)
{
    //contains the number of characters read or written
    int n;

	//block the process until a client connects to the server
     newsockfd = accept(sockfd, (struct sockaddr *) &clientAddress, &clientLen);
     if (newsockfd < 0)
    	 error("ERROR on accept");

     do {
		 bzero(buffer,256);
		 printf("Waiting:\n");
		 n = read(newsockfd,buffer,255);
		 if (n < 0)
			 error("ERROR reading from socket");

		 buffer[strlen(buffer)-1]='\0';
		 printf("Message: %s\n",buffer);

		 string inputStr  = buffer;
		 string outputStr = buffer;
		 if (strcmp(buffer,"SENSORS") == 0){
			 outputStr = "SENSOR1=5,SENSOR2=4,SENSOR3=3";
		 }
		 else if (inputStr.find("SPEED") != string::npos){
			 outputStr = "OK";
		 }
		 n = write(newsockfd, outputStr.c_str(), outputStr.length());
		 if (n < 0)
			 error("ERROR writing to socket");

//		 for (int i = 0; i < strlen(buffer); ++i)
//			 printf("%d\t", (int)(buffer[i]));

     }
     while (strcmp(buffer,"!!") != 0);

     close(newsockfd);
     close(sockfd);

     return NULL;
}



void closeServer()
{

}



void *runServer2 (void *m_sensors_map)
{
	//contains the number of characters read or written
	int n;

	//block the process until a client connects to the server
	 newsockfd = accept(sockfd, (struct sockaddr *) &clientAddress, &clientLen);
	 if (newsockfd < 0)
		 error("ERROR on accept");

	 do {
		 bzero(buffer,256);
		 printf("Waiting:\n");
		 n = read(newsockfd,buffer,255);
		 if (n < 0)
			 error("ERROR reading from socket");

		 buffer[strlen(buffer)-1]='\0';
		 printf("Message: %s\n",buffer);

		 string inputStr  = buffer;
		 string outputStr = buffer;
		 if (strcmp(buffer,"SENSORS") == 0){
			outputStr.clear();
			UUV::sensorsMap *sensMap =  (UUV::sensorsMap *) m_sensors_map;
			for (UUV::sensorsMap::iterator it = sensMap->begin();  it != sensMap->end(); it++){
				outputStr += it->first +"="+ doubleToString(it->second.averageRate,2) +",";
				//reset sensors information
				it->second.reset();
			}
		 }
		 else if (inputStr.find("SPEED") != string::npos){
			 outputStr = "OK";
		 }
		 n = write(newsockfd, outputStr.c_str(), outputStr.length());
		 if (n < 0)
			 error("ERROR writing to socket");

//		 for (int i = 0; i < strlen(buffer); ++i)
//			 printf("%d\t", (int)(buffer[i]));

	 }
	 while (strcmp(buffer,"!!") != 0);

	 close(newsockfd);
	 close(sockfd);

	 return NULL;
}
