#include "robot.h"

#define WIDTH 160
#define HEIGHT 135
#define LEFT_FRONT_MOTOR 0
#define RIGHT_FRONT_MOTOR 1
#define LEFT_BACK_MOTOR 2
#define RIGHT_BACK_MOTOR 3

extern int current_driver;

void turn_off_motors(){
    current_driver = LEFT_FRONT_MOTOR;
    softhiZ();
    current_driver = RIGHT_FRONT_MOTOR;
    softhiZ();
    current_driver = LEFT_BACK_MOTOR;
    softhiZ();
    current_driver = RIGHT_BACK_MOTOR;
    softhiZ();
}

void left_front_motor(int speed){
    current_driver = LEFT_FRONT_MOTOR;
    if(speed == 0){
        return;
    } else if (speed < 0){
        run(FWD, -speed);
    } else {
        run(FWD, speed);
    }
}

void right_front_motor(int speed){
    current_driver = RIGHT_FRONT_MOTOR;
    if(speed == 0){
        return;
    } else if (speed < 0){
        run(FWD, -speed);
    } else {
        run(FWD, speed);
    }
}

void left_back_motor(int speed){
    current_driver = LEFT_BACK_MOTOR;
    if(speed == 0){
        return;
    } else if (speed < 0){
        run(FWD, -speed);
    } else {
        run(FWD, speed);
    }
}

void right_back_motor(int speed){
    current_driver = RIGHT_BACK_MOTOR;
    if(speed == 0){
        return;
    } else if (speed < 0){
        run(FWD, -speed);
    } else {
        run(FWD, speed);
    }
}

void robot_move(double x, double y, double rotation){
    int leftfront = (y + x - rotation*(WIDTH/2.0 + HEIGHT/2.0));
    int rightfront = (y - x + rotation*(WIDTH/2.0 + HEIGHT/2.0));
    int leftback = (y - x - rotation*(WIDTH/2.0 + HEIGHT/2.0));
    int rightback = (y + x + rotation*(WIDTH/2.0 + HEIGHT/2.0));
    left_front_motor(leftfront);
    right_front_motor(rightfront);
    left_front_motor(leftback);
    right_front_motor(rightback);
}

int main( int argc, char *argv[] ) {
   int sockfd, newsockfd, portno, clilen;
   char buffer[256];
   struct sockaddr_in serv_addr, cli_addr;
   int  n;
   
   /* First call to socket() function */
   sockfd = socket(AF_INET, SOCK_STREAM, 0);
   
   if (sockfd < 0) {
      perror("ERROR opening socket");
      exit(1);
   }
   
   /* Initialize socket structure */
   bzero((char *) &serv_addr, sizeof(serv_addr));
   portno = 1100;
   
   serv_addr.sin_family = AF_INET;
   serv_addr.sin_addr.s_addr = INADDR_ANY;
   serv_addr.sin_port = htons(portno);
   
   /* Now bind the host address using bind() call.*/
   if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
      perror("ERROR on binding");
      exit(1);
   }

   init();
      
   /* Now start listening for the clients, here process will
      * go in sleep mode and will wait for the incoming connection
   */
   
   listen(sockfd,5);
   clilen = sizeof(cli_addr);
   printf("IP Address of this machine: %d.%d.%d.%d\n",
      (int)(cli_addr.sin_addr.s_addr&0xFF),
      (int)((cli_addr.sin_addr.s_addr&0xFF00)>>8),
      (int)((cli_addr.sin_addr.s_addr&0xFF0000)>>16),
      (int)((cli_addr.sin_addr.s_addr&0xFF000000)>>24));
   
   while(1){
       /* Accept actual connection from the client */
       turn_off_motors();
       printf("Waiting on socket connection...\n");
       newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);
        
       if (newsockfd < 0) {
          perror("ERROR on accept");
          exit(1);
       }
       
       /* If connection is established then start communicating */
       bzero(buffer,256);
       n = read( newsockfd,buffer,255 );
       while(n != 0){
           bzero(buffer,256);
           n = read( newsockfd,buffer,255 );
           
           if (n < 0) {
              perror("ERROR reading from socket");
              exit(1);
           }
           printf("%d %d %d %d %d\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
       }
   }
   return 0;
}
