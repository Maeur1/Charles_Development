#include "robot.h"

#define WIDTH 160
#define HEIGHT 135

#define LEFT_FRONT_MOTOR 1
#define RIGHT_FRONT_MOTOR 0
#define LEFT_BACK_MOTOR 2
#define RIGHT_BACK_MOTOR 3

#define ROTATION_SCALE 10.0

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
    if (speed < 0){
        run(REV, -speed);
    } else {
        run(FWD, speed);
    }
}

void right_front_motor(int speed){
    current_driver = RIGHT_FRONT_MOTOR;
    if (speed < 0){
        run(REV, -speed);
    } else {
        run(FWD, speed);
    }
}

void left_back_motor(int speed){
    current_driver = LEFT_BACK_MOTOR;
    if (speed < 0){
        run(REV, -speed);
    } else {
        run(FWD, speed);
    }
}

void right_back_motor(int speed){
    current_driver = RIGHT_BACK_MOTOR;
    if (speed < 0){
        run(REV, -speed);
    } else {
        run(FWD, speed);
    }
}

void robot_move(double x, double y, double rotation){
    int leftfront = (int)(y + x - rotation*(WIDTH/2.0 + HEIGHT/2.0));
    int rightfront = (int)(y - x + rotation*(WIDTH/2.0 + HEIGHT/2.0));
    int leftback = (int)(y - x - rotation*(WIDTH/2.0 + HEIGHT/2.0));
    int rightback = (int)(y + x + rotation*(WIDTH/2.0 + HEIGHT/2.0));
    left_front_motor(leftfront);
    right_front_motor(rightfront);
    left_back_motor(leftback);
    right_back_motor(rightback);
}

int main( int argc, char *argv[] ) {
    int sockfd, newsockfd, portno, clilen;
    char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    int family, s, n;
    struct ifaddrs *ifaddr, *ifa;
    char host[256];

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

    init(4);

    /* Now start listening for the clients, here process will
     * go in sleep mode and will wait for the incoming connection
     */

    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    printf("IP Address(es) of this machine: \n");
    getifaddrs(&ifaddr);
    for (ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++) {
        if (ifa->ifa_addr == NULL)
            continue;
        family = ifa->ifa_addr->sa_family;
        if(family == AF_INET){
            s = getnameinfo(ifa->ifa_addr,
                    sizeof(struct sockaddr_in),
                    host, NI_MAXHOST,
                    NULL, 0, NI_NUMERICHOST);
            if(strcmp(ifa->ifa_name, "lo"))
                printf("%s\t\taddress: <%s>\n", ifa->ifa_name, host);
        }
    }

    while(1){
        /* Accept actual connection from the client */
        turn_off_motors();
        printf("Waiting on socket connection...\n");
        newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);
        printf("Connected\n");

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
            int x = buffer[0] - 127;
            int y = buffer[1] - 127;
            double rotation = buffer[2] - 127;
            rotation = -(rotation/127.0)*(2*3.141592653589);
            double speed = buffer[4];
            speed = 2*speed/255.0;
            robot_move(x*speed, y*speed, rotation*speed/ROTATION_SCALE);
        }
    }
    freeifaddrs(ifaddr);
    exit(EXIT_SUCCESS);
}
