#include "music.h"

#define MOTOR_1 0
#define MOTOR_2 1
#define MOTOR_3 2
#define MOTOR_4 3

#define NUMBER_OF_DRIVERS 4
#define FREQ_DIV 1.0

extern int current_driver;

void turn_off_motors(){
    for(int i = 0; i < NUMBER_OF_DRIVERS; i++){
        current_driver = i;
        hardhiZ();
    }
}

void setup_motors(){
    init(NUMBER_OF_DRIVERS);
    for(int i  = 0; i < NUMBER_OF_DRIVERS; i++){
	current_driver = i;
	resetDev();
    }
    for(int i = 0; i < NUMBER_OF_DRIVERS; i++){
        current_driver = i;
	setMicroSteps(STEP_SEL_1);
	setMaxSpeed(15000.0);
	setThresholdSpeed(15000.0);
	setAcc(50000.0);
	setDec(50000.0);
	setSlewRate(CONFIG_SR_530V_us);
	//setOverCurrent(OCD_TH_750mA);
	setPWMFreq(CONFIG_PWM_DIV_2, CONFIG_PWM_MUL_2);
	setOCShutdown(CONFIG_OC_SD_DISABLE);
	setVoltageComp(CONFIG_VS_COMP_DISABLE);
	setAccKVAL(65);
	setDecKVAL(65);
	setRunKVAL(65);
	setHoldKVAL(20);
	softhiZ();
    }
}

int main(int argc, char *argv[]){

    long length1 = 0, length2 = 0, length3 = 0, length4 = 0;
    long prevLength1 = 0, prevLength2 = 0, prevLength3 = 0, prevLength4 = 0;
    double frequency1, frequency2, frequency3, frequency4;
    double prevFrequency1, prevFrequency2, prevFrequency3, prevFrequency4;
    int prevDir1 = FWD, prevDir2 = FWD, prevDir3 = FWD, prevDir4 = FWD;
    struct timeval currentTime, startTime;
    setup_motors();
    FILE *firstMotor = fopen(argv[1], "r");
    FILE *secondMotor = fopen(argv[2], "r");
    FILE *thirdMotor = fopen(argv[3], "r");
    FILE *fourthMotor = fopen(argv[4], "r");
    if(!firstMotor || !secondMotor){
	printf("File not Found\n");
	exit(0);
    }
    gettimeofday(&startTime, NULL);
    printf("Begin to play file\n");
    gettimeofday(&currentTime, NULL);
    while(1){
	double ms_since_start = ((currentTime.tv_sec - startTime.tv_sec) * 1000 ) + ((currentTime.tv_usec - startTime.tv_usec)/1000);
	if(ms_since_start >= length1){
	    current_driver = 0;
	    prevLength1 = length1;
	    prevFrequency1 = frequency1;
            int ret = fscanf(firstMotor, "%ld,%lf", &length1, &frequency1);
	    if(ret != 2){
	        turn_off_motors();
		printf("Motor 1 finished\n");
	    	return 0;
	    }
	    if(prevLength1 == length1){
	        run(prevDir1, 0);
		run(prevDir1, frequency1/FREQ_DIV);
	        prevDir1 = (prevDir1 == FWD) ? REV : FWD;
	    } else {
		run(prevDir1, 0);
		run(prevDir1, prevFrequency1/FREQ_DIV);
	    }
	    prevDir1 = (prevDir1 == FWD) ? REV : FWD;
	    //printf("Frequency 1 = %f\n", frequency1);
	}
	if(ms_since_start >= length2){
	    current_driver = 1;
	    prevLength2 = length2;
	    prevFrequency2 = frequency2;
            int ret = fscanf(secondMotor, "%ld,%lf", &length2, &frequency2);
	    if(ret != 2){
	        turn_off_motors();
		printf("Motor 2 finished\n");
	    	return 0;
	    }
	    if(prevLength2 == length2){
	        run(prevDir2, 0);
		run(prevDir2, frequency2/FREQ_DIV);
	        prevDir2 = (prevDir2 == FWD) ? REV : FWD;
	    } else {
		run(prevDir2, 0);
		run(prevDir2, prevFrequency2/FREQ_DIV);
	    }
	    prevDir2 = (prevDir2 == FWD) ? REV : FWD;
	}
	if(thirdMotor)
	    if(ms_since_start >= length3){
	        current_driver = 2;
	        prevLength3 = length3;
	        prevFrequency3 = frequency3;
                int ret = fscanf(thirdMotor, "%ld,%lf", &length3, &frequency3);
	        if(ret != 2){
	            turn_off_motors();
		    printf("Motor 3 finished\n");
	    	    return 0;
                }
                if(prevLength3 == length3){
	            run(prevDir3, 0);
		    run(prevDir3, frequency3/FREQ_DIV);
	            prevDir3 = (prevDir3 == FWD) ? REV : FWD;
	        } else {
		    run(prevDir3, 0);
		    run(prevDir3, prevFrequency3/FREQ_DIV);
	        }
	        prevDir3 = (prevDir3 == FWD) ? REV : FWD;
	    }
	if(fourthMotor)
	    if(ms_since_start >= length4){
	        current_driver = 3;
	        prevLength4 = length4;
	        prevFrequency4 = frequency4;
                int ret = fscanf(fourthMotor, "%ld,%lf", &length4, &frequency4);
	        if(ret < 2){
	            turn_off_motors();
	    	    return 0;
                }
                if(prevLength4 == length4){
	            run(prevDir4, 0);
		    run(prevDir4, frequency4/FREQ_DIV);
	            prevDir4 = (prevDir4 == FWD) ? REV : FWD;
	        } else {
		    run(prevDir4, 0);
		    run(prevDir4, prevFrequency4/FREQ_DIV);
	        }
	        prevDir4 = (prevDir4 == FWD) ? REV : FWD;
	    }
	gettimeofday(&currentTime, NULL);
    }
    return 0;
}
