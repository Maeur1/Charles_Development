#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <math.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <wiringPi.h>
#include <time.h>
#include "L6470.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define NUM_OF_DRIVERS 4

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static uint8_t mode;
static int fd;
static int num_of_drivers;
static const char *device = "/dev/spidev0.0";
static uint8_t bits = 8;
static uint32_t speed = 5000000;

int current_driver;

void init(){
    current_driver = 0;
    wiringPiSetup();
    pinMode(0, OUTPUT);
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(5, OUTPUT);
    digitalWrite(0, 0);
    digitalWrite(2, 0);
    digitalWrite(3, 0);
    digitalWrite(5, 0);
    digitalWrite(0, 1);
    digitalWrite(2, 1);
    digitalWrite(3, 1);
    digitalWrite(5, 1);

	int ret = 0;
	fd = open(device, O_RDWR);
    mode |= SPI_MODE_3;

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");
    
	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");
    
	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	// First things first: let's check communications. The CONFIG register should
	//  power up to 0x2E88, so we can use that to check the communications.
    printf("Status = %x\n", GetParam(CONFIG));
	if (GetParam(CONFIG) != 0x2E88){
		printf("Comm issue\n");
	}
}

bool isBusy(){
	int status = getStatus();
	return !((status >> 1) & 0b1);
}

void setMicroSteps(int microSteps){
	  uint8_t stepVal;
	  
	  for(stepVal = 0; stepVal < 8; stepVal++){
	  	if(microSteps == 2) break;
	  	microSteps = microSteps >> 1;
	  }
	  
	  SetParam(STEP_MODE, !SYNC_EN | stepVal | SYNC_SEL_1);
}

void setThresholdSpeed(float thresholdSpeed){
	// Configure the FS_SPD register- this is the speed at which the driver ceases
	//  microstepping and goes to full stepping. FSCalc() converts a value in steps/s
	//  to a value suitable for this register; to disable full-step switching, you
	//  can pass 0x3FF to this register.
	
	if(thresholdSpeed == 0.0){
		SetParam(FS_SPD, 0x3FF);
	}else{
		SetParam(FS_SPD, FSCalc(thresholdSpeed));	
	}
}

void setMaxSpeed(int speed){
	// Configure the MAX_SPEED register- this is the maximum number of (micro)steps per
	//  second allowed. You'll want to mess around with your desired application to see
	//  how far you can push it before the motor starts to slip. The ACTUAL parameter
	//  passed to this function is in steps/tick; MaxSpdCalc() will convert a number of
	//  steps/s into an appropriate value for this function. Note that for any move or
	//  goto type function where no speed is specified, this value will be used.
	SetParam(MAX_SPEED, MaxSpdCalc(speed));
}

void setMinSpeed(int speed){
	// Configure the MAX_SPEED register- this is the maximum number of (micro)steps per
	//  second allowed. You'll want to mess around with your desired application to see
	//  how far you can push it before the motor starts to slip. The ACTUAL parameter
	//  passed to this function is in steps/tick; MaxSpdCalc() will convert a number of
	//  steps/s into an appropriate value for this function. Note that for any move or
	//  goto type function where no speed is specified, this value will be used.
	SetParam(MIN_SPEED, MinSpdCalc(speed));
}

void setAcc(float acceleration){
	// Configure the acceleration rate, in steps/tick/tick. There is also a DEC register;
	//  both of them have a function (AccCalc() and DecCalc() respectively) that convert
	//  from steps/s/s into the appropriate value for the register. Writing ACC to 0xfff
	//  sets the acceleration and deceleration to 'infinite' (or as near as the driver can
	//  manage). If ACC is set to 0xfff, DEC is ignored. To get infinite deceleration
	//  without infinite acceleration, only hard stop will work.
	unsigned long accelerationBYTES = AccCalc(acceleration);
	SetParam(ACC, accelerationBYTES);
}

void setDec(float deceleration){
	unsigned long decelerationBYTES = DecCalc(deceleration);
	SetParam(DEC, decelerationBYTES);
}


long getPos(){
	unsigned long position = GetParam(ABS_POS);
	return convert(position);
}

float getSpeed(){
	/*
	 SPEED
	The SPEED register contains the current motor speed, expressed in step/tick (format unsigned fixed point 0.28).
	In order to convert the SPEED value in step/s the following formula can be used:
	Equation 4
	where SPEED is the integer number stored into the register and tick is 250 ns.
	The available range is from 0 to 15625 step/s with a resolution of 0.015 step/s.
	Note: The range effectively available to the user is limited by the MAX_SPEED parameter.
	*/
	
	return (float) GetParam(SPEED);
	//return (float) speed * pow(8, -22);
	//return FSCalc(speed); NEEDS FIX
}


void setOverCurrent(unsigned int ma_current){
	// Configure the overcurrent detection threshold. 
	uint8_t OCValue = floor(ma_current / 375);
	if(OCValue > 0x0F)OCValue = 0x0F;
	SetParam(OCD_TH, OCValue);
}

void setStallCurrent(float ma_current){
	
	uint8_t STHValue = (uint8_t)floor(ma_current / 31.25);
	if(STHValue > 0x80)STHValue = 0x80;
	if(STHValue < 0)STHValue = 0;
	SetParam(STALL_TH, STHValue);
}

void SetLowSpeedOpt(bool enable){
	// Enable or disable the low-speed optimization option. If enabling,
	//  the other 12 bits of the register will be automatically zero.
	//  When disabling, the value will have to be explicitly written by
	//  the user with a SetParam() call. See the datasheet for further
	//  information about low-speed optimization.
	Xfer(SET_PARAM | MIN_SPEED);
	if (enable) Param(0x1000, 13);
	else Param(0, 13);
}
	

void run(uint8_t dir, float spd){
	// RUN sets the motor spinning in a direction (defined by the constants
	//  FWD and REV). Maximum speed and minimum speed are defined
	//  by the MAX_SPEED and MIN_SPEED registers; exceeding the FS_SPD value
	//  will switch the device into full-step mode.
	// The SpdCalc() function is provided to convert steps/s values into
	//  appropriate integer values for this function.
	unsigned long speedVal = SpdCalc(spd);

	Xfer(RUN | dir);
	if (speedVal > 0xFFFFF) speedVal = 0xFFFFF;
	Xfer((uint8_t)(speedVal >> 16));
	Xfer((uint8_t)(speedVal >> 8));
	Xfer((uint8_t)(speedVal));
}


void Step_Clock(uint8_t dir){
	// STEP_CLOCK puts the device in external step clocking mode. When active,
	//  pin 25, STCK, becomes the step clock for the device, and steps it in
	//  the direction (set by the FWD and REV constants) imposed by the call
	//  of this function. Motion commands (RUN, MOVE, etc) will cause the device
	//  to exit step clocking mode.
	Xfer(STEP_CLOCK | dir);
}

void move(long n_step){
	// MOVE will send the motor n_step steps (size based on step mode) in the
	//  direction imposed by dir (FWD or REV constants may be used). The motor
	//  will accelerate according the acceleration and deceleration curves, and
	//  will run at MAX_SPEED. Stepping mode will adhere to FS_SPD value, as well.

	uint8_t dir;
	
	if(n_step >= 0){
		dir =  FWD;
	}else{
		dir =  REV;
	}
	
	long n_stepABS = abs(n_step);
	
	Xfer(MOVE | dir); //set direction
	if (n_stepABS > 0x3FFFFF) n_step = 0x3FFFFF;
	Xfer((uint8_t)(n_stepABS >> 16));
	Xfer((uint8_t)(n_stepABS >> 8));
	Xfer((uint8_t)(n_stepABS));
}

void goTo(long pos){
	// GOTO operates much like MOVE, except it produces absolute motion instead
	//  of relative motion. The motor will be moved to the indicated position
	//  in the shortest possible fashion.
	
	Xfer(GOTO);
	if (pos > 0x3FFFFF) pos = 0x3FFFFF;
	Xfer((uint8_t)(pos >> 16));
	Xfer((uint8_t)(pos >> 8));
	Xfer((uint8_t)(pos));
}


void goTo_DIR(uint8_t dir, long pos){
	// Same as GOTO, but with user constrained rotational direction.
	
	Xfer(GOTO_DIR);
	if (pos > 0x3FFFFF) pos = 0x3FFFFF;
	Xfer((uint8_t)(pos >> 16));
	Xfer((uint8_t)(pos >> 8));
	Xfer((uint8_t)(pos));
}

void goUntil(uint8_t act, uint8_t dir, unsigned long spd){
	// GoUntil will set the motor running with direction dir (REV or
	//  FWD) until a falling edge is detected on the SW pin. Depending
	//  on bit SW_MODE in CONFIG, either a hard stop or a soft stop is
	//  performed at the falling edge, and depending on the value of
	//  act (either RESET or COPY) the value in the ABS_POS register is
	//  either RESET to 0 or COPY-ed into the MARK register.
	Xfer(GO_UNTIL | act | dir);
	if (spd > 0x3FFFFF) spd = 0x3FFFFF;
	Xfer((uint8_t)(spd >> 16));
	Xfer((uint8_t)(spd >> 8));
	Xfer((uint8_t)(spd));
}

void releaseSW(uint8_t act, uint8_t dir){
	// Similar in nature to GoUntil, ReleaseSW produces motion at the
	//  higher of two speeds: the value in MIN_SPEED or 5 steps/s.
	//  The motor continues to run at this speed until a rising edge
	//  is detected on the switch input, then a hard stop is performed
	//  and the ABS_POS register is either COPY-ed into MARK or RESET to
	//  0, depending on whether RESET or COPY was passed to the function
	//  for act.
	Xfer(RELEASE_SW | act | dir);
}

void goHome(){
	// GoHome is equivalent to GoTo(0), but requires less time to send.
	//  Note that no direction is provided; motion occurs through shortest
	//  path. If a direction is required, use GoTo_DIR().
	Xfer(GO_HOME);
}

void goMark(){
	// GoMark is equivalent to GoTo(MARK), but requires less time to send.
	//  Note that no direction is provided; motion occurs through shortest
	//  path. If a direction is required, use GoTo_DIR().
	Xfer(GO_MARK);
}


void setMark(bool currentPlace, long value){
	if(currentPlace)
        value = getPos();

	Xfer(MARK);
	if (value > 0x3FFFFF) value = 0x3FFFFF;
	if (value < -0x3FFFFF) value = -0x3FFFFF;
	
	
	Xfer((uint8_t)(value >> 16));
	Xfer((uint8_t)(value >> 8));
	Xfer((uint8_t)(value));
}

void setAsHome(){
	// Sets the ABS_POS register to 0, effectively declaring the current
	//  position to be "HOME".
	Xfer(RESET_POS);
}

void resetDev(){
	// Reset device to power up conditions. Equivalent to toggling the STBY
	//  pin or cycling power.
	Xfer(RESET_DEVICE);
}
	
void softStop(){
	// Bring the motor to a halt using the deceleration curve.
	Xfer(SOFT_STOP);
}

void hardStop(){
	// Stop the motor right away. No deceleration.
	Xfer(HARD_STOP);
}

void softhiZ(){
	// Decelerate the motor and disengage
	Xfer(SOFT_HIZ);
}

void hardhiZ(){
	// disengage the motor immediately with no deceleration.
	Xfer(HARD_HIZ);
}

int getStatus(){
	// Fetch and return the 16-bit value in the STATUS register. Resets
	//  any warning flags and exits any error states. Using GetParam()
	//  to read STATUS does not clear these values.
	int temp = 0;
	Xfer(GET_STATUS);
	temp = Xfer(0)<<8;
	temp |= Xfer(0);
	return temp;
}

unsigned long AccCalc(float stepsPerSecPerSec){
	// The value in the ACC register is [(steps/s/s)*(tick^2)]/(2^-40) where tick is 
	//  250ns (datasheet value)- 0x08A on boot.
	// Multiply desired steps/s/s by .137438 to get an appropriate value for this register.
	// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
	float temp = stepsPerSecPerSec * 0.137438;
	if( ((unsigned long)(temp)) > 0x00000FFF) return 0x00000FFF;
	else return ((unsigned long)(temp));
}


unsigned long DecCalc(float stepsPerSecPerSec){
	// The calculation for DEC is the same as for ACC. Value is 0x08A on boot.
	// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
	float temp = stepsPerSecPerSec * 0.137438;
	if( ((unsigned long)(temp)) > 0x00000FFF) return 0x00000FFF;
	else return ((unsigned long)(temp));
}

unsigned long MaxSpdCalc(float stepsPerSec){
	// The value in the MAX_SPD register is [(steps/s)*(tick)]/(2^-18) where tick is 
	//  250ns (datasheet value)- 0x041 on boot.
	// Multiply desired steps/s by .065536 to get an appropriate value for this register
	// This is a 10-bit value, so we need to make sure it remains at or below 0x3FF
	float temp = stepsPerSec * .065536;
	if( ((unsigned long)(temp)) > 0x000003FF) return 0x000003FF;
	else return ((unsigned long)(temp));
}

unsigned long MinSpdCalc(float stepsPerSec){
	// The value in the MIN_SPD register is [(steps/s)*(tick)]/(2^-24) where tick is 
	//  250ns (datasheet value)- 0x000 on boot.
	// Multiply desired steps/s by 4.1943 to get an appropriate value for this register
	// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
	float temp = stepsPerSec * 4.1943;
	if( ((unsigned long)(temp)) > 0x00000FFF) return 0x00000FFF;
	else return ((unsigned long)(temp));
}

unsigned long FSCalc(float stepsPerSec){
	// The value in the FS_SPD register is ([(steps/s)*(tick)]/(2^-18))-0.5 where tick is 
	//  250ns (datasheet value)- 0x027 on boot.
	// Multiply desired steps/s by .065536 and subtract .5 to get an appropriate value for this register
	// This is a 10-bit value, so we need to make sure the value is at or below 0x3FF.
	float temp = (stepsPerSec * .065536)-.5;
	if( ((unsigned long)(temp)) > 0x000003FF) return 0x000003FF;
	else return ((unsigned long)(temp));
}

unsigned long IntSpdCalc(float stepsPerSec){
	// The value in the INT_SPD register is [(steps/s)*(tick)]/(2^-24) where tick is 
	//  250ns (datasheet value)- 0x408 on boot.
	// Multiply desired steps/s by 4.1943 to get an appropriate value for this register
	// This is a 14-bit value, so we need to make sure the value is at or below 0x3FFF.
	float temp = stepsPerSec * 4.1943;
	if( ((unsigned long)(temp)) > 0x00003FFF) return 0x00003FFF;
	else return ((unsigned long)(temp));
}

unsigned long SpdCalc(float stepsPerSec){
	// When issuing RUN command, the 20-bit speed is [(steps/s)*(tick)]/(2^-28) where tick is 
	//  250ns (datasheet value).
	// Multiply desired steps/s by 67.106 to get an appropriate value for this register
	// This is a 20-bit value, so we need to make sure the value is at or below 0xFFFFF.

	float temp = stepsPerSec * 67.106;
	if( ((unsigned long)(temp)) > 0x000FFFFF) return 0x000FFFFF;
	else return (unsigned long)temp;
}

uint8_t Xfer(uint8_t data){
	uint8_t ret;
    uint8_t tx[NUM_OF_DRIVERS] = {0};
    uint8_t rx[NUM_OF_DRIVERS] = {0};

    memcpy(&tx[current_driver], &data, sizeof(data));
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)&tx,
		.rx_buf = (unsigned long)&rx,
		.len = NUM_OF_DRIVERS,
		.delay_usecs = 0,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

    memcpy(&ret, &rx[current_driver], sizeof(ret));
    return ret;
}

void SetParam(uint8_t param, unsigned long value){
	Xfer(SET_PARAM | param);
	ParamHandler(param, value);
}

unsigned long GetParam(uint8_t param){
	// Realize the "get parameter" function, to read from the various registers in
	//  the dSPIN chip.
	Xfer(GET_PARAM | param);
	return ParamHandler(param, 0);
}

long convert(unsigned long val){
	//convert 22bit 2s comp to signed long  
	int MSB = val >> 21;
	
	val = val << 11;
	val = val >> 11;
	
	if(MSB == 1) val = val | 0b11111111111000000000000000000000;
	return val;
}

unsigned long Param(unsigned long value, uint8_t bit_len){
	// Generalization of the subsections of the register read/write functionality.
	//  We want the end user to just write the value without worrying about length,
	//  so we pass a bit length parameter from the calling function.
	unsigned long ret_val=0;        // We'll return this to generalize this function
	                                //  for both read and write of registers.
	uint8_t byte_len = bit_len/8;      // How many BYTES do we have?
	if (bit_len%8 > 0) byte_len++;  // Make sure not to lose any partial byte values.
	// Let's make sure our value has no spurious bits set, and if the value was too
	//  high, max it out.
	unsigned long mask = 0xffffffff >> (32-bit_len);
	if (value > mask) value = mask;
	// The following three if statements handle the various possible uint8_t length
	//  transfers- it'll be no less than 1 but no more than 3 uint8_ts of data.
	// Xfer() sends a uint8_t out through SPI and returns a uint8_t received
	//  over SPI- when calling it, we typecast a shifted version of the masked
	//  value, then we shift the received value back by the same amount and
	//  store it until return time.
	if (byte_len == 3) {
	  ret_val |= ((long)(Xfer((uint8_t)(value>>16)))) << 16;
	  //Serial.println(ret_val, HEX);
	}
	if (byte_len >= 2) {
	  ret_val |= ((long)(Xfer((uint8_t)(value>>8)))) << 8;
	  //Serial.println(ret_val, HEX);
	}
	if (byte_len >= 1) {
	  ret_val |= Xfer((uint8_t)value);
	  //Serial.println(ret_val, HEX);
	}
	// Return the received values. Mask off any unnecessary bits, just for
	//  the sake of thoroughness- we don't EXPECT to see anything outside
	//  the bit length range but better to be safe than sorry.
	return (ret_val & mask);
}

unsigned long ParamHandler(uint8_t param, unsigned long value){
	// Much of the functionality between "get parameter" and "set parameter" is
	//  very similar, so we deal with that by putting all of it in one function
	//  here to save memory space and simplify the program.
	unsigned long ret_val = 0;   // This is a temp for the value to return.
	// This switch structure handles the appropriate action for each register.
	//  This is necessary since not all registers are of the same length, either
	//  bit-wise or uint8_t-wise, so we want to make sure we mask out any spurious
	//  bits and do the right number of transfers. That is handled by the dSPIN_Param()
	//  function, in most cases, but for 1-uint8_t or smaller transfers, we call
	//  Xfer() directly.
	switch (param)
	{
	  // ABS_POS is the current absolute offset from home. It is a 22 bit number expressed
	  //  in two's complement. At power up, this value is 0. It cannot be written when
	  //  the motor is running, but at any other time, it can be updated to change the
	  //  interpreted position of the motor.
	  case ABS_POS:
	    ret_val = Param(value, 22);
	    break;
	  // EL_POS is the current electrical position in the step generation cycle. It can
	  //  be set when the motor is not in motion. Value is 0 on power up.
	  case EL_POS:
	    ret_val = Param(value, 9);
	    break;
	  // MARK is a second position other than 0 that the motor can be told to go to. As
	  //  with ABS_POS, it is 22-bit two's complement. Value is 0 on power up.
	  case MARK:
	    ret_val = Param(value, 22);
	    break;
	  // SPEED contains information about the current speed. It is read-only. It does 
	  //  NOT provide direction information.
	  case SPEED:
	    ret_val = Param(0, 20);
	    break; 
	  // ACC and DEC set the acceleration and deceleration rates. Set ACC to 0xFFF 
	  //  to get infinite acceleration/decelaeration- there is no way to get infinite
	  //  deceleration w/o infinite acceleration (except the HARD STOP command).
	  //  Cannot be written while motor is running. Both default to 0x08A on power up.
	  // AccCalc() and DecCalc() functions exist to convert steps/s/s values into
	  //  12-bit values for these two registers.
	  case ACC: 
	    ret_val = Param(value, 12);
	    break;
	  case DEC: 
	    ret_val = Param(value, 12);
	    break;
	  // MAX_SPEED is just what it says- any command which attempts to set the speed
	  //  of the motor above this value will simply cause the motor to turn at this
	  //  speed. Value is 0x041 on power up.
	  // MaxSpdCalc() function exists to convert steps/s value into a 10-bit value
	  //  for this register.
	  case MAX_SPEED:
	    ret_val = Param(value, 10);
	    break;
	  // MIN_SPEED controls two things- the activation of the low-speed optimization
	  //  feature and the lowest speed the motor will be allowed to operate at. LSPD_OPT
	  //  is the 13th bit, and when it is set, the minimum allowed speed is automatically
	  //  set to zero. This value is 0 on startup.
	  // MinSpdCalc() function exists to convert steps/s value into a 12-bit value for this
	  //  register. SetLowSpeedOpt() function exists to enable/disable the optimization feature.
	  case MIN_SPEED: 
	    ret_val = Param(value, 12);
	    break;
	  // FS_SPD register contains a threshold value above which microstepping is disabled
	  //  and the dSPIN operates in full-step mode. Defaults to 0x027 on power up.
	  // FSCalc() function exists to convert steps/s value into 10-bit integer for this
	  //  register.
	  case FS_SPD:
	    ret_val = Param(value, 10);
	    break;
	  // KVAL is the maximum voltage of the PWM outputs. These 8-bit values are ratiometric
	  //  representations: 255 for full output voltage, 128 for half, etc. Default is 0x29.
	  // The implications of different KVAL settings is too complex to dig into here, but
	  //  it will usually work to max the value for RUN, ACC, and DEC. Maxing the value for
	  //  HOLD may result in excessive power dissipation when the motor is not running.
	  case KVAL_HOLD:
	    ret_val = Xfer((uint8_t)value);
	    break;
	  case KVAL_RUN:
	    ret_val = Xfer((uint8_t)value);
	    break;
	  case KVAL_ACC:
	    ret_val = Xfer((uint8_t)value);
	    break;
	  case KVAL_DEC:
	    ret_val = Xfer((uint8_t)value);
	    break;
	  // INT_SPD, ST_SLP, FN_SLP_ACC and FN_SLP_DEC are all related to the back EMF
	  //  compensation functionality. Please see the datasheet for details of this
	  //  function- it is too complex to discuss here. Default values seem to work
	  //  well enough.
	  case INT_SPD:
	    ret_val = Param(value, 14);
	    break;
	  case ST_SLP: 
	    ret_val = Xfer((uint8_t)value);
	    break;
	  case FN_SLP_ACC: 
	    ret_val = Xfer((uint8_t)value);
	    break;
	  case FN_SLP_DEC: 
	    ret_val = Xfer((uint8_t)value);
	    break;
	  // K_THERM is motor winding thermal drift compensation. Please see the datasheet
	  //  for full details on operation- the default value should be okay for most users.
	  case K_THERM: 
	    ret_val = Xfer((uint8_t)value & 0x0F);
	    break;
	  // ADC_OUT is a read-only register containing the result of the ADC measurements.
	  //  This is less useful than it sounds; see the datasheet for more information.
	  case ADC_OUT:
	    ret_val = Xfer(0);
	    break;
	  // Set the overcurrent threshold. Ranges from 375mA to 6A in steps of 375mA.
	  //  A set of defined constants is provided for the user's convenience. Default
	  //  value is 3.375A- 0x08. This is a 4-bit value.
	  case OCD_TH: 
	    ret_val = Xfer((uint8_t)value & 0x0F);
	    break;
	  // Stall current threshold. Defaults to 0x40, or 2.03A. Value is from 31.25mA to
	  //  4A in 31.25mA steps. This is a 7-bit value.
	  case STALL_TH: 
	    ret_val = Xfer((uint8_t)value & 0x7F);
	    break;
	  // STEP_MODE controls the microstepping settings, as well as the generation of an
	  //  output signal from the dSPIN. Bits 2:0 control the number of microsteps per
	  //  step the part will generate. Bit 7 controls whether the BUSY/SYNC pin outputs
	  //  a BUSY signal or a step synchronization signal. Bits 6:4 control the frequency
	  //  of the output signal relative to the full-step frequency; see datasheet for
	  //  that relationship as it is too complex to reproduce here.
	  // Most likely, only the microsteps per step value will be needed; there is a set
	  //  of constants provided for ease of use of these values.
	  case STEP_MODE:
	    ret_val = Xfer((uint8_t)value);
	    break;
	  // ALARM_EN controls which alarms will cause the FLAG pin to fall. A set of constants
	  //  is provided to make this easy to interpret. By default, ALL alarms will trigger the
	  //  FLAG pin.
	  case ALARM_EN: 
	    ret_val = Xfer((uint8_t)value);
	    break;
	  // CONFIG contains some assorted configuration bits and fields. A fairly comprehensive
	  //  set of reasonably self-explanatory constants is provided, but users should refer
	  //  to the datasheet before modifying the contents of this register to be certain they
	  //  understand the implications of their modifications. Value on boot is 0x2E88; this
	  //  can be a useful way to verify proper start up and operation of the dSPIN chip.
	  case CONFIG: 
	    ret_val = Param(value, 16);
	    break;
	  // STATUS contains read-only information about the current condition of the chip. A
	  //  comprehensive set of constants for masking and testing this register is provided, but
	  //  users should refer to the datasheet to ensure that they fully understand each one of
	  //  the bits in the register.
	  case STATUS:  // STATUS is a read-only register
	    ret_val = Param(0, 16);
	    break;
	  default:
	    ret_val = Xfer((uint8_t)(value));
	    break;
	}
	return ret_val;
}

int main(){
    init();
}
