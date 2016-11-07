#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "L6470.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static int fd;
static const char *device = "/dev/spidev0.0";
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay;

void init(){
	fd = open(device, O_RDWR);
}

uint8_t Xfer(uint8_t data){
	int ret;
    uint8_t* rx;
	struct spi_ioc_transfer tr = {
		.tx_buf = &data,
		.rx_buf = rx,
		.len = 1,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
    return *rx;
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
	// L6470::Xfer() sends a uint8_t out through SPI and returns a byte received
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
