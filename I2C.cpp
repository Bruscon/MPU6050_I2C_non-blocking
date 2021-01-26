#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"

#include "MPU6050.h"


//Defines
#define MCS_ERROR 0b10011110															//if anything goes wrong in the MCS reg
#define MCS_BUSY  0b11011111															//basically to know if you can use the line again

#define		MPU_WRITE_ADDRESS	0xD0
#define		MPU_READ_ADDRESS	0xD1

#define		ACCEL_FSR			2														//G's
#define		DIVISOR				10000000000
#define 	RADIANS_TO_DEGREES  57.2958


//Variable Declarations
								//big enough to hold accel/gyro x y z and Temp

enum i2c_state_enum {							//for i2c state machine
	WRITE_SINGLE_REG,
	READ_SINGLE_REG1,
	BURST_READ_REGS1,
	ALMOST_IDLE,
	ALMOST_DONE_WRITING,
	READ_SINGLE_REG2,
	BURST_READ_REGS2,
	IDLE
};

struct MPU_struct {																		//structure for I2c transactsions and MPU stuff
	enum i2c_state_enum		i2c_state;
	uint8_t  				new_MDR;
	uint8_t  				index;
	uint8_t  				stopAt;
	uint32_t				num_frames;
	int16_t					gyro_calib[3];
	int16_t					accel_calib[3];
	float					alpha;														//for complementary filter
	float					gyro_RPY[3];												//floats are 32 bits
	float					accel_RPY[3];
	float					RPY[3];
	int16_t 				data_buff_14b[7];											//stores last motion frame from MPU
};


struct MPU_struct MPU = {IDLE,0,0,0,0,{0,0,0},{0,0,0},.95,{0,0,0},{0,0,0}};

//Functions
void wtimer0_init(void){				//inits wide timer to let us get dt
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);												//enable wtimer0
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_PERIODIC_UP);										//setup wide timer0 as periodic
    TimerEnable(WTIMER0_BASE, TIMER_BOTH);														//should only miss a frame about every 7000 years
}										//when you want the time, use TimerValueGet64(WTIMER0_BASE);


/*
 * Efficiency:
 * when calculating accel every single loop, about 63% of the CPU is used (running at 1khz)
 * when using accel %5, about 25%of the CPU is used.
 *
 * TODO: replace math library with faster math functions of your own desing. Should be able to get this <5%
 */

inline void calc_euler(void){					//calculate the euler angles
	static uint64_t last_time;

	uint64_t this_time = TimerValueGet64(WTIMER0_BASE);
	static uint64_t dt;																//time since last function call, in clk ticks
	dt = this_time - last_time;
	last_time = this_time;


	MPU.gyro_RPY[0] = MPU.RPY[0] + ((float)(MPU.data_buff_14b[4] - MPU.gyro_calib[0])*dt) / DIVISOR;
	MPU.gyro_RPY[1] = MPU.RPY[1] + ((float)(MPU.data_buff_14b[5] - MPU.gyro_calib[1])*dt) / DIVISOR;
	MPU.gyro_RPY[2] = MPU.RPY[2] + ((float)(MPU.data_buff_14b[6] - MPU.gyro_calib[2])*dt) / DIVISOR;


	//calculating euler from accelerometer is intensive and inacurate, so only do it every certain number of cycles then weight alpha higher
	//also, we disregard the accelerometer if the values add up to more than 1.5x gravity or less than .5x
	if((MPU.num_frames % 5 == 1)
			&& (((abs(MPU.data_buff_14b[0]) + abs(MPU.data_buff_14b[1]) + abs(MPU.data_buff_14b[2])) < 24000) //remains untested
			&& ((abs(MPU.data_buff_14b[0]) + abs(MPU.data_buff_14b[1]) + abs(MPU.data_buff_14b[2])) > 8000))){
		MPU.accel_RPY[0] = atan(-1*MPU.data_buff_14b[0]/sqrt(pow(MPU.data_buff_14b[1],2) + pow(MPU.data_buff_14b[2],2)))*RADIANS_TO_DEGREES;
		MPU.accel_RPY[1] = atan(MPU.data_buff_14b[1]/sqrt(pow(MPU.data_buff_14b[0],2) + pow(MPU.data_buff_14b[2],2)))*RADIANS_TO_DEGREES;
	}

	MPU.RPY[0] = MPU.alpha*MPU.gyro_RPY[0] + (1 - MPU.alpha)*MPU.accel_RPY[0];
	MPU.RPY[1] = MPU.alpha*MPU.gyro_RPY[1] + (1 - MPU.alpha)*MPU.accel_RPY[1];
	MPU.RPY[2] = MPU.alpha*MPU.gyro_RPY[2];

}	//could DEFINITELY use some optimization



void calibrate_MPU(int64_t num_samples){				//MAKE SURE TO PLACE MPU FLAT FOR THIS
	int64_t samples[6] = {0,0,0,0,0,0};														//stores the sum of all gyro readings
	uint16_t count;
	uint32_t old_num_frames = MPU.num_frames;

	for(count=0;count<num_samples;count++){
		while(MPU.num_frames == old_num_frames);											//if there isnt new data for us
		old_num_frames = MPU.num_frames;

		samples[0] += (int64_t)MPU.data_buff_14b[0];
		samples[1] += (int64_t)MPU.data_buff_14b[1];
		samples[2] += (int64_t)MPU.data_buff_14b[2];
		samples[3] += (int64_t)MPU.data_buff_14b[4];
		samples[4] += (int64_t)MPU.data_buff_14b[5];
		samples[5] += (int64_t)MPU.data_buff_14b[6];
	}

	MPU.accel_calib[0] = samples[0] / num_samples;
	MPU.accel_calib[1] = samples[1] / num_samples;
	MPU.accel_calib[2] = samples[2] / num_samples;
	MPU.gyro_calib[0] = samples[3] / num_samples;
	MPU.gyro_calib[1] = samples[4] / num_samples;
	MPU.gyro_calib[2] = samples[5] / num_samples;

}


void MPUwriteReg(uint8_t reg, uint8_t data){
	while((MPU.i2c_state != IDLE) || (I2C0_MCS_R & MCS_BUSY));							//wait while I2C is doing stuff
	MPU.i2c_state = WRITE_SINGLE_REG;

    I2C0_MSA_R = MPU_WRITE_ADDRESS;												//specifies slave address, write operation
    HWREG(I2C0_BASE + I2C_O_MDR) = (reg);										//which reg to write to
    I2C0_MCS_R |= I2C_MCS_RUN + I2C_MCS_START;									//initiate transmission, START RUN

    MPU.new_MDR = data;
}

void MPUReadReg(uint8_t reg){
	while((MPU.i2c_state != IDLE) || (I2C0_MCS_R & MCS_BUSY));							//wait while I2C is doing stuff
	MPU.i2c_state = READ_SINGLE_REG1;

    I2C0_MSA_R = MPU_WRITE_ADDRESS;														//specifies slave address, write operation
	HWREG(I2C0_BASE + I2C_O_MDR) = (reg);												//which reg to write on the line
	I2C0_MCS_R |= I2C_MCS_DATACK + I2C_MCS_RUN + I2C_MCS_START;							//initiate transmission, ACK START RUN

}

void MPUBurstReadRegs(uint8_t startReg, uint8_t numRegs){		//also assembles them into 16b signed. for 8b, use MPUReadReg
	while((MPU.i2c_state != IDLE) || (I2C0_MCS_R & MCS_BUSY));								//wait while I2C is doing stuff
	MPU.index = 0;
	MPU.i2c_state = BURST_READ_REGS1;
	MPU.stopAt = numRegs;

    I2C0_MSA_R = MPU_WRITE_ADDRESS;														//specifies slave address, write operation
	HWREG(I2C0_BASE + I2C_O_MDR) = (startReg);											//which reg to write on the line
	I2C0_MCS_R |= I2C_MCS_DATACK + I2C_MCS_RUN + I2C_MCS_START;							//initiate transmission, ACK START RUN

}

void i2c0_init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);											//enable I2C module0
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);											//reset it
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);										//enable GPIOB
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);													//configure pin B2 as SCL
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);													//configure pin B3 as SDA
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);										//configure pin B2 as SCL
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);										//configure pin B3 as SDA

    I2C0_MCR_R = I2C_MCR_MFE;															//initialize master mode (unnecessary)
    //I2C0_MTPR_R = 0x39;																//clock period register
    I2C0_MIMR_R = 0x3;																	//enable interrupts for master and clk timeout

    //this function sucks.  adjust MTPR reg directly in the future to speed up.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);								//400kbps data rate setup

    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;										//clear the FIFO just in case

	IntEnable(INT_I2C0);

}

void MPU_init(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);									//enable port C to power MPU
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4 );
	GPIO_PORTC_DATA_R = 0x10;														//enable power to MPU

	int i;
	for (i=0;i<200000;i++);															//give MPU some time to power up

    MPUwriteReg(MPU6050_RA_PWR_MGMT_1, 0x80);										//reset device
    for (i=0;i<200000;i++);															//give MPU some time to reset

    MPUwriteReg(MPU6050_RA_PWR_MGMT_1, 0x01);										//wake up the MPU, xgyro as clk
    for (i=0;i<200000;i++);															//wait for the device to wake up

    MPUwriteReg(MPU6050_RA_CONFIG, 0b00000001);										//set DLPF to 1
}

void MPU_DRDY_int_handler(void){
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);  												// Clear interrupt flag
	if((MPU.i2c_state == IDLE) && !(I2C0_MCS_R & MCS_BUSY)){
		MPUBurstReadRegs(MPU6050_RA_ACCEL_XOUT_H,14);
	}
}

void MPU_int_config(void){																		//also inits LED's

	MPUwriteReg(MPU6050_RA_INT_PIN_CFG, 0b00010000);								//pulsed interrupt, open drain, active high.
	MPUwriteReg(MPU6050_RA_INT_ENABLE, 0b00000001);									//DRDY interrupt. no FIFO, no Master, no nothing

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);												//enable port F for LEDs
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);												//enable port C for PPM
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 |GPIO_PIN_3);				//LED pins as outputs

    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);  										// Init PC5 as input
    GPIOIntDisable(GPIO_PORTF_BASE, GPIO_PIN_4);        										// Disable interrupt for PC4 (in case it was enabled)
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);      											// Clear pending interrupts for PC4
	GPIOIntRegister(GPIO_PORTF_BASE, MPU_DRDY_int_handler);     								// Register our handler function for port C
	GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_RISING_EDGE);             					// Configure PC4 for rising edge trigger
	GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);     											// Enable interrupt for PC4

	int i;
	for (i=0;i<20000;i++);																		//wait for samples to begin

}


void I2C0_int_handler(void){						//State machine for various I2C functions

	I2C0_MICR_R = 1;																	//clear interrupt

	if (MPU.i2c_state == WRITE_SINGLE_REG){
		HWREG(I2C0_BASE + I2C_O_MDR) = (MPU.new_MDR);										//load the data to be written to the slave reg
		I2C0_MCS_R |= I2C_MCS_RUN + I2C_MCS_STOP;										//load new control bits
		MPU.i2c_state = ALMOST_DONE_WRITING;
	}

	else if (MPU.i2c_state == ALMOST_DONE_WRITING){
		MPU.i2c_state = ALMOST_IDLE;
	}

	else if (MPU.i2c_state == READ_SINGLE_REG1){
		MPU.i2c_state = READ_SINGLE_REG2;

		I2C0_MSA_R = MPU_READ_ADDRESS;													//specifies slave address, read operation
		I2C0_MCS_R |=  I2C_MCS_RUN + I2C_MCS_START + I2C_MCS_STOP;						//initiate transmission, ACK START RUN
	}

	else if (MPU.i2c_state == READ_SINGLE_REG2){
		MPU.i2c_state = ALMOST_IDLE;
		MPU.data_buff_14b[0] = HWREG(I2C0_BASE + I2C_O_MDR);
		MPU.num_frames++;
	}

	else if (MPU.i2c_state == BURST_READ_REGS1){
		MPU.i2c_state = BURST_READ_REGS2;

		I2C0_MSA_R = MPU_READ_ADDRESS;													//specifies slave address, read operation
		I2C0_MCS_R |=  I2C_MCS_DATACK + I2C_MCS_RUN + I2C_MCS_START;					//initiate transmission, ACK START RUN
	}

	else if (MPU.i2c_state == BURST_READ_REGS2){
		if(!(MPU.index&0x0001))															//if even data
			MPU.data_buff_14b[MPU.index/2] = HWREG(I2C0_BASE + I2C_O_MDR)<<8;
		else																			//if odd data
			MPU.data_buff_14b[MPU.index/2] |= HWREG(I2C0_BASE + I2C_O_MDR);
		I2C0_MCS_R |= I2C_MCS_DATACK + I2C_MCS_RUN;										//ACK, run
		MPU.index++;

		if(MPU.index >= MPU.stopAt){
			I2C0_MCS_R |= I2C_MCS_STOP;													//NACK then stop bit
			MPU.i2c_state = ALMOST_IDLE;
			MPU.num_frames++;
		}
	}

	if (MPU.i2c_state == ALMOST_IDLE){
		MPU.i2c_state = IDLE;
		calc_euler();
	}
}


//Main
int main(void) {

	//80 MHz
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

	wtimer0_init();
	i2c0_init();
	MPU_init();
	MPU_int_config();
	calibrate_MPU(500);																//calibrate MPU with 500 samples

	IntMasterEnable();

	while(1){
	}

}
