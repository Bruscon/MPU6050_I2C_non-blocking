# MPU6050_I2C_non-blocking
Note: I originally wrote this in January 2017. Only in 2021 did I dig it up and publish it here, unmodified. 


 My implementation of a non-blocking MPU specific I2C module.
 Three fuctions are available:
 MPUWriteReg which writes to one 8b register on the MPU
 MPUReadReg which reads a single register on the MPU
 MPUBurstReadReg which reads as many regs as you specify.

 Presently, it assembles the 8b data into a 16b signed global buffer.
 I dont anticipate needing anything other than this, but it can be modified as needed.

 TODO: optimize euler approximation for speed
 TODO: remove math library
 TODO: Digital Motion Processor utilization

ISSUE: even with waiting for IDLE, a delay must be inserted between R/W cycles. This is not truely non-blocking. fix it.
		it appears 50000 is the minimum delay before something messes up.
		powering off of 5v instead of GPIO pin makes SCL not sag, but does not lower threshhold.
		this is only for getting valid data, not for I2C fucking up
		SOLVED:you have to add a bitwise on the MCS register
		while((i2c_state != IDLE) || (I2C0_MCS_R & MCS_BUSY));							//while I2C is doing stuff

ISSUE: when trying to burst read on the interrupts, it asks for reg 3B and then MASTER holds CLK low???
ISSUE: CLUE: INT pin on MPU is triggering DRDY at 4Khz... something is wrong here. will look into it more tomorrow.
	SOLVED, needed a sample rate divider / DLPF

TESTING: currently uses less than 1% of CPU time.
TESTING: sampling at 1khz, we are getting new data extremely rapidly. The I2c bus visually appears about half full at 400kpbs
TESTING:
TESTING: calibration stability: with 10,000 samples calibration: the results were:
TESTING: -460	-27 49
TESTING: -459	-26	49
TESTING: -458	-26	50
TESTING:
TESTING: so calibration is very accurate and consistent. With 1k samples: -459 -26 50. with 100 samples: -455 -26 50
TESTING:
See notes on calc_euler function for specific performance tests and TODO's
