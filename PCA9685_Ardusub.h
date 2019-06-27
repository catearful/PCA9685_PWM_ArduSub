/*
 * I2C driver for External PCA9685 PWM Controller
 *
 * Parts of the code are adapted from arduino library for PCA9865 board. 
 * License of the parts is located in arduino_Adafruit_PWM_Servo_Driver_Library_license.txt file. 
 * See https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library for contributors. 
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD

#define PCA9685_ADDR 0x40 //I2C Address for PCA9685
#define PCA9685_PWMFREQ 100.0f
#define PCA9685_NCHANS 16 // total number of pwm outputs

#define PCA9685_PWMMIN 1200
#define PCA9685_PWMMAX 1800

#define PCA9685_PWMCENTER ((PCA9685_PWMMAX + PCA9685_PWMMIN)/2)

class PCA9685 {
public:
    PCA9685(void);

    void init(void);
	
	bool reset(void);
	
	/**
	 * Helper function to set the pwm frequency
	 */
	bool setPWMFreq(float freq);
	
	/**
	 * Helper function to provide access to PWM_val array. 
	 * @param num pwm output number from 1 to 16, inclusive.
	 * @param val value from -1 to 1, inclusive.
	 */
	void setPWM_array(uint8_t num, float val);
	
	/** 
	 * Helper function to start timer and set values.
	 * @param amount of time in milliseconds you want the timer to run for. 
	 * @param init_arr is the initial array of PWM values -1 to 1, inclusive. 
	 * @param fin_arr is the final array of PWM values you'd like.
	 */
	void setPWM_timer(uint16_t time_ms, float init_arr[PCA9685_NCHANS], float fin_arr[PCA9685_NCHANS]); 
	 
	/**
	 * Helper function to check if timer is done. When time has elapsed, change PWM to final. 
	 */
	void check_timer(void);
	
	/**
	 * Helper function to get the demanded pwm value
	 * @param num pwm output number from 1 to 16
	 */
	uint16_t getPWM(uint8_t num) { return PWM2norm(PWM_val[num-1]); }
	
	bool is_healthy(void) { return _healthy; }

private:

	AP_HAL::OwnPtr<AP_HAL::Device> _dev;
	bool _healthy;						// if it's unhealthy, we have a problem. 
	bool _update;						// true when PWM_array has been changed. 
	bool _timer_on;						// if timer is on, disallow changing PWM values. 
	uint32_t _timer_start;				// milliseconds when timer began.	
	uint16_t _time_passed;				// milliseconds how long the timer goes for. 
	uint8_t _msg[6];					
	uint16_t PWM_val[PCA9685_NCHANS];	// list of pwm values the board pins have been set to
	uint16_t timer_buf[PCA9685_NCHANS]; // buffer for saving future values as timer runs. 
	
	/* converts PWM value to normalized value (-1 to 1 inclusive) */
	float PWM2norm(uint16_t val);
	
	/* check input for validity, and convert to PWM values */
	uint16_t convert2PWM(float val);
	
	/*
	 * Sets pin without having to deal with on/off tick placement and properly handles
	 * a zero value as completely off.  Optional invert parameter supports inverting
	 * the pulse for sinking to ground.
	 * @param num pwm output number
	 * @param val should be a value from 0 to 4095 inclusive.
	 */
	void setPin(uint8_t num, uint16_t val, bool invert=false);

	/* Helper function to set the demanded pwm value */
	bool setPWM(uint8_t num, uint16_t on, uint16_t off);
	
	/* Wrapper to read a byte from addr */
	bool read8(uint8_t addr, uint8_t &value);

	/* Wrapper to write a byte to addr */
	bool write8(uint8_t addr, uint8_t value);
	
	void timer(void); // main loop of the driver, updates pwm values. 
};