#include "PCA9685_Ardusub.h"

#include <utility>
#include <stdio.h>
#include <string.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

PCA9685::PCA9685() :
	_dev(nullptr),
	_healthy(false),
	_update(false),
	_timer_on(false),
	_timer_start(0),
	_time_passed(0)
{
    memset(&_msg, 0, sizeof(_msg));
	
	for (uint8_t i = 0; i < PCA9685_NCHANS; i++) {
		PWM_val[i] = uint16_t(PCA9685_PWMCENTER);
	}
	
	memset(&timer_buf, 0, sizeof(timer_buf));
		
}	

void PCA9685::init() 
{
	// try pinging device on external bus 
	_dev = std::move(hal.i2c_mgr->get_device(1, PCA9685_ADDR));
	if (!_dev) {
		printf("PCA9685 not found!");
		_healthy = false;
	}
	
	if (!_dev->get_semaphore()->take(0)) {
		AP_HAL::panic("PANIC: PCA9685: failed to take serial semaphore for init");
	}
	
	_dev->set_retries(10);
	
	// try reset 
	if (!reset()) {
		printf("PCA9685 reset failed");
		_dev->get_semaphore()->give();
		_healthy = false;
	}
	
	// set retries back to 3. 
	_dev->set_retries(3);
	
	// give semaphore back. 
	_dev->get_semaphore()->give();
	
	// set chip pulse frequency
	if (!setPWMFreq(PCA9685_PWMFREQ)) {
		printf("Failed to set PCA9685 frequency");
		_dev->get_semaphore()->give();
		_healthy = false;
	}
	
	// 20 hz update of pwm signals
    _dev->register_periodic_callback(50 * USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&PCA9685::timer, void));
	_healthy = true;
	
}

bool PCA9685::reset()
{	
	_healthy = write8(PCA9685_MODE1, 0x0);
	
	return _healthy;
}

bool PCA9685::setPWMFreq(float freq)
{
	if (_timer_on) {
		return false;
	}
	
	freq *= 0.9f; /* Correct for overshoot in frequency setting, see issue on 
					https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/issues/11 */
	
	float prescaleval = 25000000;
	prescaleval /= 4096;
	prescaleval /= freq;
	prescaleval -= 1;
	
	uint8_t prescale = uint8_t(prescaleval + 0.5f); //implicit floor
	
	uint8_t oldmode;
	bool ret = read8(PCA9685_MODE1, oldmode);
	
	if (!ret) {
		return ret;
	}
	
	uint8_t newmode = (oldmode & 0x7F) | 0x10; //sleep
	
	ret = write8(PCA9685_MODE1, newmode); //go to sleep
	
	if (!ret) {
		return ret;
	}
	
	ret = write8(PCA9685_PRESCALE, prescale); //set the prescaler 
	
	if (!ret) {
		return ret;
	}
	
	hal.scheduler->delay(5); //5 ms delay
	
	ret = write8(PCA9685_MODE1, oldmode | 0xa0); //sets MODE1 register to turn on auto increment
	
	return ret;
}

void PCA9685::setPWM_array(uint8_t num, float val)
{
	// if timer is ticking, stop user from changing PWM_array. 
	if (_timer_on) {
		return;
	}
	
	if (num > PCA9685_NCHANS) {
		num = PCA9685_NCHANS;
	}
	
	if (num < 1) {
		num = 1;
	}
	
	uint16_t pwm_value = convert2PWM(val);
	
	// check to see if PWM value has already been assigned, avoid unnecessary transfers. 
	if (pwm_value == PWM_val[num-1]) {
		return;
	} else {
		// save values
		PWM_val[num-1] = pwm_value;
		_update = true;
	}
	
}

void PCA9685::setPWM_timer(uint16_t time_ms, float init_arr[PCA9685_NCHANS], float fin_arr[PCA9685_NCHANS])
{
	// if timer is already ticking, stop resetting the timer. 
	if (_timer_on){
		return;
	}
	
	_timer_on = true;
	
	// add initial values to PWM_val to be sent out. 
	for (uint8_t i = 0; i < PCA9685_NCHANS; i++) {
		PWM_val[i] = convert2PWM(init_arr[i]);
	}
	
	// set the timer buffer array with the expected final values. 
	for (uint8_t i = 0; i < PCA9685_NCHANS; i++) {
		timer_buf[i] = convert2PWM(fin_arr[i]);
	}
	
	_timer_start = AP_HAL::millis();
	
	_time_passed = time_ms;
	
	_update = true;
	
}

void PCA9685::check_timer() 
{
	if (!_timer_on) {
		return;
	}
	
	uint32_t _time_now = AP_HAL::millis();
	
	if (_time_now - _timer_start > _time_passed) {
		memcpy(PWM_val, timer_buf, PCA9685_NCHANS*sizeof(int8_t));		
		memset(&timer_buf, 0, sizeof(timer_buf));
	} else {
		return;
	}
	
	_timer_on = false;
	
	_time_passed = 0;
	
	_update = true;

}

float PCA9685::PWM2norm(uint16_t val) 
{
	// convert to normalized values
	float norm_val = static_cast<float>((val - PCA9685_PWMCENTER) / (PCA9685_PWMCENTER - PCA9685_PWMMIN)); 
	
	return norm_val;
}

uint16_t PCA9685::convert2PWM(float val)
{
	// validate user input
	val = constrain_float(val, -1.0f, 1.0f);
	
	// convert to PWM
	uint16_t pwm_value = static_cast<uint16_t>(PCA9685_PWMCENTER + (val * (PCA9685_PWMCENTER - PCA9685_PWMMIN)));
	
	return pwm_value;
}

void PCA9685::setPin(uint8_t num, uint16_t val,  bool invert)
{
	if (val > 4095) {
		val = 4095;
	}
	
	if (invert) {
		if (val == 0) {
			//Special value for signal fully on. 
			_healthy = setPWM(num, 4096, 0);
			
		} else if (val == 4095) {
			// Special value for signal fully off. 
			_healthy = setPWM(num, 0, 4096);
			
		} else {
			_healthy = setPWM(num, 0, 4095 - val);
		}
	} else {
		if (val == 4095) {
			// Special value for signal fully on. 
			_healthy = setPWM(num, 4096, 0);
			
		} else if (val == 0) {
			// Special value for signal fully off. 
			_healthy = setPWM(num, 0, 4096);
			
		} else {
			_healthy = setPWM(num, 0, val);
		}
	}
}

bool PCA9685::setPWM(uint8_t num, uint16_t on, uint16_t off) 
{
	// convert to correct message
	_msg[0] = LED0_ON_L + 4 * num;
	_msg[1] = on;
	_msg[2] = on >> 8;
	_msg[3] = off;
	_msg[4] = off >> 8;
	
	return _dev->transfer(_msg, 5, nullptr, 0);
}

/* Wrapper to read a byte from an address */
bool PCA9685::read8(uint8_t addr, uint8_t &value)
{
	bool ret = _dev->transfer(&addr, sizeof(addr), nullptr, 0);
	
	if (!ret) {
		return ret;
	}
	
	return _dev->transfer(nullptr, 0, &value, 1);
}

/* Wrapper to write a byte to an address */
bool PCA9685::write8(uint8_t addr, uint8_t value)
{
	_msg[0] = addr;
	_msg[1] = value;
	/* send addr and value */
	return _dev->transfer(_msg, 2, nullptr, 0);
}
	
void PCA9685::timer()
{	
	// check timer periodically to see if timer has been engaged. 
	check_timer();
		
	if (_update && _healthy) {
		for (uint8_t i = 0; i < PCA9685_NCHANS; i++) {
				setPin(i, PWM_val[i]);
		}
	}

	_update = false;
}

