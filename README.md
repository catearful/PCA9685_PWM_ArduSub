# PCA9685 Driver for use with ArduSub 

A simple driver for interfacing an Adafruit 16-Channel 12-Bit PWM/Servo Driver with Ardusub. Includes timer code to output a certain PWM value for a set amount of time. 

## To install:

1. Place in a library folder of your choice (like AP_Motors). 

2. Edit AP_Motors.h, add the line #include PCA9685_Ardusub.h. 

3. In Sub.h, create driver object in the private variables section with 

```
PCA9685 your_obj_name;
```

4. Use APM_Config.h to activate the functions in UserCode.cpp. Put 

```
your_obj_name.init();
```

## To use:

Default for the driver is to use all 16 channels. Change `PCA9685_NCHANS` if you want to limit the number of active channels.

Use `setPWM_array(uint8_t num, float val)` to select and change the pwm value a channel outputs. 

Use `setPWM_timer(uint16_t time_ms, float init_arr[PCA9685_NCHANS], float fin_arr[PCA9685_NCHANS])` to set the initial PWM state of the channels and the final state after `time_ms`. 

To associate them with a button command, place either of the above functions in joystick.cpp, under `k_custom_1`. Then map the custom_1 function to a key using a mission control like QGroundControl. 

