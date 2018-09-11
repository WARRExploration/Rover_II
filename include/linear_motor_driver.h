#pragma once
#include <serial/serial.h>
#include <string>
#include <iostream>

// Control values
#define SET_ACCURACY 0x01
#define	SET_RETRACT_LIMIT 0x02
#define	SET_EXTEND_LIMIT 0x03
#define	SET_MOVEMENT_THRESHOLD 0x04
#define	SET_STALL_TIME 0x05
#define	SET_PWM_THRESHOLD 0x06
#define	SET_DERIVATIVE_THRESHOLD 0x07 
#define	SET_DERIVATIVE_MAXIMUM 0x08
#define	SET_DERIVATIVE_MINIMUM 0x09
#define	SET_PWM_MAXIMUM 0x0A
#define	SET_PWM_MINIMUM 0x0B
#define	SET_PROPORTIONAL_GAIN 0x0C
#define	SET_DERIVATIVE_GAIN 0x0D
#define	SET_AVERAGE_RC 0x0E
#define	SET_AVERAGE_ADC 0x0F

#define	GET_FEEDBACK 0x10

#define	SET_POSITION 0x20
#define	SET_SPEED 0x21

#define	DISABLE_MANUAL 0x30

#define	RESET 0xFF

#define FALSE 0
#define TRUE 1

typedef uint8_t UCHAR;
typedef int32_t INT;

class linear_motor_driver {
	public:
		UCHAR init(std::string device, INT baudrate);
		void SendCmd(UCHAR Control, INT Value);
		UCHAR GetResult(UCHAR *Control, INT *Value);

		~linear_motor_driver();

	private: 
		serial::Serial ser;
};
