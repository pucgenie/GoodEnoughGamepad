/*
 *  Project     Arduino XInput Library
 *  @author     David Madison
 *  @link       github.com/dmadison/ArduinoXInput
 *  @license    MIT - Copyright (c) 2019 David Madison
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 *  Example:      GamepadPins
 *  Description:  Uses all of the available pin inputs to build a 'complete'
 *                Xbox gamepad, with both analog joysticks, both triggers,
 *                and all of the main buttons.
 *
 *                * Joysticks should be your typical 10k dual potentiometers.
 *                  To prevent random values caused by floating inputs,
                    joysticks are disabled by default.
 *                * Triggers can be either analog (pots) or digital (buttons).
 *                  Set the 'TriggerButtons' variable to change between the two.
 *                * Buttons use the internal pull-ups and should be connected
 *                  directly to ground.
 *
 *                These pins are designed around the Leonardo's layout. You
 *                may need to change the pin numbers if you're using a
 *                different board type
 *
 * Power optimizations, timing alignments (Reflex-like): pucgenie
 */

#include <avr/power.h>
#include <avr/sleep.h>
// workaround because of Arduino IDEs compilation units
#include "XInput_impl.h"


// Setup
#define UseLeftJoystick 0  // set to 1 to enable left joystick
static const boolean InvertLeftYAxis   = false;  // set to true to use inverted left joy Y

#define UseRightJoystick 0  // set to 1 to enable right joystick
static const boolean InvertRightYAxis  = false;  // set to true to use inverted right joy Y

#define UseTriggerButtons 1   // set to 0 if using analog triggers

#define ADC_Max 0b0000001111111111  // 10 bits

struct Pin_State {
	const XInputMap_Button& control;
	const uint8_t pinNumber;
	// high or low. 1-cycle-access (don't use bit-manipulation or :1;)
	boolean lastState;
};

static XInputController XInput;

#if UseLeftJoystick == 1
	// Joystick Pins
	static const uint8_t Pin_LeftJoyX  = A0;
	static const uint8_t Pin_LeftJoyY  = A1;
	// pucgenie: don't need to zero-initialize - is always written to at first.
	static int16_t joyLeftOld[2];
#endif
#if UseRightJoystick == 1
	static const uint8_t Pin_RightJoyX = A2;
	static const uint8_t Pin_RightJoyY = A3;
	static int16_t joyRightOld[2];
#endif

// Trigger Pins
static const uint8_t Pin_TriggerL = 40;
static const uint8_t Pin_TriggerR = 41;

#if UseTriggerButtons == 1
	// less complexity here
#else
	// pucgenie: don't need to zero-initialize - is always written to at first.
	static uint8_t triggerOld[2];
#endif

// Directional Pad Pins
#define ProcessDpadButtons 1

// Button Pins. enum XInputControl = index+1
static struct Pin_State PinButton[] = {
	{control: XInputController::Map_ButtonA, pinNumber: 2, lastState: true},
	{control: XInputController::Map_ButtonB, pinNumber: 3, lastState: true},
	{control: XInputController::Map_ButtonX, pinNumber: 4, lastState: true},
	{control: XInputController::Map_ButtonY, pinNumber: 5, lastState: true},
	{control: XInputController::Map_ButtonLB, pinNumber: 6, lastState: true},
	{control: XInputController::Map_ButtonRB, pinNumber: 7, lastState: true},
	{control: XInputController::Map_ButtonStart, pinNumber: 0, lastState: true},
	{control: XInputController::Map_ButtonBack, pinNumber: 1, lastState: true},
	{control: XInputController::Map_ButtonL3, pinNumber: 8, lastState: true},
	{control: XInputController::Map_ButtonR3, pinNumber: 9, lastState: true},
// button LOGO unused by design.


#if ProcessDpadButtons == 1
	// There are dependencies of these controls being at the end of array PinButton[]: UseSOCD
	{control: XInputController::Map_DpadUp, pinNumber: 14, lastState: true},
	{control: XInputController::Map_DpadDown, pinNumber: 16, lastState: true},
	{control: XInputController::Map_DpadLeft, pinNumber: 10, lastState: true},
	{control: XInputController::Map_DpadRight, pinNumber: 15, lastState: true},
#endif
};

#ifndef COMPLETELY_UNTOUCH_TIMER1
  ISR(TIMER1_COMPA_vect) {
    // disable timer1 interrupts
    TIMSK1 &= ~(1 << OCIE1A);
  }
#endif

// interruptable pins: 0,1,2,3,7

void setup() {
  // try to achieve <200 µA
  power_spi_disable();
  power_twi_disable();

	// If using buttons for the triggers, use internal pull-up resistors
	#if UseTriggerButtons == 1
		pinMode(Pin_TriggerL, INPUT_PULLUP);
		pinMode(Pin_TriggerR, INPUT_PULLUP);
		#if (UseLeftJoystick | UseRightJoystick) == 0
			// turn off ADC (#powersaving)
			ADCSRA = 0;
			power_adc_disable();
		#endif
	#else // If using potentiometers for the triggers, set range
		auto &triggerRange = XInput.triggerInputRange;
		triggerRange.min = 0;
		triggerRange.max = ADC_Max;
	#endif

	// Set buttons as inputs, using internal pull-up resistors
	for (int i = sizeof(PinButton); i --> 0;) {
		auto abc = PinButton[i];
		pinMode(PinButton[i].pinNumber, INPUT_PULLUP);
	}

	#if (UseLeftJoystick | UseRightJoystick)
	// Set joystick range to the ADC
	auto &joyRange = XInput.joystickInputRange;
	joyRange.min = 0;
	joyRange.max = ADC_Max;
	#endif

	//XInput.setAutoSend(false);  // Wait for all controls before sending

	XInput_USB_enable(XInput);
	XInput.begin();
}

static inline void computeTriggerValue(const XInputMap_Trigger &control_trigger, const uint8_t pin_Trigger) {
	// Read trigger buttons
	#if UseTriggerButtons == 1
		// pucgenie: cancel-out remapping values by directly using outputRange values
		// high->min, low->max
		auto triggerValue = digitalRead(pin_Trigger) ? XInputMap_Joystick::outputRange.min : XInputMap_Joystick::outputRange.max;
	#else
		// Read trigger potentiometer values
		auto triggerValue = analogRead(pin_Trigger);
	#endif
	
	XInput.setTrigger(control_trigger, triggerValue);
}

/**
"Only two switches exhibited bounces exceeding 6200 μsec. Switch E, what seemed like a
nice red pushbutton, had a worst case bounce when it opened of 157 msec – almost a 1/6
of a second! Yuk. Yet it never exceeded a 20 μsec bounce when closed." - https://my.eng.utah.edu/~cs5780/debouncing.pdf A Guide to Debouncing, Jack G. Ganssle
**/
void loop() {
	// Read pin values and store in variables
	// (Note the "!" to invert the state, because LOW = pressed)
	for (int i = sizeof(PinButton); i --> 0;) {
		struct Pin_State& currentPinButton = PinButton[i];
		currentPinButton.lastState = digitalRead(currentPinButton.pinNumber);
	}
	
	// Set XInput buttons
	for (int i = sizeof(PinButton)
		#ifdef UseSOCD
		-4
		#endif
			; i --> 0;) {
		struct Pin_State& currentPinButton = PinButton[i];
		XInput.setButton(currentPinButton.control, !currentPinButton.lastState);
	}

	#if defined(UseSOCD) && ProcessDpadButtons == 1
		// Set XInput DPAD values
		XInput.setDpad(
			!PinButton[sizeof(PinButton)-4].lastState,
			!PinButton[sizeof(PinButton)-3].lastState,
			!PinButton[sizeof(PinButton)-2].lastState,
			!PinButton[sizeof(PinButton)-1].lastState)
		;
	#endif

	// Set XInput trigger values

	computeTriggerValue(XInput.Map_TriggerLeft, Pin_TriggerL);
	computeTriggerValue(XInput.Map_TriggerRight, Pin_TriggerR);

	// Set left joystick
	#if UseLeftJoystick == 1
	{
		int leftJoyX = analogRead(Pin_LeftJoyX);
		int leftJoyY = analogRead(Pin_LeftJoyY);

		// White lie here... most generic joysticks are typically
		// inverted by default. If the "Invert" variable is false
		// then we'll take the opposite value with 'not' (!).

		XInput.setJoystickX(XInput.Map_JOY_LEFT, leftJoyX);
		XInput.setJoystickY(XInput.Map_JOY_LEFT, leftJoyY, !InvertLeftYAxis);
	}
	#endif

	// Set right joystick
	#if UseRightJoystick == 1
	{
		int rightJoyX = analogRead(Pin_RightJoyX);
		int rightJoyY = analogRead(Pin_RightJoyY);

		XInput.setJoystickX(XInput.Map_JOY_RIGHT, rightJoyX);
		XInput.setJoystickY(XInput.Map_JOY_RIGHT, rightJoyY, !InvertRightYAxis);
	}
	#endif

	#ifndef COMPLETELY_UNTOUCH_TIMER1
		OCR1A = 8;
		// OCR1A: how many scaled ticks to count
		TCNT1 = 0; // reset timer
		// Output Compare Match A Interrupt Enable
		TIMSK1 |= (1 << OCIE1A);

		set_sleep_mode(SLEEP_MODE_IDLE);
		sleep_enable();
		sleep_mode();
		sleep_disable();
	#endif

	// TODO: read again and calculate debouncing/filtering
	// joystick output = (A&&B) || (A&&C) || (B&&C) // because it doesn't matter if all three are 

	// TODO: reflex fine-tune
	// pucgenie: without SerialUSB, measuring time is complicated.
	// 13 : 11 should be good, but I don't know.
	if (XInput.send() == 0) {
		#ifndef COMPLETELY_UNTOUCH_TIMER1
			OCR1A = 2;
			// OCR1A: how many scaled ticks to count
			TCNT1 = 0; // reset timer
			// Output Compare Match A Interrupt Enable
			TIMSK1 |= (1 << OCIE1A);

			set_sleep_mode(SLEEP_MODE_IDLE);
			sleep_enable();
			sleep_mode();
			sleep_disable();
		#endif
	}
}
