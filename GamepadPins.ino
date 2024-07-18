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
#include "XInput.h"

// Setup
#define UseLeftJoystick 0  // set to 1 to enable left joystick
const boolean InvertLeftYAxis   = false;  // set to true to use inverted left joy Y

#define UseRightJoystick 0  // set to 1 to enable right joystick
const boolean InvertRightYAxis  = false;  // set to true to use inverted right joy Y

#define UseTriggerButtons 1   // set to 0 if using analog triggers

#define ADC_Max 0b0000001111111111  // 10 bit

struct Pin_State {
	const uint8_t pinNumber;
	boolean lastState;
};

#if UseLeftJoystick == 1
// Joystick Pins
const uint8_t Pin_LeftJoyX  = A0;
const uint8_t Pin_LeftJoyY  = A1;// pucgenie: don't need to zero-initialize - is always written to at first.
int16_t joyLeftOld[2];
#endif
#if UseRightJoystick == 1
const uint8_t Pin_RightJoyX = A2;
const uint8_t Pin_RightJoyY = A3;
int16_t joyRightOld[2];
#endif

// Trigger Pins
const uint8_t Pin_TriggerL = 14;
const uint8_t Pin_TriggerR = 15;

// pucgenie: don't need to zero-initialize - is always written to at first.
uint8_t triggerOld[2];

// Button Pins
// pucgenie: don't need to false-initialize lastState - is always written to at first.
struct Pin_State Pin_ButtonA = {pinNumber: 2};
struct Pin_State Pin_ButtonB = {pinNumber: 3};
struct Pin_State Pin_ButtonX = {pinNumber: 4};
struct Pin_State Pin_ButtonY = {pinNumber: 5};
struct Pin_State Pin_ButtonLB = {pinNumber: 6};
struct Pin_State Pin_ButtonRB = {pinNumber: 7};
struct Pin_State Pin_ButtonBack  = {pinNumber: 8};
struct Pin_State Pin_ButtonStart = {pinNumber: 9};
struct Pin_State Pin_ButtonL3 = {pinNumber: 10};
struct Pin_State Pin_ButtonR3 = {pinNumber: 16};
// button LOGO unused by design.

// Directional Pad Pins
#define ProcessDpadButtons 0
#if ProcessDpadButtons == 1
struct Pin_State Pin_DpadUp    = {pinNumber: 0};
struct Pin_State Pin_DpadDown  = {pinNumber: 1};
struct Pin_State Pin_DpadLeft  = {pinNumber: 40};
struct Pin_State Pin_DpadRight = {pinNumber: 41};
#endif


#ifndef COMPLETELY_UNTOUCH_TIMER1
  ISR(TIMER1_COMPA_vect) {
    // disable timer1 interrupts
    TIMSK1 &= ~(1 << OCIE1A);
  }
#endif

// interruptable pins: 0,1,2,3,7

void setup() {
  // try to reach <200 µA
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
		XInput.setTriggerRange(0, ADC_Max);
	#endif

	// Set buttons as inputs, using internal pull-up resistors
	pinMode(Pin_ButtonA.pinNumber, INPUT_PULLUP);
	pinMode(Pin_ButtonB.pinNumber, INPUT_PULLUP);
	pinMode(Pin_ButtonX.pinNumber, INPUT_PULLUP);
	pinMode(Pin_ButtonY.pinNumber, INPUT_PULLUP);

	pinMode(Pin_ButtonLB.pinNumber, INPUT_PULLUP);
	pinMode(Pin_ButtonRB.pinNumber, INPUT_PULLUP);

	pinMode(Pin_ButtonBack.pinNumber, INPUT_PULLUP);
	pinMode(Pin_ButtonStart.pinNumber, INPUT_PULLUP);

	pinMode(Pin_ButtonL3.pinNumber, INPUT_PULLUP);
	pinMode(Pin_ButtonR3.pinNumber, INPUT_PULLUP);

  #if ProcessDpadButtons == 1
	pinMode(Pin_DpadUp.pinNumber, INPUT_PULLUP);
	pinMode(Pin_DpadDown.pinNumber, INPUT_PULLUP);
	pinMode(Pin_DpadLeft.pinNumber, INPUT_PULLUP);
	pinMode(Pin_DpadRight.pinNumber, INPUT_PULLUP);
  #endif

	XInput.setJoystickRange(0, ADC_Max);  // Set joystick range to the ADC
	//XInput.setAutoSend(false);  // Wait for all controls before sending

	XInput.begin();
}

static inline void computeTriggerValue(const XInputMap_Trigger &control_trigger, const uint8_t pin_Trigger) {
	// Read trigger buttons
	#if UseTriggerButtons == 1
	int triggerValue  = !digitalRead(pin_Trigger) ? ADC_Max : 0;
	#else
	// Read trigger potentiometer values
	int triggerValue = analogRead(pin_Trigger);
	#endif
	
	XInput.setTrigger(control_trigger, triggerValue);
}

void loop() {
	// Read pin values and store in variables
	// (Note the "!" to invert the state, because LOW = pressed)
	Pin_ButtonA.lastState = !digitalRead(Pin_ButtonA.pinNumber);
	Pin_ButtonB.lastState = !digitalRead(Pin_ButtonB.pinNumber);
	Pin_ButtonX.lastState = !digitalRead(Pin_ButtonX.pinNumber);
	Pin_ButtonY.lastState = !digitalRead(Pin_ButtonY.pinNumber);

	Pin_ButtonLB.lastState = !digitalRead(Pin_ButtonLB.pinNumber);
	Pin_ButtonRB.lastState = !digitalRead(Pin_ButtonRB.pinNumber);

	Pin_ButtonBack.lastState  = !digitalRead(Pin_ButtonBack.pinNumber);
	Pin_ButtonStart.lastState = !digitalRead(Pin_ButtonStart.pinNumber);

	Pin_ButtonL3.lastState = !digitalRead(Pin_ButtonL3.pinNumber);
	Pin_ButtonR3.lastState = !digitalRead(Pin_ButtonR3.pinNumber);

  #if ProcessDpadButtons == 1
	Pin_DpadUp.lastState    = !digitalRead(Pin_DpadUp.pinNumber);
	Pin_DpadDown.lastState  = !digitalRead(Pin_DpadDown.pinNumber);
	Pin_DpadLeft.lastState  = !digitalRead(Pin_DpadLeft.pinNumber);
	Pin_DpadRight.lastState = !digitalRead(Pin_DpadRight.pinNumber);
  #endif

	// Set XInput buttons
	XInput.setButton(*XInputController::getButtonFromEnum(BUTTON_A), Pin_ButtonA.lastState);
	XInput.setButton(*XInputController::getButtonFromEnum(BUTTON_B), Pin_ButtonB.lastState);
	XInput.setButton(*XInputController::getButtonFromEnum(BUTTON_X), Pin_ButtonX.lastState);
	XInput.setButton(*XInputController::getButtonFromEnum(BUTTON_Y), Pin_ButtonY.lastState);

	XInput.setButton(*XInputController::getButtonFromEnum(BUTTON_LB), Pin_ButtonLB.lastState);
	XInput.setButton(*XInputController::getButtonFromEnum(BUTTON_RB), Pin_ButtonRB.lastState);

	XInput.setButton(*XInputController::getButtonFromEnum(BUTTON_BACK), Pin_ButtonBack.lastState);
	XInput.setButton(*XInputController::getButtonFromEnum(BUTTON_START), Pin_ButtonStart.lastState);

	XInput.setButton(*XInputController::getButtonFromEnum(BUTTON_L3), Pin_ButtonL3.lastState);
	XInput.setButton(*XInputController::getButtonFromEnum(BUTTON_R3), Pin_ButtonR3.lastState);

  #if ProcessDpadButtons == 1
	// Set XInput DPAD values
	XInput.setDpad(Pin_DpadUp.lastState, Pin_DpadDown.lastState, Pin_DpadLeft.lastState, Pin_DpadRight.lastState);
  #endif

	// Set XInput trigger values

	computeTriggerValue(*XInputController::getTriggerFromEnum(TRIGGER_LEFT), Pin_TriggerL);
	computeTriggerValue(*XInputController::getTriggerFromEnum(TRIGGER_LEFT), Pin_TriggerL);

	// Set left joystick
	#if UseLeftJoystick == 1
  {
		int leftJoyX = analogRead(Pin_LeftJoyX);
		int leftJoyY = analogRead(Pin_LeftJoyY);

		// White lie here... most generic joysticks are typically
		// inverted by default. If the "Invert" variable is false
		// then we'll take the opposite value with 'not' (!).

		XInput.setJoystickX(JOY_LEFT, leftJoyX);
		XInput.setJoystickY(JOY_LEFT, leftJoyY, !InvertLeftYAxis);
  }
	#endif

	// Set right joystick
	#if UseRightJoystick == 1
  {
		int rightJoyX = analogRead(Pin_RightJoyX);
		int rightJoyY = analogRead(Pin_RightJoyY);

		XInput.setJoystickX(JOY_RIGHT, rightJoyX);
		XInput.setJoystickY(JOY_RIGHT, rightJoyY, !InvertRightYAxis);
  }
	#endif

	// TODO: reflex fine-tune
	// pucgenie: without SerialUSB, measuring time is complicated.
	// 13 : 11 should be good, but I don't know.
	OCR1A = XInput.send() == 0 ? 12 : 10;

	#ifndef COMPLETELY_UNTOUCH_TIMER1
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
