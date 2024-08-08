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
 */

#ifndef XInput_h
#define XInput_h

#include <Arduino.h>

enum XInputControl : uint8_t {
	BUTTON_LOGO = 0,
	BUTTON_A = 1,
	BUTTON_B = 2,
	BUTTON_X = 3,
	BUTTON_Y = 4,
	BUTTON_LB = 5,
	BUTTON_RB = 6,
	BUTTON_BACK = 7,
	BUTTON_START = 8,
	BUTTON_L3 = 9,
	BUTTON_R3 = 10,
	DPAD_UP = 11,
	DPAD_DOWN = 12,
	DPAD_LEFT = 13,
	DPAD_RIGHT = 14,
	TRIGGER_LEFT = 15,
	TRIGGER_RIGHT = 16,
	JOY_LEFT,
	JOY_RIGHT,
};

enum class XInputReceiveType : uint8_t {
	Rumble = 0x00,
	LEDs = 0x01,
};

enum class XInputLEDPattern : uint8_t {
	Off = 0x00,
	Blinking = 0x01,
	Flash1 = 0x02,
	Flash2 = 0x03,
	Flash3 = 0x04,
	Flash4 = 0x05,
	On1 = 0x06,
	On2 = 0x07,
	On3 = 0x08,
	On4 = 0x09,
	Rotating = 0x0A,
	BlinkOnce = 0x0B,
	BlinkSlow = 0x0C,
	Alternating = 0x0D,
};

// --------------------------------------------------------
// XInput Button Maps                                     |
// (Matches ID to tx index with bitmask)                  |
// --------------------------------------------------------

struct XInputMap_Button {
	constexpr XInputMap_Button(const uint8_t i, const uint8_t o)
		: index(i), mask(o) {}
	const uint8_t index :5;
	const uint8_t mask  :3;
};

// Control Input Ranges
struct XIRange { int32_t min; int32_t max; };

// --------------------------------------------------------
// XInput Trigger Maps                                    |
// (Matches ID to tx index)                               |
// --------------------------------------------------------

struct XInputMap_Trigger {
	constexpr XInputMap_Trigger(uint8_t i, XIRange *inputRange)
		: index(i), inputRange(inputRange) {}
	static const XIRange outputRange;
	const uint8_t index;
	XIRange *inputRange;
};

// --------------------------------------------------------
// XInput Joystick Maps                                   |
// (Matches ID to tx x/y high/low indices)                |
// --------------------------------------------------------

struct XInputMap_Joystick {
	constexpr XInputMap_Joystick(uint8_t xl, uint8_t xh, uint8_t yl, uint8_t yh, XIRange *inputRange)
		: x_low(xl), x_high(xh), y_low(yl), y_high(yh), inputRange(inputRange) {}
	static const XIRange outputRange;
	const uint8_t x_low;
	const uint8_t x_high;
	const uint8_t y_low;
	const uint8_t y_high;
	XIRange *inputRange;
};

// --------------------------------------------------------
// XInput Rumble Maps                                     |
// (Stores rx index and buffer index for each motor)      |
// --------------------------------------------------------

struct XInputMap_Rumble {
	constexpr XInputMap_Rumble(const uint8_t rIndex, const uint8_t bIndex)
		: rxIndex(rIndex), bufferIndex(bIndex) {}
	const uint8_t rxIndex     :4;
	const uint8_t bufferIndex :4;
};

class XInputController {
public:
	XInputController();

	void begin();

	//static const XInputMap_Button * getButtonFromEnum(XInputControl ctrl);
	//static const XInputMap_Trigger * getTriggerFromEnum(XInputControl ctrl);
	//static const XInputMap_Joystick * getJoyFromEnum(XInputControl ctrl);

	// Set Control Surfaces
	void setButton(const XInputMap_Button &button, boolean state);

	void setDpad(boolean up, boolean down, boolean left, boolean right, boolean useSOCD=true);

	void setTrigger(const XInputMap_Trigger &triggerData, int32_t val);

	void setJoystick(const XInputMap_Joystick &joyData, int32_t x, int32_t y);
	void setJoystick(const XInputMap_Joystick &joyData, boolean up, boolean down, boolean left, boolean right, boolean useSOCD=true);
	void setJoystickX(const XInputMap_Joystick &joyData, int32_t x, boolean invert=false);
	void setJoystickY(const XInputMap_Joystick &joyData, int32_t y, boolean invert=false);

	void releaseAll();

	// Read Control Surfaces
	boolean getButton(const XInputMap_Button &button) const;
	boolean getDpad(const XInputMap_Button &dpad) const;
	uint8_t getTrigger(const XInputMap_Trigger &trigger) const;
	int16_t getJoystickX(const XInputMap_Joystick &joyData) const;
	int16_t getJoystickY(const XInputMap_Joystick &joyData) const;

	// Received Data
	uint8_t getPlayer() const;  // Player # assigned to the controller (0 is unassigned)

	uint16_t getRumble() const;  // Rumble motors. MSB is large weight, LSB is small
	uint8_t  getRumbleLeft() const;  // Large rumble motor, left grip
	uint8_t  getRumbleRight() const; // Small rumble motor, right grip

	XInputLEDPattern getLEDPattern() const;  // Returns LED pattern type

	// Received Data Callback
	using RecvCallbackType = void(*)(XInputReceiveType packetType);
	void setReceiveCallback(RecvCallbackType);

	// USB IO
	boolean connected();
	int send();
	int receive();


	void setTriggerRange(int32_t rangeMin, int32_t rangeMax);
	void setJoystickRange(int32_t rangeMin, int32_t rangeMax);

	// Setup
	void reset();

	bool isDataUnsent();

	static const XInputMap_Button Map_DpadUp;
	static const XInputMap_Button Map_DpadDown;
	static const XInputMap_Button Map_DpadLeft;
	static const XInputMap_Button Map_DpadRight;
	static const XInputMap_Button Map_ButtonStart;
	static const XInputMap_Button Map_ButtonBack;
	static const XInputMap_Button Map_ButtonL3;
	static const XInputMap_Button Map_ButtonR3;

	static const XInputMap_Button Map_ButtonLB;
	static const XInputMap_Button Map_ButtonRB;
	static const XInputMap_Button Map_ButtonLogo;
	static const XInputMap_Button Map_ButtonA;
	static const XInputMap_Button Map_ButtonB;
	static const XInputMap_Button Map_ButtonX;
	static const XInputMap_Button Map_ButtonY;

	static const XInputMap_Rumble RumbleLeft;
	static const XInputMap_Rumble RumbleRight;

	XIRange triggerInputRange;
	XInputMap_Trigger Map_TriggerLeft;
	XInputMap_Trigger Map_TriggerRight;

	XIRange joystickInputRange;
	XInputMap_Joystick Map_JoystickLeft;
	XInputMap_Joystick Map_JoystickRight;

private:
	// Sent Data
	uint8_t tx[20];  // USB transmit data
	boolean newData;  // Flag for tx data changed
	
	void setJoystickDirect(const XInputMap_Joystick &joyData, int16_t x, int16_t y);

	// Received Data
	volatile uint8_t player;  // Gamepad player #, buffered
	volatile uint8_t rumble[2];  // Rumble motor data in, buffered
	volatile XInputLEDPattern ledPattern;  // LED pattern data in, buffered
	RecvCallbackType recvCallback;  // User-set callback for received data

	void parseLED(uint8_t leds);  // Parse LED data and set pattern/player data

	static int32_t rescaleInput(int32_t val, const XIRange& in, const XIRange &out);
	static int16_t invertInput(int16_t val, const XIRange& range);
};

#endif