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

#include "XInput.h"

 // AVR Board with USB support
#if defined(USBCON)
	#ifndef USB_XINPUT
		#warning "Non-XInput version selected in boards menu! Using debug print - board will not behave as an XInput device"
	#endif

// Teensy Boards
#elif defined(TEENSYDUINO)
	// Teensy 3.1-3.2:            __MK20DX256__
	// Teensy LC:                 __MKL26Z64__
	// Teensy 3.5:                __MK64FX512__
	// Teensy 3.6:                __MK66FX1M0__
	// Teensy 4.0, 4.1, MicroMod  __IMXRT1062__
	#if  !defined(__MK20DX256__) && !defined(__MKL26Z64__) && \
		 !defined(__MK64FX512__) && !defined(__MK66FX1M0__) && \
		 !defined(__IMXRT1062__)
		#warning "Not a supported board! Must use Teensy 3.1/3.2, LC, 3.5, 3.6, 4.0, 4.1, or MicroMod"
	#elif !defined(USB_XINPUT)
		#warning "USB type is not set to XInput in boards menu! Using debug print - board will not behave as an XInput device"
	#endif /* if supported Teensy board */

// Everything else
#else
	#ifdef USB_XINPUT
		#warning "Unknown board. XInput may not work properly."
	#else
		#error "This board does not support XInput! You must use a USB capable board with the corresponding XInput boards package. See the list of supported boards in the 'extras' folder for more information"
	#endif
#endif /* if supported board */

static const XInputMap_Button Map_DpadUp(2, 0);
static const XInputMap_Button Map_DpadDown(2, 1);
static const XInputMap_Button Map_DpadLeft(2, 2);
static const XInputMap_Button Map_DpadRight(2, 3);
static const XInputMap_Button Map_ButtonStart(2, 4);
static const XInputMap_Button Map_ButtonBack(2, 5);
static const XInputMap_Button Map_ButtonL3(2, 6);
static const XInputMap_Button Map_ButtonR3(2, 7);

static const XInputMap_Button Map_ButtonLB(3, 0);
static const XInputMap_Button Map_ButtonRB(3, 1);
static const XInputMap_Button Map_ButtonLogo(3, 2);
static const XInputMap_Button Map_ButtonA(3, 4);
static const XInputMap_Button Map_ButtonB(3, 5);
static const XInputMap_Button Map_ButtonX(3, 6);
static const XInputMap_Button Map_ButtonY(3, 7);

const XInputController::Range XInputMap_Trigger::outputRange = { 0, 255 };  // uint8_t

static XInputMap_Trigger Map_TriggerLeft(4, new XInputController::Range());
static XInputMap_Trigger Map_TriggerRight(5, Map_TriggerLeft.inputRange);

XInputMap_Trigger * const XInputController::getTriggerFromEnum(XInputControl ctrl) {
	switch (ctrl) {
	case(TRIGGER_LEFT): return &Map_TriggerLeft;
	case(TRIGGER_RIGHT): return &Map_TriggerRight;
	default: return nullptr;
	}
}

const XInputController::Range XInputMap_Joystick::outputRange = { -32768, 32767 };  // int16_t

static XInputMap_Joystick Map_JoystickLeft(6, 7, 8, 9, new XInputController::Range());
static XInputMap_Joystick Map_JoystickRight(10, 11, 12, 13, Map_JoystickLeft.inputRange);

XInputMap_Joystick * const XInputController::getJoyFromEnum(XInputControl ctrl) {
	switch (ctrl) {
	case(JOY_LEFT): return &Map_JoystickLeft;
	case(JOY_RIGHT): return &Map_JoystickRight;
	default: return nullptr;
	}
}

// --------------------------------------------------------
// XInput Rumble Maps                                     |
// (Stores rx index and buffer index for each motor)      |
// --------------------------------------------------------

struct XInputMap_Rumble {
	constexpr XInputMap_Rumble(uint8_t rIndex, uint8_t bIndex)
		: rxIndex(rIndex), bufferIndex(bIndex) {}
	const uint8_t rxIndex     :4;
	const uint8_t bufferIndex :4;
};

static const XInputMap_Rumble RumbleLeft(3, 0);   // Large motor
static const XInputMap_Rumble RumbleRight(4, 1);  // Small motor

// --------------------------------------------------------
// XInput USB Receive Callback                            |
// --------------------------------------------------------

#ifdef USB_XINPUT
static void XInputLib_Receive_Callback() {
	XInput.receive();
}
#endif


// --------------------------------------------------------
// XInputController Class (API)                           |
// --------------------------------------------------------

XInputController::XInputController() :
	tx(), rumble() // Zero initialize arrays
{
	tx[0] = 0x00;  // Set tx message type
	tx[1] = 0x14;  // Set tx packet size (20)
	reset();
#ifdef USB_XINPUT
	XInputUSB::setRecvCallback(XInputLib_Receive_Callback);
	while(this->receive());  // flush USB OUT buffer
#endif
}

void XInputController::begin() {
	// Empty for now
}

const XInputMap_Button * const XInputController::getButtonFromEnum(XInputControl ctrl) {
	switch (ctrl) {
	case(DPAD_UP):      return &Map_DpadUp;
	case(DPAD_DOWN):    return &Map_DpadDown;
	case(DPAD_LEFT):    return &Map_DpadLeft;
	case(DPAD_RIGHT):   return &Map_DpadRight;
	case(BUTTON_A):     return &Map_ButtonA;
	case(BUTTON_B):     return &Map_ButtonB;
	case(BUTTON_X):     return &Map_ButtonX;
	case(BUTTON_Y):     return &Map_ButtonY;
	case(BUTTON_LB):    return &Map_ButtonLB;
	case(BUTTON_RB):    return &Map_ButtonRB;
	case(JOY_LEFT): // pucgenie: scuffed
	case(BUTTON_L3):    return &Map_ButtonL3;
	case(JOY_RIGHT): // pucgenie: scuffed
	case(BUTTON_R3):    return &Map_ButtonR3;
	case(BUTTON_START): return &Map_ButtonStart;
	case(BUTTON_BACK):  return &Map_ButtonBack;
	case(BUTTON_LOGO):  return &Map_ButtonLogo;
	default: return nullptr;
	}
}

void XInputController::setButton(const XInputMap_Button &buttonData, boolean state) {
	if (getButton(buttonData) == state) return;  // Button hasn't changed

	if (state) { tx[buttonData.index] |= (1 << buttonData.mask); }  // Press
	else { tx[buttonData.index] &= ~(1 << buttonData.mask); }  // Release
	newData = true;
	//else {
	//	Range * triggerRange = getRangeFromEnum(button);
	//	if (triggerRange == nullptr) return;  // Not a trigger (or joystick, but the trigger function will ignore that)
	//	setTrigger(button, state ? triggerRange->max : triggerRange->min);  // Treat trigger like a button
	//}
}

void XInputController::setDpad(boolean up, boolean down, boolean left, boolean right, boolean useSOCD) {
	// Simultaneous Opposite Cardinal Directions (SOCD) Cleaner
	if (useSOCD) {
		if (up && down) { down = false; }  // Up + Down = Up
		if (left && right) { left = false; right = false; }  // Left + Right = Neutral
	}

	//const boolean autoSendTemp = autoSendOption;  // Save autosend state
	//autoSendOption = false;  // Disable temporarily

	setButton(*getButtonFromEnum(DPAD_UP), up);
	setButton(*getButtonFromEnum(DPAD_DOWN), down);
	setButton(*getButtonFromEnum(DPAD_LEFT), left);
	setButton(*getButtonFromEnum(DPAD_RIGHT), right);

	//autoSendOption = autoSendTemp;  // Re-enable from option
	//autosend();
}

void XInputController::setTrigger(const XInputMap_Trigger &triggerData, int32_t val) {
	val = rescaleInput(val, *triggerData.inputRange, XInputMap_Trigger::outputRange);
	if (getTrigger(triggerData) == val) return;  // Trigger hasn't changed

	tx[triggerData.index] = val;
	newData = true;
}

void XInputController::setJoystick(const XInputMap_Joystick &joyData, int32_t x, int32_t y) {
	x = rescaleInput(x, *joyData.inputRange, XInputMap_Joystick::outputRange);
	y = rescaleInput(y, *joyData.inputRange, XInputMap_Joystick::outputRange);

	setJoystickDirect(joyData, x, y);
}

void XInputController::setJoystickX(const XInputMap_Joystick &joyData, int32_t x, boolean invert) {
	x = rescaleInput(x, *joyData.inputRange, XInputMap_Joystick::outputRange);
	if (invert) x = invertInput(x, XInputMap_Joystick::outputRange);

	if (getJoystickX(joyData) == x) return;  // Axis hasn't changed

	tx[joyData.x_low] = lowByte(x);
	tx[joyData.x_high] = highByte(x);

	newData = true;
}

void XInputController::setJoystickY(const XInputMap_Joystick &joyData, int32_t y, boolean invert) {
	y = rescaleInput(y, *joyData.inputRange, XInputMap_Joystick::outputRange);
	if (invert) y = invertInput(y, XInputMap_Joystick::outputRange);

	if (getJoystickY(joyData) == y) return;  // Axis hasn't changed

	tx[joyData.y_low] = lowByte(y);
	tx[joyData.y_high] = highByte(y);

	newData = true;
}

void XInputController::setJoystick(const XInputMap_Joystick &joyData, boolean up, boolean down, boolean left, boolean right, boolean useSOCD) {
	const Range &range = XInputMap_Joystick::outputRange;

	int16_t x = 0;
	int16_t y = 0;

	// Simultaneous Opposite Cardinal Directions (SOCD) Cleaner
	if (useSOCD) {
		if (up && down) { down = false; }  // Up + Down = Up
		if (left && right) { left = false; right = false; }  // Left + Right = Neutral
	}
	
	// Analog axis means directions are mutually exclusive. Only change the
	// output from '0' if the per-axis inputs are different, in order to
	// avoid the '-1' result from adding the int16 extremes
	if (left != right) {
		if (left == true) { x = range.min; }
		else if (right == true) { x = range.max; }
	}
	if (up != down) {
		if (up == true) { y = range.max; }
		else if (down == true) { y = range.min; }
	}

	setJoystickDirect(joyData, x, y);
}

void XInputController::setJoystickDirect(const XInputMap_Joystick &joyData, int16_t x, int16_t y) {
	if (getJoystickX(joyData) != x) {
		tx[joyData.x_low] = lowByte(x);
		tx[joyData.x_high] = highByte(x);
		newData = true;
	}

	if (getJoystickY(joyData) != y) {
		tx[joyData.y_low] = lowByte(y);
		tx[joyData.y_high] = highByte(y);
		newData = true;
	}
}

void XInputController::releaseAll() {
	const uint8_t offset = 2;  // Skip message type and packet size
	memset(tx + offset, 0x00, sizeof(tx) - offset);  // Clear TX array
	newData = true;  // Data changed and is unsent
}

boolean XInputController::getButton(const XInputMap_Button &buttonData) const {
	return tx[buttonData.index] & (1 << buttonData.mask);
}

boolean XInputController::getDpad(const XInputMap_Button &dpad) const {
	return getButton(dpad);
}

uint8_t XInputController::getTrigger(const XInputMap_Trigger &triggerData) const {
	return tx[triggerData.index];
}

int16_t XInputController::getJoystickX(const XInputMap_Joystick &joyData) const {
	return (tx[joyData.x_high] << 8) | tx[joyData.x_low];
}

int16_t XInputController::getJoystickY(const XInputMap_Joystick &joyData) const {
	return (tx[joyData.y_high] << 8) | tx[joyData.y_low];
}

uint8_t XInputController::getPlayer() const {
	return player;
}

uint16_t XInputController::getRumble() const {
	return rumble[RumbleLeft.bufferIndex] << 8 | rumble[RumbleRight.bufferIndex];
}

uint8_t XInputController::getRumbleLeft() const {
	return rumble[RumbleLeft.bufferIndex];
}

uint8_t XInputController::getRumbleRight() const {
	return rumble[RumbleRight.bufferIndex];
}

XInputLEDPattern XInputController::getLEDPattern() const {
	return ledPattern;
}

void XInputController::setReceiveCallback(RecvCallbackType cback) {
	recvCallback = cback;
}

boolean XInputController::connected() {
#ifdef USB_XINPUT
	return XInputUSB::connected();
#else
	return false;
#endif
}

//Send an update packet to the PC
int XInputController::send() {
	if (!newData) return 0;  // TX data hasn't changed
	newData = false;
#ifdef USB_XINPUT
	return XInputUSB::send(tx, sizeof(tx));
#else
	printDebug();
	return sizeof(tx);
#endif
}

int XInputController::receive() {
#ifdef USB_XINPUT
	if (XInputUSB::available() == 0) {
		return 0;  // No packet available
	}

	// Grab packet and store it in rx array
	uint8_t rx[8];
	const int bytesRecv = XInputUSB::recv(rx, sizeof(rx));

	// Only process if received 3 or more bytes (min valid packet size)
	if (bytesRecv >= 3) {
		const XInputReceiveType PacketType = (XInputReceiveType) (rx[0]);

		switch (PacketType) {
			// Rumble Packet
			case XInputReceiveType::Rumble: {
				rumble[RumbleLeft.bufferIndex] = rx[RumbleLeft.rxIndex];   // Big weight (Left grip)
				rumble[RumbleRight.bufferIndex] = rx[RumbleRight.rxIndex];  // Small weight (Right grip)
			}
			break;
			// LED Packet
			case XInputReceiveType::LEDs: {
				parseLED(rx[2]);
			}
			break;
		}

		// User-defined receive callback
		if (recvCallback != nullptr) {
			recvCallback(PacketType);
		}
	}

	return bytesRecv;
#else
	return 0;
#endif
}

void XInputController::parseLED(uint8_t leds) {
	if (leds > 0x0D) return;  // Not a known pattern

	ledPattern = (XInputLEDPattern) leds;  // Save pattern
	switch (ledPattern) {
	case(XInputLEDPattern::Off):
	case(XInputLEDPattern::Blinking):
		player = 0;  // Not connected
		break;
	case(XInputLEDPattern::On1):
	case(XInputLEDPattern::Flash1):
		player = 1;
		break;
	case(XInputLEDPattern::On2):
	case(XInputLEDPattern::Flash2):
		player = 2;
		break;
	case(XInputLEDPattern::On3):
	case(XInputLEDPattern::Flash3):
		player = 3;
		break;
	case(XInputLEDPattern::On4):
	case(XInputLEDPattern::Flash4):
		player = 4;
		break;
	default: return;  // Pattern doesn't affect player #
	}
}

int32_t XInputController::rescaleInput(int32_t val, const Range& in, const Range& out) {
	if (val <= in.min) return out.min;  // Out of range -
	if (val >= in.max) return out.max;  // Out of range +
	if (in.min == out.min && in.max == out.max) return val;  // Ranges identical
	return map(val, in.min, in.max, out.min, out.max);
}

int16_t XInputController::invertInput(int16_t val, const Range& range) {
	return range.max - val + range.min;
}

void XInputController::setTriggerRange(int32_t rangeMin, int32_t rangeMax) {
	Range &range_TriggerLeft = *(getTriggerFromEnum(TRIGGER_LEFT)->inputRange);
	range_TriggerLeft.min = rangeMin;
	range_TriggerLeft.max = rangeMax;
	getTriggerFromEnum(TRIGGER_RIGHT)->inputRange = &range_TriggerLeft;
}

void XInputController::setJoystickRange(const int32_t rangeMin, const int32_t rangeMax) {
	Range &range_JoyLeft = *(getJoyFromEnum(JOY_LEFT)->inputRange);
	range_JoyLeft.min = rangeMin;
	range_JoyLeft.max = rangeMax;
	getJoyFromEnum(JOY_RIGHT)->inputRange = &range_JoyLeft;
}

// Resets class back to initial values
void XInputController::reset() {
	// Reset control data (tx)
	releaseAll();  // Clear TX buffer

	// Reset received data (rx)
	player = 0;  // Not connected, no player
	memset((void*) rumble, 0x00, sizeof(rumble));  // Clear rumble values
	ledPattern = XInputLEDPattern::Off;  // No LEDs on

	// Reset rescale ranges
	setTriggerRange(XInputMap_Trigger::outputRange.min, XInputMap_Trigger::outputRange.max);
	setJoystickRange(XInputMap_Joystick::outputRange.min, XInputMap_Joystick::outputRange.max);

	// Clear user-set options
	recvCallback = nullptr;
	//autoSendOption = true;
}

XInputController XInput;
