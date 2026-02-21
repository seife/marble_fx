/*
   combined version of the Arduino HID and Mouse libraries

   Copyright (c) 2015, Arduino LLC
   Original code (pre-library): Copyright (c) 2011, Peter Barrett

   combined (for self-containedness) and modified for high resolution
   horizontal / vertical scrolling by Stefan Seyfried, 2026

   Permission to use, copy, modify, and/or distribute this software for
   any purpose with or without fee is hereby granted, provided that the
   above copyright notice and this permission notice appear in all copies.

   THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
   WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
   BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
   OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
   WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
   ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
   SOFTWARE.
 */

#include "HIDMouse.h"

#if defined(USBCON)

static const uint8_t _hidReportDescriptor[] PROGMEM = {
	0x05, 0x01,                       // Usage Page (Generic Desktop)
	0x09, 0x02,                       // Usage (Mouse)
	0xa1, 0x01,                       // Collection (Application)
		0x85, 0x01,               // REPORT ID (1) (Mouse)
		0x09, 0x01,               // Usage (Pointer)
		0xa1, 0x00,               // Collection (Physical)
			// --- Mouse standard buttons / move --- 4 buttons for marble_fx
			0x05, 0x09,       // Usage Page (Button)
			0x19, 0x01,       // Usage Minimum (Button 1)
			0x29, 0x04,       // Usage Maximum (Button 4)
			0x15, 0x00,       // Logical Minimum (0)
			0x25, 0x01,       // Logical Maximum (1)
			0x95, 0x04,       // Report Count (4)
			0x75, 0x01,       // Report Size (1)
			0x81, 0x02,       // Input (Data, Var, Abs)
			0x95, 0x01,       // Report Count (1)
			0x75, 0x04,       // Report Size (4)
			0x81, 0x03,       // Input (Cnst, Var, Abs)
			0x05, 0x01,       // Usage Page (Generic Desktop)
			0x09, 0x30,       // Usage (X)
			0x09, 0x31,       // Usage (Y)
			0x15, 0x81,       // Logical Minimum (-127)
			0x25, 0x7f,       // Logical Maximum (127)
			0x95, 0x02,       // Report Count (2)
			0x75, 0x08,       // Report Size (8)
			0x81, 0x06,       // Input (Data, Var, Rel)
			// --- Wheel with Resolution Multiplier ---
			0xa1, 0x02,       // Collection (Logical)
				0x09, 0x48,       // Usage (Resolution Multiplier)
				0x95, 0x01,       // Report Count (1)
				0x75, 0x02,       // Report Size (2)
				0x15, 0x00,       // Logical Minimum (0)
				0x25, 0x01,       // Logical Maximum (1)
				0x35, 0x01,       // Physical Minimum (1)
				0x45, 0x0c,       // Physical Maximum (12)
				0x55, 0x00,       // Unit Exponent (0)
				0xb1, 0x02,       // Feature (Data, Var, Abs)
				0x35, 0x00,       // Physical Minimum (0)
				0x45, 0x00,       // Physical Maximum (0)
				0x75, 0x06,       // Report Size (6)
				0xb1, 0x01,       // Feature (Constant)
				0x09, 0x38,       // Usage (Wheel)
				0x15, 0x81,       // Logical Minimum (-127)
				0x25, 0x7f,       // Logical Maximum (127)
				0x95, 0x01,       // Report Count (1)
				0x75, 0x08,       // Report Size (8)
				0x81, 0x06,       // Input (Data, Var, Rel)
				0x05, 0x0c,       // Usage Page (Consumer)
				0x0A, 0x38, 0x02, // Usage (AC Pan Horizontal
				0x15, 0x81,       // Logical Minimum (-127)
				0x25, 0x7f,       // Logical Maximum (127)
				0x95, 0x01,       // Report Count (1)
				0x75, 0x08,       // Report Size (8)
				0x81, 0x06,       // Input (Data, Var, Rel)
			0xc0,             //   End Collection
		0xc0,                     //   End Collection
	0xc0                              // End Collection
};


HIDMouse_& HIDMouse()
{
	static HIDMouse_ obj;
	return obj;
}

int HIDMouse_::getInterface(uint8_t* interfaceCount)
{
	*interfaceCount += 1; // uses 1
	HIDDescriptor hidInterface = {
		D_INTERFACE(pluggedInterface, 1, USB_DEVICE_CLASS_HUMAN_INTERFACE, HID_SUBCLASS_NONE, HID_PROTOCOL_NONE),
		D_HIDREPORT(descriptorSize),
		D_ENDPOINT(USB_ENDPOINT_IN(pluggedEndpoint), USB_ENDPOINT_TYPE_INTERRUPT, USB_EP_SIZE, 0x01)
	};
	return USB_SendControl(0, &hidInterface, sizeof(hidInterface));
}

int HIDMouse_::getDescriptor(USBSetup& setup)
{
	// Check if this is a HID Class Descriptor request
	if (setup.bmRequestType != REQUEST_DEVICETOHOST_STANDARD_INTERFACE) { return 0; }
	if (setup.wValueH != HID_REPORT_DESCRIPTOR_TYPE) { return 0; }

	// In a HID Class Descriptor wIndex contains the interface number
	if (setup.wIndex != pluggedInterface) { return 0; }

	int total = 0;
	HIDSubDescriptor* node;
	for (node = rootNode; node; node = node->next) {
		int res = USB_SendControl(TRANSFER_PGM, node->data, node->length);
		if (res == -1)
			return -1;
		total += res;
	}
	
	// Reset the protocol on reenumeration. Normally the host should not assume the state of the protocol
	// due to the USB specs, but Windows and Linux just assumes its in report mode.
	protocol = HID_REPORT_PROTOCOL;
	
	return total;
}

uint8_t HIDMouse_::getShortName(char *name)
{
	name[0] = 'H';
	name[1] = 'I';
	name[2] = 'D';
	name[3] = 'A' + (descriptorSize & 0x0F);
	name[4] = 'A' + ((descriptorSize >> 4) & 0x0F);
	return 5;
}

void HIDMouse_::AppendDescriptor(HIDSubDescriptor *node)
{
	if (!rootNode) {
		rootNode = node;
	} else {
		HIDSubDescriptor *current = rootNode;
		while (current->next) {
			current = current->next;
		}
		current->next = node;
	}
	descriptorSize += node->length;
}

int HIDMouse_::SendReport(uint8_t id, const void* data, int len)
{
	auto ret = USB_Send(pluggedEndpoint, &id, 1);
	if (ret < 0) return ret;
	auto ret2 = USB_Send(pluggedEndpoint | TRANSFER_RELEASE, data, len);
	if (ret2 < 0) return ret2;
	return ret + ret2;
}

bool HIDMouse_::setup(USBSetup& setup)
{
	if (pluggedInterface != setup.wIndex) {
		return false;
	}

	uint8_t request = setup.bRequest;
	uint8_t requestType = setup.bmRequestType;

	if (requestType == REQUEST_DEVICETOHOST_CLASS_INTERFACE)
	{
		if (request == HID_GET_REPORT) {
			getReportCalled++; // DEBUG
			uint8_t multiplier = (uint8_t)setReportVal;
			USB_SendControl(0, &multiplier, 1);
			return true;
		}
		if (request == HID_GET_PROTOCOL) {
			// TODO: Send8(protocol);
			return true;
		}
		if (request == HID_GET_IDLE) {
			// TODO: Send8(idle);
		}
	}

	if (requestType == REQUEST_HOSTTODEVICE_CLASS_INTERFACE)
	{
		if (request == HID_SET_PROTOCOL) {
			// The USB Host tells us if we are in boot or report mode.
			// This only works with a real boot compatible device.
			protocol = setup.wValueL;
			return true;
		}
		if (request == HID_SET_IDLE) {
			idle = setup.wValueL;
			return true;
		}
		if (request == HID_SET_REPORT)
		{
			setReportCalled++; // DEBUG
			if (setup.wValueH == 0x03 && setup.wValueL == 0x01) {
				uint8_t multiplier;
				USB_RecvControl(&multiplier, 1);
				setReportVal = multiplier;
				return true;
			}
			// Make sure to not read more data than USB_EP_SIZE.
			// You can read multiple times through a loop.
			// The first byte (may!) contain the reportID on a multreport.
			//USB_RecvControl(data, length);
		}
	}

	return false;
}

HIDMouse_::HIDMouse_(void) : PluggableUSBModule(1, 1, epType),
                   rootNode(NULL), descriptorSize(0),
                   protocol(HID_REPORT_PROTOCOL), idle(1)
{
	setReportVal = -1;
	setReportCalled = 0;
	getReportCalled = 0;
	epType[0] = EP_TYPE_INTERRUPT_IN;
	PluggableUSB().plug(this);

	static HIDSubDescriptor node(_hidReportDescriptor, sizeof(_hidReportDescriptor));
	AppendDescriptor(&node);
}

int HIDMouse_::begin(void)
{
	return 0;
}

void HIDMouse_::move(signed char x, signed char y, signed char vwheel, signed char hwheel)
{
	uint8_t m[5];
	m[0] = _buttons;
	m[1] = x;
	m[2] = y;
	m[3] = vwheel;
	m[4] = hwheel;
	SendReport(1,m,5);
}

void HIDMouse_::buttons(uint8_t b)
{
	if (b != _buttons)
	{
		_buttons = b;
		move(0,0,0);
	}
}

void HIDMouse_::press(uint8_t b)
{
	buttons(_buttons | b);
}

void HIDMouse_::release(uint8_t b)
{
	buttons(_buttons & ~b);
}

HIDMouse_ Mouse;

#endif /* if defined(USBCON) */
