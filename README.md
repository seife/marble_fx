# USB converter for Trackman MarbleFX
The famous Logitech Trackman MarbleFX trackball has four buttons, of which the 4th (red) button was used to emulate a scroll wheel.
To enable this, the trackball needs to be switched into a special Logitech PS/2 protocol, which enables the reporting of the additional button.
This works well on Linux, but only if the trackball is connected via a native PS/2 port.

If a simple PS/2 <-> USB converter is used, the trackball operates in "legacy" 3-button Mouse mode, because those converters do not know how to switch and interpret the logitech protocol.

This project uses an Arduino Leonardo (or compatible boards, like the CMCU Beetle) to interface the TrackmanFX's PS/2 port to USB HID.

The smallest boards can be integrated directly into the trackball, so that the TrackmanFX appears as a "native" USB device.

One example of such a modification is [here](doc/README.md).
