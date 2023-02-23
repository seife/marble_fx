/*
 * Logitech TrackMan Marble FX wheel driver
 * also supports Logitech TrackMan Marble model T-BC21 (in ps/2 mode)
 *
 * Copyright © 2018-2021 Stefan Seyfried <seife@tuxbox-git.slipkontur.de>
 *
 * This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * http://www.wtfpl.net/ for more details.
 *
 *  tested on: paradisetronic Pro Micro (mini leonardo-compatible board)
 *             CMCU beetle (nano USB-connector-only leonardo-compatible)
 *             Arduino Leonardo, probably a cheap clone board
 *
 *  PS2++ protocol specs from http://web.archive.org/web/20030714000535/http://dqcs.com/logitech/ps2ppspec.htm
 *
 * based on this:
 *   Arduino Forum > Topics > Device Hacking > Logitech TrackMan Marble FX USB converter
 *   https://forum.arduino.cc/index.php?topic=365472.0
 *
 * SAMPLE_RATE and STREAM_MODE setting idea from
 *   https://github.com/dkao/Logitech_Trackman_Marble_FX_PS2_to_USB_converter
 *
 * depends on these libraries:
 *   HID-Project (https://github.com/NicoHood/HID)
 *
 * The T-BC21 trackball has four buttons. It has an USB cable, but can also speak PS/2
 * We use the PS/2 mode
 *
 *  default HW setup
 *   wire PS/2 connector to arduino PIN 2 (data) and 3 (clk)
 *   see: http://playground.arduino.cc/ComponentLib/Ps2mouse
 *   for the T-BC21, wire USB D- (next to VBUS) is data and USB D+ (next to GND) is clk
 *
 *  driver limitations:
 *   use at your own risk.
 *   super hack. tested on my own TrackMan Marble FX(T-CJ12) only
 *               now also tested on TrackMan Marble (model T-BC21)
 *
 *  functionality:
 *   Marble FX: press red button to emulate wheel movement with the ball
 *   T-BC21: left small button is "red button", scroll wheel emulation,
 *           right small button is "middle button" (button 3)
 */

//# define SERIALDEBUG 1

#ifdef USE_LEGACY_HID
#include "Mouse.h"
#else
#include "HID-Project.h"
#endif

/*
 * Pin definitions
 */
#define DATA_PIN 2
#define CLK_PIN  3
/* configuration input switches */
#define LEFTHAND_PIN 8
#define JIGGLE_PIN   7

/* STREAM_MODE and non-standard SAMPLE_RATE are disabled by default.
 * while it sounds useful at first glance, it results in problems when
 * (really) fast moving the ball, and it does not give considerable
 * benefits in standard usage etiher
 * you can uncomment those defines to use them if you want */
//#define STREAM_MODE
/*
 * Set sample rate.
 * PS/2 default sample rate is 100Hz.
 * Valid sample rates: 10, 20, 40, 60, 80, 100, 200
 */
//#define SAMPLE_RATE 200

/* will be set from switches on pins 7 and 8 */
bool lefthanded = false;
bool jiggler = true;
/* global variables */
uint8_t xtrabutton = 0;
bool buttons[3] = { false, false, false };
// lucky us, the definitions of MOUSE_LEFT,_RIGHT,_MIDDLE are also 1,2,4...
uint8_t bmask[3] = { 0x01, 0x02, 0x04 };
int scroll_sum = 0;

void led_invert(void)
{
  static bool led = HIGH; /* start with LED on */
  digitalWrite(LED_BUILTIN, led);
  led = !led;
}

/*
 * https://www.arduino.cc/reference/en/language/functions/digital-io/pinmode/
 * correctly set the mouse clock and data pins for
 * various conditions.
 */
void setpin(int pin, bool value)
{
  if (value) {
    pinMode(pin, INPUT_PULLUP);
  } else {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
}

void mouse_write(uint8_t data)
{
  uint8_t i;
  uint8_t parity = 1;

  /* put pins in output mode */
  setpin(DATA_PIN, HIGH);
  setpin(CLK_PIN, HIGH);
  delayMicroseconds(300);
  setpin(CLK_PIN, LOW);
  delayMicroseconds(300);
  setpin(DATA_PIN, LOW);
  delayMicroseconds(10);
  /* start bit */
  setpin(CLK_PIN, HIGH);
  /* wait for mouse to take control of clock); */
  while (digitalRead(CLK_PIN) == HIGH)
    ;
  /* clock is low, and we are clear to send data */
  for (i=0; i < 8; i++) {
    if (data & 0x01) {
      setpin(DATA_PIN, HIGH);
    } else {
      setpin(DATA_PIN, LOW);
    }
    /* wait for clock cycle */
    while (digitalRead(CLK_PIN) == LOW)
      ;
    while (digitalRead(CLK_PIN) == HIGH)
      ;
    parity = parity ^ (data & 0x01);
    data >>= 1;
  }
  /* parity */
  setpin(DATA_PIN, parity);
  while (digitalRead(CLK_PIN) == LOW)
    ;
  while (digitalRead(CLK_PIN) == HIGH)
    ;
  /* stop bit */
  setpin(DATA_PIN, HIGH);
  delayMicroseconds(50);
  while (digitalRead(CLK_PIN) == HIGH)
    ;
  /* wait for mouse to switch modes */
  while ((digitalRead(CLK_PIN) == LOW) || (digitalRead(DATA_PIN) == LOW))
    ;
  /* put a hold on the incoming data. */
  setpin(CLK_PIN, LOW);
}

/*
 * Get a byte of data from the mouse
 * ret is the return code (true if data was delivered, false if timeout)
 * timeout is 1000 millis
 * timeout reporting is needed so that we can block in stream mode, but
 * the mouse jiggler can still do its job ;-)
 */
uint8_t mouse_read(bool *ret = NULL)
{
  uint8_t data = 0x00;
  int i;
  uint8_t bit = 0x01;

  if (ret)
    *ret = true;
  setpin(CLK_PIN, HIGH);
  setpin(DATA_PIN, HIGH);
  delayMicroseconds(50);
  long start = millis();
  while (digitalRead(CLK_PIN) == HIGH) {
    if (ret && (millis() - start) > 1000) {
       *ret = false;
       goto out;
    }
  }
  delayMicroseconds(5);               /* debounce */
  while (digitalRead(CLK_PIN) == LOW) /* eat start bit */
    ;
  for (i=0; i < 8; i++) {
    while (digitalRead(CLK_PIN) == HIGH)
      ;
    if (digitalRead(DATA_PIN) == HIGH) {
      data = data | bit;
    }
    while (digitalRead(CLK_PIN) == LOW)
      ;
    bit <<= 1;
  }
  /* eat parity bit, (ignored) */
  while (digitalRead(CLK_PIN) == HIGH)
    ;
  while (digitalRead(CLK_PIN) == LOW)
    ;
  /* eat stop bit */
  while (digitalRead(CLK_PIN) == HIGH)
    ;
  while (digitalRead(CLK_PIN) == LOW)
    ;

out:
  /* stop incoming data. */
  setpin(CLK_PIN, LOW);
  return data;
}

void mouse_init()
{
  setpin(CLK_PIN, HIGH);
  setpin(DATA_PIN, HIGH);
  delay(250);    /* allow mouse to power on */
  /* reset */
  mouse_write(0xff);
  mouse_read();  /* ack byte */
  mouse_read();  /* blank */
  mouse_read();  /* blank */
  led_invert();  /* led off to see we passsed first init */
#ifdef SAMPLE_RATE
  mouse_write(0xf3);  /* sample rate */
  mouse_read();  /* ack */
  mouse_write(SAMPLE_RATE);
  mouse_read();  /* ack */
#endif
#ifndef STREAM_MODE
    mouse_write(0xf0);  /* remote mode */
    mouse_read();  /* ack */
    delayMicroseconds(100);
#endif
}

void mouse_enable_report()
{
  mouse_write(0xf4); /* enable report */
  mouse_read(); /* ack */
  delayMicroseconds(100);
}

// PS2++, extended ps/2 protocol spec.
// http://web.archive.org/web/20030714000535/http://dqcs.com/logitech/ps2ppspec.htm
// also, linux kernel ps2 mouse drivers have extensive code to look up the protocol.
static uint8_t magic[] = { 0xe8, 0x00, 0xe8, 0x03, 0xe8, 0x02, 0xe8, 0x01, 0xe6, 0xe8, 0x03, 0xe8, 0x01, 0xe8, 0x02, 0xe8, 0x03 };
void ps2pp_write_magic_ping()
{
  /* e8 00 e8 03 e8 02 e8 01 e6 e8 03 e8 01 e8 02 e8 03 */
  for (uint8_t i = 0; i < sizeof(magic); i++)
    mouse_write(magic[i]);
}

bool ps2pp_decode(uint8_t b0, uint8_t b1, uint8_t b2)
{
  /* values from linux/drivers/input/mouse/logips2pp.c */
  if ((b0 & 0x48) != 0x48 || (b1 & 0x02) != 0x02)
    return false;
  // mouse extra info
  if ((b0 & 0x30) == 0x0 && (b1 & 0xf0) == 0xd0) {
    xtrabutton = (b2 & 0x30);
#ifdef SERIALDEBUG
    Serial.print("xtrabutton: ");
    Serial.println((int)xtrabutton, HEX);
#endif
  }
  return true;
}

/* the main() program code */
void setup()
{
  pinMode(JIGGLE_PIN, INPUT_PULLUP);
  pinMode(LEFTHAND_PIN, INPUT_PULLUP);
  jiggler =    (digitalRead(JIGGLE_PIN) == HIGH);  /* default on if pin open */
  lefthanded = (digitalRead(LEFTHAND_PIN) == LOW); /* default off */
#ifdef SERIALDEBUG
  Serial.begin(115200); /* baudrate does not matter */
  delay(100);
  while(! Serial) {};
  Serial.println("HELLO!");
  Serial.print("Jiggler:\t");
  Serial.println(jiggler);
  Serial.print("Lefthanded:\t");
  Serial.println(lefthanded);
#endif
  pinMode(LED_BUILTIN, OUTPUT);
  led_invert();
  mouse_init();
  ps2pp_write_magic_ping();
#ifdef STREAM_MODE
    mouse_enable_report();
#endif
  Mouse.begin();
}

long last_move = 0;
int jigglecount = 0;

void move(int8_t x, int8_t y, int8_t z)
{
  Mouse.move(x, y, z);
  last_move = millis();
  jigglecount = 0;
}

/*
 * mstat bit 0 = 0x01 left;
 * mstat bit 1 = 0x02 right;
 * xtra  bit 4 = 0x10 small left on Marble T-BC21, red button on Marble FX;
 * xtra  bit 5 = 0x20 small right on Marble T-BC21;
 * red button will be scroll, 0x20 will be mapped to 0x04 => middle button
 * return value: bit 0,1,2 = left, right, middle, bit 4 = scroll
 * if lefthanded == true, then buttons will be swapped (only useful with T-BC21)
 */
uint8_t map_buttons(uint8_t mstat, uint8_t xtra)
{
  uint8_t ret = 0;
  if (! lefthanded) {
    ret = mstat & 0x07; /* standard left/right/middle buttons */
    if (xtra & 0x20)
      ret |= 0x04;
    if (xtra & 0x10)
      ret |= 0x10; /* scroll button */
  } else { /* invert */
    if (mstat & 0x01)
      ret = 0x02;
    if (mstat & 0x02)
      ret |= 0x01;
    if (xtra & 0x10)
      ret |= 0x04;
    if (xtra & 0x20)
      ret |= 0x10; /* scroll button */
  }
  return ret;
}

void loop()
{
  bool ret;
  /* update the switch state.
     Does this even make sense at run time? but it does not hurt anyway ;-) */
  jiggler =    (digitalRead(JIGGLE_PIN) == HIGH);  /* default on if pin open */
  lefthanded = (digitalRead(LEFTHAND_PIN) == LOW); /* default off */
  led_invert();
#ifndef STREAM_MODE
    mouse_write(0xeb);  /* give me data! */
    mouse_read();      /* ignore ack */
#endif
  uint8_t mstat = mouse_read(&ret);
#ifdef STREAM_MODE
  if (ret) { /* no timeout */
#endif
    int8_t mx    = (int8_t)mouse_read();
    int8_t my    = (int8_t)mouse_read();
#ifdef SERIALDEBUG
    Serial.print((int)mstat, HEX);
    Serial.print("\t");
    Serial.print((int)mx);
    Serial.print("\t");
    Serial.println((int)my);
#endif
    if (ps2pp_decode(mstat, mx, my) || USBDevice.isSuspended())
      return; // do nothing.

    uint8_t btn = map_buttons(mstat, xtrabutton);
    bool redbutton = btn & 0x10;
    if (redbutton) { /* translate y scroll into wheel-scroll */
      int8_t scroll = my / 8;
      if (! scroll) {
        scroll_sum += my;
        scroll = scroll_sum / 8;
      }
      if (scroll != 0) {
        scroll_sum = 0;
#ifdef SERIALDEBUG
        Serial.print("SCRL ");
        Serial.println((int)scroll);
#endif
        move(0, 0, scroll);
      }
    } else {
      /* -my to get the direction right... */
      if (mx != 0 || my != 0) {
        move(mx, -my, 0);
#ifdef SERIALDEBUG
        Serial.print("MOVE ");
        Serial.print((int)mx);
        Serial.print(" ");
        Serial.println((int)my);
#endif
      }
      scroll_sum = 0;
    }

    /* handle normal buttons */
    for (uint8_t i = 0; i < sizeof(buttons); i++) {
      bool button = btn & bmask[i];
      if (!buttons[i] && button)
        Mouse.press(bmask[i]);
      else if (buttons[i] && !button)
        Mouse.release(bmask[i]);
      buttons[i] = button;
    }

#ifndef STREAM_MODE
      delay(20);
#endif
#ifdef STREAM_MODE
  }
#endif
  if (! jiggler)
    return;

  long  jiggle = (millis() - last_move);
  if (jiggle > 30000L * (jigglecount + 1) && jiggle < 1800000) {
    jigglecount++;
    if (!USBDevice.isSuspended()) {
#ifdef SERIALDEBUG
      Serial.print("JIGGLE! ");
      Serial.print(jiggle);
      Serial.print(" ");
      Serial.println(jigglecount);
#endif
      Mouse.move(1,0,0);
      Mouse.move(-1,0,0);
    }
  }
}
