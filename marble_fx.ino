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
 * SAMPLE_RATE and stream_mode setting idea from
 *   https://github.com/dkao/Logitech_Trackman_Marble_FX_PS2_to_USB_converter
 *
 * interrupt routines inspired by the qmk firmware:
 *   https://github.com/qmk/qmk_firmware
 *
 * depends on these libraries:
 *   HID-Project  (https://github.com/NicoHood/HID, install via library manager)
 *   Arduino-GPIO (https://github.com/mikaelpatel/Arduino-GPIO, install from source)
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
//# define LED_DEBUG

#ifdef USE_LEGACY_HID
#include "Mouse.h"
#else
#include "HID-Project.h"
#endif
#include "GPIO.h"

#include <avr/wdt.h>
/*
 * Pin definitions
 */
#define DATA_PIN 2
#define CLK_PIN  3
/* configuration input switches */
#define LEFTHAND_PIN 8
#define JIGGLE_PIN   7

/* int2board(CLK_PIN) == BOARD::D3 */
#define B_CONCAT(x) BOARD::D##x
#define int2board(x) B_CONCAT(x)

GPIO<int2board(DATA_PIN)>     pin_DATA;
GPIO<int2board(CLK_PIN)>      pin_CLK;
GPIO<int2board(LEFTHAND_PIN)> pin_LEFTHAND;
GPIO<int2board(JIGGLE_PIN)>   pin_JIGGLE;
GPIO<int2board(LED_BUILTIN)>  pin_LED;

#ifdef LED_DEBUG
#define LED1_PIN 10
#define LED2_PIN 11
GPIO<int2board(LED1_PIN)> pin_LED1;
GPIO<int2board(LED2_PIN)> pin_LED2;
#define DBG(PIN, FUNC) do { PIN.FUNC; } while (0)
#else
#define DBG(PIN, FUNC) do {} while (0)
#endif

#ifdef SERIALDEBUG
#define DSERIAL(WHAT) Serial.WHAT
#else
#define DSERIAL(WHAT)
#endif

#define pin_high(PIN) do { PIN.input().pullup(); } while (0)
#define pin_low(PIN)  do { PIN.output(); PIN.low(); } while (0)
#define pin_set(PIN, STATE) do { if (STATE) pin_high(PIN); else pin_low(PIN); } while (0)

/* and non-standard SAMPLE_RATE setting is disabled by default.
 * while it sounds useful at first glance, it results in problems when
 * (really) fast moving the ball, and it does not give considerable
 * benefits in standard usage etiher
 * you can uncomment this define to use it if you want
 *
 * Set sample rate.
 * PS/2 default sample rate is 100Hz.
 * Valid sample rates: 10, 20, 40, 60, 80, 100, 200
 */
//#define SAMPLE_RATE 200

class JiggleTimer
{
public:
  JiggleTimer() { start_time = 0; count = 0; enabled = true; };

  void reset(void) { start_time = millis(); count = 0; };
  bool jiggle(void) {
    if (! enabled || count > 60)
      return false;
    unsigned long now = millis();
    if (now - start_time < 30000)
      return false;
    while (now - start_time >= 30000) {
      count++;
      start_time += 30000;
    }
    return count < 60;
  };
  int count;
  bool enabled;
private:
  unsigned long start_time;
};

JiggleTimer jiggletimer;

/* will be set from switches on pins 7 and 8 */
bool lefthanded = false;
/* global variables */
uint8_t xtrabutton = 0;
bool buttons[3] = { false, false, false };
// lucky us, the definitions of MOUSE_LEFT,_RIGHT,_MIDDLE are also 1,2,4...
uint8_t bmask[3] = { 0x01, 0x02, 0x04 };
int8_t scroll_sum = 0;

bool stream_mode = false;

const uint8_t clk_interrupt = digitalPinToInterrupt(3);
uint8_t ps2_error = 0;

class Mousebuffer
{
  /* ring buffer for received bytes */
  #define MBUF_SIZE 16
private:
  uint8_t buf[MBUF_SIZE];
  uint8_t head;
  uint8_t tail;
public:
  Mousebuffer(void) { head = tail = 0; };
  void push(uint8_t data) {
    uint8_t sreg = SREG;
    cli();
    uint8_t next = (head + 1) % MBUF_SIZE;
    if (next != tail) {
      buf[head] = data;
      head = next;
    }
    SREG = sreg;
  };
  uint8_t pull(void) {
    uint8_t ret = 0;
    uint8_t sreg = SREG;
    cli();
    if (head != tail) {
      ret = buf[tail++];
      tail %= MBUF_SIZE;
    }
    SREG = sreg;
    return ret;
  };
  bool empty(void) {
    uint8_t sreg = SREG;
    cli();
    bool ret = (head == tail);
    SREG = sreg;
    return ret;
  };
  void reset(void) {
    uint8_t sreg = SREG;
    cli();
    head = tail = 0;
    SREG = sreg;
  };
};
Mousebuffer mbuf;

enum {
  NONE, START,
  BIT0, BIT1, BIT2, BIT3, BIT4, BIT5, BIT6, BIT7,
  PARITY, STOP
};

void ps2_ISR(void)
{
  static uint8_t bit = NONE;
  static uint8_t data = 0;
  static bool parity = false;

  if (pin_CLK) /* ???, we trigger on FALLING */
    return;

  bit++;
  switch (bit) {
    case START:
      if (pin_DATA)
        goto error;
      break;
    case BIT0...BIT7:
      data >>= 1;
      if (pin_DATA) {
        data |= 0x80;
        parity = parity ^ 1;
      }
      break;
    case PARITY:
      if (pin_DATA == parity)
        goto error;
      break;
    case STOP:
      if (!pin_DATA)
        goto error;
      mbuf.push(data);
      DBG(pin_LED1, low());
      goto done;
      break;
    default:
      goto error;
  }
  return;
error:
  ps2_error = bit;
  DBG(pin_LED1, high());
done:
  bit = NONE;
  data = 0;
  parity = false;
  return;
}

void bus_idle(void)
{
  pin_high(pin_DATA);
  pin_high(pin_CLK);
}

void bus_stop(void)
{
  pin_low(pin_CLK);
  pin_high(pin_DATA);
}

class Timeout
{
public:
  Timeout() {};
  void reset() { start = millis(); };
  bool elapsed() { return (millis() - start) > 500; };
  void die(void) {
    if (! elapsed())
      return;
#ifdef LED_DEBUG
    /* signal the error reset with one second flashing debug LEDs */
    DBG(pin_LED2, high());
    DBG(pin_LED1, low());
    for (uint8_t i = 0; i < 10; i++) {
      delay(100);
      DBG(pin_LED1, toggle());
      DBG(pin_LED2, toggle());
    }
#endif
    wdt_enable(WDTO_30MS);
    while(true) {};
  };
private:
  unsigned long start;
};
Timeout SendTimeout;

void send_bit(bool data)
{
  pin_set(pin_DATA, data);
  /* wait for clock cycle */
  while (!pin_CLK)
    SendTimeout.die();
  while (pin_CLK)
    SendTimeout.die();
}

void mouse_write(uint8_t data)
{
  uint8_t i;
  uint8_t parity = 1;
  SendTimeout.reset();

  detachInterrupt(clk_interrupt);
  /* put pins in output mode */
  bus_idle();
  delayMicroseconds(300);
  pin_low(pin_CLK);
  delayMicroseconds(300);
  pin_low(pin_DATA);
  delayMicroseconds(10);
  DBG(pin_LED1, high());
  /* start bit */
  pin_high(pin_CLK);
  delayMicroseconds(10); /* Arduino-GPIO is too fast, so wait until CLK is actually high */
  /* wait for mouse to take control of clock); */
  while (pin_CLK)
    SendTimeout.die();
  /* clock is low, and we are clear to send data */
  for (i=0; i < 8; i++) {
    send_bit(data & 1);
    parity = parity ^ (data & 0x01);
    data >>= 1;
  }
  /* parity */
  send_bit(parity);
  /* stop bit */
  pin_high(pin_DATA);
  delayMicroseconds(50);
  while (pin_CLK)
    SendTimeout.die();
  /* wait for mouse to switch modes */
  while (! pin_CLK || ! pin_DATA)
    SendTimeout.die();
  DBG(pin_LED1, low());
  bus_idle();             /* enable incoming data, will be handled by ISR */
  delayMicroseconds(10);  /* to allow pin_CLK to settle */
  attachInterrupt(clk_interrupt, ps2_ISR, FALLING);
}

/*
 * check ringbuffer state
 * wait for max ~200ms for byte to arrive, then return
 * normally, this should not take longer than 20ms, but the logitechs seem
 * to take longer... And it should not hurt as it is only a worst-case value.
 */
bool mouse_ready(void)
{
  uint8_t retry = 200;
  while (retry-- && mbuf.empty())
    delay(1);
  bool ret = !mbuf.empty();
  DBG(pin_LED2, write(ret));
  return ret;
}

/* convenience function: fetch a byte with wait */
uint8_t mouse_read()
{
  if (mouse_ready())
    return mbuf.pull();
  return 0;
}

void mouse_init()
{
  bus_idle();
  delay(250);    /* allow mouse to power on */
  /* reset */
  mouse_write(0xff);
  mouse_read();  /* ack */
  delay(400);    /* the reset takes almost 400ms */
  mouse_read();  /* 0xAA */
  mouse_read();  /* Device ID 0x00 */
  pin_LED.toggle();  /* led off to see we passsed first init */
#ifdef SAMPLE_RATE
  mouse_write(0xf3);  /* sample rate */
  mouse_read();  /* ack */
  mouse_write(SAMPLE_RATE);
  mouse_read();  /* ack */
#endif
  if (!stream_mode) {
    mouse_write(0xf0);  /* remote mode */
    mouse_read();  /* ack */
    delayMicroseconds(100);
  }
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

void mouse_setup(void)
{
  mouse_init();
  ps2pp_write_magic_ping();
  if (stream_mode)
    mouse_enable_report();
  /* mouse will return a ps2++ P0 packet, which loop() will handle */
}

bool ps2pp_decode(uint8_t b0, uint8_t b1, uint8_t b2)
{
#ifdef SERIALDEBUG
  if ((b0 & 0x48) == 0x48) {
    uint8_t ptype = ((b0 & 0x30) >> 2) | ((b1 & 0x30) >> 4);
    Serial.print("ps2++ packet type P");
    Serial.print(ptype, HEX);
    switch (ptype) {
      case 0xb:
        {
          if (b2 & 0x80)
            Serial.print(" BTN: 0x");
          else
            Serial.print(" SHP: 0x");
          Serial.print(b2 & 0x7f, HEX);
        }
    }
    Serial.print(" (raw: 0x");
    Serial.print(b0, HEX);
    Serial.print(" 0x");
    Serial.print(b1, HEX);
    Serial.print(" 0x");
    Serial.print(b2, HEX);
    Serial.println(")");
  }
#endif
  /* values from linux/drivers/input/mouse/logips2pp.c */
  if ((b0 & 0x48) != 0x48 || (b1 & 0x02) != 0x02)
    return false;
  // mouse extra info
  if ((b0 & 0x30) == 0x0 && (b1 & 0xf0) == 0xd0) {
    xtrabutton = (b2 & 0x30);
    DSERIAL(print("xtrabutton: "));
    DSERIAL(println((int)xtrabutton, HEX));
  }
  return true;
}

/* the main() program code */
void setup()
{
  Mouse.begin(); /* does not actually "begin" anything but defines USB descriptors */
  pin_low(pin_DATA);
  pin_low(pin_CLK);
  pin_JIGGLE.input().pullup();
  pin_LEFTHAND.input().pullup();
  jiggletimer.enabled = pin_JIGGLE; /* default on if pin open */
  lefthanded = !pin_LEFTHAND; /* default off */
#ifdef SERIALDEBUG
  Serial.begin(115200); /* baudrate does not matter */
  delay(100);
  while(! Serial) {};
  Serial.println("HELLO!");
  Serial.print("Jiggler:\t");
  Serial.println(jiggletimer.enabled);
  Serial.print("Lefthanded:\t");
  Serial.println(lefthanded);
#endif
  pin_LED.output();
  pin_LED.high();
  DBG(pin_LED1, output());
  DBG(pin_LED2, output());
  DBG(pin_LED1, low());
  DBG(pin_LED2, low());
  mouse_setup();
}

void move(int8_t x, int8_t y, int8_t z)
{
  Mouse.move(x, y, z);
  jiggletimer.reset();
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
#ifdef SERIALDEBUG
  if (Serial.available())
    Serial.read(); /* avoid host side blocking if something is typed in terminal */
#endif
  /* update the switch state.
     Does this even make sense at run time? but it does not hurt anyway ;-) */
  jiggletimer.enabled = pin_JIGGLE; /* default on if pin open */
  lefthanded = !pin_LEFTHAND; /* default off */
  pin_LED.toggle();
  if (ps2_error) {
    DSERIAL(println("PS2_ERROR => Reset"));
    detachInterrupt(clk_interrupt);
    bus_stop();
    ps2_error = 0;
    mbuf.reset();
    mouse_setup();
  }
  if (!stream_mode) {
    mouse_write(0xeb);  /* give me data! */
    mouse_read();      /* ignore ack */
  }
  bool ret = mouse_ready();
  if (ret) { /* no timeout */
    uint8_t mstat = mbuf.pull();
    int8_t mx    = (int8_t)mouse_read();
    int8_t my    = (int8_t)mouse_read();
    DSERIAL(print((int)mstat, HEX));
    DSERIAL(print("\t"));
    DSERIAL(print((int)mx));
    DSERIAL(print("\t"));
    DSERIAL(println((int)my));
    if (ps2pp_decode(mstat, mx, my) || USBDevice.isSuspended())
      return; // do nothing.

    if (mstat & 0xC0) { /* the overflow bits are never set on ps2pp */
      ps2_error = mstat;
      DSERIAL(println("OVERFLOW"));
      return; /* will reenter loop() and reset from there */
    }

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
        DSERIAL(print("SCRL "));
        DSERIAL(println((int)scroll));
        move(0, 0, scroll);
      }
    } else {
      /* -my to get the direction right... */
      if (mx != 0 || my != 0) {
        move(mx, -my, 0);
        DSERIAL(print("MOVE "));
        DSERIAL(print((int)mx));
        DSERIAL(print(" "));
        DSERIAL(println((int)my));
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

    if (!stream_mode)
      delay(20);
  }

  if (jiggletimer.jiggle()) {
    if (!USBDevice.isSuspended()) {
      DSERIAL(print("JIGGLE! "));
      DSERIAL(println(jiggletimer.count));
      Mouse.move(1,0,0);
      Mouse.move(-1,0,0);
    }
  }
}
