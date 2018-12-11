/* * Logitech TrackMan Marble FX wheel driver
 *  tested on: paradisetronic Pro Micro (mini leonardo-compatible board)
 *             CMCU beetle (nano USB-connector-only leonardo-compatible)
 *
 *  PS2++ protocol specs from http://web.archive.org/web/20030714000535/http://dqcs.com/logitech/ps2ppspec.htm
 *
 * based on this: 
 *   Arduino Forum > Topics > Device Hacking > Logitech TrackMan Marble FX USB converter
 *   https://forum.arduino.cc/index.php?topic=365472.0
 *
 *  default HW setup
 *   wire PS/2 connector to arduino
 *   see: http://playground.arduino.cc/ComponentLib/Ps2mouse
 *
 *  driver limitations:
 *   use at your own risk.
 *   super hack. tested on my own TrackMan Marble FX(T-CJ12) only
 *
 *  functionality:
 *   press red button to emulate wheel movement with the ball
 */

#include "Mouse.h"

/*
 * Pin definitions
 */
#define DATA_PIN 2
#define CLK_PIN  3

/* global variables */
bool redbutton = false;
bool buttons[3] = { false, false, false };
// lucky us, the definitions of MOUSE_LEFT,_RIGHT,_MIDDLE are also 1,2,4...
char bmask[3] = { 0x01, 0x02, 0x04 };
int scroll_sum = 0;

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

void mouse_write(char data)
{
  char i;
  char parity = 1;

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
 */
char mouse_read(void)
{
  char data = 0x00;
  int i;
  char bit = 0x01;

  setpin(CLK_PIN, HIGH);
  setpin(DATA_PIN, HIGH);
  delayMicroseconds(50);
  while (digitalRead(CLK_PIN) == HIGH)
    ;
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

  /* stop incoming data. */
  setpin(CLK_PIN, LOW);
  return data;
}

void mouse_init()
{
  setpin(CLK_PIN, HIGH);
  setpin(DATA_PIN, HIGH);
  /* reset */
  mouse_write(0xff);
  mouse_read();  /* ack byte */
  mouse_read();  /* blank */
  mouse_read();  /* blank */
  mouse_write(0xf0);  /* remote mode */
  mouse_read();  /* ack */
  delayMicroseconds(100);
}

// PS2++, extended ps/2 protocol spec.
// http://web.archive.org/web/20030714000535/http://dqcs.com/logitech/ps2ppspec.htm
// also, linux kernel ps2 mouse drivers have extensive code to look up the protocol.
static uint8_t magic[] = { 0xe8, 0x00, 0xe8, 0x03, 0xe8, 0x02, 0xe8, 0x01, 0xe6, 0xe8, 0x03, 0xe8, 0x01, 0xe8, 0x02, 0xe8, 0x03 };
void ps2pp_write_magic_ping()
{
  /* e8 00 e8 03 e8 02 e8 01 e6 e8 03 e8 01 e8 02 e8 03 */
  for (char i = 0; i < sizeof(magic); i++)
    mouse_write(magic[i]);
}

bool ps2pp_decode(char b0, char b1, char b2)
{
  if ((b0 & 0x48) != 0x48)
    return false;
  char t = ((b0 & 0x30) << 4) || (b1 & 0x30);
  char data = b2;
  // int check = b1 & 0x0f;
  // if ((check & 0x03 == 2) && (check >> 2) == (data & 0x03)) {
  //   Serial.print("\t valid");
  // }
  // else {
  //   Serial.print("\t ignore");
  // }
  // mouse extra info
  if (t == 1)
    redbutton = (data & 0x10);
  return true;
}

/* the main() program code */
void setup()
{
  mouse_init();
  ps2pp_write_magic_ping();
  Mouse.begin();
}

void loop()
{
  mouse_write(0xeb);  /* give me data! */
  mouse_read();      /* ignore ack */
  char mstat = mouse_read();
  char mx    = mouse_read();
  char my    = mouse_read();

  if (!ps2pp_decode(mstat, mx, my)) {
    if (redbutton) { /* translate y scroll into wheel-scroll */
      char scroll = my / 8;
      if (! scroll) {
        scroll_sum += my;
        scroll = scroll_sum / 8;
      }
      if (scroll != 0) {
        scroll_sum = 0;
        Mouse.move(0, 0, scroll);
      }
    } else {
      /* -my to get the direction right... */
      if (mx != 0 || my != 0) {
        Mouse.move(mx, -my, 0);
      }
      scroll_sum = 0;
    }

    /* handle normal buttons */
    for (char i = 0; i < sizeof(buttons); i++) {
      bool button = mstat & bmask[i];
      if (!buttons[i] && button)
        Mouse.press(bmask[i]);
      else if (buttons[i] && !button)
        Mouse.release(bmask[i]);
      buttons[i] = button;
    }
  }

  delay(20);
}
