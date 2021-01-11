# How I modified my MarbleFX
I used a CJMCU Beetle board, because the integrated USB plug makes it very easy to solder the USB cable due to the large pin spacing.

## This is the MarbleFX before modification:
![The unmodified MarbleFX](01-marble_fx-original.jpg)

## The bottom opened and the interface PCB removed:
(beware, there's a screw hidden below the label sticker)
![Bottom opened](02-marble_fx-open1.jpg)
![PCB removed](03-marble_fx-open2.jpg)

## Close ups of the removed daughter PCB
(you need to solder the pins from the leonardo board to this board)
![PCB closeup 1](04-marble_fx-pcb1.jpg)
![PCB closeup 2](05-marble_fx-pcb2.jpg)

## Soldered the cables (USB and to the MarbleFX PCB) to the beetle
![Beetle USB cable](06-marble_fx-beetle1.jpg)
![Beetle USB+PCB cable](07-marble_fx-beetle2.jpg)

## Now connect it to the daughter PCB
I used a connector from a Logitech USB mouse cable, so that I did not need to cut the original PS/2 cable. So everything can be returned to the original state.
![Beetle + PCB1](08-marble_fx-beetle-pcb1.jpg)
![Beetle + PCB2](09-marble_fx-beetle-pcb2.jpg)

## This is how the Beetle board fits into the MarbleFX
![Mounted](10-marble_fx-beetle-mounted.jpg)
The Beetle is kept in place by the daughter board.

The additional blue and black cables are RESET and GND, stored below the bottom plate, so that reflashing is easier by triggering a board reset, even if the ACM interface has been patched out of the USB core.
