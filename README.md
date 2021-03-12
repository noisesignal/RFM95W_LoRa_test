# RFM95W LoRa Test Program

This is a simple application demonstrating how to transmit and receive LoRa messages using the RFsolutions.co.uk "RF-LORA-868" radio modules, which seem to use `SX1272` chips:


Note that these modules don't include an antenna. You can either make one out of a length of wire, or buy one for your target frequency (868MHz in this example). Either way, solder it to the pin marked `ANA` on the silkscreen.


If you get a cheap radio module from the usual bargain-bin sources, it may have an older revision of the chip. You can tell by checking the `RegVersion` register at address `0x42`. Current versions should return `0x22`; if you get `0x11`, this datasheet will apply instead:

https://www.rfsolutions.co.uk/downloads/1463993415RFM95_96_97_98W.pdf

This application targets an `STM32L432KC` "Nucleo-32" board:

https://www.digikey.com/short/zzfznf

The program requires two boards and radios: one to transmit, and one to receive. Un-comment the appropriate `MODE` definition in `src/main.h` to build each half of the application.
