Heavily modified version of https://github.com/makermoekoe/Reflow-Controller

Removed unnecessary parts of code (2x PWM controls, etc).

Changed thermocouples to use MAX31855K with Adafruit MAX31855 library.

Now using SH1106 132x64 dispaly (much larger, 1.54" vs 0.96" diagonal).
Product link: https://www.amazon.com/dp/B0CHDSKMBQ
Requires modified u8g2 library: https://github.com/captmicr0/u8g2
