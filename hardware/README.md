# effinhover hardware list

#### Betaflight board
There are many boards that can be used here, they just need to be running betaflight, and have at least one UART available. The one on the current hover is this one [here](https://betafpv.com/collections/flight-controller-1/products/alienwhoop-f4-brushed-flight-controller), but this is likely overkill

A six-motor board that could be interesting is [here](https://www.banggood.com/fr/Eachine-32bits-F3-Brushed-Flight-Control-Board-Based-On-SP-RACING-F3-EVO-For-Micro-FPV-Frame-p-1076530.html?akmClientCountry=CH&p=LD020411878172015024&cur_warehouse=CN)

A lot of the boards 'stack' and have the flight controller seperate from the power distribution board. This could be useful if we're running, for example, a car or something with servos or external ESCs.

#### Arduino
If you want WIFI, then we would either use an ESP32 (fast) or a ESP8266 (a little slower). The one on the current hover is the HUZZAH32 from adafruit [here](https://www.adafruit.com/product/3405?gclid=Cj0KCQiAvqDiBRDAARIsADWh5Tc5BtZsTGiUqhi3eE-MHOtxFO1m3n9ski37MPf_gij0hS6TomrOwv0aAtMwEALw_wcB). A much lighter board could easily be used, as we don't need the battery management or charging capabilities of this board.

#### Props / motors
Anything will do, but the current over uses the betafpv ones [here](https://betafpv.com/collections/motors)

#### Hovercraft
The current hover uses [these](https://www.dronejunkie.co.uk/tiny-whoover-hovercraft-kit)