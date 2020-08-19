# ESP32 NTP Server

forked from DennisSc/PPS-ntp-server

NTP time server implementation using  GPS module as a reference clock. 
## Used Hardware

* Microcontroller: `M5Stack`
* GPS time reference: `g18u8ttl`


## Wiring
```
GPS <-> ESP32
-----------------
GND --> GND
Tx  --> 16 (Rx)
Rx  --> 17 (Tx)
VCC --> +3.3V

