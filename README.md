# GPS-assisted stand-alone NTP server

## What is this any why is it good to have one of this

This device uses a GPS as a time source, can be used as a super-cheap Stratum-1 [NTP](https://en.wikipedia.org/wiki/Network_Time_Protocol) server. My intended use is for ham radio, where in certain digital communication modes, (such as [FT-8](https://physics.princeton.edu/pulsar/k1jt/wsjtx.html)) it is necessary to keep the local clocks of the stations synchronised.  
This is normally done via a decent NTP server over the Internet, usually provided by your ISP or network admin.  
In field days where ham radio operators are in remote locations usually without access to the Internet, this server can be used as a time source.

### How to use it?

Once you assemble the hardware and load the code, the device will create a wifi network called 'NTP Server'. Connect to it, and sync your computer's time with 192.168.4.1.

## How does the code work and what components are required

You need an ESP module: I made this with the NodeMCU board, but you can get it to work anything that uses the ESP8266 processor, as long as you have enough GPIO pins available.

The temperature sensor is a Bosch BME280, and the display is a cheap SSD1306 one, both are using the same I2C bus.  
The GPS I used is an [Adafruit Ultimate GPS](https://www.adafruit.com/product/746), but I have got it to work with other much cheaper modules too. If your GPS doesn't have a PPS output, you still can build and use this device, but it will operate at a reduced precision.  

In my case, the 3.3V LDO can't provide enough current to supply the GPS module and the OLED screen. You may want a separate 3.3V supply to power everything.

You can download the necessary libraries from the Arduino library manager:
- Adafruit Graphics
- Adafruit BME280
- Adafruit SSD1306

The rest (`minmea.c`, `minmea.h`, `DateTime.cpp` and `DateTime.h`) are included.

The following hardware pins are needed:
- `GPS_EN_PIN`, which turns the GPS module on and off, if supported
- `GPS_PPS_PIN`, which should be connected to the PPS output of the GPS, if any.
- `SDApin` and `SCLpin` for the I2C bus
- ...and of course the UART RX pin for the GPS.

**NOTE: Disconnect the UART RX pin from the GPS while uploading firmware!**

## Brief description of the code

After initialising the hardware, the loop function fetches a [$GPRMC NMEA sentence](https://www.gpsinformation.org/dale/nmea.htm) via the built-in UART, and if the sentence is correct, it sets the reference time (with or without the use of the PPS-pin, depending on your configuration).  

In the loop, the code also checks for NTP requests, and if found one, it responds to it. The response times are calculated from the reference time and the number of microseconds derived from CPU cycles since the last time the reference time was set.

The update of the display and the read operation of the sensor is done via timer interrupts. The PPS-pin is also set up as an interrupt.

## Difference from the previous implementations

I got the code to work on the stock Arduino development app, mostly with stock libraries and the stock board support package.  
However, the GPS sentences are now parsed with [minmea](https://github.com/kosma/minmea), which is a really neat library for this application. Instead of the `$GPZDA` sentences, the code parses the `$GPRMC` sentences, so you can use pretty much any GPS module.  

The reference time and NTP functions came from [DennisSc](https://github.com/DennisSc/PPS-ntp-server)'s implementation, only minor things were changed, such as the reported precision and some mnemonics.  

## A note on accuracy

**Do NOT run your data center from this device! Go and buy a proper NTP server with an atomic clock in it!**  

There is no leap-second check and fine tuning is done via a manual set in a `#define` directive. If you decide to use the device in wifi client mode, the timing accuracy will also depend on the network the device is working on.  
The timing data might be erroneous when the 32-bit microsecond counter value overflows (which happens every 71.583 minutes), or when the GPS decides to send back garbage data due to incorrect information or weak satellite signal.  
Nothing checks whether the GPS data is actually realistic, so if you see on the display that it's 18th of January 1972 or 9th of May 2025, this is the time and date the NTP server will report.

According to my measurements, the achievable timing accuracy is about +/- 50 milliseconds 90% confidence interval when the offset is adjusted correctly.
Sometimes, when the server is processing packets rapidly due to hammering, an extra second delay might be introduced. A normal output with `sntp` should be something like:  
```
$ sntp 192.168.4.1
sntp 4.2.8p10@1.3728-o Tue Mar 21 14:36:42 UTC 2017 (136.200.1~2544)
2019-03-07 16:14:05.328536 (-0400) +0.010 +/- 0.008126 192.168.4.1 s1 no-leap
```
If you see that the +/- deviation is in the range of a few hundred milliseconds, try syncing time again until you get a few milliseconds like in the example above.


## What's on the display:

- The first like is showing the network the device is connected to or created, and it alternates with the IP address the NTP server is running on.  

- The big numbers are UTC time

- Under the time in the left column is temperature, air pressure and humidity. If the sensor is not present or it's a different type (the BMP280 does not measure humidity but the BME280 does) these numbers will be zeros.

- Under the time in the right column is the date (YYYY/MM/DD) format, and Unix time.

Note that the display's update and time accuracy is not a priority, so you might find that the colons don't entirely blink in sync with the changing of the seconds digit. This is because the blinking is controlled from the 500 ms timer interrupt, but the time is updated from the GPS. It is possible for the time to freeze and jump on the screen, depending on when the last `$GPRMC` sentence was received.  
Also, when the seconds are 59, the display will 'shake', as a random pixel coordinate offset is added to minimise burn-in. This is also a good way of marking the end of a minute, which comes handy in ham radio contesting.