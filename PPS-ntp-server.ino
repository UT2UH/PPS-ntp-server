/*
 * This piece of code uses the hardware UART to connect to the GPS, and fetches the time.
 * ntp seconds: Seconds since 01/01/1900!
 */

#include <Wire.h>
#include <time.h>
#include <string.h>


/// ESP-related stuff. You can get these via the board manager
#include <ESP.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiClient.h>
#include <WiFiUDP.h>
#include <ESP8266WebServer.h>
#include <Ticker.h>

// You can get these libraries from the arduino library manager
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)

#include <Adafruit_BME280.h>
 /*
  * IMPORTANT:
  * Make sure you change the address of the BME chip from 0x77 to 0x76 in Adafruit_BME280.h!
  * This one is easy to find.
  */

  
// This is bundled with the Adafruit gfx library.
#include <Fonts/FreeMonoBold12pt7b.h>

// Local stuff.
#include "minmea.h" // from https://github.com/kosma/minmea.git
#include "DateTime.h" // this helps with the NTP time stamp calculations.

/*
 * Hardware pins
 */
 // i2c
#define SDApin 4 // 'D1' on the NodeMCU v3 baord
#define SCLpin 5 // 'D2 on the NodeMCU v3 baord
// GPS
#define GPS_BAUDRATE 9600
#define GPS_EN_PIN 12 // D6 on the NodeMCU v3 board

// If your GPS doesn't have a PPS output, just comment out this line.
#define GPS_PPS_PIN 14 // D5 on the NodeMCU v3 board


/* IMPORTANT:
 * ALWAYS VERIFY TIMING ACCURACY BEFORE USING THIS DEVICE!!!
 *  
 * Since this device doesn't take leap seconds into account, the 'correct' time will be calculated with a simple offset.
 * This offset affects the microsecond counter so fine control can be achieved.
 * NOTE that this only ever can be a positive number!
 * 
 */
#ifdef GPS_PPS_PIN
#define TIMING_OFFSET_US 60000 // Are you using the PPS, you can fine tune this.
#else
#define TIMING_OFFSET_US 500000 // Not using PPS? You will need a larger value.
#endif

/*
 * Library-provided high-level objects
 */
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // There is no reset pin due to my cheapo modules.
Adafruit_BME280 bme_sensor; // Temperature - humidity - air pressure sensor
Ticker sensor_timer; // this controls how often the temperature sensor is being read.
Ticker display_timer; // This controls how often the screen is updated.
WiFiUDP Udp; // UDP handler.

/*
 * Global variables
 */

bool gps_fix_found = false; // True if the GPS returns data
bool can_respond_to_packets = false; // Set to true when the data is parsed, set to false just after sending an NTP packet.
bool update_the_display = false; // This is controlled from a timer.
char there_is_new_data = 0; // This is for the UART
String uart_string; // A GPS string should not be any longer than this.
unsigned int uart_string_length = 0; // This tells how long an NMEA sentence is, in bytes.
char message_sequence_id = 0; // This one is used for cycling between messages on the OLED screen.

struct minmea_sentence_rmc rmc_frame; // $GPRMC frame, after minmea parsed it.
char gps_sentence[MINMEA_MAX_LENGTH]; // Character array, initialised as per the minmea lib.

// these are for keeping time.
unsigned long microsecond_counter = 0; // CPU microseconds
DateTime reference_time; // This is being updated by the GPS
DateTime uart_time; // this is the DateTime structure decoded by the GPS.
DateTime receive_time; // This is set on an incoming NTP request
DateTime transmit_time; // This is set when transmitting the NTP packet.
byte origTimeTs[9]; // The remote host's local time stamp.

float temperature; // This is in celsius
float humidity; // This is in percent
float pressure; // This is in millibars (or hectopascals)
unsigned char x_offset = 0; // this is for the oled screen
unsigned char y_offset = 0; // This is for the oled screen.


// Wi-Fi stuff.
// Comment this line out if you want to run this server on the local network.
#define OPERATE_AS_AP

#ifdef OPERATE_AS_AP
#ifndef APSSID
#define APSSID "NTP Server"
#define APPSK NULL // Password. It's open.
const char* ssid     = APSSID;
const char* password = APPSK;

#define CHANNEL 9 // Wifi channel. Between 1 and 13, to your taste. 2.4 GHz.
#define HIDE_SSID false // Don't hide SSID.
#define MAX_CONNECTION 3 // How many clients we should handle simultaneously. between 0 and 8.
// the IP will be 192.168.4.1. It can be cinfigured further, but I don't think it matters.

#else
#ifndef STASSID
// Change this info with your network's name and password, if you want to use the client mode.
#define STASSID "Species8472"
#define STAPSK  "Project2501!"
#endif
const char* ssid     = STASSID;
const char* password = STAPSK;
String ip = ""; // we use DHCP. Add your IP here, if you want it to be fixed.
#endif
#endif




#define NTP_PORT 123
#define NTP_PACKET_SIZE 48
byte packetBuffer[NTP_PACKET_SIZE];





void setup() {

    // For randomisation, we need this: the ADC is not connected, so it reads noise,
    randomSeed(analogRead(A0));
  
    // Wifi.
    WiFi.disconnect(true); // This re-initialises the wifi.
    #ifdef OPERATE_AS_AP
    // Stand-alone access point.
    WiFi.softAP(APSSID, APPSK, CHANNEL, HIDE_SSID, MAX_CONNECTION);
    
    #else
    // If client, use these.
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
    }
    #endif
  
    

    
    pinMode(GPS_EN_PIN, OUTPUT);
    digitalWrite(GPS_EN_PIN, 0); // Disable the GPS receiver


    // Software Serial
    //gps.begin(GPS_BAUDRATE);

    // GPS: Serial port and enable pin
    Serial.begin(GPS_BAUDRATE); // GPS is connected to the uart's RX pin.
    digitalWrite(GPS_EN_PIN, 1); // This turns on the GPS receiver, if hooked up.

    // GPS: PPS-pin handling, if the PPS-pin is specified.
    #ifdef GPS_PPS_PIN
    pinMode(GPS_PPS_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN), pps_interrupt, FALLING); // Interrupt is triggered on falling edge.
    #endif
    uart_string.reserve(200);

    // i2c
    Wire.begin(SDApin,SCLpin);

    // OLED screen
    oled.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Create the 5 Volts internally, my display is at address 0x3C
    oled.clearDisplay(); // Clears the frame buffer from power-up related garbage
    oled.display(); // Put the framebuffer's contents to the display
    oled.setTextColor(WHITE); // The OLED is monochrome, but the Adafruit GFX library needs this apparently.
    displayText("Waiting for GPS fix.");

    // BME sensor
    bme_sensor.begin();

    // Timer.
    sensor_timer.attach_ms(3117, read_bme_sensor); // Read the sensor out every 3 seconds or so.
    display_timer.attach_ms(500, set_display_to_update); // This starts a display frame update, every 500 milliseconds.

    // Network.
    Udp.begin(NTP_PORT); // start udp server

    // Debug message via uart.
    Serial.println("Hardware initialised, now entering loop.");
}

void loop() {
  /*
   * Do we have a GPS sentence?
   */
  noInterrupts(); // disable interrupts while waiting for a sentence.
  if(Serial.available())
  {
    uart_string = Serial.readStringUntil('\n'); // Read string until carriage return.
    there_is_new_data = 1;
    // Echo the GPS sentence.
    //Serial.println(uart_string);
  }
  interrupts(); // Re-enable interrupts after the string is received.
  /*
   * Parse the GPS sentence.
   */
  if(there_is_new_data)
  {
    //displayText(uart_string); // Display the NMEA sentence.
    uart_string_length = uart_string.length();
    uart_string.toCharArray(gps_sentence, uart_string_length); // Copy the string to a character array so minmea can process it.
    there_is_new_data = 0;
    uart_string = ""; // Clear this string.
    Serial.flush(); // If there was something remaining in the buffer, it's gone now.
    parse_rmc(); // Update the date and time using gps_sentence
    can_respond_to_packets = true; // Now we can respond to NTP requests.
   
  }

  /*
   * Can we update the display?
   * (Ticker calls timer function, sets semaphore, and then execute this statement once.)
   */

  if(update_the_display && gps_fix_found)
  {
    if(gps_fix_found)
    {
      drive_display(); // Update the contents of the screen.
    }
    else
    {
      displayText("Waiting for GPS fix.");
    }
    update_the_display = false; // Make sure this is being executed once per call.
  }

  /*
   * Have we got a time sync request??
   */

    // process NTP requests
    IPAddress remoteIP; // this will store the remote hosts's IP address
    int remotePort; // the port it was sent from

    int packetSize = Udp.parsePacket();

    if (packetSize && gps_fix_found && can_respond_to_packets) // we've got a packet, and there is GPS fix, and we have received new GPS data since the last NTP reply
    {

      /*
       * Process NTP request.
       */

      // Disable interrupts for this time.
      //noInterrupts();
       
      receive_time = get_time_now(); // Log the time the packet came in
      
      
      //store sender ip and port for later use
      remoteIP = Udp.remoteIP();
      remotePort = Udp.remotePort();
    /*
      // Some very useful debug stuff. But it takes time!
      Serial.print("Received UDP packet with ");
      Serial.print(packetSize);
      Serial.print(" bytes size - ");
      Serial.print("SourceIP ");
      
      for (uint8_t i =0; i < 4; i++)
      {
          Serial.print(remoteIP[i], DEC);
          if (i < 3)
          {
              Serial.print(".");
          }
      }
      
      Serial.print(", Port ");
      Serial.println(remotePort);
      
      
      Serial.print("query: ");
      Serial.print(receive_time.toString());
      Serial.print(",");
      Serial.print(receive_time.microsfraction());
      Serial.println();
      */
      
      
      // We've received a packet, read the data from it
      // read the packet into the buffer
      Udp.read(packetBuffer, NTP_PACKET_SIZE);
      
      //get client transmit time (becomes originTime in reply packet)
      for (int i = 0; i < 8; i++)
      {
          origTimeTs[i] = packetBuffer[40+i];
      }

      
      //send NTP reply
      sendNTPpacket(remoteIP, remotePort);
      Serial.print("reply: ");
      Serial.print(transmit_time.toString());
      Serial.print(",");
      Serial.println(transmit_time.microsfraction());
      Serial.println("");
      
      //output "done"
      //updateLCDtime();
      //Serial.println("NTP reply sent.\r\n*********************************");

      //Re-enable the interrupts
      //interrupts();
      can_respond_to_packets == false; // This is set so no NTP responses will be sent until we have new GPS data.
      
    }
}


/*
 * Custom functions
 */

void displayText(String text_to_display)
{
  //This function does the necessary motions, and displays a string on the screen.
  oled.clearDisplay();
  oled.setCursor(random(4), random(4)); // The setcursor randomisation slows down burn-in.
  oled.println(text_to_display);
  oled.display();
}

void parse_rmc()
{
  // This function parses the $GPRMC NMEA sentence. This function works with global variables, and updates the reference_time accordingly.
  switch(minmea_sentence_id(gps_sentence, true))
  {
    // In this function, the boolean value enables GPS sentence checksum verification. 
    case MINMEA_SENTENCE_RMC:
      //displayText("RMC sentence captured.");
      
      if(minmea_parse_rmc(&rmc_frame, gps_sentence))
      {
        gps_fix_found = true;
        // While we are here, save the data to the uart_time, as per the header file.
        //DateTime(uint16_t year, uint16_t month, uint16_t day, uint16_t hour = 0, uint16_t minute = 0, uint16_t second = 0, unsigned long microsfraction = 0);
        uart_time = DateTime((uint16_t)rmc_frame.date.year, (uint16_t)rmc_frame.date.month, (uint16_t)rmc_frame.date.day, (uint16_t)rmc_frame.time.hours, (uint16_t)rmc_frame.time.minutes, (uint16_t)rmc_frame.time.seconds, TIMING_OFFSET_US);
        //uart_time = DateTime((uint16_t)rmc_frame.date.year, (uint16_t)rmc_frame.date.month, (uint16_t)rmc_frame.date.day, (uint16_t)rmc_frame.time.hours, (uint16_t)rmc_frame.time.minutes, (uint16_t)61, (unsigned long)rmc_frame.time.microseconds);    

        // For some reason, my GPS returns dummy data when it can't find fix. Not sure why this happens.
        if(reference_time.unixtime() == (uint32_t)2085978497)
        {
          // well, this will be a problem in 2036. This was written in 2019, so there is time to fix it :)
          gps_fix_found = false;
        }
        
        // Update the microsecond counter too, if no PPS pin is assigned.
        #ifndef GPS_PPS_PIN
        pps_interrupt(); // If the system had a GPS_PPS_PIN, this function would be executed in an interrupt.
        #endif
        //Serial.println("RMC sentence received.");
      }
      else
      {
        // If the parsing failed
        displayText("Bad RMC sentence!");
      }
      break;

    case MINMEA_SENTENCE_GGA:
      //Serial.println("GGA sentence received.");
      break;
    case MINMEA_SENTENCE_GSA:
      //Serial.println("GSA sentence received.");
      break;
    
    case MINMEA_SENTENCE_VTG:
      //Serial.println("VTG sentence received.");
      break;

    case MINMEA_SENTENCE_ZDA:
      //Serial.println("ZDA sentence received.");
      break;

    default:
      //Do nothing.
    break;
      
  }
  
}

void drive_display()
{
  /*
   * This function drives the OLED screen. Again, we are touching global variables, so no need for passing on arguments.
   * I kinda did some formatting, and and font increases. Since I know the screen size and I won't change it, I hard-coded everything.
   */
   // These are required for string formatting. The +1 character is for the \0 character.
  char date_today_string[11]; // Date string to display.
  char time_now_string[9]; // Time string to display.
  char temp_string[4];
  char humidity_string[4];
  char pressure_string[5];
  bool show_colons = false;
  
  if(reference_time.second() == 59)
  {
    // Randomise display position when the seconds just hit 59. This is to prevent OLED burn-in. Not like it really matters :)
    x_offset = random(5);
    y_offset = random(4);
  }
  

  
  oled.clearDisplay();
  // Display the BME sensor data.
  oled.setCursor(0 + x_offset, 36 + y_offset);
  oled.setFont(); // This resores the font.
  sprintf(temp_string, "T=%0.1f", temperature);
  oled.print(temp_string);
  oled.print(char(247)); //Degrees sign.
  oled.print("C");
  
  oled.setCursor(0 + x_offset, 45 + y_offset);
  sprintf(pressure_string, "P=%0.0fhPa", pressure);
  oled.print(pressure_string);
  
  oled.setCursor(0 + x_offset, 53 + y_offset);
  sprintf(humidity_string, "RH=%0.0f%%", humidity);
  oled.print(humidity_string);

  
  // Display time.
  oled.setCursor(0 + x_offset, 0 + y_offset);
  //oled.print("UNIX time:");
  //oled.print(reference_time.unixtime()); // I will put the unix time here once it's ready :) 
  switch(message_sequence_id)
  {
    case 0:
      oled.print("SSID:");
      oled.print(ssid); // Pint SSID.
      #ifdef OPERATE_AS_AP
      // Add the number of connected clients
      oled.print(" (");
      oled.print(WiFi.softAPgetStationNum());
      oled.print(")");
      #endif
      
      show_colons = true; // The colons between hours, minutes and seconds
      message_sequence_id++; // Move to the next message.
      break;
    
    case 1:
      oled.print("SSID:");
      oled.print(ssid); // Pint SSID.
      #ifdef OPERATE_AS_AP
      // Add the number of connected clients
      oled.print(" (");
      oled.print(WiFi.softAPgetStationNum());
      oled.print(")");
      #endif
      show_colons = false;
      message_sequence_id++; // Move to the next message
      break;
    
    case 2:
      oled.print("IP: ");
      #ifdef OPERATE_AS_AP
      oled.print(WiFi.softAPIP());
      #else
      oled.print(WiFi.localIP()); // Display the IP address.
      #endif
      show_colons = true;
      message_sequence_id++;
      break;
    
    case 3:
      oled.print("IP: ");
      #ifdef OPERATE_AS_AP
      oled.print(WiFi.softAPIP());
      #else
      oled.print(WiFi.localIP()); // Display the IP address.
      #endif
      show_colons = false;
      message_sequence_id = 0;
      break;

    default:
      message_sequence_id = 0;
      
  }
  

  
  oled.setCursor(30 + x_offset, 10 + y_offset);
  oled.print("UTC time is:");
  oled.setCursor(8 + x_offset, 27 + y_offset);
  oled.setFont(&FreeMonoBold12pt7b); // This will change the font.
  // Prepare the time string to display.
  if(show_colons)
  {
    sprintf(time_now_string, "%02d:%02d:%02d", reference_time.hour(), reference_time.minute(), reference_time.second());
  }
  else
  {
    sprintf(time_now_string, "%02d %02d %02d", reference_time.hour(), reference_time.minute(), reference_time.second());
  }
  oled.print(time_now_string);

  oled.setFont();
  oled.setCursor(60 + x_offset, 36 + y_offset);
  oled.print(" Today is:");
  
  // You don't like the time format? Change it to your taste here.
  sprintf(date_today_string, "%4d/%02d/%02d", reference_time.year(), reference_time.month(), reference_time.day());
  
  oled.setCursor(60 + x_offset, 45 + y_offset);
  oled.print(date_today_string);

  // I keep this line for debugging.
  oled.setCursor(60 + x_offset, 53 + y_offset);
  oled.print(reference_time.unixtime()); // I will put the unix time here once it's ready :) 
  //oled.print(reference_time.microsfraction());
  //oled.print(WiFi.localIP());

  
  

  // Update screen contents.
  oled.display();

   
}

void read_bme_sensor()
{
  /*
   * This function reads the BME sensor and updates the global variables.
   */
   temperature = bme_sensor.readTemperature(); // celsius
   humidity = bme_sensor.readHumidity(); // percent
   pressure = bme_sensor.readPressure() / 100.0F; // hectopascals
}

DateTime get_time_now(void)
{
  /*
   * this function derives the time from the reference time, and gets the number of microseconds since the last GPS update.
   * It derives time from reference_time, which is set by the GPS, and the PPS interrupt, if any.
   */
    DateTime stuff_to_return; // This is what we are going to return.
    unsigned long current_microsecond_counter = micros(); // Get the number of microseconds
    uint32_t current_reference_time = reference_time.ntptime(); // This gets the time as per NTP
    // This calculates the time difference since the last $GPRMC NMEA sentence in microseconds, and takes a manual offset into account
    unsigned long microsecond_difference = (current_microsecond_counter - microsecond_counter) + TIMING_OFFSET_US;

    /*
    
    // debug stuff
    Serial.print("current_microsecond_counter: ");
    Serial.println(current_microsecond_counter);
    Serial.print("microsecond_counter: ");
    Serial.println(microsecond_counter);
    Serial.print("microsecond_difference: ");
    Serial.println(microsecond_difference);
    */
    // Did the PPS impulses stop happening? Make this crash!
    if( (microsecond_difference > 5000000) || (microsecond_difference > -5000000))
    {
      displayText("The NTP packet could not be sent because no GPS data or PPS pulse was received in the past 5 seconds. Check GPS and reset device!");
      while(1);
    }

    // Did we have a variable overflow?
    if(microsecond_counter > current_microsecond_counter)
    {
      microsecond_difference = 0; // This is going to introduce a an error up to a second at variable overflow.
      //microsecond_difference = (current_microsecond_counter - ((unsigned long)-1 - microsecond_counter)) + TIMING_OFFSET_US;
    }
    // Did we experience time longer than a second? A Lot longer too? Compensate.
    while( (microsecond_difference >= 10000000))
    {
      // If we get a larger than 1 second here, we have a problem.
      microsecond_difference = microsecond_difference - 1000000; // Remove the extra second offset
      //current_reference_time += 1; //...and add it to the time.
      
    }
    

    // Now let's assemble the new DateTime object so we can return it.
    stuff_to_return = DateTime(current_reference_time, microsecond_difference);
    return stuff_to_return;
}

uint64_t DateTimeToNtp64(DateTime time_to_send) 
{ 
    /*
     * This function generates the time information required for the NTP packet.
     */
    uint64_t time_stamp; // 64-bit time stamp.

    time_stamp = (((uint64_t)time_to_send.ntptime()) << 32); // Shove it to the top 32 bits.
    time_stamp |= (uint64_t)(time_to_send.microsfraction() * 4294.967296); // Add the lower 32-bit nibble, which is the precise information

    return (time_stamp);
}

// send NTP reply to the given address
void sendNTPpacket(IPAddress remoteIP, int remotePort) 
{
  /*
   * This function assembles an NTP packet, and sends it back to the host requesting it.
   */
  
  // LI: 0, Version: 4, Mode: 4 (server)
  //packetBuffer[0] = 0b00100100;
  // Not a leap second (LI=0), NTP version: 3, Mode: 4 (server)
  packetBuffer[0] = 0b00011100;

  // Stratum, or type of clock. Since we have the clock derived from a GPS, we are stratum 1, no matter how inaccurate are we.
  packetBuffer[1] = 0b00000001;
  
  // Polling Interval: the log2 value of the maximum interval between souccessive messages
  packetBuffer[2] = 2; // Was 4.

  // Peer Clock Precision
  // log2(sec)
  // 0xF6 <--> -10 <--> 0.0009765625 s
  // 0xF7 <--> -9 <--> 0.001953125 s
  // 0xF8 <--> -8 <--> 0.00390625 s
  // 0xF9 <--> -7 <--> 0.0078125 s
  // 0xFA <--> -6 <--> 0.0156250 s
  // 0xFB <--> -5 <--> 0.0312500 s 
  // 0xFC <--> -4 <--> 0.0625 s 
  // 0xFD <--> -3 <--> 0.125 s

  #ifndef GPS_PPS_PIN
  // report a worse precision if a GPS without PPS output was used.
  packetBuffer[3] = 0xFC;
  #else
  packetBuffer[3] = 0xF6; // the actual clock precision is better, but let's be conservative, this is just a microcontroller!
  #endif
  
  
  // 8 bytes for Root Delay & Root Dispersion
  // Root delay is 0, becuase we got our clock from a GPS.
  packetBuffer[4] = 0; 
  packetBuffer[5] = 0;
  packetBuffer[6] = 0; 
  packetBuffer[7] = 0;
  
  // Root dispersion. Refers to clock frequency tolerance. Well, I guess this is a bit optimistic :)
  packetBuffer[8] = 0;
  packetBuffer[9] = 0;
  packetBuffer[10] = 0;
  packetBuffer[11] = 0x50;
  
  
  // Time source is GPS. The external reference source code is GPS.
  packetBuffer[12] = 71; // G
  packetBuffer[13] = 80; // P
  packetBuffer[14] = 83; // S
  packetBuffer[15] = 0;

  
  // Reference Time.
  uint64_t refT = DateTimeToNtp64(get_time_now()); // This one fetches the current time.
  
  packetBuffer[16] = (int)((refT >> 56) & 0xFF);
  packetBuffer[17] = (int)((refT >> 48) & 0xFF);
  packetBuffer[18] = (int)((refT >> 40) & 0xFF);
  packetBuffer[19] = (int)((refT >> 32) & 0xFF);
  packetBuffer[20] = (int)((refT >> 24) & 0xFF);
  packetBuffer[21] = (int)((refT >> 16) & 0xFF);
  packetBuffer[22] = (int)((refT >> 8) & 0xFF);
  packetBuffer[23] = (int)(refT & 0xFF);
 
  // Origin Time
  //copy old transmit time to origtime 

  for (int i = 24; i < 32; i++)
  {
        packetBuffer[i] = origTimeTs[i-24];
        //Serial.write(origTimeTs[i-24]);
  }

  // write Receive Time to bytes 32-39
  refT = DateTimeToNtp64(receive_time);
  
  packetBuffer[32] = (int)((refT >> 56) & 0xFF);
  packetBuffer[33] = (int)((refT >> 48) & 0xFF);
  packetBuffer[34] = (int)((refT >> 40) & 0xFF);
  packetBuffer[35] = (int)((refT >> 32) & 0xFF);
  packetBuffer[36] = (int)((refT >> 24) & 0xFF);
  packetBuffer[37] = (int)((refT >> 16) & 0xFF);
  packetBuffer[38] = (int)((refT >> 8) & 0xFF);
  packetBuffer[39] = (int)(refT & 0xFF);
  
  
  // get current time + write  as Transmit Time to bytes 40-47
  transmit_time = get_time_now();
  refT = DateTimeToNtp64(transmit_time);
  
  packetBuffer[40] = (int)((refT >> 56) & 0xFF);
  packetBuffer[41] = (int)((refT >> 48) & 0xFF);
  packetBuffer[42] = (int)((refT >> 40) & 0xFF);
  packetBuffer[43] = (int)((refT >> 32) & 0xFF);
  packetBuffer[44] = (int)((refT >> 24) & 0xFF);
  packetBuffer[45] = (int)((refT >> 16) & 0xFF);
  packetBuffer[46] = (int)((refT >> 8) & 0xFF);
  packetBuffer[47] = (int)(refT & 0xFF);
  
  // send reply:
  Udp.beginPacket(remoteIP, remotePort);
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();

}

void set_display_to_update(void)
{
  // This function adjusts a semaphore, which controls the display update. The Adafruit library doesn't seem to like being used from an interrupt function.
  update_the_display = true; // Set this global variable. The display is updated from the loop() function.
}


// This is the interrupt function.
void pps_interrupt()
{
  // Log the microseconds counter, so we know what the time was when the interrupt happened.
  microsecond_counter = micros();
  // This function sets the reference time, every second.
  uint32_t reference_time_to_process_in_ntp = uart_time.ntptime(); // Save the current time as NTP time.
  #ifdef GPS_PPS_PIN
  reference_time_to_process_in_ntp++; // Increase the number of seconds, when using PPS interrupt.
  #endif
  reference_time = DateTime(reference_time_to_process_in_ntp, (unsigned long)TIMING_OFFSET_US); // This updates the global reference time.



  // Print the current time, so we can check the fraction stuff.
  //reference_time.print();
}
