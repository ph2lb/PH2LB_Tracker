/*******************************************************************************
  Demo program TvB LoRaWAN Node Rev.2 - RFM95W
  Based on Arduino LMIC: https://github.com/matthijskooijman/arduino-lmic

  Serial port: 115200 baud

  Transmitting a LoRaWAN message:
    Blinks 3x blue at startup
    Blinks 10x green when transmitting
    While waiting for response: blinks blue every second
    Green ON when message was send correctly
 

  Add LoRaWAN keys (USE YOUR OWN KEYS):
    Device Address:                         0xFFFFFFFF;
    Network Session Key (default Semtech):  { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
    Application Key (default Semtech):      { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

 *******************************************************************************
 
 Payload function : 
 
 function Decoder(bytes, port) {
  var code = bytes[0];
  var counter = bytes[1];
  var lat = (bytes[2] + (bytes[3] << 8) + (bytes[4] << 16) + (bytes[5] << 24)) / 100000;
  var lng = (bytes[6] + (bytes[7] << 8) + (bytes[8] << 16) + (bytes[9] << 24)) / 100000;
  
  var voltage = bytes[10]/10.0;
  return {
    code:code,
    counter:counter,
    location: {
      lat: lat,
      lng: lng
    },
    
    voltage: voltage
  };
}

 
 
*******************************************************************************/

// Include & enable library functions
#include <TvBLoRaWANNodeRev2.h>
TvBLoRaWANNodeRev2 TvBLoRaWANNodeRev2;
 
#include <TinyGPS++.h>
 
// Include LMIC library (= LoRaWAN protocol stack)
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
 
// THE TINGS NETWORK
static const u4_t DEVADDR = 0x26011403;
static const u1_t PROGMEM NWKSKEY[16] = { 0x18, 0x7F, 0x8E, 0x4B, 0x06, 0x14, 0xFE, 0x13, 0x1C, 0x9D, 0xC5, 0x8A, 0xCF, 0x35, 0xA2, 0x8B };
static const u1_t PROGMEM APPSKEY[16] = { 0xD1, 0xAA, 0x60, 0x76, 0x84, 0x2E, 0x18, 0x77, 0x75, 0x80, 0x9F, 0x9E, 0x3C, 0x02, 0x28, 0x49 };

#define SW1         0   // D00
#define SW2         1   // D01
#define SW3         3   // D03
#define SW4         2   // D02
#define BATTADC     2   // A02
 
#define debugSerial Serial  

#define debugPrintLn(...) { if (debugSerial) debugSerial.println(__VA_ARGS__); }
#define debugPrint(...) { if (debugSerial) debugSerial.print(__VA_ARGS__); }
 
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
 
// Pin mapping for RFM9X
const lmic_pinmap lmic_pins = {
   .nss  = LORAWAN_NSS,
   .rxtx = LMIC_UNUSED_PIN,
   .rst  = LMIC_UNUSED_PIN,
   .dio  = {LORAWAN_DIO0, LORAWAN_DIO1, LORAWAN_DIO2},
};
 
// Restrict to channel0 and SF7 if uncommented; otherwise all channels & SF12
//#define CHANNEL0
 
byte Transmitted = 0;
byte Event_Timeout = 0;
 
byte counter = 0;
 
int_fast32_t prevtimepassed = millis() - 30000; // int to hold the arduino miilis since startup
 
// The TinyGPS++ object
TinyGPSPlus gps;

//----------------------------------------------------------------------------------------
  
// Setup for startup
void setup() 
{ 
  // Init the basic node hardware, blinks 3x blue at startup
  TvBLoRaWANNodeRev2.Init();
  pinMode(SW1, INPUT_PULLUP); 
  pinMode(SW2, INPUT_PULLUP); 
  pinMode(SW3, INPUT_PULLUP); 
  pinMode(SW4, INPUT_PULLUP); 
  analogReference(EXTERNAL); 
  
  // Init message:
  debugPrint("Initializing RFM95W, DevAddr: ");
  debugPrint(DEVADDR, HEX);
 
  Serial1.begin(9600);
 
  // Display channel info
  #ifdef CHANNEL0
  debugPrintLn(" (TX on channel 0 only)\n");
  #else
  debugPrintLn(" (all TX channels active)\n");
  #endif
 
  delay(2000);
}
 
// Main
void loop() {
  bool gpsupdate = false;

  while (Serial1.available() > 0)
  {
    byte data = Serial1.read();
    gpsupdate = gps.encode(data);
    debugPrint((char)data);
  }
 
  int_fast32_t timepassed = millis(); // int to hold the arduino milis since startup  
  if ( prevtimepassed + 30000 < timepassed )
  {
    prevtimepassed = timepassed;
 
    byte sw1 = digitalRead(SW1) == LOW;
    byte sw2 = digitalRead(SW2) == LOW;
    byte sw3 = digitalRead(SW3) == LOW;
    byte sw4 = digitalRead(SW4) == LOW;

    byte code =  sw1 | sw2 << 1 | sw3 << 2;
    uint8_t mydata[11];
    mydata[0] = code;
    mydata[1] = counter++;
    
    mydata[2] = 0;
    mydata[3] = 0;
    mydata[4] = 0;
    mydata[5] = 0;
    
    mydata[6] = 0;
    mydata[7] = 0;
    mydata[8] = 0;
    mydata[9] = 0;
 
    if (gps.location.isValid())
    {
      int32_t lat =   gps.location.lat() * 100000;
      int32_t lon =   gps.location.lng() * 100000;
      debugPrintLn(gpsupdate);     
      debugPrintLn(lat);
      debugPrintLn(lon);
      // Pad 2 int32_t to 6 8uint_t, skipping the last byte (x >> 24)
      mydata[2] = lat;
      mydata[3] = lat >> 8;
      mydata[4] = lat >> 16;
      mydata[5] = lat >> 24;
   
      mydata[6] = lon;
      mydata[7] = lon >> 8;
      mydata[8] = lon >> 16;
      mydata[9] = lon >> 24; 
    }

    if (sw4 == HIGH)
    {
      byte batvalue = (uint8_t)(TvBLoRaWANNodeRev2.Voltage() * 10.0); 
      // use lithium battery
      mydata[10] =  batvalue;
    }
    else
    {
      int batt = analogRead(BATTADC); // max 1023 = 6.6V/2 because ref = 3.3V resize to 0...66
      unsigned int batvaluetmp = batt * 66;
      batvaluetmp = batvaluetmp / 1023;
      byte batvalue = (byte)batvaluetmp; // no problem putting it into a int.
      // use external battery 
      mydata[10] =  batvalue;
    }
    
    TX_Data(mydata, sizeof(mydata)); 

    // Wait for response of the queued message (check if message is send correctly)

      // Needed to check "onEvent" status
      os_runloop_once();

      // Continue until message is transmitted correctly
      while(Transmitted != 1) {
       os_runloop_once();
        // Blink blue while waiting
        TvBLoRaWANNodeRev2.Blink(LED_ONBOARD, 20, 900); // LED_BLUE
        // Add timeout counter when nothing happens:
        Event_Timeout++;
        if (Event_Timeout >= 30) {
          // Timeout when there's no "EV_TXCOMPLETE" event after 30 seconds
          debugPrintLn("\tEvent Timeout, message not transmitted correctly\n");
          TvBLoRaWANNodeRev2.Blink(LED_ONBOARD, 5000, 0); // LED_RED
          break;
        }
      }
      Transmitted = 0;
      TvBLoRaWANNodeRev2.Blink(LED_ONBOARD, 5000, 0); // LED_GREEN
      Event_Timeout = 0;
  }
}
//----------------------------------------------------------------------------------------

// Transmit LoRaWAN message:
void TX_DataString(String New_Message)
{
      // Dispay the transmitted message:
      debugPrint("TX message:\t");
      debugPrintLn(New_Message);
     
      // Calculate message length:
      debugPrint("\tLength: ");
      const uint8_t StringLength = New_Message.length() + 1;
      uint8_t mydata[StringLength];
      debugPrintLn(StringLength);
 
      // Transform message depending on the length:
      for(int i = 0; i <= StringLength; i++)
      {
        mydata[i] = New_Message.charAt(i);
      }
      TX_Data(mydata, StringLength);
}
 
void TX_Data(uint8_t * mydata, uint8_t length)
{
      // Init LMIC
      os_init();

      // Reset the MAC state. Session and pending data transfers will be discarded.
      LMIC_reset();

      // LMIC setup
      #ifdef PROGMEM
      uint8_t appskey[sizeof(APPSKEY)];
      uint8_t nwkskey[sizeof(NWKSKEY)];
      memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
      memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
      LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
      #else
      LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
      #endif
   
      // Channel config
      LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
      LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
      LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
      LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
      LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
      LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
      LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
      LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
      LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);
 
      // For single channel gateways: Restrict to channel 0 when defined above
      #ifdef CHANNEL0
      LMIC_disableChannel(1);
      LMIC_disableChannel(2);
      LMIC_disableChannel(3);
      LMIC_disableChannel(4);
      LMIC_disableChannel(5);
      LMIC_disableChannel(6);
      LMIC_disableChannel(7);
      LMIC_disableChannel(8);
      #endif
       
      // Disable link check validation
      LMIC_setLinkCheckMode(0);

      // Only for TTN network users: SF9 for RX2 window
      //LMIC.dn2Dr = DR_SF9;

      // Set data rate and transmit power
      LMIC_setDrTxpow(DR_SF7,14);

      // Check if there is not a current TX/RX job running before transmitting
      if (LMIC.opmode & OP_TXRXPEND)
      {
        debugPrintLn(F("OP_TXRXPEND, not sending"));
      }
      else
      {
        // Send message
        LMIC_setTxData2(1, mydata, length, 0);
 
        // Display TX channel
        debugPrint("\tChannel: ");
        debugPrintLn(LMIC.txChnl);
 
        debugPrintLn("\tPacket queued");
       
        for (byte i = 0; i < 10; i++)
        {
          TvBLoRaWANNodeRev2.Blink(LED_ONBOARD, 50, 50); // LED_GREEN
        }
      }
}

// When something happens after transmitting (status check):
void onEvent (ev_t ev) {
    debugPrint("\tEvent: ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            debugPrintLn(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            debugPrintLn(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            debugPrintLn(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            debugPrintLn(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            debugPrintLn(F("EV_JOINING"));
            break;
        case EV_JOINED:
            debugPrintLn(F("EV_JOINED"));
            break;
        case EV_RFU1:
            debugPrintLn(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            debugPrintLn(F("EV_JOIN_FAILED"));
            TvBLoRaWANNodeRev2.Blink(LED_ONBOARD, 5000, 0); //  LED_RED
            break;
        case EV_REJOIN_FAILED:
            debugPrintLn(F("EV_REJOIN_FAILED"));
            TvBLoRaWANNodeRev2.Blink(LED_ONBOARD, 5000, 0);//  LED_RED
            break;
        case EV_TXCOMPLETE:
            // Message transmitted
            debugPrintLn(F("EV_TXCOMPLETE (Packet send)"));
            // Set "transmitted OK" status high
            Transmitted = 1;
            // Data received in RX slot after transmission
            if(LMIC.dataLen) {
                debugPrint(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                debugPrintLn();
            }
            break;
        case EV_LOST_TSYNC:
            debugPrintLn(F("EV_LOST_TSYNC"));
            TvBLoRaWANNodeRev2.Blink(LED_ONBOARD, 5000, 0); // LED_RED
            break;
        case EV_RESET:
            debugPrintLn(F("EV_RESET"));
            TvBLoRaWANNodeRev2.Blink(LED_ONBOARD, 5000, 0);  //  LED_RED
            break;
        case EV_RXCOMPLETE:
            // Data received in ping slot
            debugPrintLn(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            debugPrintLn(F("EV_LINK_DEAD"));
            TvBLoRaWANNodeRev2.Blink(LED_ONBOARD, 5000, 0); //  LED_RED
            break;
        case EV_LINK_ALIVE:
            debugPrintLn(F("EV_LINK_ALIVE"));
            break;
         default:
            debugPrintLn(F("Unknown event"));
            TvBLoRaWANNodeRev2.Blink(LED_ONBOARD, 5000, 0); //  LED_RED
            break;
    }
    debugPrintLn("");
}


