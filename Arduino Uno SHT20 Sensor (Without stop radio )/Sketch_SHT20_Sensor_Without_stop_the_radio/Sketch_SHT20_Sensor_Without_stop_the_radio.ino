#include <lmic.h>

#include <hal/hal.h>

#include <SPI.h>

#include "DHT.h"

#include "DFRobot_SHT20.h"

//#define DHTTYPE DHT11

//#define DHTPIN 5

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0xFC, 0xBC, 0xE5, 0x2D, 0xCF, 0x24, 0xC7, 0xCF, 0x58, 0x9C, 0x45, 0xF3, 0x0F, 0x5E, 0x68, 0x17 };
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

const unsigned TX_INTERVAL = 1;

unsigned long starttime;
unsigned long cycle_length = TX_INTERVAL * 1000UL; // cycle in secs;

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).


// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7}, //io1 pin is connected to pin 6
};

//Sensor Variables

int moisture = 0;
// temperature in Celcius
int tempC = 0;
// temperature in Fahrenheid
int tempF = 0;

DFRobot_SHT20    sht20;

unsigned int counter = 0; 

// data to send
static uint8_t dataTX[2];


/* **************************************************************
 * setup
 * *************************************************************/
void setup() {
  //Set baud rate
  Serial.begin(115200);
  // Wait (max 10 seconds) for the Serial Monitor
   sht20.initSHT20();
  delay(100);
  sht20.checkSHT20();
  while ((!Serial) && (millis() < 10000)){ }
  Serial.println(F("im Niki"));

  init_node();
  init_sensor();
  
  
  starttime = millis();
}


/* **************************************************************
 * loop
 * *************************************************************/
void loop() {
  
  do_sense();
  
    float humd = sht20.readHumidity();                  // Read Humidity
    float temp = sht20.readTemperature();               // Read Temperature
    Serial.print("Time:");
    Serial.print(millis());
    Serial.print(" Temperature:");
    Serial.print(temp, 1);
    Serial.print("C");
    Serial.print(" Humidity:");
    Serial.print(humd, 1);
    Serial.print("%");
    Serial.println();
    delay(1000);
    
  // check if need to send
  if ((millis() - starttime) > cycle_length) { build_data(); do_send(); starttime = millis(); }
  
}


/* **************************************************************
 * sensor code, typical would be init_sensor(), do_sense(), build_data()
 * *************************************************************/
/* **************************************************************
 * init the sensor
 * *************************************************************/
void init_sensor() {
  sht20.checkSHT20(); // reset sensor
  delay(1000); // give some time to boot up 
}

/* **************************************************************
 * do the reading
 * *************************************************************/
void do_sense() {
  
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  moisture = sht20.readHumidity();
  // Read temperature as Celsius (the default)
  tempC = sht20.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  tempF = sht20.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(moisture) || isnan(tempC) || isnan(tempF)) {
    #ifdef DEBUG   
      Serial.println("Failed to read from DHT sensor!");
    #endif  
    return;
  }

      
   
  #ifdef DEBUG
    Serial.print(F(" temp Celcius:"));
    Serial.print(tempC);
    Serial.print(F(" temp Fahrenheit:"));
    Serial.print(tempF);
    Serial.print(F(" moisture:"));
    Serial.print(moisture);
    Serial.println(F(""));
  #endif
}

/* **************************************************************
 * build data to transmit in dataTX
 *
 * Suggested payload function for this data
 *
 * function Decoder(bytes, port) {
 *  var temp = parseInt(bytes[0]);
 *  var moisture = parseInt(bytes[1]);
 *  
 *  return { temp: temp,
 *           moisture: moisture };
 * }
 * *************************************************************/
void build_data() {
  dataTX[0] = tempC;
  dataTX[1] = moisture;
}

/* **************************************************************
 * radio code, typical would be init_node(), do_send(), etc
 * *************************************************************/
/* **************************************************************
 * init the Node
 * *************************************************************/
void init_node() {
  #ifdef VCC_ENABLE
     // For Pinoccio Scout boards
     pinMode(VCC_ENABLE, OUTPUT);
     digitalWrite(VCC_ENABLE, HIGH);
     delay(1000);
  #endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  LMIC_setLinkCheckMode(1);
  LMIC_setAdrMode(1);

}

/* **************************************************************
 * send the message
 * *************************************************************/
void do_send() {

  Serial.print(millis());
  Serial.print(F(" Sending.. "));  
   

  send_message(&sendjob);

  // wait for send to complete
  Serial.print(millis());
  Serial.print(F(" Waiting.. "));  
      
 
  while ( (LMIC.opmode & OP_JOINING) or (LMIC.opmode & OP_TXRXPEND) ) { os_runloop_once();  }
  Serial.print(millis());
  Serial.println(F(" TX_COMPLETE"));
     
}
  
/* *****************************************************************************
* send_message
* ****************************************************************************/
void send_message(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));    
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, dataTX, sizeof(dataTX), 1);
    Serial.println(F("Packet queued"));      
  }
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.print(millis());
      Serial.print(" - ");
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.print(millis());
      Serial.print(" - ");
      Serial.println(F("EV_JOINED"));

      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      //LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
      break;
    case EV_TXCOMPLETE:
      Serial.print(millis());
      Serial.print(" - ");
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
        Serial.print("txCnt :"); Serial.println(LMIC.txCnt);
        Serial.print("txrxFlags :"); Serial.println(LMIC.txrxFlags);
        Serial.print("dataBeg :"); Serial.println(LMIC.dataBeg);
        for (int i = 0; i < LMIC.dataLen; i++) {
          if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
            Serial.print(F("0"));
          }
          Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
        }
      }
      Serial.println("");
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}


#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

/*  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
 
  //LMIC_setDrTxpow(DR_SF9,20);
  //LMIC.dn2Dr = DR_SF9;
*/
  // Start job (sending automatically starts OTAA too)
  //do_send(&sendjob);
//}

//void loop() {
  //os_runloop_once();
//}
