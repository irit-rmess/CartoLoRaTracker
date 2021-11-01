// CartoLoRaTracker: a M5stack-based (ESP32) tracker node to evaluate the coverage of a LoRa network, using RawLoRa messages
// Authors: Adrien van den Bossche <vandenbo@irit.fr>, Anaïs Thillaye <Anais.Thillaye@irit.fr>
// Code under GPLv3 licence

#include <Locapack.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <math.h>
#include <M5Stack.h>
#include <WiFi.h>
#include <WiFiMulti.h> 
#include "AsyncUDP.h"
#include <PubSubClient.h>
#include "Display.h"
#include <SD.h>
#include "cartolora_logo.h"

const char* ssid = "xxx";
const char* password = "xxx";
const char* mqtt_server = "loraserver.tetaneutral.net";
bool wifiConnSuccess = false;
const int mqttPort = 1883; 
int buzzerActive = 0;

#define NODE_ADDRESS 0x1234
#define LOCAPACK_PACKET_PERIOD_NORMAL 20000
#define LOCAPACK_PACKET_PERIOD_FAST 6000
#define UPDATE_LCD_PERIOD 1000

#define RFM95_CS 5 // M5-stack
#define RFM95_DIO0 36 // M5-stack

//#define GNSS_UDP
//#define GNSS_UDP_PORT 3333
#define GNSS_SERIAL Serial2

#define MAX_RAWLORA_DATA_LEN 64 // Maximum RawLoRa frame size (from radiohead)
#define MQTT_BUFFER_SIZE 4096
#define SPEAKER_BEEP_DURATION 100
#define BEEPS_SIZE 64

RH_RF95 rf95(RFM95_CS, RFM95_DIO0);
WiFiMulti WiFiMulti;
WiFiClient espClient;
AsyncUDP udp;
PubSubClient mqttClient(espClient);
TinyGPSPlus gps;
Locapack locapack;
Locapack::universalGnssPacket_t gnss;
Locapack::locallyReferencedPacket_t locRef;
bool gnss_valid = false;
uint32_t gnss_age = 0;
uint32_t timetosendlocapack = 0;
uint32_t timeToUpdateLcd = 0;
uint8_t rawLoRaFrame[MAX_RAWLORA_DATA_LEN];
uint8_t* rawLoRaSqn = &rawLoRaFrame[0];
uint8_t* rawLoRaPayload = &rawLoRaFrame[0];
uint8_t rawLoRaHeaderSize = 0;
uint32_t _rgb_timeout = 0;
uint16_t nodeAddress = NODE_ADDRESS;
long signalBandwidth = 125000;
uint8_t spreadingFactor = 10;
uint8_t spreadingFactorsList[] = {7, 8, 9, 10, 11, 12};
uint8_t codingRate4 = 5;
float frequency = 867.7;
float frequencyList[] = {867.1, 867.3, 867.5, 867.7, 867.9, 868.1, 868.3, 868.5};
int8_t txPower = 14;
char mqttTopicPub[1024];
char mqttTopicSub[1024];
int rawLoRaSenderFastMode = 0;

typedef struct {
  uint16_t freq;
  uint32_t duration;
} beep_t;

beep_t beeps[32];
int beeps_read, beeps_write = 0;


const int protocol_version = 1160;
const int packet_type = 1 ;

// SD management
int bandwith = signalBandwidth/1000;
byte hour,minute,sec;
int month,day,year;
const int chipSelect = 25;
File file;
static uint32_t sqn = 0;
uint8_t buffer[64];
int tmp;


void setup(void)
{
  M5.begin();
  M5.Power.begin();
  Serial.print("Starting ...\r\n");
  DisplayInit();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(20, 0);
  M5.Lcd.print("CartoLoraTracker");
  M5.Lcd.drawXBitmap(60, 40, (const uint8_t *)cartolora_logo, CARTOLORA_LOGO_WIDTH, CARTOLORA_LOGO_HEIGHT, TFT_WHITE);
  delay(1000);
  beeps_init();

#ifdef GNSS_SERIAL
  GNSS_SERIAL.begin(9600);
#endif

  M5.Lcd.fillRect(60, 40, CARTOLORA_LOGO_WIDTH, CARTOLORA_LOGO_HEIGHT, TFT_BLACK);

  CoverScrollText("Connecting to Wifi", TFT_WHITE);
  wifiSetup();
  if(wifiConnSuccess)
  {
    CoverScrollText("Connecting to MQTT", TFT_WHITE);
    mqttSetup();
  }else
  {
    CoverScrollText("No WiFi", TFT_WHITE);
    delay(1000);
  }

  CoverScrollText("Initializing SD card", TFT_WHITE);
  if(SD.begin(chipSelect))
    CoverScrollText("SD init done", TFT_WHITE);
  else 
    CoverScrollText("SD init failed", TFT_WHITE);
  delay(1000);

  CoverScrollText("Configuring raw LoRa", TFT_WHITE);
  rawLoRaSetup();

#ifdef GNSS_UDP
  if (udp.listen(GNSS_UDP_PORT)) {
    udp.onPacket([](AsyncUDPPacket packet) {
      while (packet.available()) {
        char c = packet.read();
        gps_process(c);
      }
    });
  }
#endif

  locapack.setdevice_id16(nodeAddress);
  randomSeed(nodeAddress);
  timetosendlocapack = LOCAPACK_PACKET_PERIOD_NORMAL + random(LOCAPACK_PACKET_PERIOD_NORMAL);

  M5.Lcd.setTextSize(2);
  M5.Lcd.fillScreen(BLACK);
}

void loop(void)
{
  uint8_t len = 0;
  uint16_t srcMacAddr;
  char mqtt_payload_buffer[MQTT_BUFFER_SIZE];

#ifdef GNSS_SERIAL
  // Get chars from GPS receiver
  while (GNSS_SERIAL.available())
  {
    char c = GNSS_SERIAL.read();
    gps_process(c);
  }
#endif

  // Wifi and MQTT
  if(wifiConnSuccess)
  {
    mqttReconnect();
    mqttClient.loop(); 
  }

  // Short press on button A to mute or unmute speaker 
  if (M5.BtnA.wasReleased()) {
    buzzerActive = !buzzerActive;
    M5.update();
  }

  // Short press on button B to enable/disable fast mode and reschedule next message
  if (M5.BtnB.wasReleased()) {
    rawLoRaSenderFastMode = !rawLoRaSenderFastMode;
    if ( rawLoRaSenderFastMode ) timetosendlocapack = millis() + LOCAPACK_PACKET_PERIOD_FAST;
    else timetosendlocapack = millis() + LOCAPACK_PACKET_PERIOD_NORMAL;
    M5.update(); 
  }

  // Update speaker
  beeps_engine();

  // Time to send RawLoRa packet
  if (millis() > timetosendlocapack)
  {
    if ( rawLoRaSenderFastMode ) timetosendlocapack = millis() + LOCAPACK_PACKET_PERIOD_FAST + random(1000);
    else timetosendlocapack = millis() + LOCAPACK_PACKET_PERIOD_NORMAL + random(1000);

    if (gnss_valid)
    {
      gnss.altitude_present = true;
      gnss.dop_present = true;
      gnss.valid_gnss_position = true;
      Serial.print("I create a new packet with position recorded ");
      Serial.print(millis() - gnss_age);
      Serial.println("ms ago.");
      rawLoRaPayload[0] = '1';
      rawLoRaPayload[1] = '0';
      int len = locapack.createUniversalGnssPacket(&gnss, &rawLoRaPayload[2]);
      len += 2;
      *rawLoRaSqn++;
      //for (int i=0; i<len+rawLoRaHeaderSize; i++) Serial.printf("|%02x",rawLoRaFrame[i]); Serial.println("|");
      //spreadingFactor = spreadingFactorsList[random(sizeof(spreadingFactorsList))];
      //Serial.println(spreadingFactor);
      //rf95.setSpreadingFactor(spreadingFactor); delay(100);
      frequency = frequencyList[random(sizeof(frequencyList)/sizeof(frequencyList[0]))];
      Serial.println(frequency);
      rf95.setFrequency(frequency); delay(100);
      rf95.send(rawLoRaFrame, len+rawLoRaHeaderSize);
      rf95.waitPacketSent();
      printGeneratedLocapackPacket(&rawLoRaPayload[2],mqtt_payload_buffer);
      if(wifiConnSuccess)
      {
        mqttClient.publish(mqttTopicPub, mqtt_payload_buffer);
      }
      ecriture_sd();
      Serial.println(mqtt_payload_buffer);
      if ( buzzerActive )
      {
        beeps_schedule(220, SPEAKER_BEEP_DURATION);
        beeps_schedule(0, SPEAKER_BEEP_DURATION);
      }
    }
  }

  // Update LCD after all blocking actions in the loop
  if ( millis() > timeToUpdateLcd )
  {
    timeToUpdateLcd = millis() + UPDATE_LCD_PERIOD;
    updateLcd();
  }

  M5.update();
}

void wifiSetup()
{
  uint8_t mac[6];
  uint8_t retries = 0;

  WiFiMulti.addAP(ssid, password);
  while (( WiFiMulti.run() != WL_CONNECTED ) && (retries < 6))
  {
    delay ( 500 );
    retries++;
    Serial.print ( "." );
  }
  if(retries < 6)
  {
    wifiConnSuccess = true;
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }

  // Set RawLoRa MAC address from two last WiFi MAC address
  WiFi.macAddress(mac);
  nodeAddress = mac[4]<<8|mac[5];
}


void mqttSetup()
{
  sprintf(mqttTopicPub,"rawlorasender/device_id/%d/sent", nodeAddress);
  sprintf(mqttTopicSub,"rawloraprobe/%d/#", nodeAddress);

  if ( !mqttClient.setBufferSize(MQTT_BUFFER_SIZE) ) Serial.println("Error while setting MQTT buffer size");
  mqttClient.setServer(mqtt_server, mqttPort);
  mqttClient.setCallback(mqttCallback);
  mqttReconnect();
}


void rawLoRaSetup()
{
  while (!rf95.init())
  {
    Serial.println("RF95 init failed");
    delay(1000);
  }

  rf95.setPromiscuous(true);
  rf95.setFrequency(frequency);
  rf95.setPreambleLength(8);
  rf95.setTxPower(txPower);
  rf95.setSyncWord(0x34); // LoRaWAN header -> must be set for being received by LoRaWAN gateways
  rf95.setDisable_RH_RF95_HEADER(true);
  rf95.setModemConfig(RH_RF95::Bw125Cr45Sf256);
  rf95.setSignalBandwidth(signalBandwidth);
  rf95.setSpreadingFactor(spreadingFactor);
  rf95.setCodingRate4(codingRate4);
  
  rawLoRaFrame[0] = 0x41; rawLoRaFrame[1] = 0x88; // Header, packet type
  rawLoRaFrame[2] = 0; // MAC sequence number
  rawLoRaFrame[3] = 0xfe; rawLoRaFrame[4] = 0xca; // PAN identificator
  rawLoRaFrame[5] = 0xff; rawLoRaFrame[6] = 0xff; // Destination is broadcast
  rawLoRaFrame[7] = (nodeAddress)&0xff; rawLoRaFrame[8] = (nodeAddress >> 8)&0xff; // Source address
  rawLoRaHeaderSize = 9;
  rawLoRaSqn = &rawLoRaFrame[2];
  rawLoRaPayload = &rawLoRaFrame[rawLoRaHeaderSize];
}


void mqttCallback(char* topic, byte *payload, unsigned int length)
{
  Serial.println("New MQTT message");
  Serial.print("Topic:");
  Serial.println(topic);
  Serial.print("Payload:");
  Serial.write(payload, length);
  Serial.println();

  if ( buzzerActive )
  {
    beeps_schedule(440, SPEAKER_BEEP_DURATION);
    beeps_schedule(0, SPEAKER_BEEP_DURATION);
  }
}


void mqttReconnect(){
  while (!mqttClient.connected())
  {
    Serial.print("Connecting to MQTT broker... ");
    if (mqttClient.connect("ESP32Client")) {
      Serial.println("connected.");
    }
    else
    {
      Serial.print("failed. Error code=");
      Serial.println(mqttClient.state());
      Serial.println(". Retrying.");
      delay(2000);
    }
  }
  mqttClient.subscribe(mqttTopicSub);
}


void updateLcd(void)
{
  int y=10;
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE);

  M5.Lcd.setCursor(10, y);
  M5.Lcd.print(String("Battery: ") + String(M5.Power.getBatteryLevel()) + String("%"));
  y+=20;
  // Networking information
  M5.Lcd.setCursor(10, y);
  M5.Lcd.print("SSID:");
  M5.Lcd.print(WiFi.SSID());
  y+=20;
  M5.Lcd.setCursor(10, y);
  M5.Lcd.print("MAC:");
  M5.Lcd.print(WiFi.macAddress());
  y+=20;
  M5.Lcd.setCursor(10, y);
  M5.Lcd.print("IP:");
  M5.Lcd.print(WiFi.localIP());
  y+=20;
  M5.Lcd.setCursor(10, y);
  M5.Lcd.print("RawLoRa:");
  M5.Lcd.print(nodeAddress);
  M5.Lcd.print(" (0x");
  M5.Lcd.print(nodeAddress, 16);
  M5.Lcd.print(")");

  // Location from GPS
  y+=30;
  M5.Lcd.setCursor(10, y);
  if (gps.location.isValid())
  {
    M5.Lcd.print("gnss:");
    M5.Lcd.print(gps.location.lat(), 5);
    M5.Lcd.print("/");
    M5.Lcd.print(gps.location.lng(), 5);
  }
  else
  {
    M5.Lcd.print("(wait for GPS fix)");    
  }

  // Date and time
  if (gps.date.isValid() && gps.time.isValid())
  {
    y+=30;
    M5.Lcd.setCursor(10, y);
    M5.Lcd.print(gps.date.day());
    M5.Lcd.print("/");
    if (gps.date.month() < 10) M5.Lcd.print("0");
    M5.Lcd.print(gps.date.month());
    M5.Lcd.print("/");
    M5.Lcd.print(gps.date.year());
    M5.Lcd.print(" ");
    if (gps.time.hour() < 10) M5.Lcd.print("0");
    M5.Lcd.print(gps.time.hour());
    M5.Lcd.print(":");
    if (gps.time.minute() < 10) M5.Lcd.print("0");
    M5.Lcd.print(gps.time.minute());
    M5.Lcd.print(":");
    if (gps.time.second() < 10) M5.Lcd.print("0");
    M5.Lcd.print(gps.time.second());
  }

  y+=30;
  M5.Lcd.setCursor(10, y);
  M5.Lcd.print(frequency);
  M5.Lcd.print("MHz/SF");
  M5.Lcd.print(spreadingFactor);
  
  M5.Lcd.setCursor(35, 220);
  if ( buzzerActive ) M5.Lcd.print(" mute ");
  else M5.Lcd.print("unmute");

  M5.Lcd.setCursor(125, 220);
  if ( rawLoRaSenderFastMode ) M5.Lcd.print("normal");
  else M5.Lcd.print(" fast ");

  M5.update();
}


void gps_process(char c)
{
  if (gps.encode(c))
  {
    // Did a new valid sentence come in?
    if ( gps.location.isValid() )
    {
      gnss.latitude = gps.location.lat();
      gnss.longitude = gps.location.lng();
      gnss.altitude = gps.altitude.meters();
      gnss.dop = gps.hdop.hdop();
      gnss_age = millis();
      gnss_valid = true;
    }
  }
}


void printGeneratedLocapackPacket (uint8_t* buffer, char* str_temp)
{
  Locapack::protocol_version_t protocol_version;
  uint8_t movement_id_presence_flag;
  uint8_t timestamp_presence_flag;
  Locapack::packet_type_t packet_type;
  uint16_t sequence_number;
  uint64_t timestamp;
  uint8_t movement_id;
  Locapack::device_id_t device_id;
  uint8_t payload[128];
  uint8_t payload_len;
  Locapack::locapacket_t locapacket;
  int decoded_payload_len;
  int ret;
  char header_data[512];
  char payload_data[512];
  char physical_data[512];
  char timestamp_str[24];

  ret = locapack.decodePacket(&protocol_version, &movement_id_presence_flag, &timestamp_presence_flag, &packet_type, &sequence_number, &timestamp, &movement_id, &device_id, payload, &payload_len, buffer);

  printUint64(timestamp, timestamp_str);
  sprintf(header_data,
    "\"protocol_version\":%d"
    ",\"timestamp\":%s"
    ",\"millisSinceUnixEpoch\":%d"
    ",\"packet_type\":%d"
    ",\"sequence_number\":%d"
    ",\"device_id\":%d"
    , protocol_version, timestamp_str, 0, packet_type, sequence_number, nodeAddress);

  switch ( packet_type )
  {
    case Locapack::PACKET_TYPE_UNIVERSAL_GNSS:
      decoded_payload_len = locapack.decodeUniversalGnssPayload(&locapacket.universalGnssPacket.valid_gnss_position,
                            &locapacket.universalGnssPacket.latitude,
                            &locapacket.universalGnssPacket.longitude,
                            &locapacket.universalGnssPacket.altitude_present,
                            &locapacket.universalGnssPacket.altitude,
                            &locapacket.universalGnssPacket.dop_present,
                            &locapacket.universalGnssPacket.dop,
                            payload);
      if (decoded_payload_len == payload_len)
      {
        if (locapacket.universalGnssPacket.valid_gnss_position)
        {
          sprintf(payload_data,    
            "\"latitude\":%.8f"
            ",\"longitude\":%.8f"
            ",\"altitude\":%.1f"
            ",\"dop\":%.3f"
            , locapacket.universalGnssPacket.latitude, locapacket.universalGnssPacket.longitude, locapacket.universalGnssPacket.altitude, locapacket.universalGnssPacket.dop
          );
        }
      }
      break;
  }

  sprintf(physical_data,    
    "\"speed\":%.1f"
    ",\"tx_power\":%d"
    ",\"frequency\":%f"
    ",\"bandwidth\":%.3f"
    ",\"spreadingFactor\":%d"
    , gps.speed.kmph(), txPower, frequency*1000000, signalBandwidth/1000.0, spreadingFactor
  );

  sprintf(str_temp,"{%s,%s,%s}",header_data, payload_data, physical_data);
}


void printUint64(uint64_t value, char* sz)
{
  const int NUM_DIGITS = log10(value) + 1;
  sz[NUM_DIGITS] =  0;
  for ( size_t i = NUM_DIGITS; i--; value /= 10)
  {
    sz[i] = '0' + (value % 10);
  }
}

void ecriture_sd(void)
{
    
   char FILENAME[8];//chaîne de caractères pour le nom des fichier de la carte SD
   char TEST[200];//message qui contiendra date et heure
   char data[200];//message qui contiendra protocol_version, timestamp, millisSinceUnixEpoch, packet_type, sequence_number et device_id
   char data2[200];//message qui contiendra lat, lon, alt, dop et speed
   char data3[200];//message qui contiendra tx_power, freq, bandwith et Spreading factor

   //Déclaration des variables 
   day = gps.date.day();
   month = gps.date.month();
   hour = gps.time.hour();
   sec = gps.time.second();
   minute = gps.time.minute();
   year = gps.date.year();
   int jour, annee;
    
   if (minute == 0 || minute ==10 || minute ==20 || minute == 30 || minute == 40 || minute == 50 ) //verification de la valeur de la variable minute
   {
Serial.println("it's time");
      file.close(); //fermeture de l'ancien fichier
      jour = conv_date(); //appel de la fonction servant à convertir la date en format quantien
      annee = year%10;// extraction du dernier chiffre de l'année
      sprintf(FILENAME,"//%d%d%02d%02d.txt",jour,annee,hour,minute);//mise en forme du nom du fichier à créer
      file = SD.open(FILENAME, FILE_WRITE);//création du fichier

        M5.Speaker.tone(220, SPEAKER_BEEP_DURATION);
        M5.update();
        M5.Speaker.tone(220, -1);
        M5.update();

   }
        
   
   if (file) 
   {
      int millisSinceUnixEpoch =0;  
      int vitesse = gps.speed.kmph();
      //int sequence_number = simplewino.decodeUi16(&buffer[4]);
      //tmp = simplewino.decodeUi40(&buffer[6]);

      int sequence_number = buffer[4]+buffer[5]*256;
      tmp = buffer[6] + buffer[7] << 8 + buffer[8] << 16 + buffer[9] << 24 + buffer[10] << 32;
      
      sprintf(TEST,"{\"Date\":\"%02d/%02d/%d\",\"Heure\":\"%02d:%02d:%02d\",",day,month,year,hour,minute,sec);
      sprintf(data,"\"protocol_version\":%d,\"timestamp\":%d,\"millisSinceUnixEpoch\":%d,\"packet_type\":%d,\"sequence_number\":%d,\"device_id\":%d,",protocol_version,tmp,millisSinceUnixEpoch,packet_type,sequence_number,NODE_ADDRESS);
      sprintf(data2,"\"latitude\":%.8f,\"longitude\":%.8f,\"altitude\":%.2f,\"dop\":%.2f,\"speed\":%d,",(gnss.latitude),(gnss.longitude),(gnss.altitude),(gnss.dop),vitesse);
      sprintf(data3,"\"tx_power\":%d,\"frequency\":%d,\"bandwith\":%d,\"spreadingFactor\":%d}",txPower,frequency,bandwith,spreadingFactor);
      file.print(TEST);
      file.print(data);
      file.print(data2);
      file.println(data3);
Serial.println("file updated");
   }  
}

int conv_date(void)//Fonction pour convertir la date en quantième
{
  int nb = 0;
  for (int i=1; month > i; i++)//nombre de jour entre le début de l'année et le début du mois actuel
    {
      if (i == 1 || i == 3 || i == 5 || i == 7 || i == 8 || i == 10 || i == 15 ) //pour les mois qui ont 31 jours
        nb = nb +31;
      else  
        if (i == 2) // mois de fevrier
          if (year%4 == 0)// année bisextile
            nb=nb+29;
          else // année classique
            nb = nb +28;
            
        else //mois avec 30 jours
          nb = nb +30;
    }
   nb = nb + day; // ajout du nombre de jour depuis le début du mois
   return nb;
}

int conv_millis(void)//Fonction pour obtenir millisSinceUnixEpoch (ne fonctionne pas)
{
  int nb = 1;
  for (int i = 1970; i < year ; i++) // nombre de millis entre 1970 et l'année en cours
  {
    if (i%4 ==0) // année bisextile
      nb = nb + (366 * 24 * 60 * 60* 1000);
    else //année classique
      nb = nb + ( 365 * 24 * 60 * 60* 1000);
  }
  nb = nb + (conv_date()* 24 * 60 *60*1000);//ajout des millis depeuis début de l'année
  nb = nb + (hour * 60 *60 *1000) + (minute *60  *1000 + sec * 1000);//ajout des millis depuis le début du jour actuel
  return nb;
}

void beeps_init (void)
{
  for (int i=0; i<BEEPS_SIZE; i++)
  {
    beeps[i].freq = 0;
    beeps[i].duration = 0;
  }
  beeps_read = 0;
  beeps_write = 0;
}


void beeps_engine (void)
{
  static uint32_t timeout = 0;

  if (millis() > timeout)
  {
    if (beeps_read != beeps_write)
    {
      timeout = millis()+beeps[beeps_read].duration;
      if (beeps[beeps_read].freq == 0) M5.Speaker.mute();
      else M5.Speaker.tone(beeps[beeps_read].freq, beeps[beeps_read].duration);
      if (++beeps_read == BEEPS_SIZE) beeps_read = 0;
    }
  }
}

void beeps_schedule (uint16_t freq, uint32_t duration)
{
  if (++beeps_write == BEEPS_SIZE) beeps_write = 0;
  beeps[beeps_write].freq = freq;
  beeps[beeps_write].duration = duration;
}

