//Jani Raami
//Koodi on tarkoitettu käytettäväksi LoRa-piirissä. Koodilla luetaan lämpötila ja kosteus anturilta sekä 
//LoRa-piirin MAC-osoite. LoRa-piiri lähettää datan eteenpäin LoRa-radioyhteydellä lisäten datapakettiin 
//GPS-sijaintitietonsa. 

//GPS-piiri:                    NEO-6M GPS
//Lämpötila- ja kosteusanturi:  HTU21D
//LoRa-piiri:                   Heltec WiFi LoRa 32

//Kytkentä:
//GPS Vcc     --- LoRa Pin 3V3 (PRG-napin puoleinen)
//GPS GND     --- LoRa Pin GND (PRG-napin puoleinen)
//GPS RXD     --- LoRa Pin 34
//GPS TXD     --- LoRa Pin 35
//GPS PPS     --- (Ei kytketä)
//HTU21D 3.3V --- LoRa Pin 3V3 (PRG-napin puoleinen)
//HTU21D GND  --- LoRa Pin GND (PRG-napin puoleinen)
//HTU21D SDA  --- LoRa Pin 21 (SDA)
//HTU21D SCL  --- LoRa Pin 22 (SCL)
//LoRa-piirin jännitteet USB:llä ulkoiselta akulta

//Yhteysasetukset:
//Kortti:           Heltec_WIFI_LoRa_32
//Flash Frequency:  80 MHz
//Upload Speed:     921600 (Jos ei toimi, kokeile 115200)

//Sisällytetään seuraavat kirjastot koodiin
#include <WiFi.h>                                           //WiFi-verkon käyttö
#include <SPI.h>                                            //Sarjayhteys kommunikointi
#include <LoRa.h>                                           //Radioverkon kommunikointi
#include <U8x8lib.h>                                        //WiFi-LoRa 32 OLED-näyttö
#include <HardwareSerial.h>                                 //LoRan sarjamuotoinen kommunikointi GPS:n kanssa 
#include <TinyGPS++.h>                                      //GPS:n käyttö
#include <Wire.h>                                           //I2C-väylän käyttö
#include <HTU21D.h>                                         //Lämpötila/kosteusanturin käyttö

//Määritetään WiFi-Lora 32:n pinneille nimet ja asetukset
#define SCK       5                                         //GPIO5  -- SX1278's SCK
#define SS        18                                        //GPIO18 -- SX1278's CS
#define RST       14                                        //GPIO14 -- SX1278's RESET
#define MISO      19                                        //GPIO19 -- SX1278's MISO
#define DI0       26                                        //GPIO26 -- SX1278's IRQ(Keskeytyspyyntö)
#define MOSI      27                                        //GPIO27 -- SX1278's MOSI
#define GPS_RX    35                                        //GPIO35 -- GPS vastaanotto
#define GPS_TX    34                                        //GPIO34 -- GPS lähetys
#define SDA       21                                        //GPIO21 -- HTU21D data
#define SCL       22                                        //GPIO22 -- HTU21D kello
#define AnalogPin 39                                        //ADC1_3 -- Analogia pin 3
#define BAND    869.5E6                                     //Käytettävä taajuus 869.5 MHz. Taajuusalueella 869.400 - 869.650 MHz saa säteilyteho olla enintään 500 mW ERP ja toimintasuhde enintään 10 %
#define signalBandwidth 31.25E3                             //Asetetaan kaistanleveydeksi 31.25 kHz
#define PABOOST true                                        //Asetetaan Power Amplifier Boost päälle

// Wifi-Lora 32:n OLED-näytön määrittely
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

//Muuttujien määrittely
static const uint32_t GPSBaud = 9600;                       //GPS:n sarjamuotoisen tiedonsiirtonopeuden määrittely
float temp;                                                 //Lämpötilan muuttuja
float hum;                                                  //Kosteuden muuttuja
float lat;                                                  //Latitudin muuttuja
float lng;                                                  //Longitudin muuttuja
float hdop;                                                 //Hdopin muuttuja
byte MAC[6];                                                //LoRan MAC-osoitteen muuttuja
String MAC_String;                                          //MAC-osoitteen merkkijonomuuttuja
char MAC_char[13];                                          //MAC-osoitteen merkkijonomuuttuja (merkkitaulukko)
int viive;                                                  //Lähetysväli millisekunneissa
bool gpsActive;

TinyGPSPlus gps;                                            //Luodaan TinyGPS++ objekti ja nimetään se gps:ksi
HardwareSerial GPSSerial(1);                                //Luodaan HardwareSerial objekti ja nimetään se GPSSerialiksi     
HTU21D myHTU21D(HTU21D_RES_RH12_TEMP14);                    //Luodaan HTU21D objekti ja nimetään se myHTU21D:ksi

String mac2String(byte ar[]);
static void smartDelay(unsigned long ms);
void gpsOff();
void sendUBX(byte *msg, byte len);

//Setup-koodi ajetaan kerran ohjelman käynnistyessä
void setup()                           
{
  u8x8.begin();                                             //WiFi LoRa 32:n OLED-näytön kommuninoinnin käynnistäminen
  u8x8.setFont(u8x8_font_chroma48medium8_r);                //Asetetaan OLED-näytölle fontti
  Serial.begin(115200);                                     //Sarjayhteyden käynnistäminen nopeudella 115200
  Serial.println();                                         //Tyhjän rivin tulostus 
  SPI.begin(SCK, MISO, MOSI, SS);                           //SPI väylän käynnistys ja pinnien asetukset. Nämä pitää tehdä ennen LoRa.begin komentoa           
  LoRa.setPins(SS, RST, DI0);
  
  
  while (!LoRa.begin(BAND,PABOOST))                         //Käynnistetään LoRa-yhteys asetetulla taajuudella ja PABOOST kytkettynä
  {
    Serial.println("Starting LoRa failed!");                //Tulostetaan sarjayhteydellä, jos LoRa-yhteydenmuodostus epäonnistuu
    u8x8.drawString(0, 0, "LoRa failed");                   //Tulostetaan näyttöön, jos LoRa-yhteydenmuodostus epäonnistuu
    delay(2000);                                            //Odotetaan 2 sekuntia ja yritetään uudestaan
  }                                     
    Serial.println("Starting LoRa OK.");                    //Tulostetaan sarjayhteydellä, jos LoRa-yhteyden muodostus onnistuu
    u8x8.drawString(0, 0, "Starting LoRa OK");              //Tulostetaan OLED-näytölle, jos LoRa-yhteyden muodostus onnistuu
    delay(5000);                                            //Odotetaan 5 sekuntia, jotta OLED-näytön viestit ehtii lukea
    u8x8.clearDisplay();                                    //Tyhjennetään OLED-näyttö 
    u8x8.drawString(0, 0, "LoRa Transmitter"); 
                                                            //Alla olevat neljä LoRa asetusta tulee tehdä LoRa.begin komennon jälkeen
    LoRa.crc();                                             //Asetetaan LoRa-viestin CRC-tarkistuskoodaus ominaisuus päälle
    LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);            //Asetetaan lähetysteho 20 dBm käyttäen PA_BOOST_PINniä
    LoRa.setSpreadingFactor(7);                             //Asetetaan LoRa-yhteyden leviämistekijäksi 7
    LoRa.setSignalBandwidth(signalBandwidth);               //Asetetaan määritelty kaistanleveys
    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);      //Sarjayhteyden käynnistäminen, jotta GPS-piirin kanssa voidaan kommunikoida
    GPSSerial.setTimeout(2);  
    gpsActive = true;
    
    u8x8.drawString(0, 4, "GPS coordinates:");
    while (myHTU21D.begin() != true)                        //Jos yhteydenmuodostus lämpötila/kosteusanturille ei onnistu, niin tulostetaan siitä virheviesti
  {
    Serial.println("HTU21D sensor is failed or not connected");
    delay(5000);
  }
    Serial.println("HTU21D, SHT21 sensor is active");

    WiFi.begin();                                           //Käynnistetään WiFi-yhteys, jotta voidaan lukea LoRa-piirin MAC-osoite
    delay(1000);                                            //Odotetaan WiFin käynnistymistä
    WiFi.macAddress(MAC);                                   //Luetaan LoRan MAC-osoite
    WiFi.disconnect();                                      //Katkaistaan WiFi-yhteys
    MAC_String = mac2String(MAC);                           //Muutetaan MAC-osoite merkkijonoksi ilman kaksoispisteitä
    MAC_String.toCharArray(MAC_char,13);                    //Muutetaan MAC-osoite string-muotoisesta char-muotoiseksi
    randomSeed(analogRead(AnalogPin));                      //Sekoitetaan satunnaislukugeneraattori (luetaan analogia-inputista 39 satunnaista kohinaa), jotta joka käynnistyskerralla saadaan erilaisia satunnaislukuja
    viive = random(1000, 10000);                            //Arvotaan ensimmäiselle viestin lähetykselle viive 1-10 s väliltä. Tämä auttaa tarkistamaan nopeasti, että LoRa toimii, kytkettäessä se päälle
}


//Loop-koodi ajetaan ohjelman käynnistyttyä jatkuvasti
void loop()
{ 
    delay(viive);                                           //Odotetaan x sekuntia
    viive = random(480000, 600000);                         //Arvotaan lähetysväli 8 - 10 minuutin väliltä
    u8x8.clearLine(1);                                      //Tyhjennetään OLED-näytön rivit 1, 2 ja 6                                      
    u8x8.clearLine(2);
    u8x8.clearLine(6);
    smartDelay(500);
    temp = myHTU21D.readTemperature();                      //Luetaan lämpötila anturilta   
    hum = myHTU21D.readCompensatedHumidity();               //Luetaan kosteus anturilta

    u8x8.setCursor(0, 1);                                   //Asetetaan näytön kursori riville 1
    u8x8.printf("Temp: %1.1f",temp);                        //Tulostetaan näytölle lämpötila yhden desimaalin tarkkuudella
    u8x8.setCursor(0, 2);                                 
    u8x8.printf("Hum: %1.1f",hum);                          //Tulostetaan näytölle kosteus yhden desimaalin tarkkuudella
    u8x8.setCursor(0, 3);                                   //Asetetaan näytön kursori riville 3
    u8x8.printf("MAC:%3s", MAC_char);                       //Tulostetaan OLED-näytölle LoRa-piirin MAC-osoite    
    
    if(gpsActive == true)
    {
      lat = gps.location.lat();                               //Luetaan sijainnin latitudi
      lng = gps.location.lng();                               //Luetaan sijainnin longitudi
      hdop = gps.hdop.value();                                //Luetaan paikannuksen horizontal diminution of precision eli "tarkkuus vaakatasossa". Luku pitää jakaa sadalla, 
      hdop = hdop/100;                                        //jotta se vastaa todellista hdop-arvoa. <1 ideaalinen tarkkuus, >20 huono tarkkuus


      u8x8.setCursor(0, 5);                                   //Asetetaan näytön kursori riville 4
      u8x8.printf("Lat: %1.6f",lat);                          //Tulostetaan näytölle latitudi kuuden desimaalin tarkkuudella
      u8x8.setCursor(0, 6);                                 
      u8x8.printf("Lon: %1.6f",lng);                          //Tulostetaan näytölle longitudi kuuden desimaalin tarkkuudella
      u8x8.setCursor(0, 7);                                 
      u8x8.printf("HDOP: %1.2f",hdop);                        //Tulostetaan näytölle Hdop-arvo yhdellä desimaalilla  
#if 0
      if(hdop < 5)
      {
        gpsActive = false;

        gpsOff();
        delay(100); // Little delay before flushing.
        GPSSerial.flush();
      }
#endif
    }
    else
    {
      u8x8.setCursor(0, 5);                                   //Asetetaan näytön kursori riville 4
      u8x8.printf("Lat: %1.6f",lat);                          //Tulostetaan näytölle latitudi kuuden desimaalin tarkkuudella
      u8x8.setCursor(0, 6);                                 
      u8x8.printf("Lon: %1.6f",lng);                          //Tulostetaan näytölle longitudi kuuden desimaalin tarkkuudella
      u8x8.setCursor(0, 7);                                 
      u8x8.printf("GPS off   ");   
    }

    Serial.print("Temperature: ");
    Serial.println(temp, 1);                                //Tulostetaan sarjaporttiväylään lämpötila yhden desimaalin tarkkuudella
    Serial.print("Humidity: ");
    Serial.println(hum, 1);                                 //Tulostetaan sarjaporttiväylään kosteus yhden desimaalin tarkkuudella
    Serial.print("GPS Lat: ");
    Serial.println(lat, 6);                                 //Tulostetaan sarjaporttiväylään latitudi 6 desimaalin tarkkuudella
    Serial.print("GPS Lon: ");
    Serial.println(lng, 6);                                 //Tulostetaan sarjaporttiväylään longitudi 6 desimaalin tarkkuudella
    Serial.print("HDOP: ");
    Serial.println(hdop); 
    Serial.print("MAC: ");                                  //Tulostetaan LoRa-piirin MAC-osoite sarjaporttiväylään
    Serial.println(MAC_char);
    Serial.println();
    
    LoRa.beginPacket();                                     //Käynnistetään datapaketin lähettäminen LoRa-radioyhteydellä
    LoRa.printf("Temp:%1.1f\n", temp);                      //Lähetetään 1. rivi (lämpötila)
    LoRa.printf("Hum:%1.1f\n", hum);                        //Lähetetään 2. rivi (kosteus)
    LoRa.printf("Lat:%1.6f\n", lat);                        //Lähetetään 3. rivi (latitudi)
    LoRa.printf("Lon:%1.6f\n", lng);                        //Lähetetään 4. rivi (longitudi)
    LoRa.printf("HDOP:%1.2f\n", hdop);                      //Lähetetään 5. rivi (hdop)  
    LoRa.printf("MAC:%3s\n",MAC_char);                      //Lähetetään 6. rivi (MAC-osoite) 
    LoRa.endPacket();                                       //Lopetetaan datapaketin lähettäminen radioyhteydellä   
} 

//Funktiot

String mac2String(byte ar[]) {                              //Funktio, jolla muutetaan MAC-osoite tavu-muodosta String-muotoiseksi merkkijonoksi
  String s;
  for (byte i = 0; i < 6; ++i)
  {
    char buf[3];
    sprintf(buf, "%2X", ar[i]);
    s += buf;
    //if (i < 5) s += ':';                                  //Lisää MAC-osoitteeseen kaksoispisteet 
  }
  return s;
}

static void smartDelay(unsigned long ms)                    //Funktio, joka ylläpitää yhteyttä GPS-piiriin ja lukee GPS-sijaintiviestin. Jos yhteyttä ei ylläpidetä, GPS-sijaintiviestin luku ei toimi.                   
{
  unsigned long start = millis();
  do 
  {
    while ((GPSSerial.available() > 0) && (gpsActive == true))
      gps.encode(GPSSerial.read());                         //Luetaan GPS-piirin sijaintiviesti ja koodataan se luettavaan muotoon
  } while (millis() - start < ms);
}    

void gpsOff()
{
    // CFG-MSG packet.
    byte packet[] = {
        0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B
    };

    sendUBX(packet, sizeof(packet));
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(byte *msg, byte len) {
  for(byte i = 0; i < len; i++) {
    GPSSerial.write(msg[i]);
  }
}
