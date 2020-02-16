//Jani Raami
//Koodi on tarkoitettu käytettäväksi Heltecin LoRa-piirissä yhdessä GPS-moduulin kanssa. GPS-piiri antaa
//sarjaväylän välityksellä sijaintitiedon LoRa-piirille, joka tulostaa sijainnin näytölle sekä lähettää 
//sijaintitiedon eteenpäin käyttäen LoRa-radioverkkoa

//GPS-piiri:  NEO-6M GPS
//LoRa-piiri: Heltec WiFi LoRa 32 

//Kytkentä:
//GPS Vcc --- LoRa Pin 3V3 (PRG-napin puoleinen)
//GPS GND --- LoRa Pin GND (PRG-napin puoleinen)
//GPS TXD --- LoRa Pin 22
//GPS RXD --- LoRa Pin 23
//GPS PPS --- (Ei kytketä)
//LoRan jännitteet USB:llä ulkoiselta akulta

//Yhteysasetukset:
//Kortti:           Heltec_WIFI_LoRa_32
//Flash Frequency:  80 MHz
//Upload Speed:     921600 (Jos ei toimi, kokeile 115200)

//Sisällytetään seuraavat kirjastot koodiin
#include <HardwareSerial.h>                             //LoRan sarjamuotoinen kommunikointi GPS:n kanssa 
#include <TinyGPS++.h>                                  //GPS:n käyttö
#include <WiFi.h>                                       //WiFi-verkon käyttö
#include <SPI.h>                                        //Sarjayhteys kommunikointi
#include <LoRa.h>                                       //LoRa-radioverkon kommunikointi
#include <U8x8lib.h>                                    //LoRa OLED-näytön käyttö

//Määritetään LoRan pinneille nimet ja asetukset
#define SCK       5                                     //GPIO5  -- SX1278's SCK
#define SS        18                                    //GPIO18 -- SX1278's CS
#define RST       14                                    //GPIO14 -- SX1278's RESET
#define MISO      19                                    //GPIO19 -- SX1278's MISO
#define DI0       26                                    //GPIO26 -- SX1278's IRQ(Keskeytyspyyntö)
#define MOSI      27                                    //GPIO27 -- SX1278's MOSI
#define GPS_RX    22                                    //GPIO22 -- GPS vastaanotto
#define GPS_TX    23                                    //GPIO23 -- GPS lähetys
#define AnalogPin 39                                    //ADC1_3 -- Analogia pin 3
#define BAND    869.5E6                                 //Käytettävä taajuus 869.5 MHz. Taajuusalueella 869.400 - 869.650 MHz saa säteilyteho olla enintään 500 mW ERP ja toimintasuhde enintään 10 %
#define signalBandwidth 31.25E3                         //Asetetaan kaistanleveydeksi 31.25 kHz
#define PABOOST true                                    //Asetetaan Power Amplifier Boost päälle

// LoRan OLED-näytön määrittely
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

//Määriteltävät muuttujat
static const uint32_t GPSBaud = 9600;                   //GPS:n sarjamuotoisen tiedonsiirtonopeuden määrittely
float lat;                                              //Latitudin muuttuja
float lng;                                              //Longitudin muuttuja
float hdop;                                             //Hdopin muuttuja
String mac;                                             //LoRa:n MAC-osoitteen muuttuja
int NumOfSat;                                           //Satelliittien määrä, joilta GPS-signaali vastaanotetaan
String MAC_String;                                      //MAC-osoitteen merkkijonomuuttuja
char MAC_char[13];                                      //MAC-osoitteen merkkijonomuuttuja (merkkitaulukko)
int viive;                                              //Lähetysväli millisekunneissa

TinyGPSPlus gps;                                        //Luodaan TinyGPS++ objekti ja nimetään se gps:ksi
HardwareSerial GPSSerial(1);                            //Luodaan HardwareSerial objekti ja nimetään se GPSSerialiksi                    

String mac2String(String mac);
static void smartDelay(unsigned long ms);

//Setup-koodi ajetaan kerran ohjelman käynnistyessä
void setup()
{
  u8x8.begin();                                         //LoRan OLED-näytön kommuninoinnin käynnistäminen
  u8x8.setFont(u8x8_font_chroma48medium8_r);            //Asetetaan OLED-näytölle fontti
  Serial.begin(115200);                                 //Sarjayhteyden käynnistäminen nopeudella 115200
  Serial.println();                                     //Tyhjän rivin tulostus
  Serial.println("LoRa Transmitter");                   //Tulostetaan sarjayhteydellä, että kyseinen LoRa-piiri toimii lähettimenä 
  u8x8.drawString(0, 0, "LoRa Transmitter");            //Tulostetaan OLED-näytölle vastaava sanoma
  SPI.begin(SCK, MISO, MOSI, SS);                       //SPI väylän käynnistys ja pinnien asetukset. Nämä pitää tehdä ennen LoRa.begin komentoa
  LoRa.setPins(SS, RST, DI0);
  
  while (!LoRa.begin(BAND,PABOOST))                     //Käynnistetään LoRa-yhteys asetetulla taajuudella ja PABOOST kytkettynä  
  {
    Serial.println("Starting LoRa failed!");            //Tulostetaan sarjayhteydellä, jos LoRa-yhteydenmuodostus epäonnistuu
    u8x8.drawString(0, 1, "LoRa failed");               //Tulostetaan näyttöön, jos LoRa-yhteydenmuodostus epäonnistuu
    delay(2000);                                        //Odotetaan 2 sekuntia ja yritetään uudestaan
  }
    Serial.println("Starting LoRa OK.");                //Tulostetaan sarjayhteydellä, jos LoRa-yhteyden muodostus onnistuu
    u8x8.drawString(0, 1, "Starting LoRa OK");          //Tulostetaan OLED-näytölle, jos LoRa-yhteyden muodostus onnistuu
                                                        //Alla olevat neljä LoRa asetusta tulee tehdä LoRa.begin komennon jälkeen
    LoRa.crc();                                         //Asetetaan LoRa-viestin CRC-tarkistuskoodaus ominaisuus päälle
    LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);        //Asetetaan lähetysteho 20 dBm käyttäen PA_BOOST_PINniä
    LoRa.setSpreadingFactor(7);                         //Asetetaan LoRa-yhteyden leviämistekijäksi 7
    LoRa.setSignalBandwidth(signalBandwidth);           //Asetetaan määritelty kaistanleveys
    WiFi.begin();                                       //Käynnistetään WiFi-yhteys, jotta voidaan lukea LoRa-piirin MAC-osoite
    delay(1000);                                        //Odotetaan 1 sekunti, jotta WiFi ehtii käynnistyä
    mac = WiFi.macAddress();                            //Luetaan ESP32:n MAC-osoite
    MAC_String = mac2String(mac);                       //Muutetaan MAC-osoite string-muotoiseksi ilman kaksoispisteitä
    MAC_String.toCharArray(MAC_char,13);                //Muutetaan string-muotoinen MAC-osoite merkkijonotaulukoksi
    WiFi.disconnect();                                  //Katkaistaan WiFi-yhteys
    delay(2000);                                        //Odotetaan 2 sekuntia, jotta OLED-näytön viestit ehtii lukea
    u8x8.clearLine(1);                                  //Tyhjennetään OLED-näytön rivi 1    
    
    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);  //Sarjayhteyden käynnistäminen, jotta GPS-piirin kanssa voidaan kommunikoida
    GPSSerial.setTimeout(2);                            
    u8x8.setCursor(0, 2);                               //Asetetaan näytön kursori riville 1
    u8x8.printf("MAC:%3s", MAC_char);                   //Tulostetaan OLED-näytölle LoRa-piirin MAC-osoite          
    u8x8.drawString(0, 3, "GPS coordinates:");
    randomSeed(analogRead(AnalogPin));                  //Sekoitetaan satunnaislukugeneraattori (luetaan analogia-inputista 39 satunnaista kohinaa), jotta joka käynnistyskerralla saadaan erilaisia satunnaislukuja
}


//Loop-koodi ajetaan ohjelman käynnistyttyä jatkuvasti
void loop()
{
  u8x8.clearLine(6);
  u8x8.clearLine(7);
  smartDelay(500);                                      
  
  if (gps.location.isUpdated())                         //Jos GPS-sijainti on päivitetty, niin suoritetaan alla oleva koodi 
  {
  lat = gps.location.lat();                             //Luetaan sijainnin latitudi
  lng = gps.location.lng();                             //Luetaan sijainnin longitudi
  hdop = gps.hdop.value();                              //Luetaan paikannuksen horizontal diminution of precision eli "tarkkuus vaakatasossa". Luku pitää jakaa sadalla, 
  hdop = hdop/100;                                      //jotta se vastaa todellista hdop-arvoa. <1 ideaalinen tarkkuus, >20 huono tarkkuus
  NumOfSat = gps.satellites.value();                    //Luetaan satelliittien määrä, joilta GPS-signaali vastaanotetaan                         
  u8x8.setCursor(0, 4);                                 //Asetetaan näytön kursori riville 4
  u8x8.printf("Lat: %1.6f", lat);                       //Tulostetaan näytölle latitudi kuuden desimaalin tarkkuudella
  u8x8.setCursor(0, 5);                                 
  u8x8.printf("Lon: %1.6f", lng);                       //Tulostetaan näytölle longitudi kuuden desimaalin tarkkuudella
  u8x8.setCursor(0, 6);                                 
  u8x8.printf("Num of Sat: %d", NumOfSat);              //Tulostetaan näytölle satelliittien määrä
  u8x8.setCursor(0, 7);                                 
  u8x8.printf("HDOP: %1.2f", hdop);                     //Tulostetaan näytölle Hdop-arvo yhdellä desimaalilla
  Serial.print("\nGPS Lat: ");
  Serial.println(lat, 6);                               //Tulostetaan sarjaporttiväylään latitudi 6 desimaalin tarkkuudella
  Serial.print("GPS Lon: ");
  Serial.println(lng, 6);                               //Tulostetaan sarjaporttiväylään longitudi 6 desimaalin tarkkuudella
  Serial.print("HDOP: ");
  Serial.println(hdop);
  Serial.print("MAC: ");
  Serial.println(MAC_char);
  
  LoRa.beginPacket();                                   //Käynnistetään datapaketin lähettäminen radioyhteydellä
  LoRa.printf("Lat:%1.6f\n", lat);                      //Lähetetään 1. rivi (latitudi)
  LoRa.printf("Lon:%1.6f\n", lng);                      //Lähetetään 2. rivi (longitudi)
  LoRa.printf("HDOP:%1.2f\n", hdop);                    //Lähetetään 3. rivi (hdop)
  LoRa.printf("MAC:%3s\n",MAC_char);                    //Lähetetään 4. rivi (MAC-osoite)                        
  LoRa.endPacket();                                     //Lopetetaan datapaketin lähettäminen radioyhteydellä
  viive = random(20000, 35000);                         //Arvotaan lähetysväli 20 - 35 sekunnin väliltä
  delay(viive);                                         //Odotetaan x sekuntia  
  }
}


//Funktiot:

//Funktio, jolla poistetaan MAC-osoitteesta kaksoispisteet
String mac2String(String mac){
  String s;
  char MAC_char[18]; 
  mac.toCharArray(MAC_char,18);               //Muutetaan string-muotoinen MAC-osoite merkkijonotaulukoksi, jotta se on oikeassa muodossa strtok-käskylle
  char* buf = strtok(MAC_char, ":");          //Etsitään ensimmäinen merkkijono, joka päättyy kaksoispisteeseen
  while (buf != 0)                            //Toistetaan kunnes muuttuja buf on tyhjä
  {
    s += buf;                                 //Lisätään etsityt merkit muuttujaan s
     buf = strtok(0, ":");                    //Jatketaan merkkijonojen etsimistä, kunnes merkkijonotaulukko on tyhjä
  }
  return s;
}


static void smartDelay(unsigned long ms)                //Funktio, joka ylläpitää yhteyttä GPS-piiriin ja lukee GPS-sijaintiviestin. Jos yhteyttä ei ylläpidetä, GPS-sijaintiviestin luku ei toimi.                   
{
  unsigned long start = millis();
  do 
  {
    while (GPSSerial.available() >0)
      gps.encode(GPSSerial.read());                      //Luetaan GPS-piirin sijaintiviesti ja koodataan se luettavaan muotoon
  } while (millis() - start < ms);
}                        
