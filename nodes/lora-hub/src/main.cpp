//Jani Raami
//Koodi on tarkoitettu käytettäväksi LoRa-piirissä. Koodilla otetaan vastaan ESP32-piirien lähettämät UDP-datapaketit, 
//jotka sisältävät anturin mittaustuloksen ja ESP32-piirin MAC-osoitteen. LoRa-piiri lähettää datapaketit
//eteenpäin LoRa-radioyhteydellä lisäten datapakettiin GPS-sijaintitietonsa. LoRa-piiri toimii anturi-joukon 
// keskittimenä sekä niiden sijainnin määrittäjänä.

//GPS-piiri:  NEO-6M GPS
//LoRa-piiri: Heltec WiFi LoRa 32

//Kytkentä:
//GPS Vcc --- LoRa Pin 3V3 (PRG-napin puoleinen)
//GPS GND --- LoRa Pin GND (PRG-napin puoleinen)
//GPS TXD --- LoRa Pin 22
//GPS RXD --- LoRa Pin 23
//GPS PPS --- (Ei kytketä)
//LoRa-piirin jännitteet USB:llä ulkoiselta akulta

//Yhteysasetukset:
//Kortti:           Heltec_WIFI_LoRa_32
//Flash Frequency:  80 MHz
//Upload Speed:     921600 (Jos ei toimi, kokeile 115200)

//Sisällytetään seuraavat kirjastot koodiin
#include <WiFi.h>                                           //WiFi-verkon käyttö
#include <WiFiUdp.h>                                        //UDP-protokolla kommunikointi
#include <SPI.h>                                            //Sarjayhteys kommunikointi
#include <LoRa.h>                                           //Radioverkon kommunikointi
#include <U8x8lib.h>                                        //WiFi-LoRa 32 OLED-näyttö
#include <HardwareSerial.h>                                 //LoRan sarjamuotoinen kommunikointi GPS:n kanssa 
#include <TinyGPS++.h>                                      //GPS:n käyttö

//Määritetään WiFi-Lora 32:n pinneille nimet ja asetukset
#define SCK     5                                           //GPIO5  -- SX1278's SCK
#define SS      18                                          //GPIO18 -- SX1278's CS
#define RST     14                                          //GPIO14 -- SX1278's RESET
#define MISO    19                                          //GPIO19 -- SX1278's MISO
#define DI0     26                                          //GPIO26 -- SX1278's IRQ(Keskeytyspyyntö)
#define MOSI    27                                          //GPIO27 -- SX1278's MOSI
#define GPS_RX  22                                          //GPIO22 -- GPS vastaanotto
#define GPS_TX  23                                          //GPIO23 -- GPS lähetys
#define BAND    869.65E6                                    //Käytettävä taajuus 869.65 MHz. Taajuusalueella 869.400 - 869.650 MHz saa säteilyteho olla enintään 500 mW ERP ja toimintasuhde enintään 10 %
#define signalBandwidth 31.25E3                             //Asetetaan kaistanleveydeksi 31.25 kHz
#define PABOOST true                                        //Asetetaan Power Amplifier Boost päälle

// Wifi-Lora 32:n OLED-näytön määrittely
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

//WiFi-yhteyden, GPS:n ja datapaketin määrittely

static const uint32_t GPSBaud = 9600;                       //GPS:n sarjamuotoisen tiedonsiirtonopeuden määrittely
float lat;                                                  //Latitudin muuttuja
float lng;                                                  //Longitudin muuttuja
float hdop;                                                 //Hdopin muuttuja
int NumOfSat;                                               //Satelliittien määrä, joilta GPS-signaali vastaanotetaan
const char* ssid = "ESP32toLoRa";                           //Access pointin SSID-tunnus
const char* password = "0N2NLXYP9T!!42FsN#UPP1899jJ";       //Access pointin salasana           
unsigned int localUdpPort = 4210;                           //UDP-protokollan käyttämä portti
char incomingPacket[40];                                    //Merkkijonomuuttuja (puskuri) saapuvalle UDP-protokollan mukaiselle datapaketille, jonka maksimi pituus on 40 merkkiä
char* token;                                                //Dynaaminen merkkijonomuuttuja
int len;                                                    //Datapaketin pituus tavuissa muuttuja
int i;                                                      //Laskuri
IPAddress local_IP(192, 168, 4, 10);                        //Access pointin staattinen IP-osoite
IPAddress gateway(192, 168, 4, 1);                          //Access pointin yhdyskäytävä
IPAddress subnet_mask(255, 255, 255, 0);                    //Access pointin aliverkon peite 

WiFiUDP Udp;                                                //Määritetään WiFiUDP-objekti Udp nimiseksi
TinyGPSPlus gps;                                            //Luodaan TinyGPS++ objekti ja nimetään se gps:ksi
HardwareSerial GPSSerial(1);                                //Luodaan HardwareSerial objekti ja nimetään se GPSSerialiksi     

static void smartDelay(unsigned long ms);
void sendUBX(uint8_t *MSG, uint8_t len);
void GpsOff();
void GpsOn();

//Setup-koodi ajetaan kerran ohjelman käynnistyessä
void setup()                           
{
  u8x8.begin();                                             //WiFi LoRa 32:n OLED-näytön kommuninoinnin käynnistäminen
  u8x8.setFont(u8x8_font_chroma48medium8_r);                //Asetetaan OLED-näytölle fontti
  Serial.begin(115200);                                     //Sarjayhteyden käynnistäminen nopeudella 115200
  Serial.println();                                         //Tyhjän rivin tulostus
  Serial.printf("Starting access point %s ", ssid);         //Tulostetaan sarjayhteydellä access pointin käynnistäminen 
  u8x8.drawString(0, 0, "Starting");                        //Tulostetaan OLED-näyttöön access pointin käynnistäminen
  u8x8.drawString(0, 1, "Access point:"); 
  WiFi.mode(WIFI_AP);                                       //Asetetaan piiri toimimaan WiFi-tukiasemana (Access Point)
  WiFi.softAP(ssid, password,1,1,10);                       //Käynnistetään access point parametreilla: WiFi-tukiaseman nimi, salasana, kanava, ssid piilotettu ja maksimi clienttien määrä 10                   
  WiFi.softAPConfig(local_IP, gateway, subnet_mask);        //Konfiguroidaan access point käyttämään määriteltyjä staattista IP-osoitetta, yhdyskäytävää ja aliverkon peitettä
  Serial.printf("\nAccess point IP %s\n", WiFi.softAPIP().toString().c_str());  //Tulostetaan sarjayhteydellä kuunneltava IP-osoite ja portti  
  u8x8.drawString(0, 2, WiFi.softAPIP().toString().c_str());//Tulostetaan OLED-näytölle kuunneltava IP-osoite
  
  SPI.begin(SCK, MISO, MOSI, SS);                           //SPI väylän käynnistys ja pinnien asetukset. Nämä pitää tehdä ennen LoRa.begin komentoa            
  LoRa.setPins(SS, RST, DI0);
  
  while (!LoRa.begin(BAND,PABOOST))                         //Käynnistetään LoRa-yhteys asetetulla taajuudella ja PABOOST kytkettynä
  {
    Serial.println("Starting LoRa failed!");                //Tulostetaan sarjayhteydellä, jos LoRa-yhteydenmuodostus epäonnistuu
    u8x8.drawString(0, 3, "LoRa failed");                   //Tulostetaan näyttöön, jos LoRa-yhteydenmuodostus epäonnistuu
    delay(2000);                                            //Odotetaan 2 sekuntia ja yritetään uudestaan
  }                                     
    Serial.println("Starting LoRa OK.");                    //Tulostetaan sarjayhteydellä, jos LoRa-yhteyden muodostus onnistuu
    u8x8.drawString(0, 3, "Starting LoRa OK");              //Tulostetaan OLED-näytölle, jos LoRa-yhteyden muodostus onnistuu
                                                            //Alla olevat kolme LoRa asetusta tulee tehdä LoRa.begin komennon jälkeen
    LoRa.crc();                                             //Asetetaan LoRa-viestin CRC-tarkistuskoodaus ominaisuus päälle. Tällä estetään korruptoituneiden viestien vastaanotto.
    LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);            //Asetetaan lähetysteho 20 dBm käyttäen PA_BOOST_PINniä (Huom! Lähetysantenni ei saa olla liian lähellä GPS-antennia tai GPS:ää. Muuten UDP-viestin vastaanotto ei toimi)
    LoRa.setSpreadingFactor(7);                             //Asetetaan LoRa-yhteyden leviämistekijäksi 7
    LoRa.setSignalBandwidth(signalBandwidth);               //Asetetaan määritelty kaistanleveys
    delay(5000);                                            //Odotetaan 5 sekuntia, jotta OLED-näytön viestit ehtii lukea
    u8x8.clearDisplay();                                    //Tyhjennetään OLED-näyttö 
    u8x8.drawString(0, 0, "Access point");
    u8x8.drawString(0, 1, "LoRa Transmitter");
    Udp.begin(localUdpPort);                                //Käynnistetään UDP-yhteys käyttäen määriteltyä porttia
    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);      //Sarjayhteyden käynnistäminen, jotta GPS-piirin kanssa voidaan kommunikoida
    GPSSerial.setTimeout(2);  
    u8x8.drawString(0, 4, "GPS coordinates:");
    //GpsOn();
}


//Loop-koodi ajetaan ohjelman käynnistyttyä jatkuvasti
void loop()
{ 
    smartDelay(500);
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
  
    memset(incomingPacket,0,40);                            //Tyhjennetään datapaketin puskuri, jotta datapaketteihin ei ala kertymään ylimääräisiä merkkejä
    len = Udp.parsePacket();                                //Aloittaa prosessoimaan seuraavaa saatavilla olevaa saapuvaa datapakettia, tarkistaa sen olemassaolon ja palauttaa datapaketin koon
    Udp.read(incomingPacket, len);                          //Luetaan muuttujan len pituinen UDP-datapaketti merkkipuskuriin
 
    if (len > 0)                                            //Jos datapaketissa on sisältöä
    {  
      u8x8.clearLine(2);                                    //Tyhjennetään OLED-näytön rivit 2 ja 7                                      
      u8x8.clearLine(7);
      Serial.printf("Datapaketin koko: %d\n",len);
      Serial.println("Datapaketin sisältö: ");               
      Serial.print(incomingPacket);                         //Tulostetaan datapaketti sarjayhteydellä
      Serial.print("GPS Lat: ");
      Serial.println(lat, 6);                               //Tulostetaan sarjaporttiväylään latitudi 6 desimaalin tarkkuudella
      Serial.print("GPS Lon: ");
      Serial.println(lng, 6);                               //Tulostetaan sarjaporttiväylään longitudi 6 desimaalin tarkkuudella
      Serial.print("HDOP: ");
      Serial.println(hdop); 
      Serial.println();
    
      token = strtok(incomingPacket, "\n");                 //Luetaan merkkijonopuskurista incomingPacket merkkijono, joka päättyy merkkeihin \n. Eli luetaan ensimmäinen merkkijonorivi (etäisyys)
      i=2;                                                  //Asetetaan rivilaskuri kutoseksi
      LoRa.beginPacket();                                   //Käynnistetään datapaketin lähettäminen LoRa-radioyhteydellä
      while (token != NULL)                                 //Luetaan merkkijonosta token rivejä (datatietueita), kunnes merkkijono on tyhjä                         
      {
        LoRa.printf("%s\n",token);                          //Lähetetään datatietue LoRa-radioyhteydellä eteenpäin
        u8x8.drawString(0, i, token);                       //Tulostetaan merkkijonorivi (datatietue) OLED-näytölle
        token = strtok(NULL, "\n");                         //Asetetaan luettu merkkijonorivi tyhjäksi
        i++;                                                //Kasvatetaan rivilaskuria
      }
      LoRa.printf("Lat:%1.6f\n", lat);                      //Lähetetään 1. rivi (latitudi)
      LoRa.printf("Lon:%1.6f\n", lng);                      //Lähetetään 2. rivi (longitudi)
      LoRa.printf("HDOP:%1.2f\n", hdop);                    //Lähetetään 3. rivi (hdop)  
      LoRa.endPacket();                                     //Lopetetaan datapaketin lähettäminen radioyhteydellä  
    } 
}   

static void smartDelay(unsigned long ms)                    //Funktio, joka ylläpitää yhteyttä GPS-piiriin ja lukee GPS-sijaintiviestin. Jos yhteyttä ei ylläpidetä, GPS-sijaintiviestin luku ei toimi.                   
{
  unsigned long start = millis();
  do 
  {
    while (GPSSerial.available() >0)
      gps.encode(GPSSerial.read());                         //Luetaan GPS-piirin sijaintiviesti ja koodataan se luettavaan muotoon
  } while (millis() - start < ms);
}                        

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for (int i = 0; i < len; i++) {
    GPSSerial.write(MSG[i]);
    //Serial.write(MSG[i]);
  }
}

void GpsOff()
{
  // ---- Set GPS to backup mode (sets it to never wake up on its own)
  uint8_t GPSoff[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
  sendUBX(GPSoff, sizeof(GPSoff) / sizeof(uint8_t));
  Serial.println(F("GPS power OFF"));
}

void GpsOn()
{
  // ---- Restart GPS
  uint8_t GPSon[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37};
  sendUBX(GPSon, sizeof(GPSon) / sizeof(uint8_t));
  Serial.println(F("GPS power ON"));
}
