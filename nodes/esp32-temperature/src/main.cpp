//Jani Raami
//Koodi on tarkoitettu käytettäväksi ESP32-piirissä yhdessä lämpötila-anturin kanssa.
//Lämpötila-anturi antaa lämpötilan ja ESP32 lähettää lämpötilan
//eteenpäin UDP-protokollalla WiFi-verkon ylitse.

//Lämpötila-anturi:         DS18B20
//ESP32:                    ESP-WROOM-32

//Kytkentä:
//Lämpötila-anturi Vcc (Punainen)   --- ESP32 Pin VIN
//Lämpötila-anturi Gnd (Musta)      --- ESP32 Pin GND
//Lämpötila-anturi Data (Keltainen) --- ESP32 Pin 13

//Yhteysasetukset:
//Kortti:           SparkFun ESP32 Thing
//Flash Frequency:  80 MHz
//Upload Speed:     921600 tai 115200

//Yhteyden muodostaminen:
//Arduino IDE-ohjelman luodessa yhteyttä ESP32-piirille, paina ESP32:n BOOT nappia niin pitkään, että yhteys muodostuu

//Sisällytetään seuraavat kirjastot koodiin
#include <WiFi.h>                             //WiFi-verkon käyttö
#include <WiFiUdp.h>                          //UDP-protokolla kommunikointi
#include "time.h"                             //Ajastus
#include <OneWire.h>                          //Yksijohdin datalinjan kommunikointi
#include <DallasTemperature.h>                //Lämpötila-anturin käyttö
//#include <esp32/include/esp_sleep.h>

//WiFi-yhteyden määrittely
const char* ssid = "ESP32toLoRa";             //WiFi-yhteyden SSID-tunnus
const char* password = "0N2NLXYP9T!!42FsN#UPP1899jJ";  //WiFi-yhteyden salasana
const char* IP = "192.168.4.10";              //IP-osoite, johon datapaketti lähetetään
unsigned int localUdpPort = 4210;             //UDP-protokollan portti, johon datapaketti lähetetään

//Lämpötila-anturin datakanavaa vastaava ESP32:n GPIO-portti  
#define ONE_WIRE_BUS 13                       //Anturin datalinja
    
//Muut määrittelyt
String mac;                                   //ESP32:n MAC-osoitteen muuttuja
String MAC_String;                            //String-muotoinen muuttuja MAC-osoitteelle
char MAC_char[13];                            //Merkkijonotaulukkomuuttuja MAC-osoitteelle
float temp = 0;                               //Lämpötilan muuttuja
byte wfConnectTry = 0;                        //Yhteysyrityslaskuri
int viive;                                    //Lähetysväli millisekunneissa

WiFiUDP Udp;                                  //Määritetään WiFiUDP-objekti Udp nimiseksi
OneWire oneWire(ONE_WIRE_BUS);                //Määritetään OneWire-objekti oneWire nimiseksi 
DallasTemperature sensors(&oneWire);          //Määritetään DallasTemperature-objekti sensors nimiseksi

String mac2String(String mac);
void wifiReconnect();
void wifiStart();
void Disconnect();

//Setup-koodi ajetaan kerran ohjelman käynnistyessä
void setup()
{
  Serial.begin(115200);                       //Sarjayhteyden käynnistäminen nopeudella 115200
  Serial.println();                           //Tyhjän rivin tulostus
  wifiStart();                                //Aliohjelmakutsu, jolla käynnistetään WiFi-yhteys
  sensors.begin();                            //Käynnistetään lämpötila-anturikommunikointi
  mac = WiFi.macAddress();                    //Luetaan ESP32:n MAC-osoite
  MAC_String = mac2String(mac);               //Muutetaan MAC-osoite string-muotoiseksi ilman kaksoispisteitä
  MAC_String.toCharArray(MAC_char,13);        //Muutetaan string-muotoinen MAC-osoite merkkijonotaulukoksi
  randomSeed(analogRead(34));                 //Sekoitetaan satunnaislukugeneraattori (luetaan analogia-inputista 34 satunnaista kohinaa), jotta joka käynnistyskerralla saadaan erilaisia satunnaislukuja

  delay(1000);
  
  sensors.requestTemperatures();              //Lähetetään anturille pyyntö lämpötila-arvosta 
  temp=sensors.getTempCByIndex(0);            //Luetaan lämpötila. Nolla viittaa ensimmäiseen lämpötila-anturiin dataväylässä
  //
  //viive = random(1000, 10000);                //Arvotaan ensimmäiselle viestin lähetykselle viive 1-10 s väliltä. Tämä auttaa tarkistamaan nopeasti, että ESP32 toimii, kytkettäessä se päälle
  delay(1000);                                //Odotetaan 1 sekunti

  viive = random(300000000, 600000000);             //Arvotaan lähetysväli 5 - 10 minuutin väliltä
  esp_sleep_enable_timer_wakeup(viive);
}


//Loop-koodi ajetaan ohjelman käynnistyttyä jatkuvasti
void loop()
{
  //viive = random(30000, 60000);             //Arvotaan lähetysväli 5 - 10 minuutin väliltä
  delay(2000);
  while (WiFi.status() != WL_CONNECTED || wfConnectTry >=5)   //Jos WiFi:n tila on muuta kuin yhdistetty tai yhteysyrityksiä on 5 tai enemmän, niin
  {
    wfConnectTry++;                           //lisätään yhteysyrityslaskuria yhdellä
    wifiReconnect();                          //Aliohjelmakutsu WiFin uudelleenyhdistämiseen
  }  
  Udp.begin(localUdpPort);                    //Käynnistetään UDP-protokollan mukainen yhteys käyttäen määriteltyä porttia
  sensors.requestTemperatures();              //Lähetetään anturille pyyntö lämpötila-arvosta 
  temp=sensors.getTempCByIndex(0);            //Luetaan lämpötila. Nolla viittaa ensimmäiseen lämpötila-anturiin dataväylässä
  Udp.beginPacket(IP, localUdpPort);          //Käynnistetään datapaketin lähettäminen määriteltyyn IP-osoitteeseen ja porttiin 
  Udp.printf("Temp:%1.1f\n",temp);            //Lähetetään anturilla mitattu lämpötila
  Udp.printf("MAC:%3s\n",MAC_char);           //Lähetetään MAC-osoite
  Udp.endPacket();                            //Lopetetaan datapaketin lähettäminen
  Serial.printf("Lämpötila: %1.1f\r\n", temp);  //Tulostetaan sarjaväylälle anturilla mitattu lämpötila
  Serial.printf("MAC: %3s\r\n",MAC_char);       //Tulostetaan MAC-osoite sarjaväylälle
  Udp.stop();                                 //Pysäytetään UDP-protokollan mukainen yhteys
  delay(2000);                                //Odotetaan 1 sekunti, jotta UDP-prokotollan mukainen yhteys ehtii pysähtyä. Jos viivettä ei ole, niin WiFi-yhteyden uudelleen muodostaminen ei onnistu
  Disconnect();                               //Aliohjelmakutsu WiFi-yhteyden katkaisuun
  Serial.printf("Viive: %d\r\n",viive/1000000);
  //delay(viive);                               //Odotetaan x sekuntia
  
  esp_err_t sleepErr = esp_light_sleep_start();
  Serial.printf("sleepErr: %d\r\n", sleepErr);
}

//Funktiot


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


//WiFi-yhteyden uudelleenmuodostaminen
void wifiReconnect()                          
{
  WiFi.disconnect();                          //Katkaistaan WiFi-yhteys
  delay(500);                                 //Odotetaan 500 ms, jotta yhteys ehtii sammua
  Serial.println(F("Starting WiFi."));
  wifiStart();                                //Luodaan WiFi-yhteys määriteltyyn verkkoon
}


//WiFi-yhteyden muodostaminen
void wifiStart()                              
{
  unsigned long timer1 = millis();            //Alustetaan timer1-laskuri ja asetetaan sen arvoksi ajanhetki (aika, joka on kulunut ohjelman käynnistymisestä)
  WiFi.disconnect(true);                      //Katkaistaan WiFi-yhteys ensin ja nollataan asetukset. Tämä on suoritettava ennen WiFin-käynnistämistä, jotta varmistetaan WiFin oikea toiminta. Myös true pitää olla tässä asetettuna
  WiFi.begin(ssid, password);                 //Käynnistetään WiFi-yhteys
  while (WiFi.status() != WL_CONNECTED && millis() - timer1 < 5000UL)     //Jos WiFin tila on muuta kuin yhdistetty ja aikaa on mennyt vähemmän kuin 5 sekuntia, niin suoritetaan alla oleva
  {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println(F("Connected"));
    wfConnectTry = 0;                         //Nollataan yhteysyritysten laskuri
  }
  else
  {
    Serial.println(F("Could not connect!"));
  }
}


//WiFi-yhteyden katkaiseminen
void Disconnect()                             
{
  WiFi.disconnect();                          //Katkaistaan WiFi-yhteys. Funktiossa ei saa käyttää tässä "true" valintaa. Aiheuttaa sen, että yhteyttä ei saada katkaistua.
  while (WiFi.status() != WL_DISCONNECTED)    //Odotetaan, kunnes WiFi-yhteys katkeaa
  {
     Serial.printf(".");
     delay(500);
  }
  Serial.println(F("WiFi disconnected"));
}
