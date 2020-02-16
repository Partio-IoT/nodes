//Jani Raami
//Koodi on tarkoitettu käytettäväksi ESP32-piirissä yhdessä lämpötila/kosteus -anturin kanssa.
//Anturi antaa lämpötilan ja kosteuden ja ESP32 lähettää nämä tiedot eteenpäin UDP-protokollalla
//WiFi-verkon ylitse.

//Lämpötila- ja kosteusanturi:          HTU21D
//ESP32:                                ESP-WROOM-32

//Kytkentä:
//HTU21D 3.3V                       --- ESP32 Pin 3V3 (BOOT-napin puoleinen)
//HTU21D GND                        --- ESP32 Pin GND (BOOT-napin puoleinen)
//HTU21D SDA                        --- ESP32 Pin 21 (SDA)
//HTU21D SCL                        --- ESP32 Pin 22 (SCL)

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
#include <Wire.h>                             //I2C-väylän käyttö
#include <HTU21D.h>                           //Lämpötila/kosteusanturin käyttö

//WiFi-yhteyden määrittely
const char* ssid = "ESP32toLoRa";             //WiFi-yhteyden SSID-tunnus
const char* password = "0N2NLXYP9T!!42FsN#UPP1899jJ";  //WiFi-yhteyden salasana
const char* IP = "192.168.4.10";              //IP-osoite, johon datapaketti lähetetään
unsigned int localUdpPort = 4210;             //UDP-protokollan portti, johon datapaketti lähetetään

//Pinnien määrittelyt
#define SDA       21                          //GPIO21 -- HTU21D data
#define SCL       22                          //GPIO22 -- HTU21D kello

//Muut määrittelyt
String mac;                                   //ESP32:n MAC-osoitteen muuttuja
String MAC_String;                            //String-muotoinen muuttuja MAC-osoitteelle
char MAC_char[13];                            //Merkkijonotaulukkomuuttuja MAC-osoitteelle
float temp = 0;                               //Lämpötilan muuttuja
float hum = 0;                                //Kosteuden muuttuja
byte wfConnectTry = 0;                        //Yhteysyrityslaskuri
int viive;                                    //Lähetysväli millisekunneissa

WiFiUDP Udp;                                  //Määritetään WiFiUDP-objekti Udp nimiseksi
HTU21D myHTU21D(HTU21D_RES_RH12_TEMP14);      //Luodaan HTU21D objekti ja nimetään se myHTU21D:ksi

String mac2String(String mac);
void wifiReconnect();
void wifiStart();
void Disconnect();

//Setup-koodi ajetaan kerran ohjelman käynnistyessä
void setup()
{
  Serial.begin(115200);                       //Sarjayhteyden käynnistäminen nopeudella 115200
  Serial.println();                           //Tyhjän rivin tulostus
  
  while (myHTU21D.begin() != true)            //Jos yhteydenmuodostus lämpötila/kosteusanturille ei onnistu, niin tulostetaan siitä virheviesti
  {
    Serial.println("HTU21D sensor is failed or not connected");
    delay(5000);
  }
    Serial.println("HTU21D, SHT21 sensor is active");
  
  wifiStart();                                //Aliohjelmakutsu, jolla käynnistetään WiFi-yhteys
  mac = WiFi.macAddress();                    //Luetaan ESP32:n MAC-osoite
  MAC_String = mac2String(mac);               //Muutetaan MAC-osoite string-muotoiseksi ilman kaksoispisteitä
  MAC_String.toCharArray(MAC_char,13);        //Muutetaan string-muotoinen MAC-osoite merkkijonotaulukoksi  
  randomSeed(analogRead(34));                 //Sekoitetaan satunnaislukugeneraattori (luetaan analogia-inputista 34 satunnaista kohinaa), jotta joka käynnistyskerralla saadaan erilaisia satunnaislukuja
  viive = random(1000, 10000);                //Arvotaan ensimmäiselle viestin lähetykselle viive 1-10 s väliltä. Tämä auttaa tarkistamaan nopeasti, että ESP32 toimii, kytkettäessä se päälle
  delay(1000);                                //Odotetaan 1 sekunti
}


//Loop-koodi ajetaan ohjelman käynnistyttyä jatkuvasti
void loop()
{
  viive = random(300000, 600000);             //Arvotaan lähetysväli 5 - 10 minuutin väliltä
  while (WiFi.status() != WL_CONNECTED || wfConnectTry >=5)   //Jos WiFi:n tila on muuta kuin yhdistetty tai yhteysyrityksiä on 5 tai enemmän, niin
  {
    wfConnectTry++;                           //lisätään yhteysyrityslaskuria yhdellä
    wifiReconnect();                          //Aliohjelmakutsu WiFin uudelleenyhdistämiseen
  }  
  Udp.begin(localUdpPort);                    //Käynnistetään UDP-protokollan mukainen yhteys käyttäen määriteltyä porttia
  temp = myHTU21D.readTemperature();          //Luetaan lämpötila anturilta   
  hum = myHTU21D.readCompensatedHumidity();   //Luetaan kosteus anturilta
  Udp.beginPacket(IP, localUdpPort);          //Käynnistetään datapaketin lähettäminen määriteltyyn IP-osoitteeseen ja porttiin 
  Udp.printf("Temp:%1.1f\n",temp);            //Lähetetään anturilla mitattu lämpötila
  Udp.printf("Hum:%1.1f\n", hum);             //Lähetetään 2. rivi (kosteus)
  Udp.printf("MAC:%3s\n",MAC_char);           //Lähetetään MAC-osoite
  Udp.endPacket();                            //Lopetetaan datapaketin lähettäminen
  Serial.print("Temperature: ");
  Serial.println(temp, 1);                    //Tulostetaan sarjaporttiväylään lämpötila yhden desimaalin tarkkuudella
  Serial.print("Humidity: ");
  Serial.println(hum, 1);                     //Tulostetaan sarjaporttiväylään kosteus yhden desimaalin tarkkuudella
  Serial.printf("MAC: %3s\n",MAC_char);       //Tulostetaan MAC-osoite sarjaväylälle
  Udp.stop();                                 //Pysäytetään UDP-protokollan mukainen yhteys
  delay(1000);                                //Odotetaan 1 sekunti, jotta UDP-prokotollan mukainen yhteys ehtii pysähtyä. Jos viivettä ei ole, niin WiFi-yhteyden uudelleen muodostaminen ei onnistu
  Disconnect();                               //Aliohjelmakutsu WiFi-yhteyden katkaisuun
  Serial.printf("Viive: %d\n",viive/1000);
  delay(viive);                               //Odotetaan x sekuntia
  
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
