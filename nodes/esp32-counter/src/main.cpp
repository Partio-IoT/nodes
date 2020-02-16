//Jani Raami
//Koodi on tarkoitettu käytettäväksi ESP32-piirissä yhdessä ultraäänianturin kanssa.
//Ultraäänianturia käytetään kävijälaskurina. Anturi antaa etäisyyden mitattavaan kohteeseen
//ja kun etäisyys on vähemmän kuin asetettu etäisyys, tulkitaan tämä anturin ohitse kulkemiseksi. 
//ESP32 lähettää kävijämäärän eteenpäin UDP-protokollalla WiFi-verkon ylitse.

//Ultraäänietäisyysanturit: HC-SR04
//ESP32:                    ESP-WROOM-32

//Kytkentä:
//Etäisyysanturi 1 Vcc  --- ESP32 VIN (+5 V)
//Etäisyysanturi 1 Trig --- ESP32 Pin 25
//Etäisyysanturi 1 Echo --- ESP32 Pin 26
//Etäisyysanturi 1 Gnd  --- ESP32 GND (EN-napin puoleinen)
//Etäisyysanturi 2 ei käytössä
//Etäisyysanturi 2 Vcc  --- ESP32 VIN (+5 V)
//Etäisyysanturi 2 Trig --- ESP32 Pin 22
//Etäisyysanturi 2 Echo --- ESP32 Pin 23
//Etäisyysanturi 2 Gnd  --- ESP32 GND (EN-napin puoleinen)

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

//WiFi-yhteyden määrittely
const char* ssid = "ESP32toLoRa";             //WiFi-yhteyden SSID-tunnus
const char* password = "0N2NLXYP9T!!42FsN#UPP1899jJ";  //WiFi-yhteyden salasana
const char* IP = "192.168.4.10";              //IP-osoite, johon datapaketti lähetetään
unsigned int localUdpPort = 4210;             //UDP-protokollan portti, johon datapaketti lähetetään

//Etäisyysanturien triggaus- ja kaikukanavia vastaavat ESP32:n GPIO-portit  
const int trigPin1 = 25;                      //Anturin 1 triggaus
const int echoPin1 = 26;                      //Anturin 1 kaiku             

//Muut määrittelyt
String mac;                                   //ESP32:n MAC-osoitteen muuttuja
String MAC_String;                            //String-muotoinen muuttuja MAC-osoitteelle
char MAC_char[13];                            //Merkkijonotaulukkomuuttuja MAC-osoitteelle
float cm1;                                    //Muuttuja mitatulle ensimmäiselle etäisyydelle anturilta 1
float cm2;                                    //Muuttuja mitatulle toiselle etäisyydelle anturilta 1

float initialDistance;

int counter = 0;                              //Kävijälaskuri
int counter2 = 0;                             //Laskuri, joka saadessaan arvon 20 tai enemmän aktivoi kävilaskurin lähettämisen WiFillä eteenpäin
bool trigger = 0;                             //Liipaisu

WiFiUDP Udp;                                  //Määritetään WiFiUDP-objekti Udp nimiseksi

float Measure_distance(const int trigPin, const int echoPin);
long microsecondsToCentimeters(long microseconds);
String mac2String(String mac);
void wifiReconnect();
void wifiStart();
void Disconnect();

//Setup-koodi ajetaan kerran ohjelman käynnistyessä
void setup()
{
  Serial.begin(115200);                       //Sarjayhteyden käynnistäminen nopeudella 115200
  Serial.println();                           //Tyhjän rivin tulostus
  mac = WiFi.macAddress();                    //Luetaan ESP32:n MAC-osoite
  MAC_String = mac2String(mac);               //Muutetaan MAC-osoite string-muotoiseksi ilman kaksoispisteitä
  MAC_String.toCharArray(MAC_char,13);        //Muutetaan string-muotoinen MAC-osoite merkkijonotaulukoksi  
  pinMode(trigPin1, OUTPUT);                  //Määritetään ESP32:n trigPin1 vastaava GPIO-portti ulostuloksi
  pinMode(echoPin1, INPUT);                   //Määritetään ESP32:n echoPin1 vastaava GPIO-portti sisääntuloksi

  initialDistance = Measure_distance(trigPin1, echoPin1); //Mitataan etäisyys
}


//Loop-koodi ajetaan ohjelman käynnistyttyä jatkuvasti
void loop()
{
  delay(70);
  cm1 = Measure_distance(trigPin1, echoPin1); //Mitataan etäisyys
  
  //cm2 = Measure_distance(trigPin1, echoPin1); //Mitataan etäisyys
  
  if (cm1 < 0.75*initialDistance && trigger == 0) //Jos etäisyydet ovat alle 100 cm ja liipaisu on 0
  {
    trigger = 1;                              //Asetetaan liipaisu ykköseksi. Tällöin joku tai jotain on anturin lähietäisyydellä  
    
    Serial.printf("etäisyys (triggering): %f\n", cm1);
    
  }
  if (trigger == 1 && cm1 > 0.75*initialDistance) //Jos liipaisu on 1 ja etäisyydet ovat suurempia kuin 150 cm
  {
    counter  = counter  + 1;                  //Kasvatetaan laskuria. Tällöin anturin editse on mennyt joku tai jotain
    counter2 = counter2 + 1;                  //Kasvatetaan laskuria 2.
    trigger = 0;                              //Asetetaan liipaisu nollaan
    //Serial.printf("etäisyys: %f\n", cm1);
    Serial.printf("Laskuri: %d\n", counter);  //Tulostetaan sarjaväylälle laskurin lukema anturilta 1
  }
  if (counter2 >= 20)                    //Jos laskurin 2 lukema on 20 tai enemmän, käynnistetään WiFi ja lähetetään lukema eteenpäin.
  {
    counter2 = 0;                             //Nollataan laskuri 2
    if (WiFi.status() != WL_CONNECTED)        //Jos WiFi:n tila on muuta kuin yhdistetty, niin
    {
      wifiReconnect();                        //Aliohjelmakutsu WiFin uudelleenyhdistämiseen
    }  
    if (WiFi.status() == WL_CONNECTED)        //Jos WiFin tila on yhdistetty, niin
    {
      Udp.begin(localUdpPort);                //Käynnistetään UDP-protokollan mukainen yhteys käyttäen määriteltyä porttia
      Udp.beginPacket(IP, localUdpPort);      //Käynnistetään datapaketin lähettäminen määriteltyyn IP-osoitteeseen ja porttiin 
      Udp.printf("Count:%d\n",counter);       //Lähetetään laskurin lukema anturilta 1
      Udp.printf("MAC:%3s\n",MAC_char);       //Lähetetään MAC-osoite
      Udp.endPacket();                        //Lopetetaan datapaketin lähettäminen
      Serial.printf("MAC: %3s\n",MAC_char);   //Tulostetaan MAC-osoite sarjaväylälle
      Udp.stop();                             //Pysäytetään UDP-protokollan mukainen yhteys
      delay(1000);                            //Odotetaan 1 sekunti, jotta UDP-prokotollan mukainen yhteys ehtii pysähtyä. Jos viivettä ei ole, niin WiFi-yhteyden uudelleen muodostaminen ei onnistu
      counter = 0;                            //Nollataan kävijälaskuri,kun tieto on saatu lähetettyä eteenpäin
      Disconnect();                           //Aliohjelmakutsu WiFi-yhteyden katkaisuun
    }
  }
}

//Funktiot

//Etäisyyden mittaus
float Measure_distance(const int trigPin, const int echoPin)
{
  long duration;                              //Etäisyysanturin vastaanottaman kaikupulssin kesto
  long cm;                                    //Etäisyysmuuttuja
  
  digitalWrite(trigPin, LOW);                 //Nollataan aluksi anturin triggauspinni asettamalla se nollaksi
  delayMicroseconds(2);                       //Odotetaan 2 mikrosekuntia
  digitalWrite(trigPin, HIGH);                //Lähetetään anturille 10 mikrosekunnin liipaisupulssi, jotta etäisyyden mittaus käynnistyy ja anturi lähettää 8 40 kHz:n ultraäänipulssia 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);          //Vastaanotetaan kaikupulssi, jonka pituus on verrannollinen mitattuun etäisyyteen
  cm = microsecondsToCentimeters(duration);   //Muutetaan pulssin kesto mitatuksi etäisyydeksi
  return cm;
}


//Muunnetaan pulssin kesto vastaavaksi mitatuksi etäisyydeksi (cm)
long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 58;                   //Mitattu etäisyys = (pulssin pituus x ilman nopeus (340 m/s))/2 tai uS / 58 = senttimetriä
}


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
  delay(1000);                                //Odotetaan 1000 ms, jotta yhteys ehtii sammua
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
  }
  else
  {
    Serial.println(F("Could not connect!"));
  }
}


//WiFi-yhteyden katkaiseminen
void Disconnect()                             
{
  unsigned long timer1 = millis();            //Alustetaan timer1-laskuri ja asetetaan sen arvoksi ajanhetki (aika, joka on kulunut ohjelman käynnistymisestä)
  WiFi.disconnect();                          //Katkaistaan WiFi-yhteys. Funktiossa ei saa käyttää tässä "true" valintaa. Aiheuttaa sen, että yhteyttä ei saada katkaistua.
  while (WiFi.status() != WL_DISCONNECTED && millis() - timer1 < 5000UL)    //Jos WiFin tila on muuta kuin katkaistu ja aikaa on mennyt vähemmän kuin 5 sekuntia, niin suoritetaan alla oleva
  {
     Serial.printf(".");
     delay(500);
  }
  Serial.println(F("WiFi disconnected"));
}
