//Jani Raami
//Koodi on tarkoitettu käytettäväksi Heltecin LoRa-piirissä. Koodilla LoRa-piiriä käytetään erilaisten
//viestien (GPS-sijaintitieto, lämpötila/kosteus, Bajamajojen pinnankorkeus, yms.) vastaanottimena.

//GPS-piiri:  NEO-6M GPS
//LoRa-piiri: Heltec WiFi LoRa 32 

//Yhteysasetukset:
//Kortti:           Heltec_WIFI_LoRa_32
//Flash Frequency:  80 MHz
//Upload Speed:     921600 (Jos ei toimi, kokeile 115200)

//Sisällytetään seuraavat kirjastot koodiin
#include <SPI.h>                                                  //Sarjayhteys kommunikointi
#include <WiFi.h>                                                 //WiFi-verkon käyttö
#include <LoRa.h>                                                 //Radioverkon kommunikointi
#include <U8x8lib.h>                                              //LoRan OLED-näytön käyttö
#include "time.h"                                                 //Ajastus


//Määritetään WiFi-Lora 32:n pinneille nimet ja asetukset
#define SCK     5                                                 //GPIO5  -- SX1278's SCK
#define SS      18                                                //GPIO18 -- SX1278's CS
#define RST     14                                                //GPIO14 -- SX1278's RESET
#define MISO    19                                                //GPIO19 -- SX1278's MISO
#define DI0     26                                                //GPIO26 -- SX1278's IRQ(Keskeytyspyyntö)
#define MOSI    27                                                //GPIO27 -- SX1278's MOSI
#define BAND    869.65E6                                           //Käytettävä taajuus 869.5 MHz. Taajuusalueella 869.400 - 869.650 MHz saa säteilyteho olla enintään 500 mW ERP ja toimintasuhde enintään 10 %
#define signalBandwidth 31.25E3                                   //Asetetaan kaistanleveydeksi 31.25 kHz
#define PABOOST true                                              //Asetetaan Power Amplifier Boost päälle

// Wifi-Lora 32:n OLED-näytön määrittely
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

//Muuttujien määrittely
const char* ssid = "Toimisto";                                   //WiFi-yhteyden SSID-tunnus
const char* password = "toimistorotta";                              //WiFi-yhteyden salasana
char dataChar;                                                    //Yksittäisen merkin muuttuja
int len;                                                          //Datapaketin pituus muuttuja
int i;                                                            //Laskuri
char incomingPacket[100];                                         //Merkkijonopuskuri saapuvalle datapaketille, jonka pituus on 100 merkkiä
const char* endPoint = "52.164.201.186";                            //Osoite, johon data lähetetään LoRalta (Raspberry)
const int Port = 80;                                              //Portti, johon data lähetetään LoRalta (Raspberry)
String JSON_message;                                              //.JSON-viestin muuttuja
byte wfConnectTry = 0;                                            //WiFin yhteysyrityslaskuri
byte EPConnectTry = 0;                                            //End pointin yhteysyrityslaskuri

WiFiClient client;                                                //Luodaan WiFiClient-objekti ja nimetään se clientiksi

String Create_JSON_Message(char incomingPacket[100]);
String add_quates_to_token(char* token);
boolean isFloat(String tString);
void EndPoint_Connect();
void EndPoint_ReConnect();
void wifiReconnect();
void wifiStart();

//Setup-koodi ajetaan kerran ohjelman käynnistyessä
void setup() {
  u8x8.begin();                                                   //WiFi LoRa 32:n OLED-näytön kommuninoinnin käynnistäminen
  u8x8.setFont(u8x8_font_chroma48medium8_r);                      //Asetetaan OLED-näytölle fontti
  Serial.begin(115200);                                           //Sarjayhteyden käynnistäminen nopeudella 115200
  Serial.println();                                               //Tyhjän rivin tulostus
  Serial.println("LoRa Receiver");                                //Tulostetaan sarjayhteydellä LoRa Receiver
  u8x8.drawString(0, 0, "LoRa Receiver");                         //Tulostetaan OLED-näytölle LoRa Receiver
  wifiStart();                                                    //Aliohjelmakutsu, jolla käynnistetään WiFi-yhteys 
  EndPoint_Connect();                                             //End pointin yhdistäminen
  SPI.begin(SCK, MISO, MOSI, SS);                                 //SPI väylän käynnistys ja pinnien asetukset. Nämä pitää tehdä ennen LoRa.begin komentoa            
  LoRa.setPins(SS, RST, DI0);
  
  
  while (!LoRa.begin(BAND,PABOOST))                               //Käynnistetään LoRa-yhteys asetetulla taajuudella ja PABOOST kytkettynä                       
  {                                             
    Serial.println("Starting LoRa failed!");                      //Tulostetaan sarjayhteydellä, jos LoRa-yhteydenmuodostus epäonnistuu
    u8x8.drawString(0, 5, "LoRa failed");                         //Tulostetaan näyttöön, jos LoRa-yhteydenmuodostus epäonnistuu
    delay(2000);                                                  //Odotetaan 2 sekuntia ja yritetään uudestaan
  }                                       
    Serial.println("Starting LoRa OK.");                          //Tulostetaan sarjayhteydellä, jos LoRa-yhteyden muodostus onnistuu
    Serial.println();                                             //Tulostetaan sarjayhteydellä tyhjä rivi
    u8x8.drawString(0, 5, "Starting LoRa OK");                    //Tulostetaan OLED-näytölle, jos LoRa-yhteyden muodostus onnistuu
                                                                  //Alla olevat kolme LoRa asetusta tulee tehdä LoRa.begin komennon jälkeen
    LoRa.crc();                                                   //Asetetaan LoRa-viestin CRC-tarkistuskoodaus ominaisuus päälle
    LoRa.setSpreadingFactor(7);                                   //Asetetaan LoRa-yhteyden leviämistekijäksi 7
    LoRa.setSignalBandwidth(signalBandwidth);                     //Asetetaan määritelty kaistanleveys
    delay(5000);                                                  //Odotetaan 5 sekuntia, jotta OLED-näytön viestit ehtii lukea
    for (i=1; i <= 5; i++)                                        //Tyhjennetään OLED-näytön rivit 1-5
    {
    u8x8.clearLine(i);                          
    }  
}


//Loop-koodi ajetaan ohjelman käynnistyttyä jatkuvasti
void loop() 
{   
  while (WiFi.status() != WL_CONNECTED || wfConnectTry >=5)       //Jos WiFi:n tila on muuta kuin yhdistetty tai yhteysyrityksiä on 5 tai enemmän, niin
  {
    wfConnectTry++;                                               //lisätään yhteysyrityslaskuria yhdellä
    wifiReconnect();                                              //Aliohjelmakutsu WiFin uudelleenyhdistämiseen
    if (WiFi.status() == WL_CONNECTED)  
    {
      EndPoint_ReConnect();                                       //Aliohjelmakutsu end pointin uudelleenyhdistämiseen (Jos WiFi-yhteys katkeaa, niin myös end point yhteys pitää uudelleenkytkeä) 
    }
  }  
  while (!client.connected() || EPConnectTry >=5)                 //Jos End pointin tila on muuta kuin yhdistetty tai yhteysyrityksiä on 5 tai enemmän, niin
  {
    EPConnectTry++;                                               //lisätään yhteysyrityslaskuria yhdellä
    EndPoint_ReConnect();                                         //Aliohjelmakutsu end pointin uudelleenyhdistämiseen                                           
  }
  
  memset(incomingPacket,0,100);                                   //Tyhjennetään datapaketin puskuri, jotta datapaketteihin ei ala kertymään ylimääräisiä merkkejä
  len = LoRa.parsePacket();                                       //Aloittaa prosessoimaan seuraavaa saatavilla olevaa datapakettia, tarkistaa sen olemassaolon ja palauttaa datapaketin koon
  if (len > 0 && len <= 99)                                       //Jos datapaketissa on sisältöä
  {  
    for (i=0; i < len; i++)                                       //Luetaan datapaketin merkit merkkitaulukkoon incomingPacket
    {                  
      dataChar=((char)LoRa.read());                               //Luetaan datapaketista tavu ja muutetaan se merkiksi
      incomingPacket[i]=dataChar;                                 //Sijoitetaan merkki indeksin i kohdalle datapaketin puskuriin
    }
    for (i=1; i <= 7; i++)                                        //Tyhjennetään OLED-näytön rivit 1-7
    {
    u8x8.clearLine(i);                          
    }
    Serial.printf("Datapaketin koko: %d", len);                   //Tulostetaan sarjayhteydellä datapaketin koko tavuissa
    Serial.print(" tavua \n");
    Serial.println("Datapaketin sisältö: ");               
    Serial.print(incomingPacket);                                 //Tulostetaan datapaketti sarjayhteydellä
       
    JSON_message = Create_JSON_Message(incomingPacket);           //Esimerkki .JSON-viestistä: {"Lat":60.275055,"Lon":25.030594,"HDOP":0.87,"MAC":"B4E62D8C918D"}

    if (WiFi.status() == WL_CONNECTED && client.connected() == 1) //Jos yhteys WiFiin ja end pointtiin on olemassa, niin suoritetaan alla oleva koodi. Näin varmistetaan vielä, että yhteydet ovat varmasti OK.
    {
      if (len == 55) // GPS
      {
        client.print("POST /api/gps?code=j2IieLtc82CcCa/GV6Pga9GYXBzPLmsd3zwty/X2/BWP/upKr2NNyQ== HTTP/1.1\r\n");  
      }
      else if (len == 63 || len == 65) // TEMP only
      {
        client.print("POST /api/temphum?code=4ht6HTay8Ly4KS4lduZBhM/FPyX3casTrwHIBrG7spWv6MsyPkDO2Q== HTTP/1.1\r\n");  
      }
      else if (len == 74) // TEMP and HUM
      {
        client.print("POST /api/temphum?code=4ht6HTay8Ly4KS4lduZBhM/FPyX3casTrwHIBrG7spWv6MsyPkDO2Q== HTTP/1.1\r\n");  
      }
      else if (len == 64) // DIST
      {
        client.print("POST /api/dist?code=PuMj4OHQv2xTgqUDpadsbJaCJnsSYHA/ncwzICXuC1Wrhgursxe1pQ== HTTP/1.1\r\n");  
      }
      else
      {
         client.print("POST /api/test?code=0U7CkT1aINiWD3HEUx9dFnDPkK5TqoWdOjjeYpGHfKEz3EbeLnNUmA== HTTP/1.1\r\n");  
      }

      client.print("Content-Type: application/json\r\n");
      client.print("User-Agent: Partio-IOT\r\n");
      client.print("Accept: */*\r\n");
      client.print("Cache-Control: no-cache\r\n");
      client.print("Host: partiot.azurewebsites.net\r\n");
      client.print("accept-encoding: gzip, deflate\r\n");
      client.print("content-length: ");
      client.print(JSON_message.length());
      client.print("\r\n");
      client.print("Connection: close\r\n");
      client.print("\r\n");
      
      //client.print("Host: partiot.azurewebsites.net\r\n");
      
      //client.println("Connection: keep-alive");
      //client.print("Content-Length: ");
      //client.println(JSON_message.length());                      //Lähetetään HTTP Post viestin sisällön pituus tavuissa
      //client.println();                                           //Tyhjä rivi tässä välissä on pakollinen HTTP Post -viestissä 
      client.println(JSON_message);                               //Lähetetään viestin varsinainen sisältö .JSON-muodossa
      delay(50);
      
      Serial.println(JSON_message);                               //Tulostetaan .JSON-viesti lisäksi sarjayhteydellä
      Serial.println();
      u8x8.setCursor(0, 2);                                       //Asetetaan OLED-näytön osoitin riville 2
      u8x8.printf("Saapunut data-");               
      u8x8.setCursor(0, 3);
      u8x8.printf("paketti: ", len);                              //Tulostetaan OLED-näytölle saapuneen datapaketin pituus tavuissa
      u8x8.printf("%d", len);                          
      u8x8.printf(" B");
      int rssi = LoRa.packetRssi();                               //Määritetään vastaanotetun datapaketin RSSI-luku
      u8x8.setCursor(0, 4);                                       //Asetetaan OLED-näytön osoitin seuraavalle riville
      u8x8.printf("RSSI: %d", rssi);                              //Tulostetaan vastaanotetun datapaketin RSSI näytölle
      u8x8.printf(" dBm");                                        //RSSI-luvun yksikkö on dBm
      float SNR = LoRa.packetSnr();                               //Määritetään vastanotetun datapaketin SNR-luku (signaali-kohinasuhde)
      u8x8.setCursor(0, 5);                                       //Asetetaan OLED-näytön osoitin seuraavalle riville
      u8x8.printf("SNR : %1.2f", SNR);                            //Tulostetaan vastaanotetun datapaketin estimoitu SNR näytölle
      u8x8.printf(" dB");                                         //SNR:n yksikkö on dB     
      delay(50);
    }
  }
}

//Funktiot:

String Create_JSON_Message(char incomingPacket[100])              //Funktio koodaa saapuneen datapaketin .JSON-muotoon
{
    char* token;                                                  //Dynaaminen merkkijonomuuttuja
    String sanoma;                                                //Muuttuja .JSON-viestin muodostamiseen
    char* tietue;                                                 //Muuttuja .JSON-viestin muodostamiseen
    char JSON_message[200];                                       //.JSON-viestin merkkijonomuuttuja
    tietue = new char[100];                                       //Luodaan 100 merkin pituinen dynaaminen merkkijonomuuttuja
    token = strtok(incomingPacket, ":");                          //Luetaan merkkijonosta incomingPacket merkkijono, joka päättyy merkkiin : . Eli luetaan ensimmäinen datatietue ja poistetaan se luettavasta merkkijonosta       
    if (token != NULL)                                            //Jos merkkijono token on muuta kuin tyhjä, niin suoritetaan alla oleva koodi (If-lause on sen takia, että jos viesti on korruptoitunut eikä
    {                                                             //kaksoispistettä löydy datapaketista, niin tällöin ohitetaan alla oleva koodi jotta vastaanotin ei kaadu)  
      sanoma = add_quates_to_token(token);                        //Lisätään datatietueeseen lainausmerkit (Jos se on muuta kuin liukuluku. Ensimmäinen datatietue sisältää aina numeerisen tiedon otsikon)
      sanoma.toCharArray(tietue,100);                             //Muutetaan String-muotoinen sanoma merkkijonomuuttujaksi  
      sprintf(JSON_message,"%s", "{");                            //Aloitetaan .JSON-viestin muodostaminen kaarisululla {
      sprintf(JSON_message + strlen(JSON_message),"%s:",tietue);  //Lisätään ensimmäinen datatietue ja kaksoispiste .JSON-viestiin
    }
    while (token != NULL)                                         //Luetaan merkkijonosta token datatietueita, kunnes merkkijono on tyhjä                         
    {      
      token = strtok(NULL, "\n");                                 //Luetaan merkkijonosta token merkkijono, joka päättyy merkkeihin \n (rivinvaihtoon). Eli luetaan datatietue ja poistetaan se luettavasta merkkijonosta 
      if (token != NULL)                                          //Jos merkkijono token on muuta kuin tyhjä, niin suoritetaan alla oleva koodi (If-lause on sen takia, että jos viesti on korruptoitunut eikä
      {                                                           //rivinvaihtoa löydy datapaketista, niin tällöin ohitetaan alla oleva koodi jotta vastaanotin ei kaadu)
        sanoma = add_quates_to_token(token);                      //Lisätään datatietueeseen lainausmerkit (jos se on muuta kuin liukuluku)
        sanoma.toCharArray(tietue,40);                            //Muutetaan String-muotoinen sanoma merkkijonomuuttujaksi
        sprintf(JSON_message + strlen(JSON_message),"%s",tietue); //Lisätään datatietue .JSON-viestiin
        token = strtok(NULL, ":");                                //Luetaan merkkijonosta token merkkijono, joka päättyy merkkiin : . Eli luetaan datatietue, ja poistetaan se luettavasta merkkijonosta
      }
      if (token != NULL)                                          //Jos merkkijono token on muuta kuin tyhjä, niin suoritetaan alla oleva koodi (If-lause on sen takia, että jos viesti on korruptoitunut
      {                                                           //tai luetaan viimeistä datatietuetta eikä kaksoispistettä löydy, niin tällöin ohitetaan alla oleva koodi jotta vastaanotin ei kaadu)  
        sanoma = add_quates_to_token(token);                      //Lisätään datatietueeseen lainausmerkit (jos se on muuta kuin liukuluku)
        sanoma.toCharArray(tietue,40);                            //Muutetaan String-muotoinen sanoma merkkijonomuuttujaksi
        sprintf(JSON_message + strlen(JSON_message),"%s", ",");   //Lisätään .JSON-viestiin , numeerisen datatietueen jälkeen
        sprintf(JSON_message + strlen(JSON_message),"%s:",tietue);//Lisätään datatietue ja kaksoispiste .JSON-viestiin
      }   
    } 
     sprintf(JSON_message + strlen(JSON_message),"%s", "}");      //Päätetäään .JSON-viestin muodostaminen kaarisululla }
     delete tietue;                                               //Poistetaan dynaaminen merkkijonomuuttuja, jotta LoRa-piirin muisti ei ala täyttymään
     sanoma = String(JSON_message);                               //Muutetaan .JSON-message merkkijono string-muotoiseksi, jotta se voidaan palauttaa funktiosta
     return sanoma;                                               //Palautetaan funktiosta muuttuja sanoma
}


String add_quates_to_token(char* token)                           //Funktio lisää merkkijonoon lainausmerkit, jos kyseessä on muuta kuin liukuluku
{
  char quates2string[100];                                        //Luodaan merkkijonomuuttuja
  quates2string[0] = 0;                                           //Asetetaan merkkijonomuuttuja tyhjäksi 
  if (isFloat(String(token)) == false)                            //Jos merkkijono ei ole liukuluku, niin 
    {
    sprintf(quates2string, "%s%s%s", "\"", token, "\"");          //Lisätään merkkijonoon lainusmerkit molemmille puolille
    }
    else                                                          //Muussa tapauksessa (on liukuluku)
    {
    sprintf(quates2string, "%s", token);                          //Ei lisätä lainausmerkkejä
    }
  return quates2string;                                           //Palautetaan merkkijonomuuttuja
}


boolean isFloat(String tString) {                                 //Funktio, jolla tunnistetaan onko kyseessä liukuluku (ottaa huomioon myös liukuluvun etumerkin)
  String tBuf;
  boolean decPt = false;
 
  if(tString.charAt(0) == '+' || tString.charAt(0) == '-') tBuf = &tString[1];
  else tBuf = tString; 
  for(int x=0;x<tBuf.length();x++)
  {
    if(tBuf.charAt(x) == '.') {
      if(decPt) return false;
      else decPt = true; 
    }   
    else if(tBuf.charAt(x) < '0' || tBuf.charAt(x) > '9') return false;
  }
  return true;
}


void EndPoint_Connect()                                           //End pointin yhdistäminen
{
  unsigned long timer1 = millis();                                //Alustetaan timer1-laskuri ja asetetaan sen arvoksi ajanhetki (aika, joka on kulunut ohjelman käynnistymisestä)
  Serial.printf("Connecting to %s", endPoint);                    //Tulostetaan sarjayhteydellä endPointin yhteydenmuodostus 
  for (i=3; i <= 7; i++)                                          //Tyhjennetään OLED-näytön rivit 3-7
  {
    u8x8.clearLine(i);                          
  }
  u8x8.drawString(0, 3, "Connecting EP");                         //Tulostetaan OLED-näyttöön EndPointin yhteydenmuodostus
  while (!client.connect(endPoint, Port) && millis() - timer1 < 5000UL) //Jos clientin tila on muuta kuin yhdistetty ja aikaa on mennyt vähemmän kuin 5 sekuntia, niin suoritetaan alla oleva
  {
    delay(500);                                                   //Odotetaan 500 ms
    Serial.print(".");                                            //Tulostetaan sarjayhteydellä "." kuvaamaan WiFi-yhteyden muodostumisen odottamista 
  }
  if (client.connected() == 1)
    {
      Serial.println("...Connected");                             //Kun yhteys endPointtiin on muodostunut, tulostetaan sarjayhteydellä "connected"
      u8x8.drawString(0, 4, "Connected");                         //Tulostetaan OLED-näytölle, jos yhteys endPointtiin muodostuu
      EPConnectTry = 0;
    }
    else
  {
    Serial.println(F(" Could not connect!"));
    u8x8.drawString(0, 4, "Not connected");                       //Tulostetaan OLED-näytölle, jos Client-yhteys ei muodostu
  }
}


void EndPoint_ReConnect()                                         //Aliohjelma End pointin uudelleen yhdistämiseen
{
  Serial.println(F("End Point connection lost."));
  u8x8.clearLine(2);                                              //Tyhjennetään OLED-näytön rivi 2
  u8x8.drawString(0, 2, "Connection lost");                       //Tulostetaan OLED-näyttöön end pointin yhteyden katkeaminen
  client.stop();                                                  //End pointin yhteyden katkaiseminen
  delay(500);                                                     //Odotetaan 500 ms
  EndPoint_Connect();                                             //End pointin uudelleenyhdistäminen
}


void wifiReconnect()                                              //WiFi-yhteyden uudelleenmuodostaminen                          
{
  WiFi.disconnect(true);                                          //Katkaistaan WiFi-yhteys, jotta sen voi muodostaa uudestaan
  delay(500);                                                     //Odotetaan 500 ms, jotta katkaisu aktivoituu
  wifiStart();                                                    //Luodaan WiFi-yhteys määriteltyyn verkkoon
}


void wifiStart()                                                  //WiFi-yhteyden muodostaminen                              
{
  unsigned long timer1 = millis();                                //Alustetaan timer1-laskuri ja asetetaan sen arvoksi ajanhetki (aika, joka on kulunut ohjelman käynnistymisestä)
  Serial.printf("Connecting to %s ", ssid);                       //Tulostetaan sarjayhteydellä määritellyn WiFi-verkon yhteydenmuodostus 
  for (i=1; i <= 7; i++)                                          //Tyhjennetään OLED-näytön rivit 1-7
  {
    u8x8.clearLine(i);                          
  }
  u8x8.drawString(0, 1, "Connecting WiFi");                       //Tulostetaan OLED-näyttöön WiFi-verkon yhteydenmuodostus
  WiFi.begin(ssid, password);                                     //Käynnistetään WiFi-yhteys
  while (WiFi.status() != WL_CONNECTED && millis() - timer1 < 7000UL)     //Jos WiFin tila on muuta kuin yhdistetty ja aikaa on mennyt vähemmän kuin 7 sekuntia, niin suoritetaan alla oleva
  {
    delay(500);                                                   //Odotetaan 500 ms
    Serial.print(".");                                            //Tulostetaan sarjayhteydellä "." kuvaamaan WiFi-yhteyden muodostumisen odottamista
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println(F(" Connected"));                              //Kun WiFi-yhteys on muodostunut, tulostetaan sarjayhteydellä "connected"   
    u8x8.drawString(0, 2, "Connected");                           //Tulostetaan OLED-näytölle, jos WiFi-yhteys muodostuu
    wfConnectTry = 0;                                             //Nollataan yhteysyritysten laskuri
  }
  else
  {
    Serial.println(F("Could not connect!"));
    u8x8.drawString(0, 2, "Not connected");                       //Tulostetaan OLED-näytölle, jos WiFi-yhteys ei muodostu
  }
}
