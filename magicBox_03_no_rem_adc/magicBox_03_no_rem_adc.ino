#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ESP8266FtpServer.h>
#include <WiFiUdp.h>
#include "FS.h"

//for LED status
#include <Ticker.h>

ESP8266WebServer server(80);
FtpServer ftpSrv;   //set #define FTP_DEBUG in ESP8266FtpServer.h to see ftp verbose on serial

//aa
//////////////////
unsigned int localPort = 12345;      // local port to listen for UDP packets
unsigned int remotePort_ = 55056;      // local port to listen for UDP packets

IPAddress IP_Remote(192,168,1,255); 

const int BUFFER_SIZE = 512;
char packetBuffer[BUFFER_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;
/////////////////


int chipId=-1;

Ticker ticker;
int wifiStatus=-1;

#define BUILTIN_LED 2
#define LIGHT_PIN 0
int lightVal=0, gpioVal16=0,gpioVal14=0,gpioVal12=0,gpioVal13=0;

bool gpioPinMode16=false, gpioPinMode14=false, gpioPinMode12=false, gpioPinMode13=false ;


#include "pageHandlers.h"

String tags[] ={"chipid","runtime","wifistatus","cpufrequency","sketchstatus","adc",
                "light","gpio16","gpio14","gpio12","gpio13" };
int tagsSize = 19;
int tmpSendCount=0;

String lastReceivedSerial="";

void tick()
{
  //toggle state
  int state = digitalRead(BUILTIN_LED);  // get the current state of GPIO1 pin
  digitalWrite(BUILTIN_LED, !state);     // set pin to the opposite state
}

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.1, tick);
}

void updateWifiStatus(){
  wifiStatus=WiFi.status();
}
void checkConnection(){
  // attempt to connect to WiFi network
  if(wifiStatus!= WL_CONNECTED){
    while ( WiFi.status() != WL_CONNECTED) {
      //WiFiManager
      //Local intialization. Once its business is done, there is no need to keep it around
      WiFiManager wifiManager;
      //reset settings - for testing
      //wifiManager.resetSettings();
      
      //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
      wifiManager.setAPCallback(configModeCallback);
    
      //fetches ssid and pass and tries to connect
      //if it does not connect it starts an access point with the specified name
      //here  "AutoConnectAP"
      //and goes into a blocking loop awaiting configuration
      if (!wifiManager.autoConnect()) {
        Serial.println("failed to connect and hit timeout");
        //reset and try again, or maybe put it to deep sleep
        ESP.reset();
        delay(1000);
      }
    
      //if you get here you have connected to the WiFi
      Serial.println("connected...yeey :)");
      ticker.detach();
      wifiStatus=WiFi.status();
      //keep LED on
      digitalWrite(BUILTIN_LED, HIGH);
      
  //    
    }
  }
}
void setLightValue(int val){
  lightVal=val;
  analogWrite(LIGHT_PIN, map(lightVal, 0,100, 0,1023) );
}

//serialData.xml?data


void getData_serial(){
  String data="";
  for ( uint8_t i = 0; i < server.args(); i++ ) {
    if(server.argName(i)=="data"){
      data=server.arg(i);
    }
  }
  if(data!=""){
    if(data.indexOf("</br>") >0 ){
      data.replace("</br>","");
      Serial.println(data);
    }
    else
      Serial.print(data);
  }
  server.send(200, "text/plain", lastReceivedSerial);
  lastReceivedSerial="";
}

void getData(){
  String cmd="";
  for ( uint8_t i = 0; i < server.args(); i++ ) {
    if(server.argName(i)=="cmd"){
      cmd=server.arg(i);
    }
  }
  if(cmd!=""){
    int index0=0,index1=0,index2=0, val=0;
    String cmd_="", k="", v="";
    index0=cmd.indexOf(':');
    cmd_= cmd.substring(0, index0);
    index1=cmd.indexOf(':', index0+1);
    k=cmd.substring(index0+1, index1);
    index2=cmd.indexOf(':', index1+1);
    v=cmd.substring(index1+1, index2);
    if( cmd_.indexOf("SETGPIOPINMODE") !=-1 ){
      if(v.toInt() ==1){
        pinMode(k.toInt(), OUTPUT);
        Serial.println("#pinMode:" + k + ":" + v);
      }
      else{
        pinMode(k.toInt(), INPUT);
        Serial.println("$pinMode:" + k + ":" + v);
      }

      switch (k.toInt() ){
        case 16:
          gpioPinMode16=v.toInt();
        break;
        case 14:
          gpioPinMode14=v.toInt();
        break;
        case 12:
          gpioPinMode12=v.toInt();
        break;
        case 13:
          gpioPinMode13=v.toInt();
        break;
      }
    }
    if( cmd_.indexOf("SETGPIO") !=-1 ){
      analogWrite( k.toInt(), map(v.toInt(), 0,100, 0,1023) );
      switch (k.toInt() ){
        case 16:
          gpioVal16=v.toInt();
        break;
        case 14:
          gpioVal14=v.toInt();
        break;
        case 12:
          gpioVal12=v.toInt();
        break;
        case 13:
          gpioVal13=v.toInt();
        break;
      }
    }
    if(k == "light"){
      setLightValue(v.toInt());
    }

    Serial.print (cmd_);
    Serial.print ("#");
    Serial.print (k);
    Serial.print ("#");
    Serial.print (v);
    Serial.println ("#");
    Serial.print (val);
    Serial.println ("#");
    
  }
  
  
  tmpSendCount++;
  if(tmpSendCount>100){
    Serial.println(">");
    tmpSendCount=0;
  }
  else{
//    Serial.print(">");
  }
  
  ///////String tags[] ={"chipid","runtime","wifistatus","updateinterval"};
  if(gpioPinMode16==0)
    gpioVal16=digitalRead(16);
  if(gpioPinMode14==0)
    gpioVal14=digitalRead(14);
  if(gpioPinMode12==0)
    gpioVal12=digitalRead(12);
  if(gpioPinMode13==0)
    gpioVal13=digitalRead(13);
  
  String a= "{";
  for(int i=0; i<tagsSize; i++){
    if(i==0){
      a=a+"\""+tags[i]+"\":"  +"\""+chipId+"\"" ;
    }
    else if(i==1){
      a=a+",\""+tags[i]+"\":"  +"\""+millis()+"\"" ;
    }
    else if(i==2){
      a=a+",\""+tags[i]+"\":"  +"\""+ "---" +"\"" ;
    }
    else if(i==3){
      a=a+",\""+tags[i]+"\":"  +"\""+ ESP.getCpuFreqMHz() +"\"" ;
    }
    else if(i==4){
      a=a+",\""+tags[i]+"\":"  +"\""+ ESP.getSketchSize() +" / "+ ESP.getFreeSketchSpace() +"\"" ;
    }
    else if(i==5){
      a=a+",\""+tags[i]+"\":"  +"\""+ analogRead(A0) +"\"" ;
    }

    
    //"light","gpio1","gpio2","gpio3","gpio4","gpio5","gpio12","gpio13","gpio14","gpio15"
    else if(i==6){
      a=a+",\""+tags[i]+"\":"  +"\""+ lightVal +"\"" ;
    }

    
    else if(i==7){  //gpio16
      a=a+",\""+tags[i]+"\":"  +"\""+ gpioVal16 +"\"" ;
    }
    else if(i==8){
      a=a+",\""+tags[i]+"\":"  +"\""+ gpioVal14 +"\"" ;
    }
    else if(i==9){
      a=a+",\""+tags[i]+"\":"  +"\""+ gpioVal12 +"\"" ;
    }
    else if(i==10){
      a=a+",\""+tags[i]+"\":"  +"\""+ gpioVal13 +"\"" ;
    }

/*
 * "chipid","runtime","wifistatus","cpufrequency","sketchstatus","adc",
                "light","gpio16","gpio14","gpio12","gpio13"
                
 */
  


    
  }
  a=a+"}";
  server.send(200, "text/plain", a);
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
//  Serial.setDebugOutput(true);
  //set led pin as output
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.6, tick);

  checkConnection();
  ticker.attach(2, updateWifiStatus);

  chipId=ESP.getChipId();
  Serial.println(chipId);
  
  server.on("/", handleHome);
  server.on("/data.xml", getData);
  server.on("/serialData.xml", getData_serial);
  
  server.begin();
  Serial.println("a");
  
//  SPIFFS.format();
  if (SPIFFS.begin()) {
//      SPIFFS.format();
//      Serial.println("Spiffs formatted");
      Serial.println("SPIFFS opened!");
      ftpSrv.begin("esp8266","esp8266");    //username, password for ftp.  set ports in ESP8266FtpServer.h  (default 21, 50009 for PASV)
  }
  Serial.println("Udp.begin");
   Udp.begin(localPort);
}



char byte_;
int sts=0;
void loop() {
  // put your main code here, to run repeatedly:
  checkConnection();
  server.handleClient();
  ftpSrv.handleFTP();  

  while (Serial.available() > 0) {
                if(sts==0){
                  lastReceivedSerial="";
                  sts++;
                }
                // read the incoming byte:
                //incomingByte = Serial.read();
                byte_=(char)Serial.read();
                lastReceivedSerial = lastReceivedSerial+ byte_;
                
                
                if(byte_=='\n'){
                  Udp.beginPacket(IP_Remote, remotePort_);
                  lastReceivedSerial.toCharArray(packetBuffer, BUFFER_SIZE);
                  Udp.write(packetBuffer);
                  Udp.endPacket();
                  sts=0;
                }
                
  }
   
}
