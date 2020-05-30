/*
      Coded for the Wemos D1 Mini (in board manager select)
      - Lolin(WEMOS) D1 R2 & Mini
      - CPU Freq: 160Mhz
      - Erase Flash: All Flash Contents

      Requires the Pololu VL530l0X Lib
      https://github.com/pololu/vl53l0x-arduino
      
      COnect VL530l0X pins as follows:
      -VCC -> ESP8266 3.3V
      -GND -> ESP8266 GND
      -SCK(CLK) -> ESP8266 D1
      -SDA(DAT) -> ESP8266 D2
      -XSHUT Not connected
      -SPIO1 Not connected
	  
	   Sensors I2C Address is hard coded to 41 (0x29)
	  
 */

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X TOFSenseModule;
unsigned short int readValue=0, peakValue=0, cnt=0, validRead=0;
float scaledValue=0;
//{Closest VALID distance from sensor, Furthest VALID distance from sensor,0,0}
float readMargins[4] = {20, 500, 0, 0};
float processedRange, broadProcessedRange;
//Clock and DATA pins for i2c buss
byte I2C_CLOCK_PIN = D1;
byte I2C_DATA_PIN = D2;


//Network Stuff
//Please enter corect details for wifi details
const char * ssid = "WIFINETWORKNAME";
const char * password = "WIFIPASSWORD";
WiFiUDP Udp;
const unsigned int udpTXPort = 1000;
//Each sensor node needs its own unique ID
const byte nodeID = 1, packetSize=4;
byte dataToSend[packetSize] = {nodeID, 0, 0, 0};
//Before uploading this code to each sensor node, set up the server node and obtanin its IP address and enter it here
//Im assuimng the IP address of the server is 192.168.1.250
IPAddress serverAddress(192,168,1,250);

void setup() 
{
  //Enable Debug to Serial Port 1
  Serial.begin(115200);
  Serial.printf("\r\n");

  //Eable WIFI
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
        delay(100);
        Serial.print(".");
  }
  Serial.printf("\r\nTOF Sense Module\t%d\tis ONLINE\r\nCurrent Ip Address:\t", nodeID);
  Serial.print(WiFi.localIP());
  Serial.print("\r\n");
  
  //Init I2C
  Wire.begin(I2C_DATA_PIN, I2C_CLOCK_PIN);
  //Init TOF Module
  TOFSenseModule.init();
  TOFSenseModule.setTimeout(500);
  TOFSenseModule.startContinuous();
  
  //set up sense margins
  readMargins[2] = readMargins[1]-readMargins[0];
}

void loop() 
{
    readValue = TOFSenseModule.readRangeContinuousMillimeters();
    //Id reading is larger than the MIN distance from sensor AND smaller than MAX distance from sensor asume the reading is valid
	if(readValue>readMargins[0] && readValue<readMargins[1])
    {
      //Create a Scaled value based on this reading from 0 to 100% based on the range which is MAXDistance-MINDistance
	  processedRange = ((readValue-readMargins[0])/readMargins[2])*100;
      //Create a Scaled value based on this reading from 0 to 255 based on the range which is MAXDistance-MINDistance
	  broadProcessedRange = ((readValue-readMargins[0])/readMargins[2])*255;
      //Reading is valid
	  validRead=1;
    }
    else
    {
      //reading from sensor is out of bounds set in readMargins 
	  validRead=0;
      processedRange = 0;
      broadProcessedRange=0;
    }
  dataToSend[1] = processedRange;
  dataToSend[2] = broadProcessedRange;
  dataToSend[3] = validRead;
  if(validRead)
  {
    Serial.printf("\r\n%d\t%f\t%d", readValue, processedRange, validRead);
  }
  txData();
  //100ms delay gives us 10 reads per second
  delay(100);
}

void txData()
{
  Udp.beginPacket(serverAddress, udpTXPort);
  Udp.write(dataToSend, packetSize);
  Udp.endPacket();
}
