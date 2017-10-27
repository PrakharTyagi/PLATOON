#define ETH2

#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>

Servo steering_servo;
Servo throttle_servo;
Servo gear_servo;

typedef struct {
  unsigned int timestamp[4]; // Timestamp is 2 uint32: s + us. The arduino doesn't need to care about this.
  unsigned int seqNum;
} pack_header;

typedef struct {
  pack_header head;
  int throttle;
  int steering;
  int gear;
} udp_msg;

int steering_pin = 5;
int throttle_pin = 6;
int gear_pin = 3;

#ifdef ETH2
IPAddress ip(192, 168, 1, 193);
#else
IPAddress ip(192, 168, 1, 194);
#endif

unsigned int localPort = 2390;
char packetBuffer[255];

int status = WL_IDLE_STATUS;
char ssid[] = "SML"; //  your network SSID (name)
char pass[] = "SML4admin.";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)


WiFiUDP Udp;

int packetSize;
String datReq;
String temp;

void setup(){
  steering_servo.attach(steering_pin);
  throttle_servo.attach(throttle_pin);
  gear_servo.attach(gear_pin);

  Serial.begin(115200);
  while(!Serial){
      ; //wait for serial port to connect
  }
  
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }
  
  WiFi.config(ip); 

  String fv = WiFi.firmwareVersion();
  if (fv != "1.1.0") {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid,pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);
  
  delay(500);
  
      
  }

void loop(){
    packetSize = Udp.parsePacket();
    if (packetSize){
        int len = Udp.read(packetBuffer, 255);
        if (len > 0){
            packetBuffer[len] = 0;
        }
        if (packetSize != sizeof(udp_msg)) {
            Serial.print("Received invalid packetlength: ");
            Serial.println(packetSize, DEC);
            return;
        }

        udp_msg * msg = (udp_msg*) packetBuffer;

        throttle_servo.writeMicroseconds(msg->throttle);
        steering_servo.writeMicroseconds(msg->steering);
        gear_servo.write(msg->gear);
        
        Udp.beginPacket(Udp.remoteIP(), localPort);
        Udp.write((char*)&msg->head, sizeof(pack_header));
        Udp.endPacket();

        Serial.print(msg->throttle, DEC);
        Serial.print(" , ");
        Serial.print(msg->steering, DEC);
        Serial.print(" , ");
        Serial.print(msg->gear, DEC);
        Serial.println("");
        
    }
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
