#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#include <ArduinoJson.h>

FirebaseData firebaseData;
String path = "/";
FirebaseJson json;

#define _SSID "iPhone"        // Your WiFi SSID
#define _PASSWORD "123456Aa"    // Your WiFi Password
#define FIREBASE_HOST  "https://mobilerobot-c559e-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH  "FUWxHEqYo9l3RnGed1zcPqyo4OES85sEbzHUYi4z"
int ManualControl = 0;
String SendEsp = "";

int DC1 = 0;
int DIR1 = 0;
int DC2 = 0;
int DIR2 = 0;
int DC3 = 0;
int DIR3 = 0;

long last = 0;
int temp = 11; 

float vx, vy;
float v1, v2, v3;
float Vtt = 50; 

void setup() {
//  // put your setup code here, to run once:
  Serial.begin(115200);

  /****************************************/
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(1000);

  // Connect to WiFi
  Serial.println();
  Serial.println();
  Serial.print("Connecting to: ");
  Serial.println(_SSID);
  WiFi.begin(_SSID, _PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("-");
  }

  Serial.println("");
  Serial.println("WiFi Connected");

  // Print the IP address
  Serial.print("Use this URL to connect: ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
  digitalWrite(LED_BUILTIN, HIGH);
  /****************************************/

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  //Firebase.reconectWifi(true);
  if (!Firebase.beginStream(firebaseData, path))
  {
    Serial.println("REASON: " + firebaseData.errorReason());
    Serial.println();
  }
}

void loop() {
//   put your main code here, to run repeatedly:
    if (Firebase.getInt(firebaseData, path + "ManualControl")) ManualControl = firebaseData.intData();
    switch (ManualControl)
    {
      case 0:
        DC1 = 0; DC2 = 0; DC3 = 0; 
        DataJson(String (DC1), String (DC2), String (DC3));
        Firebase.setInt(firebaseData, path + "/ManualControl",temp);
        break;
      case 1:
        Move(45);
//        DC1 = 0; DIR1 = 0; DC2 = 100; DIR2 = 0; DC3 = 100; DIR3 = 0;
        DataJson(String (DC1), String (DC2), String (DC3));
        Firebase.setInt(firebaseData, path + "/ManualControl",temp);
        break;
      case 2:
        Move(90);
//        DC1 = 0; DIR1 = 1; DC2 = 200; DIR2 = 1; DC3 = 200; DIR3 = 1;
        DataJson(String (DC1), String (DC2), String (DC3));
        Firebase.setInt(firebaseData, path + "/ManualControl",temp);
        break;
      case 3:
        Move(135);
//        DC1 = 0; DIR1 = 0; DC2 =300; DIR2 = 1; DC3 = 300; DIR3 = 0;
        DataJson(String (DC1), String (DC2), String (DC3));
        Firebase.setInt(firebaseData, path + "/ManualControl",temp);
        break;
      case 4:
        Move(0);
//        DC1 = 0; DIR1 = 0; DC2 =300; DIR2 = 1; DC3 = 300; DIR3 = 0;
        DataJson(String (DC1), String (DC2), String (DC3));
        Firebase.setInt(firebaseData, path + "/ManualControl",temp);
        break;
      case 5:
        Move(180);
//        DC1 = 0; DIR1 = 0; DC2 =300; DIR2 = 1; DC3 = 300; DIR3 = 0;
        DataJson(String (DC1), String (DC2), String (DC3));
        Firebase.setInt(firebaseData, path + "/ManualControl",temp);
        break;
      case 6:
        Move(315);
//        DC1 = 0; DIR1 = 0; DC2 =300; DIR2 = 1; DC3 = 300; DIR3 = 0;
        DataJson(String (DC1), String (DC2), String (DC3));
        Firebase.setInt(firebaseData, path + "/ManualControl",temp);
        break;
      case 7:
        Move(270);
//        DC1 = 0; DIR1 = 0; DC2 =300; DIR2 = 1; DC3 = 300; DIR3 = 0;
        DataJson(String (DC1), String (DC2), String (DC3));
        Firebase.setInt(firebaseData, path + "/ManualControl",temp);
        break;
      case 8:
        Move(225);
//        DC1 = 0; DIR1 = 0; DC2 =300; DIR2 = 1; DC3 = 300; DIR3 = 0;
        DataJson(String (DC1), String (DC2), String (DC3));
        Firebase.setInt(firebaseData, path + "/ManualControl",temp);
        break;
      case 9:
        DC1 = 50; DIR1 = 1; DC2 =50; DIR2 = 1; DC3 = 50; DIR3 = 1;
        DataJson(String (DC1), String (DC2), String (DC3));
        Firebase.setInt(firebaseData, path + "/ManualControl",temp);
        break;
      case 10:
        DC1 = 50; DIR1 = 0; DC2 =50; DIR2 = 0; DC3 = 50; DIR3 = 0;
        DataJson(String (DC1), String (DC2), String (DC3));
        Firebase.setInt(firebaseData, path + "/ManualControl",temp);
        break;
    }
}
void DataJson(String DC1, String DC2, String DC3)
{
  SendEsp = "";
  SendEsp = "{\"DC1\":\"" + String(DC1) + "\"," +
            "\"DIR1\":\"" + String(DIR1) + "\"," +
            "\"DC2\":\"" + String(DC2) + "\"," +
            "\"DIR2\":\"" + String(DIR2) + "\"," +
            "\"DC3\":\"" + String(DC3) + "\"," +
            "\"DIR3\":\"" + String(DIR3) + "\"}";
  Serial.println(SendEsp);
}
void Move(int deg)
{
  vx = cos(deg * PI / 180);
  vy = sin(deg * PI / 180);
  
  v1 = vx;
  v2 = sqrt(3) / 2 * vy - 0.5 * vx;
  v3 = -(sqrt(3) / 2 * vy + 0.5 * vx);
  
  DIR1 = v1 < 0 ? 1 : 0;
  DIR2 = v2 < 0 ? 1 : 0;
  DIR3 = v3 < 0 ? 1 : 0;
  
  v1 = Vtt * abs(v1);
  v2 = Vtt * abs(v2);
  v3 = Vtt * abs(v3);

  DC1 = v1;
  DC2 = v2;
  DC3 = v3;
}
