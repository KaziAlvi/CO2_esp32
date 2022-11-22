#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>

//CO2************************************************
//int analogPin = 2;
int pwmPin = 34;
float co2_ppm;

//DHT************************************************
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

//WIFI***********************************************
//const char* ssid = "Conference";
//const char* password = "Kfg#2!Fa5";

const char* ssid = "KFG_EMS_WIFI";
const char* password = "Kfg#2!EMS@Passw0rd";

//SERVER*********************************************
#define INFLUX "http://172.16.253.155:8086/write?db=kfg_ems"
#define DELAY 10000
String metrics;
void sendReadings(String post_data) {
  String dburl = INFLUX;
  HTTPClient http;
  http.begin(dburl);
  http.POST(post_data);
  http.end();
}

void setup() {
  //LED//////////////
  pinMode(21, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(2, OUTPUT);

  dht.begin();
  Serial.begin(115200);
  digitalWrite(5, HIGH);
  delay(50);
  digitalWrite(25, HIGH);
  delay(50);
  digitalWrite(26, HIGH);
  delay(50);
  digitalWrite(21, HIGH);

  WiFi.begin(ssid, password);
  Serial.print("Connecting..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print(F("Connected, IP address: "));
  Serial.println(WiFi.localIP());
  digitalWrite(2, HIGH);

  digitalWrite(5, LOW);
  delay(500);
  digitalWrite(25, LOW);
  delay(500);
  digitalWrite(26, LOW);
  delay(500);
  digitalWrite(21, LOW);

  Serial.println(F("DHTxx test!"));
  dht.begin();

  //CO2 SETUP*********************************
  pinMode(pwmPin, INPUT_PULLUP);
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);
  Serial.println("Pre-heat the Sensor.....1 min");
  delay(30000); // preheat the CO2 sensor for 1 minutes
  digitalWrite(21, LOW);
  delay(5000);
  digitalWrite(21, HIGH);
  Serial.println("Pre-heat the Sensor.....30 sec left");
  delay(25000);
  Serial.println("Done!");
  digitalWrite(21, LOW);
  delay(500);
}
void loop() {

  //CO2***********************************************

  float v = digitalRead(pwmPin) ;
  float PWM = v * 3.3 / 4095.0;

  int gas_concentration = int((v) * (5000 / 2));
  Serial.println(gas_concentration);

  if (gas_concentration = 0)
  {
    digitalWrite(5, HIGH);
  }
  else
  {
    digitalWrite(21, HIGH);
    while (digitalRead(pwmPin) == LOW) {};
    long t0 = millis();
    while (digitalRead(pwmPin) == HIGH) {};
    long t1 = millis();
    while (digitalRead(pwmPin) == LOW) {};
    long t2 = millis();
    long th = t1 - t0;
    long tl = t2 - t1;
    long ppm = 5000L * (th - 2) / (th + tl - 4);
    while (digitalRead(pwmPin) == HIGH) {};
    Serial.print(ppm);
    Serial.println("..CO2");
    co2_ppm = (ppm * 1);
    delay(5000);
    digitalWrite(5, HIGH);
    String metrics = "CO2,device=CO2_02,sensor=Sensor02 value="    + String(co2_ppm, 2);
    sendReadings(metrics);
    Serial.println(metrics);
    digitalWrite(21, LOW);
    digitalWrite(5, LOW);
    delay(2000);
  }

  //DHT******************************************************

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);

  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    delay(2000);
    return;
  }

  float hif = dht.computeHeatIndex(f, h);
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.println(F("°F"));

  if ( h > 0) {

    digitalWrite(5, HIGH);
    String metrics = "HUM,device=HUM_02,sensor=Sensor02 value="    + String(h, 2);
    sendReadings(metrics);
    Serial.println(metrics);
    delay(1000);
    digitalWrite(5, LOW);
    digitalWrite(21, LOW);
  } else
  {
    Serial.println("Failed to read from DHT sensor!");
    digitalWrite(21, HIGH);
    digitalWrite(25, HIGH);
  }
  if ( t > 0)
  {
    digitalWrite(26, HIGH);
    String metrics = "TEMP,device=TEMP_02,sensor=Sensor02 value="    + String(t, 2);
    sendReadings(metrics);
    Serial.println(metrics);
    delay(1000);
    digitalWrite(26, LOW);
    digitalWrite(25, LOW);
  } else
  {
    Serial.println("Failed to read from DHT sensor!");
    digitalWrite(25, HIGH);
    digitalWrite(21, HIGH);
  }
}
