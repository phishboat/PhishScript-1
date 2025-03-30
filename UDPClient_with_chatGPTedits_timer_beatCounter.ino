#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include <time.h>

const float periodMin = 600; // milliseconds
const float periodMax = 4000;
const float amplitudeMin = 0.01; // volts
const float amplitudeMax = 0.3;
const float baseSpeedMin = 1.0; // volts
const float baseSpeedMax = 1.0;
const int tenBitMin = 0;
const int tenBitMax = 1023;

const float pi = 3.141592653;

float amplitude = 0;
float base_speed = 0;
float period = 0;

// uint8_t userStartDelay [3] = {0,0,0}; // Commented out as requested
uint8_t startDelay = 15 * 60 * 1000; // 15 minutes in milliseconds

const uint16_t beatLimit = 500; // Define a variable for the beat limit
uint16_t BeatCount = 0;

bool runTimeComplete = false;

int status = WL_IDLE_STATUS;
char ssid[] = "fishBOATServer"; //  your network SSID (name)
char pass[] = "swimswim";    // your network password (use for WPA, or use as key for WEP)

unsigned int localPort = 1111; // local port to listen on

char packetBuffer[8]; // buffer to hold incoming packet
char ReplyBuffer[] = "stuff"; // a string to send back

WiFiUDP Udp;
int loop_counter = 0;
unsigned long lastCycleTime = 0; // Variable to track the last cycle time

void setup() {
  Serial.begin(9600);
  analogWriteResolution(12);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  Serial.println(fv);
  if (fv == "1.1.0") {
    Serial.println("Please upgrade the firmware");
  }

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();

  Udp.begin(localPort);
  Serial.println("Starting connection to server...");
}

void loop() {
  if (loop_counter % 950 == 0) {
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
      IPAddress remoteIp = Udp.remoteIP();

      Serial.print(remoteIp);
      Serial.print(", port ");
      Serial.println(Udp.remotePort());

      int len = Udp.read(packetBuffer, 8);
      Serial.println("test1");
      Serial.println(len);
      if (len == 8) {
        period = ((packetBuffer[1] - 1) << 8) | (packetBuffer[2] & ~1);
        amplitude = ((packetBuffer[3] - 1) << 8) | (packetBuffer[4] & ~1);
        base_speed = ((packetBuffer[5] - 1) << 8) | (packetBuffer[6] & ~1);

        for (int i = 0; i <= 7; i++) {
          Serial.print(packetBuffer[i], DEC);
          Serial.print(" ");
        }

        Serial.println("Contents:");
        Serial.print("Period: ");
        Serial.print(period);
        Serial.print(" Amplitude: ");
        Serial.print(amplitude);
        Serial.print(" Base Speed: ");
        Serial.println(base_speed);
      } else {
        Serial.println("Invalid packet received");
      }

      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(ReplyBuffer);
      Udp.endPacket();
    }
  } else {
    delay(10);
  }

  loop_counter++;

  int periodScaled = period * ((periodMax - periodMin) / (tenBitMax - tenBitMin)) + periodMin;
  float amplitudeScaled = amplitude * ((amplitudeMax - amplitudeMin) / (tenBitMax - tenBitMin)) + amplitudeMin;
  float baseSpeedScaled = base_speed * ((baseSpeedMax - baseSpeedMin) / (tenBitMax - tenBitMin)) + baseSpeedMin;

  float voltage_output = sinusoidalModel(baseSpeedScaled, amplitudeScaled, periodScaled);
  outputToDAC(voltage_output);
}

float sinusoidalModel(float baseSpeed, float amplitude, int period) {
  static float lastSinusoidalOutput = 0;
  float timeWithinPeriod = (millis() % period) / (period / (pi * 2));
  float sinusoidalOutput = (amplitude * sin(timeWithinPeriod)) + baseSpeed;

  if (lastSinusoidalOutput < baseSpeed && sinusoidalOutput >= baseSpeed) {
    if (BeatCount < beatLimit) {
      BeatCount++; // Increment beat counter every time a complete cycle is detected
      Serial.print("Beat Count: ");
      Serial.println(BeatCount);
    } else {
      Serial.println("Beat Count limit reached: ");
      Serial.println(beatLimit);
    }
  }
  lastSinusoidalOutput = sinusoidalOutput;

  return sinusoidalOutput;
}

void outputToDAC(float voltage_output) {
  static unsigned long startTime = millis();
  if (millis() - startTime < startDelay || BeatCount >= beatLimit) {
    return; // Delay the output for 15 minutes or stop if beat count limit is reached
  }
  
  voltage_output = constrain(voltage_output, 0, 3.3);
  int final_output = (int)((4096.0 / 3.3) * voltage_output);
  analogWrite(A6, final_output);
}

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
