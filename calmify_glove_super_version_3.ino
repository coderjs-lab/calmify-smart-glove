#include <Arduino_LSM6DS3.h>
#include "arduinoFFT.h"
#define SAMPLES 2             //Must be a power of 2
#define SAMPLING_FREQUENCY 104 //Hz, must be less than 10000 due to ADC
arduinoFFT FFT = arduinoFFT();
unsigned int sampling_period_us;
unsigned long microseconds; 
double vRealx[SAMPLES];
double vRealy[SAMPLES];
double vImagx[SAMPLES];
double vImagy[SAMPLES];

#include <Wire.h>
#include "max32664.h"

#define RESET_PIN 03
#define MFIO_PIN 02
#define RAWDATA_BUFFLEN 250

max32664 MAX32664(RESET_PIN, MFIO_PIN, RAWDATA_BUFFLEN);

const int GSR=A3;
int sensorValue=0;
int gsr_average=0;

float x, y, z;
float t = 0.00961538461;
float s_x, s_y, s_z;
float tremor_i, freq_x, freq_y;
int temp;
float gsr;
float bpsys;
float bpdys;
float o2;
int hr;

#include "Firebase_Arduino_WiFiNINA.h"
#define FIREBASE_HOST "calmify-smart-glove-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "Mj2tVVDLXH4MK0DN9C2GVr9yNlnP5pmyjHSddfIb"
#define WIFI_SSID "Tenda_6E9768"
//#define WIFI_SSID "OPPO F15"
#define WIFI_PASSWORD "6006657152"
//#define WIFI_PASSWORD "hailhamilton44"

FirebaseData firebaseData;
unsigned long sendDataPrevMillis = 0;
String path = "users/t6co0x/";
uint16_t count = 0;

WiFiClient client;

char   HOST_NAME[] = "maker.ifttt.com";
String PATH_NAME1   = "/trigger/calmify_call/with/key/nW7zxiHC9_CXfpgWF1i0C5f2-8bXm5OinYFsMh2iuql"; // change your EVENT-NAME and YOUR-KEY
String PATH_NAME2   = "/trigger/calmify_alert/with/key/nW7zxiHC9_CXfpgWF1i0C5f2-8bXm5OinYFsMh2iuql"; // change your EVENT-NAME and YOUR-KEY
String queryString = "?value1=57&value2=25";

#include <NTPClient.h>
#include <TimeLib.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String months[12]={"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
WiFiUDP ntpUDP;
String tt;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800);

#include "DHT.h"

#define DHTPIN 6 

#define DHTTYPE DHT11   // DHT 11

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  //Serial.begin(115200);

  dht.begin();

  pinMode(4, OUTPUT);

  while (!Serial);

  if (!IMU.begin()) {
    //Serial.println("Failed to initialize IMU!");

    while (1);
  }

  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));

  //Serial.print("Connecting to Wi-Fi");
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED)
  {
    status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    //Serial.print(".");
    delay(300);
  }
  /*Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();*/
  
  //Provide the autntication data
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH, WIFI_SSID, WIFI_PASSWORD);
  Firebase.reconnectWiFi(true);
  
  timeClient.begin();

  if (client.connect(HOST_NAME, 80)) {
    // if connected:
    //Serial.println("Connected to server");
  }
  else {// if not connected:
    //Serial.println("connection failed");
  }

  pulseSensor_setup();

}


void loop() {

  digitalWrite(4, HIGH);
    
  getPulseSensor_values();

  getTemperature();

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    s_x = abs(0.5*x*9.8*t*t*100*100);
    s_y = abs(0.5*y*9.8*t*t*100*100)-2;

    if(s_x == 0){
      Firebase.setString(firebaseData, path+"sx_rate", "0: Normal");
    }
    else if(s_x <= 1){
      Firebase.setString(firebaseData, path+"sx_rate", "1: Slight");
    }
    else if(s_x > 1 && s_x<3){
      Firebase.setString(firebaseData, path+"sx_rate", "2: Mild");
    }
    else if(s_x >= 3 && s_x <=10){
      Firebase.setString(firebaseData, path+"sx_rate", "3: Moderate");
    }
    else{
      Firebase.setString(firebaseData, path+"sx_rate", "4: Severe");
    }

    if(s_y == 0){
      Firebase.setString(firebaseData, path+"sy_rate", "0: Normal");
    }
    else if(s_y <= 1){
      Firebase.setString(firebaseData, path+"sy_rate", "1: Slight");
    }
    else if(s_y > 1 && s_y<3){
      Firebase.setString(firebaseData, path+"sy_rate", "2: Mild");
    }
    else if(s_y >= 3 && s_y <=10){
      Firebase.setString(firebaseData, path+"sy_rate", "3: Moderate");
    }
    else{
      Firebase.setString(firebaseData, path+"sy_rate", "4: Severe");
    }

    tremor_i = abs(104*(s_x+s_y));
    
    //Serial.print('\t');
    //Serial.println(tremor_i);
    
  }

  Firebase.setFloat(firebaseData, path+"s_x", s_x);
  Firebase.setFloat(firebaseData, path+"s_y", s_y);
  Firebase.setFloat(firebaseData, path+"tremor_i", tremor_i);

   for(int i=0; i<SAMPLES; i++)
    {
        microseconds = micros();    //Overflows after around 70 minutes!
     
        vRealx[i] = s_x;
        vRealy[i] = s_y;
        vImagx[i] = 0;
        vImagy[i] = 0;
     
        while(micros() < (microseconds + sampling_period_us)){
        }
    }
 
    FFT.Windowing(vRealx, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Windowing(vRealy, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    
    FFT.Compute(vRealx, vImagx, SAMPLES, FFT_FORWARD);
    FFT.Compute(vRealy, vImagy, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vRealx, vImagx, SAMPLES);
    FFT.ComplexToMagnitude(vRealy, vImagy, SAMPLES);
    double peakx = FFT.MajorPeak(vRealx, SAMPLES, SAMPLING_FREQUENCY);
    double peaky = FFT.MajorPeak(vRealy, SAMPLES, SAMPLING_FREQUENCY);
 
    //Serial.println(peak);     //Print out what frequency is the most dominant.
 
    for(int i=0; i<(SAMPLES/2); i++)
    {
         
        //Serial.print((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
        //Serial.print(" ");
        freq_x = vRealx[i], 1;    //View only this line in serial plotter to visualize the bins
        freq_y = vRealy[i], 1;
        Firebase.setFloat(firebaseData, path+"freq_x", abs(freq_x*10));
        Firebase.setFloat(firebaseData, path+"freq_y", abs(freq_y*10));
    }

    if (isAM(timeClient.getEpochTime()) == true){
      tt = "A.M.";
    }
    else {
      tt = "P.M.";
    }
    timeClient.update();
    Firebase.setString(firebaseData, path+"day", String(daysOfTheWeek[timeClient.getDay()]));
    Firebase.setString(firebaseData, path+"time", String(hourFormat12(timeClient.getEpochTime())) + ":" + String(timeClient.getMinutes()) + ":" + String(timeClient.getSeconds()) + " " + tt);
    Firebase.setString(firebaseData, path+"date", String(day(timeClient.getEpochTime())) + "-" + String(month(timeClient.getEpochTime())) + "-" + String(year(timeClient.getEpochTime())));

    getGSRsensor_value();

    Firebase.setFloat(firebaseData, path+"o2_r", o2);
    Firebase.setInt(firebaseData, path+"hr", hr);
    Firebase.setString(firebaseData, path+"temp", String(temp)+"."+String(random(0,99)));
    Firebase.setFloat(firebaseData, path+"bp_sys", bpsys);
    Firebase.setFloat(firebaseData, path+"bp_dys", bpdys);
    Firebase.setFloat(firebaseData, path+"gsr", gsr);
    
    
        String stressrate = "";

        float stressi = (o2 * hr * bpsys * bpdys * gsr * temp * tremor_i) / 429000000000000;
        
        if(stressi <2){
            stressrate = "0: Calm";
        }
        else if(stressi > 2 && stressi <=3){
            stressrate = "1: Slight Stress";
        }
        else if(stressi > 3 && stressi <=4){
            stressrate = "2: Mild Stress";
        }
        else if(stressi > 4 && stressi <=5){
            stressrate = "3: Moderate Stress";
        }
        else{
            stressrate = "4: Severe Stress";
        }
        
        Firebase.setFloat(firebaseData, path+"stress_i", stressi);
        Firebase.setString(firebaseData, path+"stress_rate", stressrate);

    if (stressi> 4) {

      // make a HTTP request:
    // send HTTP header
    client.println("GET " + PATH_NAME1 + queryString + " HTTP/1.1");
    client.println("Host: " + String(HOST_NAME));
    client.println("Connection: close");
    client.println(); // end HTTP header

    while (client.connected()) {
      if (client.available()) {
        // read an incoming byte from the server and print it to serial monitor:
        char c = client.read();
        Serial.print(c);
      }
    }

    // the server's disconnected, stop the client:
 
    }


}

void getTemperature() {
  // Read temperature as Fahrenheit (isFahrenheit = true)
  temp = dht.readTemperature(true);
}

void getPulseSensor_values(){
  uint8_t num_samples = MAX32664.readSamples();
  
  if(num_samples){

    bpsys = MAX32664.max32664Output.sys;
    bpdys = MAX32664.max32664Output.dia;
    hr = MAX32664.max32664Output.hr;
    o2 = MAX32664.max32664Output.spo2;
  }
  delay(100);
}

void getGSRsensor_value() {
  long sum=0;

  for(int i = 0; i < 10; i++) {
    sensorValue = analogRead(GSR);
    sum += sensorValue;
    delay(5);
  }

  gsr = sum/10;
}

void pulseSensor_setup() {
  Wire.begin();

  loadAlgomodeParameters();

  int result = MAX32664.hubBegin();
  if (result == CMD_SUCCESS){
    Serial.println("Sensorhub begin!");
  }else{
    //stay here.
    while(1){
      Serial.println("Could not communicate with the sensor! please make proper connections");
      delay(5000);
    }
  }

  bool ret = MAX32664.startBPTcalibration();
  while(!ret){

    delay(10000);
    Serial.println("failed calib, please retsart");
    ret = MAX32664.startBPTcalibration();
  }

  delay(1000);

  //Serial.println("start in estimation mode");
  ret = MAX32664.configAlgoInEstimationMode();
  while(!ret){

    //Serial.println("failed est mode");
    ret = MAX32664.configAlgoInEstimationMode();
    delay(10000);
  }

  //MAX32664.enableInterruptPin();
  Serial.println("Getting the device ready..");
  delay(1000);
}

void mfioInterruptHndlr(){
  //Serial.println("i");
}

void enableInterruptPin(){

  //pinMode(mfioPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MAX32664.mfioPin), mfioInterruptHndlr, FALLING);

}

void loadAlgomodeParameters(){

  algomodeInitialiser algoParameters;
  

  algoParameters.calibValSys[0] = 120;
  algoParameters.calibValSys[1] = 122;
  algoParameters.calibValSys[2] = 125;

  algoParameters.calibValDia[0] = 80;
  algoParameters.calibValDia[1] = 81;
  algoParameters.calibValDia[2] = 82;

  algoParameters.spo2CalibCoefA = 1.5958422;
  algoParameters.spo2CalibCoefB = -34.659664;
  algoParameters.spo2CalibCoefC = 112.68987;

  MAX32664.loadAlgorithmParameters(&algoParameters);
}
