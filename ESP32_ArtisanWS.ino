
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <HTTPClient.h>
#include <WiFi.h>
#include <PubSubClient.h> //for mqtt
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson
#include "max6675.h"
#include <SPIFFS.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ezButton.h>
//------------------------------------------ pin --------------------------------------------------------------
//buzzer
int BUZZER_PIN = 25;     //蜂鳴器控制腳位

//ET Max6675
int thermoSO1 = 18; //SO1
int thermoCS1 = 19; //CS1
int thermoSCK1 = 23; //SCK1

//BT Max6675
int thermoSO2 = 15;  //SO2
int thermoCS2 = 4; //CS2
int thermoSCK2 = 5; //SCK2

//Hottop
const int Pin_Fan = 26; 
const int Pin_Drop = 27;
const int Pin_Cooler = 14;
const int Pin_Drum = 12;
const int Pin_Heater = 13;

//For L293D
const int Pin_FAN_PWM = 32;
const int Pin_FAN_INPUT = 33;

//For Reset WiFi Button
const int btnWiFiResetPin = 17;  //TX2
//For DROP Button
const int btnDROPPin = 16;  //RX2

//------------------------------------------ global variables --------------------------------------------------

char DeviceId[32] = "HoptopRoaster";  // Device ID, SSID
char APPassword[32] = "1qaz2wsx";  // Wifi AP Password

//MQTT
char mqttServer[40] = "";  // MQTT伺服器位址
char mqttPort[6]  = "1883";
char mqttUserName[32] = "";  // 使用者名稱
char mqttPwd[32] = "";  // MQTT密碼

//------------------------------------------ OLED --------------------------------------------------------------
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

class SCREEN {
    public:
        SCREEN() { 
          display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
        } 
        void init();
        void drawAPmode();
        void drawError(String text);
        void drawText(String text);
        void drawHotTopmode(String text);
    private:
        Adafruit_SSD1306 display;
};

void SCREEN::init() {
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS, OLED_RESET, true)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
}

void SCREEN::drawAPmode(){
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("AP Mode"));
  display.setTextSize(1);
  char text[64];
  sprintf(&text[0], "\nSSID:%s\nPassword:%s", DeviceId, APPassword);
  display.println(text);
  display.display();
}


void SCREEN::drawError(String text){
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(text);
  display.display();
}


void SCREEN::drawText(String text){
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(text);
  display.display();
}

String IpAddress2String(const IPAddress& ipAddress){
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ;
}

void SCREEN::drawHotTopmode(String stat){
  IPAddress ip = WiFi.localIP();
  String ws_msg = "ws://"+IpAddress2String(ip)+"/ws";
  
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(ws_msg);
  display.println(stat);
  display.display();
}

SCREEN screen;

//------------------------------------------ Buzzer --------------------------------------------------------------
#define BUZZER_IDLE       0
#define BUZZER_BEEP_DELAY 1
#define BUZZER_BEEPING    2

class BUZZER {
    public:
        BUZZER(int pin);
        void loop();
        void beep(unsigned long beepTime);
        void beep(unsigned long beepTime, unsigned long delay);
    private:
        int _pin;
        unsigned long _delayTime;
        unsigned long _beepTime;
        unsigned long _startTime;
        int _buzzerState;
};

BUZZER::BUZZER(int pin) { 
  _pin = pin;
  _buzzerState = BUZZER_IDLE;
  _delayTime = 0;
  _beepTime  = 0;
  _startTime = 0;
  pinMode(_pin, OUTPUT);
} 
void BUZZER::beep(unsigned long beepTime) {
  beep(beepTime, 0);
}

void BUZZER::beep(unsigned long beepTime, unsigned long delay) {
  _delayTime = delay;
  _beepTime  = beepTime;
  _buzzerState = BUZZER_BEEP_DELAY;
  _startTime = millis();
}

void BUZZER::loop() {
  switch(_buzzerState) {
    case BUZZER_IDLE:
      break;

    case BUZZER_BEEP_DELAY:
      if ((unsigned long)(millis() - _startTime) >= _delayTime) {
        _buzzerState = BUZZER_BEEPING;
        _startTime = millis();

        digitalWrite(_pin, HIGH);
      }

      break;

    case BUZZER_BEEPING:
      if ((unsigned long)(millis() - _startTime) >= _beepTime) {
        _buzzerState = BUZZER_IDLE;
        digitalWrite(_pin, LOW);
      }

      break;
    default:
      break;
  }
}
BUZZER buzzer(BUZZER_PIN);

//------------------------------------------ WiFi and Configurations ----------------------------------

bool shouldSaveConfig;
void saveConfigCallback () {
  shouldSaveConfig = true;
}

class WIFI {
    public:
        WIFI(char *AP_SSID, char* AP_PASS){ 
            ap_ssid = AP_SSID;
            ap_pass = AP_PASS;
        } 
        void init();
        void reset();
        void check();
    private:
        void readConfig();
        char *ap_ssid;
        char* ap_pass;
};


void WIFI::init() {
  readConfig();
  
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  WiFiManager wm;
  shouldSaveConfig = false;
  wm.setSaveConfigCallback(saveConfigCallback);

  WiFiManagerParameter custom_mqttServer("mqttServer", "mqtt server", mqttServer, 40);
  WiFiManagerParameter custom_mqttPort("mqttPort", "mqtt port", mqttPort, 6);
  WiFiManagerParameter custom_mqttUserName("mqttUserName", "mqtt user name", mqttUserName, 32);
  WiFiManagerParameter custom_mqttPwd("mqttPwd", "mqtt password", mqttPwd, 32);
  WiFiManagerParameter custom_DeviceId("DeviceId", "Device ID", DeviceId, 32);
  
  wm.addParameter(&custom_mqttServer);
  wm.addParameter(&custom_mqttPort);
  wm.addParameter(&custom_mqttUserName);
  wm.addParameter(&custom_mqttPwd);
  wm.addParameter(&custom_DeviceId);
  

  bool res;
  screen.drawAPmode();
  wm.setConfigPortalTimeout(180);//seconds
  res = wm.autoConnect(ap_ssid,ap_pass); // password protected ap
  if(!res) {
      Serial.println("Failed to connect, restart");
      ESP.restart();
  } 
  else {
      //if you get here you have connected to the WiFi    
      Serial.println("connected...yeey :)");
      screen.drawText("WiFi connected");
  }

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    DynamicJsonDocument doc(1024);
    
    doc["mqttServer"]   = custom_mqttServer.getValue();
    doc["mqttPort"]     = custom_mqttPort.getValue();
    doc["mqttUserName"] = custom_mqttUserName.getValue();
    doc["mqttPwd"]      = custom_mqttPwd.getValue();
    doc["DeviceId"]     = custom_DeviceId.getValue();

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }
    
    serializeJsonPretty(doc, Serial);
    serializeJson(doc, configFile);
    configFile.close();
    //end save
    shouldSaveConfig = false;
    readConfig();
  }  
}

void WIFI::readConfig(){
  //clean FS, for testing
  // SPIFFS.format();
  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin(true)) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument doc(1024);
     
        deserializeJson(doc, buf.get(), DeserializationOption::NestingLimit(20));
        serializeJsonPretty(doc, Serial);

        if (!doc.isNull()) {
          Serial.println("\nparsed json");

          if (doc.containsKey("mqttServer")){
            strcpy(mqttServer, doc["mqttServer"]);  
          }
          if (doc.containsKey("mqttPort")){
            strcpy(mqttPort, doc["mqttPort"]);
          }
          if (doc.containsKey("mqttUserName")){
            strcpy(mqttUserName, doc["mqttUserName"]);
          }
          if (doc.containsKey("mqttPwd")){
            strcpy(mqttPwd, doc["mqttPwd"]);
          }
          if (doc.containsKey("DeviceId")){
            strcpy(DeviceId, doc["DeviceId"]);
          }
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
}


void WIFI::check()
{
  static unsigned long samplingTime = millis();
  static int countdown=5;
  
  if(millis()-samplingTime > 5000)
  {
      samplingTime=millis();
      
      if(WiFi.status()!= WL_CONNECTED) {
        countdown--;
        if(countdown==0){
            Serial.println("Failed to reconnect");
            ESP.restart();
        }
      }else{
        countdown=5;
      }
  }
}

//------------------------------------------ MQTT --------------------------------------------------------------
WiFiClient espClient;
PubSubClient mqttClient(espClient);
char CommandTopic[64];
char DataTopic[64];

void mqtt_init() {
    mqttClient.setServer(mqttServer, atoi(mqttPort)); 
    sprintf(&CommandTopic[0],"cmd/%s/#", DeviceId);  //"cmd/DeviceId/#";
    sprintf(&DataTopic[0],"data/%s/", DeviceId);  //"data/DeviceId/";
}

void mqtt_loop(){
  if (!mqttClient.connected()) {
      int countdown=5;
      while (!mqttClient.connected()) {
          if (mqttClient.connect(DeviceId, mqttUserName, mqttPwd)) {
            Serial.println("MQTT connected");
            screen.drawText("MQTT connected");
            mqttClient.subscribe(CommandTopic);
            mqttClient.setCallback(mqttCallback);
          } else {
            screen.drawError("Failed to connect MQTT");
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);  // 等5秒之後再重試'
            //設置 timeout, 過了 25 秒仍無法連線, 就重啟 EPS32
            countdown--;
            if(countdown==0){
              Serial.println("Failed to reconnect");
              ESP.restart();
            }
          }
    }
  }
  mqttClient.loop();
}

void mqtt_publish(const char* topic, String str){
    // 宣告字元陣列
    byte arrSize = str.length() + 1;
    char msg[arrSize];
    Serial.print("Publish topic: ");
    Serial.print(topic);
    Serial.print(" message: ");
    Serial.print(str);
    Serial.print(" arrSize: ");
    Serial.println(arrSize);
    str.toCharArray(msg, arrSize); // 把String字串轉換成字元陣列格式
    if (!mqttClient.publish(topic, msg)){
      Serial.println("Faliure to publish, maybe you should check the message size: MQTT_MAX_PACKET_SIZE 128");       // 發布MQTT主題與訊息
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
}


//------------------------------------------ Push Buttons ------------------------------------------------------
#include <functional>
#define LONG_PRESS_CALLBACK_SIGNATURE std::function<void(void)> long_pressed_callback
#define SHORT_PRESS_CALLBACK_SIGNATURE std::function<void(void)> short_pressed_callback
const int SHORT_PRESS_TIME = 1000; // 1000 milliseconds
const int LONG_PRESS_TIME  = 1000; // 1000 milliseconds
class BUTTON {
    public:
        BUTTON(int PIN)  {pin=PIN;} 
        void init();
        void loop();
        void setLongPressedCallback(LONG_PRESS_CALLBACK_SIGNATURE);
        void setShortPressedCallback(SHORT_PRESS_CALLBACK_SIGNATURE);
    private:
        ezButton *button;
        int pin;
        unsigned long pressedTime  = 0;
        unsigned long releasedTime = 0;
        bool isPressing = false;
        bool isLongDetected = false;
        LONG_PRESS_CALLBACK_SIGNATURE;
        SHORT_PRESS_CALLBACK_SIGNATURE;
};

void BUTTON::init() {
    button = new ezButton(pin);
    button->setDebounceTime(50);
}

void BUTTON::setLongPressedCallback(LONG_PRESS_CALLBACK_SIGNATURE) {
    this->long_pressed_callback = long_pressed_callback;
}

void BUTTON::setShortPressedCallback(SHORT_PRESS_CALLBACK_SIGNATURE) {
    this->short_pressed_callback = short_pressed_callback;
}

void BUTTON::loop(){
   button->loop(); // MUST call the loop() function first

  if(button->isPressed()){
    pressedTime = millis();
    isPressing = true;
    isLongDetected = false;
  }

  if(button->isReleased()) {
    isPressing = false;
    releasedTime = millis();

    long pressDuration = releasedTime - pressedTime;

    if( pressDuration < SHORT_PRESS_TIME ){
      Serial.println("A short press is detected");
      if(short_pressed_callback){
        short_pressed_callback();
      }
    }
      
  }

  if(isPressing == true && isLongDetected == false) {
    long pressDuration = millis() - pressedTime;

    if( pressDuration > LONG_PRESS_TIME ) {
      Serial.println("A long press is detected");
      isLongDetected = true;
      if(long_pressed_callback){
        long_pressed_callback();
      }
    }
  }
}

//------------------------------------------ MAX6675 -----------------------------------------------------
MAX6675 thermocouple(thermoSCK1, thermoCS1, thermoSO1);
MAX6675 thermocouple2(thermoSCK2, thermoCS2, thermoSO2);

//------------------------------------------ hottop ------------------------------------------------------

// Setting PWM properties
const int FanPWMfreq = 30000;
const int resolution = 8;
const int FanPWMChannel = 1;
const int HeaterPWMChannel = 2;
int HeaterPWMfreq = 1;
float refresh_rate = 200;                   //PID loop time in ms
float SetpointDiff = 30;   //In degrees C

class HOTTOP {
    public:
        HOTTOP() { 
          StateFan = StateDrop =StateCooler = StateDrum = false;
          StateHeater = 0;
          kp=2.5;
          ki=0.06;
          kd=0.8;
        } 
        void init();
        void loop();
        bool start_heater(int percent);
        void stop_heater();
        bool start_fan(int percent);
        void set_pid_control(String type, int set_point, float set_kp, float set_ki, float set_kd);
        void stop_fan();
        void toggle_drop_button();
        void set_drop_state(int drop_value);
        void set_drum_state(int drum_value);
        void update_state();
    private:
        bool  StateFan;
        bool  StateDrop;
        bool  StateCooler;
        bool  StateDrum;
        int   StateHeater;
        int   percent_fan;
        int   percent_heater;
        void  heater_pid_loop();
        float get_real_temp();
        void  heater_ramp_up();
        void  heater_PID_control();
        float kp;
        float ki;
        float kd;
        float Setpoint;       //In degrees C
        int   SetpointType;    // 0:ET, 1:BT 
};

void HOTTOP::init()
{
  pinMode(Pin_Fan, OUTPUT);
  pinMode(Pin_Drop, OUTPUT);
  pinMode(Pin_Cooler, OUTPUT);
  pinMode(Pin_Drum, OUTPUT);
  
  digitalWrite(Pin_Fan, LOW);
  digitalWrite(Pin_Drop, LOW);
  digitalWrite(Pin_Cooler, LOW);
  digitalWrite(Pin_Drum, LOW);

  //setup_heater
  pinMode(Pin_Heater, OUTPUT);
  // configure LED PWM functionalitites
  ledcSetup(HeaterPWMChannel, HeaterPWMfreq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(Pin_Heater, HeaterPWMChannel);
  //Stop all
  stop_heater();

  //setup_fan
  pinMode(Pin_FAN_PWM, OUTPUT);   
  pinMode(Pin_FAN_INPUT, OUTPUT);
  // configure LED PWM functionalitites
  ledcSetup(FanPWMChannel, FanPWMfreq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(Pin_FAN_PWM, FanPWMChannel);
  //Stop all
  stop_fan();
}  

void HOTTOP::loop(){
  digitalWrite(Pin_Fan, StateFan);
  digitalWrite(Pin_Drop, StateDrop);
  digitalWrite(Pin_Cooler, StateCooler);
  digitalWrite(Pin_Drum, StateDrum);
  heater_pid_loop();
  update_state();
}

void HOTTOP::update_state(){
  static float elapsedTime, prev_time;        //Variables for time control
  elapsedTime = millis() - prev_time; 
  if(elapsedTime > refresh_rate){  
    char text[64];
    float et =  thermocouple.readCelsius();  
    float bt =  thermocouple2.readCelsius();  
    sprintf(&text[0], "Heater:%d\nFan:%d\nDrum:%d\nDrop:%d\nCooler:%d\nET:%3.0f\nBT:%3.0f", percent_heater, percent_fan,StateDrum, StateDrop, StateCooler, et, bt);
    screen.drawHotTopmode(String(text));
    prev_time = millis();
  }
}
    
void HOTTOP::toggle_drop_button(){
  if (StateDrop){
      screen.drawText("Stop Drop Beans"); 
   }else{
      screen.drawText("Start Drop Beans");
   }
   StateDrop=!StateDrop;
   StateCooler=StateDrop;
}

void HOTTOP::set_drop_state(int drop_value){
  if(drop_value!=0&&drop_value!=1){
      Serial.printf("Error drop_value %d\n", drop_value);
    return;
  }
  StateDrop = drop_value;
  StateCooler = drop_value;
}

void HOTTOP::set_drum_state(int drum_value){
  if(drum_value!=0&&drum_value!=1){
    Serial.printf("Error drum_value %d\n", drum_value);
    return;
  }
  StateDrum = drum_value;
}

bool HOTTOP::start_fan(int percent) {
  if (percent > 100 || percent < 0) {
    Serial.printf("start_fan Error percent %d\n", percent);
    return false;
  }
  percent_fan = percent;
  StateFan=1;
  int dutyCycle = map(percent, 0, 100, 0, 255);
  ledcWrite(FanPWMChannel, dutyCycle);
  Serial.printf("start_fan with dutyCycle:%d percent:%d\r\n", dutyCycle, percent);
  digitalWrite(Pin_FAN_INPUT, HIGH);
  if(percent==0){
    stop_fan();
  }  
  return true;
}

void HOTTOP::stop_fan() {
  // Stop the DC motor
  StateFan=0;
  percent_fan = 0;
  digitalWrite(Pin_FAN_INPUT, LOW);
}

bool HOTTOP::start_heater(int percent) {
  if (percent > 100 || percent < 0) {
    Serial.printf("start_heater Error percent %d\n", percent);
    return false;
  }
  percent_heater = percent;
  StateHeater=0;
  int dutyCycle = map(percent, 0, 100, 0, 255);
  ledcWrite(HeaterPWMChannel, dutyCycle);
  Serial.printf("start_heater with dutyCycle:%d percent:%d\r\n", dutyCycle, percent);
  return true;
}

void HOTTOP::stop_heater() {
  ledcWrite(HeaterPWMChannel, 0);
  StateHeater=0;
  percent_heater = 0;
}

float HOTTOP::get_real_temp(){
  if (SetpointType==0){ //ET
    return thermocouple.readCelsius();  
  }else{ //BT
    return thermocouple2.readCelsius();  
  }
}

void HOTTOP::set_pid_control(String type, int set_point, float set_kp, float set_ki, float set_kd){
    if(type=="ET"){
      SetpointType = 0;        // 0:ET, 1:BT   
    }else{
      SetpointType = 1;        // 0:ET, 1:BT   
    }
    Setpoint = set_point;       //In degrees C
    kp=set_kp;
    ki=set_ki;
    kd=set_kd;
    StateHeater=1;
}

void HOTTOP::heater_pid_loop() {  
  //StateHeater
  // 0, not active
  // 1, ramp_up
  // 2, PID_control
  switch (StateHeater){
    case 1:
      heater_ramp_up();
      break;
    case 2:
      heater_PID_control(); 
      break;
  }
}

//Main PID compute and execute function
void HOTTOP::heater_PID_control(){
  static float elapsedTime, prev_time;        //Variables for time control
  float PID_p, PID_i, PID_d, PID_total;
  float now_pid_error, prev_pid_error;
  float real_temp;           //We will store here the real temp 
  
  elapsedTime = millis() - prev_time;   
  if(elapsedTime > refresh_rate){    
    //1. We get the temperature and calculate the error
    real_temp = get_real_temp();
    now_pid_error = Setpoint - real_temp;
  
    //2. We calculate PID values
    PID_p = kp * now_pid_error;
    PID_d = kd*((now_pid_error - prev_pid_error)/refresh_rate);
    //2.2 Decide if we apply I or not. Only when error is very small
    if(-3 < now_pid_error && now_pid_error < 3){
      PID_i = PID_i + (ki * now_pid_error);
    }else{
      PID_i = 0;
    }

    //3. Calculate and map total PID value
    PID_total = PID_p + PID_i + PID_d;  
    PID_total = map(PID_total, 0, 150, 0, 255);

    //4. Set limits for PID values
    if(PID_total < 0){
        PID_total = 0;
    }else if(PID_total > 255) {
        PID_total = 255; 
    } 

    percent_heater = map(PID_total, 0, 255, 0, 100);

    //5. Write PWM signal to the SSR
    ledcWrite(HeaterPWMChannel, PID_total);
    //6. Print values to the OLED dsiplay
//    display.clearDisplay();
//          
//    display.setCursor(0,0);  
//    display.print("Set: "); 
//    display.println(Setpoint,1);
//    
//    display.print((char)247); 
//    display.print("C: "); 
//    display.println(real_temp,1);  
//    display.println("PID mode");  
//    display.print(PID_total);
//    display.display();//Finally display the created image
//    
    
    //7. Save values for next loop
    prev_time = millis();                       //Store time for next loop
    prev_pid_error = now_pid_error;             //Store error for next loop
    //Serial.println(elapsedTime);                //For debug only
    //LED_State = !LED_State;
    //digitalWrite(led, LED_State);
  }  
}//End PID_control loop


//Fucntion for ramping up the temperature
void HOTTOP::heater_ramp_up(void){  
  static float elapsedTime, prev_time;        //Variables for time control
  float real_temp;           //We will store here the real temp 
  
  //Rising temperature to (Setpoint - SetpointDiff)
  elapsedTime = millis() - prev_time; 
  if(elapsedTime > refresh_rate){  
    real_temp = get_real_temp();
    
    if(real_temp < (Setpoint - SetpointDiff)){
      ledcWrite(HeaterPWMChannel, 255);//Turn on heater
      percent_heater = 100;
    }else{
      ledcWrite(HeaterPWMChannel, 0);        //Turn Off heater
      percent_heater = 0;
      StateHeater = 2;                       //Already hot so we go to PID control
    }
    
//    display.clearDisplay();  
//    display.setCursor(0,0);           
//    display.print("Set: "); 
//    display.println(Setpoint,1);
//    
//    display.print((char)247); 
//    display.print("C: "); 
//    display.println(real_temp,1);     
//    display.print("Ramp Up");   
//    display.display();//Finally display the created image
//    Serial.println(real_temp);                //For debug only
    prev_time = millis();
  }
}//End of ramp_up loop

HOTTOP hottop;

//------------------------------------------ Websocket --------------------------------------------------------------
// Create AsyncWebServer object on port 80

//AsyncWebServer server(80);
//AsyncWebSocket ws("/ws");

class WebSocket {
    public:
        WebSocket() {
        };
        void init();
        void loop();
        AsyncWebSocket *ws;
    private:
      AsyncWebServer *server;
};

const char index_html[] PROGMEM = R"rawliteral(
    <!DOCTYPE HTML><html>
    </html>
    )rawliteral";
    
void WebSocket::init(){
    server = new AsyncWebServer(80);
    ws = new AsyncWebSocket("/ws");
    ws->onEvent(onEvent);
    server->addHandler(ws);
    // Route for root / web page
    server->on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/html", index_html);
    });
    // Start server
    server->begin();
    
}

void WebSocket::loop(){
    ws->cleanupClients();
}

WebSocket websocket;

char respData[64];
void responseData(int id) {
  float ET = thermocouple.readCelsius();
  float BT = thermocouple2.readCelsius();
  sprintf(&respData[0],"{\"id\": %d,\"data\": { \"ET\" : %.0f, \"BT\" : %.0f}}", id, ET, BT); 
  Serial.println(respData);
  websocket.ws->textAll(respData);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    Serial.printf("handleWebSocketMessage %s\n", data);
    //{"command": "getData", "id": 86571, "roasterID": 2}
    //Parse Data
    DynamicJsonDocument doc(200);
    DeserializationError error = deserializeJson(doc, data);
    if (error) {
      Serial.println(F("deserializeJson() failed "));
      //Serial.println(error.f_str());=
      return;
    }
    const char* command = doc["command"];
    // Print values.
    //Serial.println(command);
    //Serial.println(id);
    if(strcmp(command, "getData") == 0){
      int id = doc["id"];
      responseData(id);
    }else if(strcmp(command, "setControlParams") == 0){
      //{"command": "setControlParams", "params": {"drum": 1}, "id": 18141, "roasterID": 2}
      JsonVariant params = doc["params"];
      if (params.isNull()) {
        Serial.println("error command arguments");
        return;
      }
      buzzer.beep(100);
      JsonVariant heater_jv = doc["params"]["heater"];
      JsonVariant fan_jv = doc["params"]["fan"];
      JsonVariant drop_jv = doc["params"]["drop"];
      JsonVariant drum_jv = doc["params"]["drum"];
      if (!heater_jv.isNull()){
        int heater_value = heater_jv.as<int>();
        hottop.start_heater(heater_value);
      }else if (!fan_jv.isNull()){
        int fan_value = fan_jv.as<int>();
        hottop.start_fan(fan_value);
      }else if (!drop_jv.isNull()){
        int drop_value = drop_jv.as<int>();
        hottop.set_drop_state(drop_value);
      }else if (!drum_jv.isNull()){
        int drum_value = drum_jv.as<int>();
        hottop.set_drum_state(drum_value);
      }
    }else if(strcmp(command, "setPIDControl") == 0){
      //{"command": "setPIDControl", "params": {{ "tempET": {} }}}
      //{"command": "setPIDControl", "params": {{ "tempBT": {} }}}
      JsonVariant params = doc["params"];
      if (params.isNull()) {
        Serial.println("error command arguments");
        return;
      }
      buzzer.beep(100);
      JsonVariant type_jv = doc["params"]["type"];
      JsonVariant temp_jv = doc["params"]["temp"];
      JsonVariant kp_jv = doc["params"]["kp"];
      JsonVariant ki_jv = doc["params"]["ki"];
      JsonVariant kd_jv = doc["params"]["kd"];
      String type = type_jv.as<String>();
      float Setpoint = temp_jv.as<float>();
      float kp = kp_jv.as<float>();
      float ki = ki_jv.as<float>();
      float kd = kd_jv.as<float>();
      hottop.set_pid_control(type, Setpoint, kp, ki, kd);
    } 
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  char msg[128];
  switch (type) {
    case WS_EVT_CONNECT:
      sprintf(msg, "WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      Serial.print(msg);
      //line_notify(msg);
      break;
    case WS_EVT_DISCONNECT:
      sprintf(msg, "WebSocket client #%u disconnected\n", client->id());
      Serial.print(msg);
      //line_notify(msg);
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
      sprintf(msg, "WebSocket WS_EVT_PONG arg:%s, data:%s\n", arg, data);
      break;
    case WS_EVT_ERROR:
      sprintf(msg, "WebSocket WS_EVT_PONG arg:%s, data:%s\n", arg, data);
      break;
  }
}

//====================================static callback =====================================================================

BUTTON wifi_btn(btnWiFiResetPin);
BUTTON drop_btn(btnDROPPin);

void wifi_btn_long_pressed(){
   Serial.println("WiFi long press is detected");
   screen.drawText("Reset Wifi");
   delay(2000); // wait for a second
   Serial.println("To reset wifi");
   WiFiManager wm;
   wm.resetSettings();
   ESP.restart();
}

void drop_btn_short_pressed(){
   Serial.println("drop_btn_short_pressed");
   hottop.toggle_drop_button();
}

//=========================================================================================================
WIFI wifi(DeviceId, APPassword);

void setup()
{
  Serial.begin(115200);
  screen.init();
  wifi.init();  
  wifi_btn.init();
  wifi_btn.setLongPressedCallback(wifi_btn_long_pressed);
  drop_btn.init();
  drop_btn.setShortPressedCallback(drop_btn_short_pressed);
  hottop.init();
  websocket.init();
}
void loop()
{
  wifi.check();
  websocket.loop();
  hottop.loop();
  wifi_btn.loop();
  drop_btn.loop();
  buzzer.loop();
}
