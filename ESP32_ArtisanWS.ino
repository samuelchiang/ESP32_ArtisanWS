
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <HTTPClient.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson
#include "max6675.h"
#include <SPIFFS.h>


//ET
int thermoSO1 = 23; //SO1
int thermoCS1 = 22; //CS1
int thermoCLK1 = 21; //SCK1

//BT
int thermoSO2 = 4; //13; //SO2
int thermoCS2 = 2;//15; //CS2
int thermoCLK2 = 15; //SCK2

const int Pin_Fan = 13; 
const int Pin_Drop = 12;
const int Pin_Cooler = 14;
const int Pin_Drum = 27;
const int Pin_Heater = 26;

int FanPins[3] = {18, 19, 5};
//{{13, 27, 12}, {26, 32, 14}, {16, 17, 4}, {18, 19, 5}};
// Setting PWM properties
const int FanPWMfreq = 30000;
const int resolution = 8;
const int FanPWMChannel = 1;

const int HeaterPWMChannel = 2;
int HeaterPWMfreq = 1;
float kp=2.5;
float ki=0.06;
float kd=0.8;
float PID_p, PID_i, PID_d, PID_total;

bool StateFan = 0;
bool StateDrop = 0;
bool StateCooler = 0;
bool StateDrum = 0;
int StateHeater = 0;

float elapsedTime, now_time, prev_time;        //Variables for time control
float refresh_rate = 200;                   //PID loop time in ms
float now_pid_error, prev_pid_error;
float real_temp;           //We will store here the real temp 
float SetpointDiff = 30;   //In degrees C
float Setpoint = 0;       //In degrees C
int   SetpointType = 0;    // 0:ET, 1:BT 

MAX6675 thermocouple(thermoCLK1, thermoCS1, thermoSO1);
MAX6675 thermocouple2(thermoCLK2, thermoCS2, thermoSO2);

//flag for saving data
bool shouldSaveConfig = false;
//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

char CH_HeaterPWMfreq[8] = "120";
char CH_kp[8] = "2.5";
char CH_ki[8] = "0.06";
char CH_kd[8] = "0.8";

const int BootBtnPin = 0;

int BUZZER_PIN = 25;     //蜂鳴器控制腳位
int Buzzer_Channel = 0;     //channel提供0-15共16個通道。
int Buzzer_resolution = 8;  //duty cycle解析度(2^10=1024解析度，變數dutyCycle最大值為1024)
int Buzzer_dutyCycle = 255; //控制PWM脈寬，即可控制音量大小。脈寬越大、音量越大。


#define AP_SSID "HoptopRoaster"
#define AP_PASS "1qaz2wsx"

//--------------- Wifi
void setup_wifi() {
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  WiFiManager wm;
  wm.setSaveConfigCallback(saveConfigCallback);

  WiFiManagerParameter custom_HeaterPWMfreq("HeaterPWMfreq", "Heater PWM frequency", CH_HeaterPWMfreq, 8);
  WiFiManagerParameter custom_kp("PIDkp", "PID kp", CH_kp, 8);
  WiFiManagerParameter custom_ki("PIDki", "PID ki", CH_ki, 8);
  WiFiManagerParameter custom_kd("PIDkd", "PID kd", CH_kd, 8);
  
  
  //add all your parameters here
  wm.addParameter(&custom_HeaterPWMfreq);
  wm.addParameter(&custom_kp);
  wm.addParameter(&custom_ki);
  wm.addParameter(&custom_kd);

  bool res;
  wm.setConfigPortalTimeout(180);//seconds
  res = wm.autoConnect(AP_SSID,AP_PASS); // password protected ap
  if(!res) {
      Serial.println("Failed to connect, restart");
      ESP.restart();
  } 
  else {
      //if you get here you have connected to the WiFi    
      Serial.println("connected...yeey :)");
  }

  //read updated parameters
  strcpy(CH_HeaterPWMfreq, custom_HeaterPWMfreq.getValue());
  strcpy(CH_kp, custom_kp.getValue());
  strcpy(CH_ki, custom_ki.getValue());
  strcpy(CH_kd, custom_kd.getValue());
  
  //save the custom parameters to FS
  if (shouldSaveConfig) {
    DynamicJsonDocument doc(1024);
    
    doc["HeaterPWMfreq"]   = CH_HeaterPWMfreq;
    doc["kp"]     = CH_kp;
    doc["ki"]     = CH_ki;
    doc["kd"]     = CH_kd;
    
    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }
    
    serializeJsonPretty(doc, Serial);
    serializeJson(doc, configFile);
    configFile.close();
    //end save
    shouldSaveConfig = false;
  }  
}

String IpAddress2String(const IPAddress& ipAddress){
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ;
}

void line_notify(String message){
  const char* serverName = "https://notify-api.line.me/api/notify";
  String httpRequestData = "message="+message;           

  HTTPClient http;
  http.begin(serverName);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.addHeader("Authorization", "Bearer 83lWzRyxKV5BtMu54VBEOZb7UNUDZYjylnq5yz1o7gi");
  int httpResponseCode = http.POST(httpRequestData);
  Serial.print("Line Notify HTTP Response code: ");
  Serial.println(httpResponseCode);
  http.end();
}

void line_notify_webservice()
{
  IPAddress ip; 
  ip = WiFi.localIP();
  Serial.println(F("WiFi connected"));
  Serial.println("");
  Serial.println(ip);
  String msg = "Hottop ESP32 start webservice ws://"+IpAddress2String(ip)+"/ws";
  line_notify(msg);
}

void check_wifi()
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

void wifi_reset(){
  Serial.println("To reset wifi");
  WiFiManager wm;
  wm.resetSettings();
  ESP.restart();
}

// Websocket

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Hottop Coffee Roster</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" href="data:,">
  <style>
  html {
    font-family: Arial, Helvetica, sans-serif;
    text-align: center;
  }
  h1 {
    font-size: 1.8rem;
    color: white;
  }
  h2{
    font-size: 1.5rem;
    font-weight: bold;
    color: #143642;
  }
  .topnav {
    overflow: hidden;
    background-color: #143642;
  }
  body {
    margin: 0;
  }
  .content {
    padding: 30px;
    max-width: 600px;
    margin: 0 auto;
  }
  .card {
    background-color: #F8F7F9;;
    box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5);
    padding-top:10px;
    padding-bottom:20px;
  }
  .button {
    padding: 15px 50px;
    font-size: 24px;
    text-align: center;
    outline: none;
    color: #fff;
    background-color: #0f8b8d;
    border: none;
    border-radius: 5px;
    -webkit-touch-callout: none;
    -webkit-user-select: none;
    -khtml-user-select: none;
    -moz-user-select: none;
    -ms-user-select: none;
    user-select: none;
    -webkit-tap-highlight-color: rgba(0,0,0,0);
   }
   /*.button:hover {background-color: #0f8b8d}*/
   .button:active {
     background-color: #0f8b8d;
     box-shadow: 2 2px #CDCDCD;
     transform: translateY(2px);
   }
   .state {
     font-size: 1.5rem;
     color:#8c8c8c;
     font-weight: bold;
   }
  </style>
<title>Hottop Coffee Roster</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="icon" href="data:,">
</head>
<body>
  <div class="topnav">
    <h1>Hottop Coffee Roster</h1>
  </div>
  <div class="content">
    <div class="card">
      <h2>Fan</h2>
      <p class="state">state: <span id="stateFan">%STATE_FAN%</span></p>
      <p><button id="buttonFan" class="button">Toggle</button></p>
    </div>
    <div class="card">
      <h2>Drop</h2>
      <p class="state">state: <span id="stateDrop">%STATE_DROP%</span></p>
      <p><button id="buttonDrop" class="button">Toggle</button></p>
    </div>
    <div class="card">
      <h2>Cooler</h2>
      <p class="state">state: <span id="stateCooler">%STATE_COOLER%</span></p>
      <p><button id="buttonCooler" class="button">Toggle</button></p>
    </div>
    <div class="card">
      <h2>Drum</h2>
      <p class="state">state: <span id="stateDrum">%STATE_DRUM%</span></p>
      <p><button id="buttonDrum" class="button">Toggle</button></p>
    </div>
    <div class="card">
      <h2>Heater</h2>
      <p class="state">state: <span id="stateHeater">%STATE_HEATER%</span></p>
      <p><button id="buttonHeater" class="button">Toggle</button></p>
    </div>
  </div>
<script>
  var gateway = `ws://${window.location.hostname}/ws`;
  //var gateway = `ws://192.168.31.235/ws`;
  
  var websocket;
  window.addEventListener('load', onLoad);
  function initWebSocket() {
    console.log('Trying to open a WebSocket connection...');
    websocket = new WebSocket(gateway);
    websocket.onopen    = onOpen;
    websocket.onclose   = onClose;
    websocket.onmessage = onMessage; // <-- add this line
  }
  function onOpen(event) {
    console.log('Connection opened');
  }
  function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
  }
  function onMessage(event) {
    var state;
    var data = JSON.parse(event.data);
    //console.log(data.state);
    //console.log(data.device);
    if(data.state){
      state = "ON";
    }else{
      state = "OFF";
    }
    switch (data.device){
      case 1:
        document.getElementById('stateFan').innerHTML = state;
        break;
      case 2:
        document.getElementById('stateDrop').innerHTML = state;
        break;
      case 3:
        document.getElementById('stateCooler').innerHTML = state;
        break;
      case 4:
        document.getElementById('stateDrum').innerHTML = state;
        break;
      case 5:
        document.getElementById('stateHeater').innerHTML = state;
        break;
      default:
        console.log('onMessage no matched device');
        break;
    }
  }
  function onLoad(event) {
    initWebSocket();
    initButton();
  }
  function initButton() {
    document.getElementById('buttonFan').addEventListener('click', toggleFan);
    document.getElementById('buttonDrop').addEventListener('click', toggleDrop);
    document.getElementById('buttonCooler').addEventListener('click', toggleCooler);
    document.getElementById('buttonDrum').addEventListener('click', toggleDrum);
    document.getElementById('buttonHeater').addEventListener('click', toggleHeater);
  }
  function toggleFan(){
    var msg = {
      command: "toggle",
      target:1
    };
    websocket.send(JSON.stringify(msg));
  }
  function toggleDrop(){
    var msg = {
      command: "toggle",
      target:2
    };
    websocket.send(JSON.stringify(msg));
  }
  function toggleCooler(){
    var msg = {
      command: "toggle",
      target:3
    };
    websocket.send(JSON.stringify(msg));
  }
  function toggleDrum(){
    var msg = {
      command: "toggle",
      target:4
    };
    websocket.send(JSON.stringify(msg));
  }
  function toggleHeater(){
    var msg = {
      command: "toggle",
      target:5
    };
    websocket.send(JSON.stringify(msg));
  }
</script>
</body>
</html>
)rawliteral";

char respData[64];
void responseData(int id) {
  float ET = thermocouple.readCelsius();
  float BT = thermocouple2.readCelsius();
  sprintf(&respData[0],"{\"id\": %d,\"data\": { \"ET\" : %.0f, \"BT\" : %.0f}}", id, ET, BT); 
  Serial.println(respData);
  ws.textAll(respData);
}

char respCmd[64];
void responseCommand(int target) {
  switch(target) { 
        case 1:
            StateFan=!StateFan;
            sprintf(&respCmd[0],"{\"device\": %d, \"state\": %s}", target, String(StateFan)); 
            break; 
        case 2:
            StateDrop=!StateDrop;
            sprintf(&respCmd[0],"{\"device\": %d, \"state\": %s}", target, String(StateDrop)); 
            break; 
        case 3:
            StateCooler=!StateCooler;
            sprintf(&respCmd[0],"{\"device\": %d, \"state\": %s}", target, String(StateCooler)); 
            break; 
        case 4:
            StateDrum=!StateDrum;
            sprintf(&respCmd[0],"{\"device\": %d, \"state\": %s}", target, String(StateDrum)); 
            break; 
        case 5:
            //StateHeater=!StateHeater;
            //sprintf(&respCmd[0],"{\"device\": %d, \"state\": %s}", target, String(StateHeater)); 
            break; 
        default: 
            break;
    } 
  Serial.println(respCmd);
  ws.textAll(respCmd);
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
    }else if(strcmp(command, "toggle") == 0){
      int target = doc["target"];
      responseCommand(target);
    }else if(strcmp(command, "resetwifi") == 0){
      wifi_reset();
    }else if(strcmp(command, "setControlParams") == 0){
      //{"command": "setControlParams", "params": {"drum": 1}, "id": 18141, "roasterID": 2}
      JsonVariant params = doc["params"];
      if (params.isNull()) {
        Serial.println("error command arguments");
        return;
      }
      //buzzer_success();
      JsonVariant heater_jv = doc["params"]["heater"];
      JsonVariant fan_jv = doc["params"]["fan"];
      JsonVariant drop_jv = doc["params"]["drop"];
      JsonVariant drum_jv = doc["params"]["drum"];
      if (!heater_jv.isNull()){
        int heater_value = heater_jv.as<int>();
        start_heater(heater_value);
      }else if (!fan_jv.isNull()){
        int fan_value = fan_jv.as<int>();
        start_fan(fan_value);
      }else if (!drop_jv.isNull()){
        int drop_value = drop_jv.as<int>();
        if(drop_value!=0&&drop_value!=1){
          Serial.printf("Error drop_value %d\n", drop_value);
          return;
        }
        StateDrop = drop_value;
        StateCooler = drop_value;
        sprintf(&respCmd[0],"{\"device\": 2, \"state\": %s}", String(StateDrop));
        ws.textAll(respCmd);
        sprintf(&respCmd[0],"{\"device\": 3, \"state\": %s}", String(StateCooler)); 
        ws.textAll(respCmd);
      }else if (!drum_jv.isNull()){
        int drum_value = drum_jv.as<int>();
        if(drum_value!=0&&drum_value!=1){
          Serial.printf("Error drum_value %d\n", drum_value);
          return;
        }
        StateDrum = drum_value;
        sprintf(&respCmd[0],"{\"device\": 4, \"state\": %s}", String(StateDrum));
        ws.textAll(respCmd);
      }

    }else if(strcmp(command, "setPIDControl") == 0){
      //{"command": "setPIDControl", "params": {{ "tempET": {} }}}
      //{"command": "setPIDControl", "params": {{ "tempBT": {} }}}
      JsonVariant params = doc["params"];
      if (params.isNull()) {
        Serial.println("error command arguments");
        return;
      }
      //buzzer_success();
      JsonVariant tempET_jv = doc["params"]["tempET"];
      JsonVariant tempBT_jv = doc["params"]["tempBT"];
      if (!tempET_jv.isNull()){
        Setpoint = tempET_jv.as<float>();
        SetpointType = 0;    // 0:ET, 1:BT
        StateHeater=1; 
      }else if (!tempBT_jv.isNull()){
        Setpoint = tempBT_jv.as<float>();
        SetpointType = 1;    // 0:ET, 1:BT 
        StateHeater=1;
      }
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

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Start server
  server.begin();
}

String processor(const String& var){
  Serial.println(var);
    if(var == "STATE_FAN"){
        if (StateFan){
          return "ON";
        }
        else{
          return "OFF";
        }
    }else if(var == "STATE_DROP"){
        if (StateDrop){
          return "ON";
        }
        else{
          return "OFF";
        }
    }else if(var == "STATE_COOLER"){
        if (StateCooler){
          return "ON";
        }
        else{
          return "OFF";
        }
    }else if(var == "STATE_DRUM"){
        if (StateDrum){
          return "ON";
        }
        else{
          return "OFF";
        }
    }else if(var == "STATE_HEATER"){
        if (StateHeater){
          return "ON";
        }
        else{
          return "OFF";
        }
    } 
}


// Websocket

void setupPins()
{
  pinMode(Pin_Fan, OUTPUT);
  pinMode(Pin_Drop, OUTPUT);
  pinMode(Pin_Cooler, OUTPUT);
  pinMode(Pin_Drum, OUTPUT);
  
  digitalWrite(Pin_Fan, LOW);
  digitalWrite(Pin_Drop, LOW);
  digitalWrite(Pin_Cooler, LOW);
  digitalWrite(Pin_Drum, LOW);
  
}  
void updatePins(){
  digitalWrite(Pin_Fan, StateFan);
  digitalWrite(Pin_Drop, StateDrop);
  digitalWrite(Pin_Cooler, StateCooler);
  digitalWrite(Pin_Drum, StateDrum);
  //digitalWrite(Pin_Heater, StateHeater);
}

void setup_heater() {
  pinMode(Pin_Heater, OUTPUT);
  // configure LED PWM functionalitites
  ledcSetup(HeaterPWMChannel, HeaterPWMfreq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(Pin_Heater, HeaterPWMChannel);
  //Stop all
  stop_heater();
}
void stop_heater() {
  ledcWrite(HeaterPWMChannel, 0);
  StateHeater=0;
}

bool start_heater(int percent) {
  if (percent > 100 || percent < 0) {
    Serial.printf("start_heater Error percent %d\n", percent);
    return false;
  }
  StateHeater=0;
  int dutyCycle = map(percent, 0, 100, 0, 255);
  ledcWrite(HeaterPWMChannel, dutyCycle);
  Serial.printf("start_heater with dutyCycle:%d percent:%d\r\n", dutyCycle, percent);
  return true;
}

void setup_fan() {
  // sets the pins as outputs:
  for (int j = 0; j < 3; j++) {
    pinMode(FanPins[j], OUTPUT);
  }
  // configure LED PWM functionalitites
  ledcSetup(FanPWMChannel, FanPWMfreq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(FanPins[2], FanPWMChannel);
  //Stop all
  stop_fan();
}

void stop_fan() {
  // Stop the DC motor
  StateFan=0;
  digitalWrite(FanPins[0], LOW);
  digitalWrite(FanPins[1], LOW);
}

bool start_fan(int percent) {
  if (percent > 100 || percent < 0) {
    Serial.printf("start_fan Error percent %d\n", percent);
    return false;
  }

  StateFan=1;
  int dutyCycle = map(percent, 0, 100, 0, 255);
  ledcWrite(FanPWMChannel, dutyCycle);
  Serial.printf("start_fan with dutyCycle:%d percent:%d\r\n", dutyCycle, percent);
  digitalWrite(FanPins[0], HIGH);
  digitalWrite(FanPins[1], LOW);
  if(percent==0){
    stop_fan();
  }  
  return true;
}


void setup_spiffs(){
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

          if (doc.containsKey("HeaterPWMfreq")){
            strcpy(CH_HeaterPWMfreq, doc["HeaterPWMfreq"]);  
            HeaterPWMfreq = atoi(CH_HeaterPWMfreq);
          }
          if (doc.containsKey("kp")){
            strcpy(CH_kp, doc["kp"]);
            kp = atof(CH_kp);
          }
      if (doc.containsKey("ki")){
            strcpy(CH_ki, doc["ki"]);
            ki = atof(CH_ki);
          }
          if (doc.containsKey("kd")){
            strcpy(CH_kd, doc["kd"]);
            kd = atof(CH_kd);
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

void heater_loop() {  
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

float get_real_temp(){
  if (SetpointType==0){ //ET
    return thermocouple.readCelsius();  
  }else{ //BT
    return thermocouple2.readCelsius();  
  }
}

//Main PID compute and execute function
void heater_PID_control(){
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
    if(PID_total < 0){PID_total = 0;}
    if(PID_total > 255) {PID_total = 255; } 

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
void heater_ramp_up(void){  
  //Rising temperature to (Setpoint - SetpointDiff)
  elapsedTime = millis() - prev_time; 
  if(elapsedTime > refresh_rate){  
    real_temp = get_real_temp();
    
    if(real_temp < (Setpoint - SetpointDiff)){
      ledcWrite(HeaterPWMChannel, 255);//Turn on heater
    }else{
      ledcWrite(HeaterPWMChannel, 0);        //Turn Off heater
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

void setup_buzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
}

void buzzer_success(){
  play_sound(2700, 100, 25, 2, 1976);
}

void buzzer_error(){
  play_sound(1000, 100, 100, 10, 1760);
}

void buzzer_notify(){
  play_sound(2700, 100, 50, 3, 4058);
}

void buzzer_done(){
  play_sound(200, 1000, 0, 1, 1760);
}

void play_sound(int freq, int play_duration, int pause_duration, int times, int note ){
  ledcSetup(Buzzer_Channel, freq, Buzzer_resolution);  //設定通道0的頻率與duty cycle解析度。
  ledcAttachPin(BUZZER_PIN, Buzzer_Channel);    //控制蜂鳴器的G15腳位綁定於通道0
  /*ledcWriteTone()在前；ledcWrite()在後*/
  ledcWriteTone(Buzzer_Channel, note); 

  for(int t=0; t<times; t++){
    ledcWrite(Buzzer_Channel, Buzzer_dutyCycle);  
    delay(play_duration);  
    ledcWrite(Buzzer_Channel, 0);
    delay(pause_duration);
  } 
}

void setup()
{
  Serial.begin(115200);
  setup_wifi();
  setup_spiffs();
  line_notify_webservice();
  initWebSocket();
  setupPins();
  setup_fan();
  setup_heater();
}
void loop()
{
  check_wifi();
  ws.cleanupClients();
  updatePins();
  heater_loop();
}
