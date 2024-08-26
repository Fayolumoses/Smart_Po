#include "../inc/utils.h"
#include <WiFi.h>

#if defined(PIN_NEOPIXEL)
  Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#endif


Adafruit_SHT4x sht4 = Adafruit_SHT4x();
GasBreakout gas(Wire, 0x18);
LiquidCrystal_I2C lcd(0x27,20,4);
_sensors sensor;
H2S_data h2s_data;
SGP30 mySensor;
GasBreakout::Reading reading;


String encodeSensorData(_sensors sensor){
  Serial.println("-----------------------------------------------------Encoding Data-----------------------------------------------------");
  String retData;
  retData.concat("{temperature:");
  retData.concat(String(sensor.temperature));
  retData.concat(",Humidity:");
  retData.concat(String(sensor.relative_humidity));
  retData.concat(",Hydrogen_Sulphide:");
  retData.concat(String(sensor.vgas));
  retData.concat(",Reducing:");
  retData.concat(String(sensor.reducing));
  retData.concat(",NH3:");
  retData.concat(String(sensor.nh3));
  retData.concat(",Oxidising:");
  retData.concat(String(sensor.oxidising));
  retData.concat(",CO2:");
  retData.concat(String(sensor.CO2));
  retData.concat(",TVOC:");
  retData.concat(String(sensor.TVOC));
  retData.concat("}");

  Serial.print("Encoded Data: ");

  Serial.println(retData);

  return retData;
}

int read_sht_data(_sensors * sensor){
  sensors_event_t humidity, temp;

  if(!sht4.getEvent(&humidity, &temp)){
    Serial.println(F("SHT 40: Error!! Could not get event"));
    return -1;
  }

  sensor->relative_humidity = humidity.relative_humidity;
  sensor->temperature = temp.temperature;

  return 0;
}

int init_sensors(){
  Serial.println(F("Initialising Sensors...."));

  initialise_H2S_sensor(&h2s_data);

  Serial.println(F("Initialising SHT sensor"));
  if (! sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
    return -1;
  }

  Serial.println(F("Found SHT4x sensor"));
  Serial.print("Serial number 0x");
  Serial.println(sht4.readSerial(), HEX);

  if(!gas.initialise()){
      return -2;
   }

  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER); 

  if (mySensor.begin() == false) {
    return -3;
  } 

  mySensor.initAirQuality();
  return 0;
}

double readADC(int adc_pin, int max_input, int min_input){
  double ret;

  size_t data = analogRead(adc_pin);
  
  ret = map_float(data, max_input,min_input,3.3, 0.0);

  return ret;
}


int lcd_progressbar(String Title, uint8_t percent, int width){
  if(percent >100){
    return -1;
  }
  
  String toPrint = Title + "("+ String(percent)+ "%)";
  int len = toPrint.length();

  lcd.setCursor(0,0);
  for(int i = 0; i < width; i++){
    lcd.print('-');
  }
  lcd.setCursor(int((width - len)/2), 1);
  lcd.print(toPrint);

    lcd.setCursor(0,2);
    int _percent = map(percent, 100,0, width, 0);
    for(int i = 0; i < _percent; i++){
      lcd.write(255);
    }
  lcd.setCursor(0,3);
  for(int i = 0; i < width; i++){
    lcd.print('-');
  }

  return 0;
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

bool Wifi_Connect(char * SSID, char * password, int Status_LED){
  bool ret = false;
  pinMode(Status_LED, OUTPUT);
  digitalWrite(Status_LED, LOW);
  Serial.println("Connecting to WiFi:" + String(SSID) + "\tPassword:" + String(password));
  WiFi.begin(SSID, password);
  for(int countdown = 30; countdown > 0; countdown--){ 
    Serial.println("---------------------------------Count Down (" + String(countdown) + ")---------------------------------" );
    if(WiFi.status() != WL_CONNECTED) {
      digitalWrite(Status_LED, HIGH);
      delay(250);
      digitalWrite(Status_LED, LOW);
      delay(250);
      ret = false;
    }
    else{
      Serial.println("Connected to WiFi:" + String(SSID));
      pixel.setPixelColor(0, 0x00FF00);
      pixel.show();
      ret = true;
      break;
    }
  }
  return ret;

}

void displayWifiConnectFailed(String SSID){
  String width = "Wi-Fi ERROR";
  String text = "Could not connect to";
  lcd.clear();
  lcd.setCursor(int((20 - width.length())/2), 0);
  lcd.print(width);

  lcd.setCursor(int((20 - text.length())/2), 2);
  lcd.print(text);

  lcd.setCursor(int((20 - SSID.length())/2), 3);
  lcd.print(SSID);

}


void enableInternalPower() {
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

#if defined(NEOPIXEL_I2C_POWER)
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  // turn on the I2C power by setting pin to opposite of 'rest state'
  pinMode(PIN_I2C_POWER, INPUT);
  delay(1);
  bool polarity = digitalRead(PIN_I2C_POWER);
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, !polarity);
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif
}

void disableInternalPower() {
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, LOW);
#endif

#if defined(NEOPIXEL_I2C_POWER)
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, LOW);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  pinMode(PIN_I2C_POWER, INPUT);
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, LOW);
#endif
}

void StartupPage(String version){
  String app_ver = "V"+ version;
  lcd.clear();
  lcd.setCursor(5, 1);
  lcd.print("Node");
  lcd.setCursor((int)((20- app_ver.length())/2),2);
  lcd.print(app_ver);
}

void read_H2S_data(_sensors * sensor, H2S_data h2s){
  double data = readADC(32,4095,0);
  //Serial.println("H2S: Data:"+ String(data));
  sensor->vgas = (double)((1/h2s.M)*(data - h2s.v_gas_o));

}

void initialise_H2S_sensor(H2S_data * h2s){
  double sensitivity_gain = 4.94;
  double TIA_gain = 49.9;
  Serial.println(F("Initialising H2S Sensor"));
  h2s->v_ref = 3.3/2;
  h2s->v_offset = 0.0;
  h2s->v_gas_o = h2s->v_ref - h2s->v_offset;
  h2s->M = 2.46506 * pow(10,-4);
  h2s->v_gas = readADC(32,15,7);
  h2s->concentration = (double)((1/h2s->M)*(h2s->v_gas - h2s->v_gas_o));

  Serial.println("V_Ref: "+ String (h2s->v_ref));
  Serial.println("V_Gas_o: "+ String (h2s->v_gas_o));
  Serial.println("M: "+ String (h2s->M));
}

double map_float(int value, int from_high, int from_low, double to_high, double to_low){
  double res = ((double)(value - from_low) * (double)(to_high - to_low))/(double)(from_high - from_low);

  return res + to_low;
}

void display(dev_info *d, enum MESSAGE_TYPE read){
    String app_ver = "V"+ d->appVersion;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("       NODE 2     ");
    lcd.setCursor((int)((20- app_ver.length())/2),1);
    lcd.print(app_ver);
    lcd.setCursor(4,2);
    lcd.print(d->ipAddress);
    lcd.setCursor(0, 3);
    switch(read){
      case READ_DATA:       lcd.print("    Reading Data  ");
                            break;
      case UPLOADING_DATA:  lcd.print("   Uploading Data ");
                            break;
      case DATA_UPLOADED:   lcd.print("    Data Uploaded ");
                            break;
      case UPLOADING_FAILED:lcd.print("    Uploading Failed");
                            break;
      case WIFI_DETAILS_ERROR:       lcd.print("WiFi Details Error  ");
                            break;
      case WIFI_NOT_CONNECTED:  lcd.print(" WiFi Not Connected ");
                            break;
      case WIFI_CONNECTED:   lcd.print("    WiFi Connected ");
                            break;
      case INIT_FAILURE:    lcd.print("    Init Failed");
                            break;
    }
  
}

void readWifiDetails(String data_in, wifi_router * wr){
  Serial.println(data_in);
  String sub = data_in.substring(1,data_in.indexOf(','));
  wr->ssid = sub.substring(sub.indexOf('\"')+1,sub.lastIndexOf('\"'));
  sub = data_in.substring(data_in.indexOf(',')+1,data_in.lastIndexOf('}'));
  wr->pass = sub.substring(sub.indexOf('\"')+1,sub.lastIndexOf('\"'));
}

int getSavedWifiDetails(wifi_router * current_router){
  if (!SPIFFS.begin()) {
    return -1;
  }

  File file = SPIFFS.open("/wifi_settings.txt", "r");
  if (!file) {
    return -2;
  }
  while (file.available()) {
    String wifi_settings = file.readString();
    readWifiDetails(wifi_settings,current_router);
  }

  Serial.println();
  file.close();

  return 0;
}

void clear_screen(){
  lcd.clear();
}

void init_pixel(){
  pixel.begin(); 
  pixel.setBrightness(20); 
  pixel.setPixelColor(0, 0xFF0000);
  pixel.show();
}


void display_message(String message){
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("     STATUS");
  lcd.setCursor(0, 2);
  lcd.print(message);
}

IPAddress get_ip_address(){
  return WiFi.localIP();
}

void read_all_sensors(_sensors * s){
  delay(1000);
  read_sht_data(s);
  read_H2S_data(s,h2s_data);
  mySensor.measureAirQuality();
  reading = gas.readAll();
  s->reducing = reading.reducing;
  s->nh3 = reading.nh3;
  s->oxidising = reading.oxidising;
  s->CO2 = mySensor.CO2;
  s->TVOC = mySensor.TVOC;
}