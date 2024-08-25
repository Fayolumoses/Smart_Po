#ifndef _UTILS
#define _UTILS

#include <Adafruit_NeoPixel.h>
#include <LiquidCrystal_I2C.h>
#include <SPIFFS.h>
#include "Adafruit_SHT4x.h"
#include "SparkFun_SGP30_Arduino_Library.h" 
#include "GasBreakout.h"
#include "Device_Info.h"



#if defined(ADAFRUIT_FEATHER_ESP32_V2)
#define PIN_NEOPIXEL 0
#define NEOPIXEL_I2C_POWER 2
#endif



typedef struct {
  float temperature;
  float relative_humidity;
  double vgas;
  int CO2;
  int TVOC;
  float reducing;
  float nh3;
  float oxidising;
} _sensors;

typedef struct{
  double v_ref;
  double v_gas_o;
  double v_offset;
  double concentration;
  double M;
  double v_gas;
} H2S_data;

typedef struct{
  String ssid;  
  String pass;  
} wifi_router;

  String encodeSensorData(_sensors sensor);
  int read_sht_data(_sensors * sensor);
  int init_sensors();
  double readADC(int adc_pin, int max_input, int min_input);
  int lcd_progressbar( String Title, uint8_t percent, int width);
  void printWifiStatus();
  bool Wifi_Connect(char * SSID, char * password, int Status_LED);
  void displayWifiConnectFailed(String SSID);
  void enableInternalPower();
  void disableInternalPower();
  void StartupPage(String version);
  void read_H2S_data(_sensors * sensor, H2S_data h2s);
  void initialise_H2S_sensor(H2S_data * h2s);
  double map_float(int value, int from_high, int from_low, double to_high, double to_low);
  void display(dev_info *d, LiquidCrystal_I2C lcd, int read);
  void readWifiDetails(String data_in, wifi_router * wr);
  int getSavedWifiDetails(wifi_router * current_router);
  void clear_screen();
  void init_pixel();
  void display_connection_status(bool isConnected);
  void display_message(String message);
  IPAddress get_ip_address();
  
#endif