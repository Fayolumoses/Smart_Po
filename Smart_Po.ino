#include "inc/utils.h"

WiFiClient client;
HTTPClient http;
const char * url = "";

void setup(){
    Serial.begin(115200);
    setAppVersion(1,0,0, Dev_Info());

    StartupPage(Dev_Info()->appVersion);
    delay(3000);
    clear_screen();
    lcd_progressbar("Loading...",0, 20);

    wifi_router wifi_details;
    if(getSavedWifiDetails(&wifi_details) != 0){
        while(1) {}
    }

    lcd_progressbar("Loading...",10, 20);

    if(init_sensors() != 0){
        while(1) {}
    }

    lcd_progressbar("Loading...",30, 20);

    enableInternalPower();
    init_pixel();
    lcd_progressbar("Loading...",65, 20);

    if(!Wifi_Connect(const_cast<char*>(wifi_details.ssid.c_str()), const_cast<char*>(wifi_details.pass.c_str()),LED_BUILTIN)){
        displayWifiConnectFailed(wifi_details.ssid);
        delay(500);
      }
    else {
        Dev_InfosetConnection(true,Dev_Info());
    }

    if(!Dev_Info()->isConnected){
        display_message("Restarting System");
        delay(2000);
        ESP.restart();
    }

    Dev_InfosetIpAddress(get_ip_address(), Dev_Info());
    lcd_progressbar("Loading...",80, 20);

}

void loop(){
    display(Dev_Info(),READ_DATA);
    _sensors sense;
    read_all_sensors(&sense);

    http.begin(client, url);
    http.addHeader("Content-Type", "application/json");
    display(Dev_Info(),UPLOADING_DATA);

    int httpResponseCode = http.POST(encodeSensorData(sense));

    if(httpResponseCode  == 200) display(Dev_Info(),DATA_UPLOADED);
    else display(Dev_Info(),UPLOADING_FAILED);
    delay(5000);

}