#include "inc/utils.h"

void setup(){
    Serial.begin(115200);
    setAppVersion(1,0,0, Dev_Info());

    StartupPage(Dev_Info()->appVersion);
    delay(3000);
    clear_screen();
    lcd_progressbar("Loading...",0, 20);

    wifi_router wifi_details;
    if(getSavedWifiDetails(&wifi_details) != 0){
        Serial.println("Error Retrieving Wifi Details. Please Restart");
    }

    lcd_progressbar("Loading...",10, 20);

    if(init_sensors() != 0){
        Serial.println("Error: Sensor not present. Please Check Connection and Restart");
    }

    lcd_progressbar("Loading...",30, 20);

    enableInternalPower();
    init_pixel();
    lcd_progressbar("Loading...",65, 20);

    if(!Wifi_Connect((char*)wifi_details.ssid.c_str(), (char*)wifi_details.pass.c_str(),LED_BUILTIN)){
        Serial.println("Could not connect to Wifi");
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

}