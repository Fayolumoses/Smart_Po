#include "../inc/Server.h"
#include <Arduino.h>
#include <WebServer.h>
#include <SPIFFS.h>


WebServer server(80);

void start_server(){
    server.on("/", handleRoot);
    server.on("/submit", handleSubmit);
    server.begin();
}

void handleRoot() {
  String html = "<!DOCTYPE html>\
                <html>\
                <head>\
                <style>\
                body {\
                  font-family: Arial, sans-serif;\
                  text-align: center;\
                }\
                form {\
                  display: inline-block;\
                  text-align: left;\
                  border: 1px solid #ccc;\
                  border-radius: 5px;\
                  padding: 20px;\
                  margin-top: 50px;\
                }\
                label {\
                  font-weight: bold;\
                }\
                input[type=text] {\
                  width: 100%;\
                  padding: 8px;\
                  margin: 6px 0;\
                  box-sizing: border-box;\
                  border: 1px solid #ccc;\
                  border-radius: 4px;\
                }\
                input[type=submit] {\
                  background-color: #4CAF50;\
                  color: white;\
                  padding: 10px 20px;\
                  border: none;\
                  border-radius: 4px;\
                  cursor: pointer;\
                  float: right;\
                }\
                input[type=submit]:hover {\
                  background-color: #45a049;\
                }\
                </style>\
                <title>WiFi Settings</title>\
                </head>\
                <body>\
                <form method='get' action='/submit'>\
                  <label for='field1'>SSID:</label><br>\
                  <input type='text' id='field1' name='field1'><br>\
                  <label for='field2'>PASSWORD:</label><br>\
                  <input type='text' id='field2' name='field2'><br><br>\
                  <input type='submit' value='Submit'>\
                </form>\
                <footer>\
                  <p>Fayolumoses 2024</p>\
                </footer>\
                </body>\
                </html>";
  server.send(200, "text/html", html);
}

void handleSubmit() {
  String message = "<!DOCTYPE html>\
                <html>\
                <head>\
                <style>\
                body {\
                  font-family: Arial, sans-serif;\
                  text-align: center;\
                }\
                .container {\
                  border-radius: 5px;\
                  background-color: #f2f2f2;\
                  padding: 20px;\
                  margin-top: 50px;\
                }\
                </style>\
                <title>WiFi Saved</title>\
                </head>\
                <body>\
                <div class='container'>\
                  <h2>Wifi Settings Saved</h2>\
                  ";
  if (server.args() > 0) {
    // for (uint8_t i = 0; i < server.args(); i++) {
    //   message += "<li>" + server.argName(i) + ": " + server.arg(i) + "</li>";
      
    // }
    File file = SPIFFS.open("/wifi_settings.txt", "w");
    if (!file) {
      Serial.println("Failed to open file for writing");
      return;
    }
    String toSave = "{SSID:\"" + server.arg(0) +"\",PASSWORD:\"" + server.arg(1) + "\"}";
    Serial.println(toSave);
    file.println(toSave);
    file.close();
  }
  message += "</div></body></html>";
  server.send(200, "text/html", message);
}