#ifndef _DEVICE_INFO
#define _DEVICE_INFO

#include<Arduino.h>

typedef struct{
  String serialNo;
  String appVersion;
  IPAddress ipAddress;
  bool isConnected;
} dev_info;

dev_info * Dev_Info();

void setSerialNo(String serial_no, dev_info *d);
String getSerialNo();
void setAppVersion(int a, int b, int c, dev_info *d);
void Dev_InfosetConnection(bool connection, dev_info * d);
void Dev_InfosetIpAddress(IPAddress ip, dev_info * d);
IPAddress getDeviceIP();


#endif