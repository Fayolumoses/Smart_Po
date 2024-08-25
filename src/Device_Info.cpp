#include "../inc/Device_Info.h"

dev_info device_info;
dev_info * ptr_dev_info = &device_info;

dev_info * Dev_Info(){
  return ptr_dev_info;
}

void setSerialNo(String serial_no, dev_info *d){
  d->serialNo = serial_no;
}

String getSerialNo(){
  return Dev_Info()->serialNo;
}

void setAppVersion(int a, int b, int c, dev_info *d){
  String temp = String(a) + "." + String(b) + "." + String(c);
  d->appVersion = temp;
}

void Dev_InfosetConnection(bool connection, dev_info * d){
  d->isConnected = connection;
}

void Dev_InfosetIpAddress(IPAddress ip, dev_info * d){
  d->ipAddress = ip;
}

IPAddress getDeviceIP(){
  return Dev_Info()->ipAddress;
}
