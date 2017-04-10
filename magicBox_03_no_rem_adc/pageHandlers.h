
void handleHome() {
  //upload page_00.h
  Serial.println("handleWebsite");
//  server.send(200, "text/css", "aaaa");
  
//  server.send(200, "text/html", home_data);
  server.send(200, "text/html", SPIFFS.open("/home.html", "r").readString() );
  
}
