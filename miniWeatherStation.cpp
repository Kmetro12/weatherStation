#include <ESP8266WiFi.h>
#include <time.h>
#include <ESP8266mDNS.h>
#include <DHT.h>
//#include "Adafruit_CCS811.h" //if using co2 and tvoc monitor 
#include <SoftwareSerial.h>
const byte rxPin = 1;
const byte txPin = 3;
SoftwareSerial pmsSerial(rxPin,txPin); //tx and rx pins of the esp8266

//unsigned long lastTime = 0;
//unsigned long timerDelay = 60000;

#ifndef STASSID
#define STASSID "yourSSID"
#define STAPSK "yourSSIDpw"
#endif

const char *ssid = STASSID;
const char *pass = STAPSK;
const int refresh = 3; 
String sensorValue = "3";
String header; 
// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

// The HTTPS server
BearSSL::WiFiServerSecure server(443);

//define dht pins and types 
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN,DHTTYPE);


float tempfarenValue; 
float humValue; 
float pm1o;
float pm25o;
float pm10o;
float pm0point3o;

//#define USE_EC //use elliptic curve signed cert

//ifdef check for the existance of macro definitions. if the identifier 
//is defined as a macro, the lines of code that immediately follow the condition are 
//passed on the the compiler, ended withj #endif 
//only passed to the compiler if the mentioned preprocessor macro is defined
//generate own keys below
#ifndef USE_EC

// The server's private key which must be kept secret
const char server_private_key[] PROGMEM = R"EOF(
-----BEGIN PRIVATE KEY-----

-----END PRIVATE KEY-----
)EOF";

// The server's public certificate which must be shared
const char server_cert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----
)EOF";

#else
////the servers private key for 
const char server_cert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----
)EOF";

// The server's private key which must be kept secret
const char server_private_key[] PROGMEM = R"EOF(
-----BEGIN EC PARAMETERS-----

-----END EC PARAMETERS-----
-----BEGIN EC PRIVATE KEY-----

-----END EC PRIVATE KEY-----

)EOF";

#endif

#define CACHE_SIZE 5  // Number of sessions to cache.
#define USE_CACHE     // Enable SSL session caching.
                      // Caching SSL sessions shortens the length of the SSL handshake.
                      // You can see the performance improvement by looking at the
                      // Network tab of the developer tools of your browser.
//#define DYNAMIC_CACHE // Whether to dynamically allocate the cache.

#if defined(USE_CACHE) && defined(DYNAMIC_CACHE)
// Dynamically allocated cache.
BearSSL::ServerSessions serverCache(CACHE_SIZE);
#elif defined(USE_CACHE)
// Statically allocated cache.
ServerSession store[CACHE_SIZE];
BearSSL::ServerSessions serverCache(store, CACHE_SIZE);
#endif

void setup() {

  dht.begin(); //start the dht sensor
      // starts to read when it gets data from buffer

  Serial.begin(115200); //being serial connection, check COM#, baud rate
  Serial.println();
  Serial.println();
  pmsSerial.begin(9600);
  //connect to a WiFi network, with above ssid and pass
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA); //set to station mode 
  WiFi.begin(ssid, pass);

  //wl connected is assigned when connected to a wifi network
  //while not connected to a wifi network, will continute to try and connect 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP()); //print out the ip address from the router
  
  if (!MDNS.begin("whtrstation")){
    Serial.println("mDNS responder started");
    }
   

  // Attach the server private cert/key combo, from the above EC or RSA 
  BearSSL::X509List *serverCertList = new BearSSL::X509List(server_cert);
  BearSSL::PrivateKey *serverPrivKey = new BearSSL::PrivateKey(server_private_key);


 //server.setRSACert library? does....
 
#ifndef USE_EC
  server.setRSACert(serverCertList, serverPrivKey);
#else
  server.setECCert(serverCertList, BR_KEYTYPE_KEYX | BR_KEYTYPE_SIGN, serverPrivKey);
#endif

//set server cache
#if defined(USE_CACHE)
  server.setCache(&serverCache);
#endif

  // Actually start accepting connections
  server.begin();
}

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
 
struct pms5003data data;


void loop(){
  BearSSL::WiFiClientSecure incoming = server.accept();
  MDNS.update();
  float f = dht.readTemperature(true); 
  float h = dht.readHumidity();
  float pm1onew = data.pm10_standard;
  float pm25onew = data.pm25_standard;
  float pm10onew = data.pm100_standard;
//  float pm0point3onew = data.particles_03um;
  tempfarenValue=f;  
  humValue=h;
  pm1o=pm1onew;
  pm25o=pm25onew;
  pm10o=pm10onew;

  if (incoming) {
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    currentTime = millis(); 
    previousTime = currentTime;
    while (incoming.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
      currentTime = millis();         
      if (incoming.available()) {             // if there's bytes to read from the client,
        char c = incoming.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n' && (readPMSdata(&pmsSerial))) {     // if the byte is a newline cha     
          if (currentLine.length() == 0) { 
          incoming.println("<!DOCTYPE html><html>");
          incoming.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
          incoming.println("<link rel=\"icon\" href=\"data:,\">"); 
          incoming.println("<body>");
//          incoming.println("<p>hello</p>");
          incoming.println("<p>Temperature: " + String(tempfarenValue,2) + "</p>");
          incoming.println("<p>Humidity: " + String(humValue,2) + "</p>"); 

          incoming.println("<p>pm 1micron: " + String(data.pm10_standard) + "</p>");
          incoming.println("<p>pm 2.5micron: " + String(data.pm25_standard) + "</p>");
          incoming.println("<p>pm 10micron: " + String(data.pm100_standard) + "</p>");
          incoming.println("<p>Particles > 0.3um / 0.1L air: " + String(data.particles_03um) + "</p>");
          incoming.println("<p>Particles > 0.5um / 0.1L air: " + String(data.particles_05um) + "</p>");
          incoming.println("<p>Particles > 1.0um / 0.1L air: " + String(data.particles_10um) + "</p>");
          incoming.println("<p>Particles > 2.5um / 0.1L air: " + String(data.particles_25um) + "</p>");
          incoming.println("<p>Particles > 5.0um / 0.1L air: " + String(data.particles_50um) + "</p>");
          incoming.println("<p>Particles > 10.0 um / 0.1L air: " + String(data.particles_100um) + "</p>"); 
          delay(5000);

          incoming.println("</body></html>");
          incoming.println();
          
            // Break out of the while loop
          break;
         //end if current line
        } else {
            currentLine = "";
            }//end else
        //end if c = new line
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        } //end else if 
    } //end if (incoming.avail)
  }//end while    
    // Clear the header variable
    header = "";
    // Close the connection
    incoming.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
    } //end if (incoming)

 
  } //end if void loop

boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
 
  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
 
  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }
 
  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  */
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
 
  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);
 
  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}

/*
const char *HTTP_RES = "HTTP/1.0 200 OK\r\n"
                              "Connection: close\r\n"
                              "Content-Length: 62\r\n"
                              "Content-Type: text/html; charset=iso-8859-1\r\n"
                              "\r\n"
                              "<html>\r\n"
                              "<body>\r\n"
                              "<p>Hello customSSL!</p>\r\n"
                                HTTP_RES += "<p style=\"font-size:20px;\">Temperature Faren:<br/>\r\n"
                                HTTP+RES += "<p style=\"color:blue; font-size:50px\">"
                                HTTP_RES += String(sensorValue, 2);
                              "</body>\r\n"
                              "</html>\r\n";

const char index_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML><html>
  <head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">
  <title>Hellossl</title>
  </head>
  <body>
  <p>hello modification test</p>
  </body>

</html>)rawliteral"; 

void sendTemp(){

  String page = "<!DOCTYPE html>\n\n";
  page += "<meta http-equiv='refresh' content='";
  page += String(refresh); 
  page += "'/>\n";
  page += "<html>\n";
  page += "<body>\n";
  page += "<p style=\"font-size:20px;\">Temperature Faren:<br/>\n";
  page += "<p style=\"color:blue; font-size:50px\">";
  page += String(sensorValue, 2);
  page += "</p>\n";
  page += "</body>\n";
  page += "</html>\n";
  server.accept(200, "text/html",page); 
}//end function that creates the webpage 

void loop() {
  static int cnt;
  BearSSL::WiFiClientSecure incoming = server.accept();
  if (!incoming) { return; }
  Serial.printf("Incoming connection...%d\n", cnt++);

  // Ugly way to wait for \r\n (i.e. end of HTTP request which we don't actually parse here)
  uint32_t timeout = millis() + 1000;
  int lcwn = 0;
  for (;;) {
    unsigned char x = 0;
    if ((millis() > timeout) || (incoming.available() && incoming.read(&x, 1) < 0)) {
      incoming.stop();
      Serial.printf("Connection error, closed\n");
      return;
    } else if (!x) {
      yield();
      continue;
    } else if (x == 0x0D) {
      continue;
    } else if (x == 0x0A) {
      if (lcwn) { break; }
      lcwn = 1;
    } else
      lcwn = 0;
  }
 

  //HTTP_RES += sensorValue;
  incoming.write((uint8_t *)HTTP_RES, strlen(HTTP_RES));
  incoming.flush();
  incoming.stop();
  Serial.printf("Connection closed.\n");
}
*/
