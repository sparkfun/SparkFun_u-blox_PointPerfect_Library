// Update this file with all your secrets:
// WiFi SSID and Password
// Thingstream PointPerfect Client ID
// Thingstream PointPerfect Client Key
// Thingstream PointPerfect Client Certificate
// Thingstream PointPerfect AWS Root Certificate

const char ssid[] = "<Your_SSID>"; // <<<=== Paste your WiFi SSID here
const char password[] =  "<Your_Password>"; // <<<=== Paste your WiFi password here

const char AWS_IOT_ENDPOINT[]       = "pp.services.u-blox.com";
const unsigned short AWS_IOT_PORT   = 8883;

// You may need to update these depending on your plan and location. Please see:
//   Location Services \ Location Things \ Thing Details \ Topics

const char MQTT_TOPIC_KEY[]        = "/pp/key/Lb"; // This topic provides the L-Band and L-Band + IP only dynamic keys in JSON format
//const char MQTT_TOPIC_KEY[]        = "/pp/key/ip"; // This topic provides the IP-only dynamic keys in JSON format

const char MQTT_TOPIC_SPARTN[]     = "/pp/Lb/us"; // This topic provides the US SPARTN corrections for L-Band and L-Band + IP
//const char MQTT_TOPIC_SPARTN[]     = "/pp/Lb/eu"; // This topic provides the EU SPARTN corrections for L-Band and L-Band + IP
//const char MQTT_TOPIC_SPARTN[]     = "/pp/Lb/au"; // This topic provides the AU SPARTN corrections for L-Band + IP
//const char MQTT_TOPIC_SPARTN[]     = "/pp/Lb/jp"; // This topic provides the Japan SPARTN corrections for L-Band + IP

//const char MQTT_TOPIC_SPARTN[]     = "/pp/ip/us"; // This topic provides the US SPARTN corrections for IP-only
//const char MQTT_TOPIC_SPARTN[]     = "/pp/ip/eu"; // This topic provides the EU SPARTN corrections for IP-only
//const char MQTT_TOPIC_SPARTN[]     = "/pp/ip/kr"; // This topic provides the KR SPARTN corrections for IP-only
//const char MQTT_TOPIC_SPARTN[]     = "/pp/ip/au"; // This topic provides the AU SPARTN corrections for IP-only
//const char MQTT_TOPIC_SPARTN[]     = "/pp/ip/jp"; // This topic provides the Japan SPARTN corrections for IP-only

// You can find the Thingstream credentials in:
//   Location Services \ Location Things \ Thing Details \ Credentials \ MQTT Credentials

static const char MQTT_CLIENT_ID[] = "<Your_Client_ID>"; // <<<=== Paste your Client ID here

// === Update this with your Client Key ===
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
<Your_Client_Key>
-----END RSA PRIVATE KEY-----
)KEY";

// === Update this with your Client Certificate ===
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
<Your_Client_Certificate>
-----END CERTIFICATE-----
)KEY";

// === Update this with the latest AWS Root Certificate - if needed ===
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";
