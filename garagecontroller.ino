#include <NewPing.h>
#include <Credentials.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

#define DHTTYPE DHT22

const char* DEVICE_NAME = "GARAGE CONTROLLER";

const char* frontdoor_topic = "garage/sensor/door/frontdoor";
const char* frontbutton_topic = "garage/relay/door/frontdoor";
const char* backdoor_topic = "garage/sensor/door/backdoor";
const char* backbutton_topic = "garage/relay/door/backdoor";
const char* garageToHouse_topic = "garage/sensor/door/garagetohouse";

const char* frontdoor_status_topic = "garage/door/status/front";
const char* backdoor_status_topic = "garage/door/status/back";

const char* esp_restart_topic = "garage/controller/restart";
const char* collin_car_topic = "garage/sensor/car/collin";
const char* cheryl_car_topic = "garage/sensor/car/cheryl";
const char* temperature_topic = "garage/sensor/dht/temperature";
const char* humidity_topic = "garage/sensor/dht/humidity";

const char* frontdoor_current_state = "UNDEFINED";
const char* backdoor_current_state = "UNDEFINED";
const char* garageToHouse_current_state = "UNDEFINED";


// Minimum stable period in milliseconds
const unsigned long stablePeriod = 3000;

// Variables to store the last time a stable state was detected for each door
unsigned long frontdoor_lastStableTime = 0;
unsigned long backdoor_lastStableTime = 0;
unsigned long garageToHouse_lastStableTime = 0;

const char* bakkieStatus = "UNDEFINED";
const char* mazdaStatus = "UNDEFINED";
const int MAX_DISTANCE = 400;
unsigned long lastCarPrensenceReadTime = 0;
const unsigned long CAR_PRESENCE_CHECK_INTERVAL = 2000;  // Interval 2 sec

float temperature = 0;
float humidity = 0;

const int DHT22_PIN = 21;
const int FRONTDOOR_RELAY_PIN = 27;
const int FRONTDOOR_PIN = 13;
const int BACKDOOR_RELAY_PIN = 26;
const int BACKDOOR_PIN = 12;
const int GARAGE_TO_HOUSE_PIN = 14;
const int TRIGGER_PIN_COLLIN = 19;
const int ECHO_PIN_COLLIN = 33;
const int TRIGGER_PIN_CHERYL = 18;
const int ECHO_PIN_CHERYL = 32;

NewPing sonarCollin(TRIGGER_PIN_COLLIN, ECHO_PIN_COLLIN, MAX_DISTANCE);
NewPing sonarCheryl(TRIGGER_PIN_CHERYL, ECHO_PIN_CHERYL, MAX_DISTANCE);

WiFiClient garageController;
PubSubClient client(garageController);

DHT dht(DHT22_PIN, DHTTYPE);

void setup() {
  Serial.begin(115200);

  //Set Relay(output) and Door(input) pins
  pinMode(FRONTDOOR_RELAY_PIN, OUTPUT);
  digitalWrite(FRONTDOOR_RELAY_PIN, HIGH);
  pinMode(FRONTDOOR_PIN, INPUT_PULLUP);

  pinMode(BACKDOOR_RELAY_PIN, OUTPUT);
  digitalWrite(BACKDOOR_RELAY_PIN, HIGH);
  pinMode(BACKDOOR_PIN, INPUT_PULLDOWN);

  pinMode(GARAGE_TO_HOUSE_PIN, INPUT_PULLUP);
  dht.begin();

  setupWiFi();
  setupMQTT();
  setupOTA();

  checkCarPresence();
}

void loop() {

  //If MQTT client can't connect to broker, then reconnect
  if (!client.connected()) {
    reconnect();
  }
  ArduinoOTA.handle();
  client.loop();

  if (millis() - lastCarPrensenceReadTime >= CAR_PRESENCE_CHECK_INTERVAL) {
    checkCarPresence();
  }
  checkDoorsState();
  checkDHT();
}

void checkDHT() {
  float newTempValue = dht.readTemperature();
  float newHumValue = dht.readHumidity();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    temperature = 0;
    humidity = 0;
  }

  if (checkAndPublishValue(newTempValue, temperature, temperature_topic, 0.5)) {
    temperature = newTempValue;
  }

  if (checkAndPublishValue(newHumValue, humidity, humidity_topic, 2.0)) {
    humidity = newHumValue;
  }
}

bool checkAndPublishValue(float newValue, float& prevValue, const char* topic, float maxDiff) {
  if (fabs(newValue - prevValue) >= maxDiff) {
    prevValue = newValue;
    client.publish(topic, String(prevValue).c_str());
    return true;
  }
  return false;
}

void checkCarPresence() {

  lastCarPrensenceReadTime = millis();

  float newDistanceCheryl = sonarCheryl.ping_cm();
  // client.publish("garage/sensor/car/cheryl/distance", String(newDistanceCheryl).c_str());
  delay(200);
  float newDistanceCollin = sonarCollin.ping_cm();
  // client.publish("garage/sensor/car/collin/distance", String(newDistanceCollin).c_str());

  if (newDistanceCheryl == 0) {
    newDistanceCheryl = sonarCheryl.ping_cm();
  }

  if (newDistanceCollin == 0) {
    newDistanceCollin = sonarCollin.ping_cm();
  }

  const char* newBakkieStatus = (newDistanceCollin > 200) ? "AWAY" : "HOME";
  if (strcmp(newBakkieStatus, bakkieStatus) != 0) {
    bakkieStatus = newBakkieStatus;
    client.publish(collin_car_topic, bakkieStatus);
  }

  const char* newMazdaStatus = (newDistanceCheryl > 200) ? "AWAY" : "HOME";
  if (newMazdaStatus != mazdaStatus) {
    mazdaStatus = newMazdaStatus;
    client.publish(cheryl_car_topic, mazdaStatus);
  }
}

void checkDoorsState() {

  auto getDoorState = [](int pin) -> const char* {
    return digitalRead(pin) == 0 ? "closed" : "open";
  };

   // Stable state logic as a lambda function
  auto isStateStable = [](unsigned long& lastStableTime, const char*& currentState, const char* newState) -> bool {
    if (strcmp(currentState, newState) != 0) {
      // If the state has changed, check if the stable period has elapsed
      if (millis() - lastStableTime > stablePeriod) {
        currentState = newState;      // Update the state
        lastStableTime = millis();    // Reset the stable time
        return true;                  // State is considered stable
      }
    } else {
      // If state is the same, reset the stable time
      lastStableTime = millis();
    }
    return false; // State is not yet stable
  };

  // Front Garage Door
  const char* frontdoor_new_state = getDoorState(FRONTDOOR_PIN);
  if (isStateStable(frontdoor_lastStableTime, frontdoor_current_state, frontdoor_new_state)) {
    client.publish(frontdoor_topic, frontdoor_current_state, true);
  }

  // Back Garage Door
  const char* backdoor_new_state = getDoorState(BACKDOOR_PIN);
  if (isStateStable(backdoor_lastStableTime, backdoor_current_state, backdoor_new_state)) {
    client.publish(backdoor_topic, backdoor_current_state, true);
  }

  // Garage to House Door
  // const char* garageToHouse_new_state = getDoorState(GARAGE_TO_HOUSE_PIN);
  // if (isStateStable(garageToHouse_lastStableTime, garageToHouse_current_state, garageToHouse_new_state)) {
  //   client.publish(garageToHouse_topic, garageToHouse_current_state, true);
  // }

  //Front Garage Door
  // const char* frontdoor_new_state = getDoorState(FRONTDOOR_PIN);
  // if (strcmp(frontdoor_new_state, frontdoor_current_state) != 0) {
  //   frontdoor_current_state = frontdoor_new_state;
  //   client.publish(frontdoor_topic, frontdoor_current_state, true);
  // }

  //Back Garage Door
  // const char* backdoor_new_state = getDoorState(BACKDOOR_PIN);
  // if (strcmp(backdoor_new_state, backdoor_current_state) != 0) {
  //   backdoor_current_state = backdoor_new_state;
  //   client.publish(backdoor_topic, backdoor_current_state, true);
  // }

  // Garage To House Door
  const char* garageToHouse_new_state = getDoorState(GARAGE_TO_HOUSE_PIN);
  if (strcmp(garageToHouse_new_state, garageToHouse_current_state) != 0) {
    garageToHouse_current_state = garageToHouse_new_state;
    client.publish(garageToHouse_topic, garageToHouse_current_state, true);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  //if the 'garage/button' topic has a payload "OPEN", then 'click' the relay
  payload[length] = '\0';
  char* strTopic = topic;
  if (strcmp(strTopic, frontbutton_topic) == 0) {
    char* frontDoorContact = (char*)payload;
    // Serial.println(frontDoorContact);
    if (strcmp(frontDoorContact, "OPEN") == 0) {
      //'click' the relay
      digitalWrite(FRONTDOOR_RELAY_PIN, LOW);
      delay(600);
      digitalWrite(FRONTDOOR_RELAY_PIN, HIGH);
      //This publish is for the garage keypad to play success tone
      client.publish(frontdoor_status_topic, frontdoor_current_state);
    }
  } else if (strcmp(strTopic, backbutton_topic) == 0) {
    char* backDoorContact = (char*)payload;
    if (strcmp(backDoorContact, "OPEN") == 0) {
      //'click' the relay
      digitalWrite(BACKDOOR_RELAY_PIN, LOW);
      delay(600);
      digitalWrite(BACKDOOR_RELAY_PIN, HIGH);
      //This publish is for the garage keypad to play success tone
      client.publish(backdoor_status_topic, backdoor_current_state);
    }
  } else if (strcmp(strTopic, esp_restart_topic) == 0) {
    restartESP();
  }
}

void setupWiFi() {
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    // Timeout if connection takes to long (15 seconds)
    if (millis() - startTime > 15000) {
      restartESP();
    }
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
}

void setupMQTT() {
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);  // callback is the function that gets called for a topic subscription

  //Reconnect to Wifi and to MQTT. If Wifi is already connected, then autoconnect doesn't do anything.
  Serial.println("Attempting MQTT connection...");
  if (client.connect(DEVICE_NAME, MQTT_USERNAME, MQTT_PASSWORD, "controller/last/will", 0, false, "Garage Controller Offline")) {
    client.publish("controller/last/will", "Garage Controller ONLINE");
    Serial.println("connected");
    subscribeMQTTTopics();
  } else {
    Serial.print("failed, rc=");
    Serial.println(client.state());
    Serial.println(" try again in 5 seconds");
    // Wait 5 seconds before retrying
    restartESP();
  }
}

void subscribeMQTTTopics() {
  client.subscribe("garage/#");
}

void setupOTA() {
  ArduinoOTA.setHostname(DEVICE_NAME);
  ArduinoOTA.begin();
}

void reconnect() {

  // Reconnect to WiFi and MQTT
  if (WiFi.status() != WL_CONNECTED) {
    setupWiFi();
  }

  // Check if MQTT connection is not established
  if (!client.connected()) {
    setupMQTT();
  }
}

void restartESP() {
  ESP.restart();
}
