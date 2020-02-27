#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "mqttcert.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Servo.h>

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

long lastMsgTimer = 0;
const int onboardLED = 2; // GPIO2 (D2) on the DOIT-ESP32-DevKitV1

// Motion detection variables
const std::string motionDetectTopic = "302CEM/Horse/Requests/MotionSensor"; // Define motion sensor topic
#define pirPin 22    // Digital pin connected to PIR sensor
bool motion_Detected = 0; // Set motion detection flag to false on startup
bool motion_Recently_Detected_Flag = 0; // Flag used to prevent a stationary person from eratically triggering the auto lights
bool motion_Detected_Previous_State = 0; // Used to compare with motion_Recently_Detected_Flag to check if a motion change has been detected
unsigned int motion_Detect_Time; // Holds the time when motion was detected
unsigned int motion_Detected_Last_Duration; // Hold the time duration for which motion was last detected by the PIR sensor
unsigned int motion_Detect_Timer = 2000;  // When no motion is detected, this period of time in ms is used to keep checking for motion to confirm that there is no movement.

// Automatic light variables
const std::string autoLightTopic = "302CEM/Horse/Requests/AutoLight"; // Define auto light topic
const std::string lightDelayOffTopic = "302CEM/Horse/Requests/AutoLight/LightDelay"; // Define auto light OFF delay topic
const std::string ambientLightThresholdTopic = "302CEM/Horse/Requests/AutoLight/AmbientLightThreshold"; // Define ambient light threshold topic
#define autoLight 23    // Digital pin connected to outdoor light
#define photosensor 36 // Analog pin connected to LDR (VP pin)
bool auto_Light_Override_Flag = 0;   // Set automatic light override flag to false on startup
bool current_light_status;   // Holds status of auto light (ON = 1, OFF = 0)
unsigned int ambientLightLevel;   // Used to store value of ambient light level
unsigned int ambientLightThreshold = 2048;  // Sets ambient light level threshold which decides whether light should be ON or OFF
unsigned int delayed_Light_OFF_Timer = 5000;  // Holds amount of time in ms the time delay to wait before turning the light off when no motion is detected
unsigned int light_last_ON_Time;    // Holds the time when the light was last ON while movement was also recently detected

// Temperature sensor variables
const std::string targetTemperatureTopic = "302CEM/Horse/Requests/TemperatureSensor/TargetTemperature"; // Define target temperature topic
const std::string autoTempManagementControl = "302CEM/Horse/Requests/TemperatureSensor/AutoTempManagement";   // Define auto temp management topic
#define DHTPIN 21    // Digital pin connected to DHT11 sensor
DHT_Unified dht(DHTPIN, DHT11);   // Initialise DHT11 sensor
int current_Temp = 20; // Holds current temperature
unsigned int last_Temp_Reading_Time = 0;    // Holds time temperature sensor was read last
int target_Temperature = 21;    // Holds user set target temperature. System will aim to keep house at this temperature
bool auto_Temp_Management_Enabled = 1; // If 1, house temperature is managed automatically by controlling windows, fans and heaters e.g.

// Servo variables
const std::string windowOperationModeTopic = "302CEM/Horse/Requests/Window/WindowOperationMode"; // Define window operation mode topic
const std::string manualWindowControlTopic = "302CEM/Horse/Requests/Window/ManualWindowControl"; // Define manual window control topic
#define servo_Pin 25    // Servo connected to this pin
Servo window_Servo;  // Create servo object to control a servo
bool window_Current_State = 0;  // Holds status of window (0 = closed, 1 = open)
bool window_Manual_Mode = 0;    // If 1, windows are not managed by auto temperature management system

// End stop switch variables - used to monitor if doors/windows are open/closed
const std::string intruderAlarmTopic = "302CEM/Horse/Requests/HomeSecurity/IntruderAlarm"; // Define intruder alarm topic
const std::string motionSensorForIntruderAlarmTopic = "302CEM/Horse/Requests/HomeSecurity/MotionSensorForIntruderAlarm"; // Define motion sensor for intruder alarm topic
const std::string acknowledgeIntruderEventTopic = "302CEM/Horse/Requests/HomeSecurity/AcknowledgeIntruderEvent";    // Define acknowledge intruder event topic
const std::string clearIntruderEventTopic = "302CEM/Horse/Requests/HomeSecurity/ClearIntruderEvent";    // Define clear intruder event topic
#define frontDoor 15   // Pin connected to end stop switch 1
#define backDoor 4   // Pin connected to end stop switch 2
bool front_Door_State = 0;  // Holds status of front door (OPEN = 1, CLOSED = 0)
bool back_Door_State = 0;   // Holds status of back door (OPEN = 1, CLOSED = 0)
bool intruder_Alarm = 0;    // Holds status of intruder alarm (ON = 1, OFF = 0)
bool use_Motion_Sensor = 1; // Defines whether the motion sensor will be used to trigger the intruder alarm (ON = 1, OFF = 0)
bool intruder_Event = 0;    // Holds the status of whether an intruder event has occurred or not (Intruder detected = 1, No intruder = 0)
bool intruder_Event_Acknowledged = 0; // Holds flag whether user has seen the intruder alert or not (1 = seen, 0 = not seen)
unsigned int last_Intruder_Message_Sent_Time;   // Holds time when last intruder event message was sent so that a message can be sent every 5 seconds

// Function used so that all sensors can publish data to a desired topic. Topic = 302CEM/ferna118/Horse/Readings/<TOPIC>
void mqttPublish (std::string TOPIC, std::string DATA_TO_SEND){
    TOPIC = MQTT_PUBLISH_TOPIC + TOPIC; // Concatenate strings to create one topic
    
    Serial.print("Publishing data:  ");
    Serial.println(DATA_TO_SEND.c_str());
    Serial.print("Publishing to:  ");
    Serial.println(TOPIC.c_str());

    mqttClient.publish(TOPIC.c_str(), DATA_TO_SEND.c_str()); // Publish data to topic
}

// Auto light light function
void autoLightFunction(){

    // Check state of auto_Light_Override_Flag. If set, do not control light automatically.
    if(!auto_Light_Override_Flag){

        // motion_Recently_Detected_Flag = 1;    // For testing lights

        // If motion was detected, read the LDR to read ambient light levels. Then control lights deending on ambient light levels
        if(motion_Recently_Detected_Flag){

            // Read ambient light level
            ambientLightLevel = analogRead(photosensor);
            //Serial.print("LDR reading: "); Serial.println(ambientLightLevel);
            
            // Check if light status needs changing and record whether to turn it on/off
            bool next_light_status;
            if(ambientLightLevel > (ambientLightThreshold + 100)){
                next_light_status = 0;  // Set flag to indicate that light needs to be turned off
            }
            else if(ambientLightLevel < (ambientLightThreshold - 100)){
                next_light_status = 1;  // Set flag to indicate that light needs to be turned on
            }

            // Only adjust lights if the ambient light threshold has been crossed - for code efficiency and to not spam MQTT server with light change updates
            if(current_light_status != next_light_status){
                // Turn light on/off depending on Light_Status variable
                if(next_light_status){
                    digitalWrite(autoLight, HIGH);  // Turn light on
                }
                else{
                    digitalWrite(autoLight, LOW);   // Turn light off
                }

                // Update light status
                current_light_status = next_light_status;
                
                // Publish status of auto light to MQTT server
                String dataToSend = "{ \"Room\" : \"Bedroom\", \"Light_status\" : " + String(current_light_status) + " }";  // Create json string
                mqttPublish("/AutoLights", dataToSend.c_str());    // Publish data
            }
        }

        // If the light is ON and motion has been detected recently, record the time - records last motion detected time while light was also ON
        if(current_light_status & motion_Recently_Detected_Flag){
            light_last_ON_Time = millis();
        }

        // If motion has not been detected, turn off light after user set delay.
        if(current_light_status & ( (millis()-light_last_ON_Time) > delayed_Light_OFF_Timer )){
            digitalWrite(autoLight, LOW);     // Turn light off
            current_light_status = 0;   // Update light status

            // Publish status of auto light to MQTT server
            String dataToSend = "{ \"Room\" : \"Bedroom\", \"Light_status\" : " + String(current_light_status) + " }";  // Create json string
            mqttPublish("/AutoLights", dataToSend.c_str());    // Publish data

            // Print confirmation that light has been automatically turned OFF
            Serial.println("No motion detected, light turned OFF.");
        }
        
    }

}

// Callback function for using auto lights manually
void manualLightControl(unsigned int data){

    // Update auto_Light_Override_Flag. If data = 0, auto mode. If data = 1, manual mode.
    if( (data == 0) || (data == 1) ){

        // Convert data to boolean
        bool data_bool;
        if(data == 0){
            data_bool = 0;
        }
        else{
            data_bool = 1;
        }

        // Only update auto_Light_Override_Flag if it is not already set to the requested mode, then publish the operation mode.
        if( auto_Light_Override_Flag ^ data_bool){
            auto_Light_Override_Flag = data_bool;

            // Publish auto light operation mode to MQTT server
            String dataToSend = "{ \"Room\" : \"Bedroom\", \"Auto_Light_Override\" : " + String(auto_Light_Override_Flag) + " }";  // Create json string
            mqttPublish("/AutoLights", dataToSend.c_str());    // Publish data
        }

        // Print confirmation message
        Serial.print("Succesfully set auto_Light_Override_Flag to: ");
        Serial.println(String(auto_Light_Override_Flag));
    }

    // Set auto light to requested state only if auto lights are in manual mode.
    // If data = 2, lights OFF. If data = 3, lights ON
    if( auto_Light_Override_Flag & ((data == 2) || (data == 3)) ){
        
        // Convert data to boolean
        bool data_bool;
        if(data == 2){
            data_bool = 0;
        }
        else{
            data_bool = 1;
        }

        // Only change light status if it is not already set to the requested mode, then publish the light status.
        if( current_light_status ^ data_bool){

            if(data_bool){
                digitalWrite(autoLight, HIGH);     // Turn light ON
            }
            else{
                digitalWrite(autoLight, LOW);     // Turn light OFF
            }

            current_light_status = data_bool;   // Update current light status

            // Publish status of the light to MQTT server
            String dataToSend = "{ \"Room\" : \"Bedroom\", \"Light_status\" : " + String(current_light_status) + " }";  // Create json string
            mqttPublish("/AutoLights", dataToSend.c_str());    // Publish data
        }

        // Print confirmation message
        Serial.print("Succesfully set current_light_status to: ");
        Serial.print(String(current_light_status));
    }

}

// Callback function for changing delayed_Light_OFF_Timer
void changeLightOffDelay(unsigned int duration_ms){

    // Update delayed_Light_OFF_Timer with given value
    delayed_Light_OFF_Timer = duration_ms;

    // Publish newly set delayed_Light_OFF_Timer
    String dataToSend = "{ \"Light_Off_Delay\" : " + String(delayed_Light_OFF_Timer) + " }";  // Create json string
    mqttPublish("/AutoLight", dataToSend.c_str());    // Publish data

    // Print change succesfull message
    Serial.print("delayed_Light_OFF_Timer updated succesfully to: ");
    Serial.println(delayed_Light_OFF_Timer);

}

// Callback function for changing ambientLightThreshold
void changeAmbientLightThreshold(unsigned int new_threshold){

    // Convert given percentage to usable number and update ambientLightThreshold with given value
    ambientLightThreshold = round(float(new_threshold)/100 * 4096);

    // Publish newly set ambientLightThreshold as percentage
    String dataToSend = "{ \"Ambient_Light_Threshold\" : " + String(new_threshold) + " }";  // Create json string
    mqttPublish("/AutoLight", dataToSend.c_str());    // Publish data

    // Print change succesfull message
    Serial.print("ambientLightThreshold updated succesfully to: ");
    Serial.println(ambientLightThreshold);
}

// Funtion for monitoring movement using PIR
void motionDetectionFunction(){
    // Check PIR sensor to see if motion was detected recently.
    motion_Detected = digitalRead(pirPin);

    // If motion has been detected recently, set motion_Recently_Detected_Flag and record time when motion was detected.
    // Else record the time duration for which no movement has been detected.
    // This is used for a timer so that devices are not controlled erratically if the user sits still and no motion is detected.
    if(motion_Detected){
        motion_Recently_Detected_Flag = 1;
        motion_Detect_Time = millis(); // Record time when motion was detected
    }

    // If motion has not been detected for longer than the set amount of time, clear motion_Recently_Detected_Flag.
    motion_Detected_Last_Duration = millis() - motion_Detect_Time; // Record how long movement has not been detected
    if(motion_Detected_Last_Duration > motion_Detect_Timer){
        motion_Recently_Detected_Flag = 0; // Clear motion_Recently_Detected_Flag
    }

    // Publish status of motion sensor to MQTT server only when a change has been detected.
    if( motion_Recently_Detected_Flag ^ motion_Detected_Previous_State){
        // Publish status of motion sensor to MQTT server
        String dataToSend = "{ \"Room\" : \"Bedroom\", \"Motion_detected\" : " + String(motion_Detected) + " }";  // Create json string
        mqttPublish("/MotionDetected", dataToSend.c_str());    // Publish data

        // Update motion_Detected_Previous_State
        motion_Detected_Previous_State = motion_Recently_Detected_Flag;
    }
}

// Callback function for motion sensor
void motionDetectorCallback(unsigned int duration_ms){

    // Update motion_Detect_Timer with user given value
    motion_Detect_Timer = duration_ms;

    // Publish newly set motion_Detect_Timer
    String dataToSend = "{ \"No_Motion_Detected_Delay\" : " + String(motion_Detect_Timer) + " }";  // Create json string
    mqttPublish("/MotionSensor", dataToSend.c_str());    // Publish data

    // Print change succesfull message
    Serial.print("motion_Detect_Timer updated succesfully to: ");
    Serial.println(duration_ms);
}

// Temperature sensor function
void temperatureSensorFunction(){

    // DHT11 can only get a reading every second, also temperature doesn't need to be monitored every second. So check temperature every 5 seconds
    if( (millis()-last_Temp_Reading_Time) > 5000){

        // Get temperature event
        sensors_event_t event;
        dht.temperature().getEvent(&event);
        int next_Temp = round(event.temperature);

        // If temperature has changed by at least 1°C, update and publish temperature
        if( (next_Temp >= (current_Temp+1)) | (next_Temp <= (current_Temp-1)) ){

            // Update current_Temp
            current_Temp = next_Temp;

            // Publish temperature to MQTT server
            String dataToSend = "{ \"Temperature\" : " + String(current_Temp) + " }";  // Create json string
            mqttPublish("/TemperatureSensor", dataToSend.c_str());    // Publish data

            // Check if target temperature reached
            if(current_Temp == target_Temperature){
                Serial.println("Target temperature achieved.");
            }

            // Print temperature reading
            Serial.print("Temperature is now: "); Serial.print(current_Temp); Serial.println("°C");          
        }

        // Record time temperature was measured
        last_Temp_Reading_Time = millis();
    }
}

// Callback funtion to change target temperature
void changeTargetTemperature(int new_Target_Temp){

    // Update target_Temperature
    target_Temperature = new_Target_Temp;

    // Publish newly set target_Temperature
    String dataToSend = "{ \"Target_Temperature\" : " + String(target_Temperature) + " }";  // Create json string
    mqttPublish("/TemperatureSensor", dataToSend.c_str());    // Publish data

    // Print change succesfull message
    Serial.print("target_Temperature updated succesfully to: ");
    Serial.println(target_Temperature);
}

// Callback function to enable/disable automatic temperature management 
void ControlAutoTemperatureManagement(unsigned int user_Option){

    // Update auto_Temp_Management_Enabled status
    if(user_Option == 0){
        auto_Temp_Management_Enabled = 0;   // Disable automatic temperature management
    }
    else if(user_Option == 1){
        auto_Temp_Management_Enabled = 1;   // Enable automatic temperature management
    }

    // Publish Automatic_Temperature_Management_Status
    String dataToSend = "{ \"Automatic_Temperature_Management_Status\" : " + String(auto_Temp_Management_Enabled) + " }";  // Create json string
    mqttPublish("/TemperatureSensor", dataToSend.c_str());    // Publish data
    
    // Print change succesfull message
    Serial.print("auto_Temp_Management_Enabled updated succesfully to: ");
    Serial.println(auto_Temp_Management_Enabled);
}

// Controls windows automatically to maintain user set target temperature
void autoWindow(){

    // Only operate windows automatically if auto_Temp_Management_Enabled and window_Manual_Mode = 0
    if(auto_Temp_Management_Enabled & !window_Manual_Mode){

        // If target temperature has been acheived, close window
        if(current_Temp == target_Temperature){

            // If window is open, close it.
            if(window_Current_State){
                window_Servo.write(0);   // Close window
                window_Current_State = 0;   // Update window state

                // Publish window status
                String dataToSend = "{ \"Window_status\" : " + String(window_Current_State) + " }";  // Create json string
                mqttPublish("/Window", dataToSend.c_str());    // Publish data

                // Print change succesfull message
                Serial.print("window_Current_State updated succesfully to: ");
                Serial.println(window_Current_State);
            }
        }

        // If current temperature is lower than target temperature, close window
        if(current_Temp < target_Temperature){

            // If window is open, close it
            if(window_Current_State){
                window_Servo.write(0);   // Close window
                window_Current_State = 0;   // Update window state

                // Publish window status
                String dataToSend = "{ \"Window_status\" : " + String(window_Current_State) + " }";  // Create json string
                mqttPublish("/Window", dataToSend.c_str());    // Publish data

                // Print change succesfull message
                Serial.print("window_Current_State updated succesfully to: ");
                Serial.println(window_Current_State);
            }
        }

        // If current temperature is greater than target temperature, open window
        if(current_Temp > target_Temperature){

            // If window is closed, open it
            if(!window_Current_State){
                window_Servo.write(90);   // Open window
                window_Current_State = 1;   // Update window state

                // Publish window status
                String dataToSend = "{ \"Window_status\" : " + String(window_Current_State) + " }";  // Create json string
                mqttPublish("/Window", dataToSend.c_str());    // Publish data

                // Print change succesfull message
                Serial.print("window_Current_State updated succesfully to: ");
                Serial.println(window_Current_State);
            }
        }
    }
}

// Callback function to enable/disable automatic window control
void changeWindowOperationMode(unsigned int user_Option){

    // Update window_Manual_Mode
    if(user_Option == 0){
        window_Manual_Mode = 0;   // Enable automatic window control
    }
    else if(user_Option == 1){
        window_Manual_Mode = 1;   // Disable automatic window control
    }

    // Publish newly set window_Manual_Mode status
    String dataToSend = "{ \"Window_manual_mode\" : " + String(window_Manual_Mode) + " }";  // Create json string
    mqttPublish("/Window", dataToSend.c_str());    // Publish data
    
    // Print change succesfull message
    Serial.print("window_Manual_Mode updated succesfully to: ");
    Serial.println(window_Manual_Mode);
}

// Callback function to control windows manually
void manualWindowControl(unsigned int user_Option){

    // Only operate if windows are set to manual mode
    if(window_Manual_Mode){

        // Update window_Current_State
        if(user_Option == 0){
            window_Servo.write(0);   // Close window
            window_Current_State = 0;   // Update window state
        }
        else if(user_Option == 1){
            window_Servo.write(90);   // Open window
            window_Current_State = 1;   // Update window state
        }

        // Publish newly set window_Manual_Mode status
        String dataToSend = "{ \"Window_status\" : " + String(window_Current_State) + " }";  // Create json string
        mqttPublish("/Window", dataToSend.c_str());    // Publish data

        // Print change succesfull message
        Serial.print("window_Current_State updated succesfully to: ");
        Serial.println(window_Current_State);
    }
}

// Door check function
void checkDoors(){

    // Check the front door status and update only if the state has been changed
    if( digitalRead(frontDoor) ^ front_Door_State){

        // Update front_Door_State
        front_Door_State = digitalRead(frontDoor);

        // Publish updated front_Door_State
        String dataToSend = "{ \"Front_door_status\" : " + String(front_Door_State) + " }";  // Create json string
        mqttPublish("/HomeSecurity", dataToSend.c_str());    // Publish data

        // Print updated front_Door_State
        Serial.print("front_Door_State changed to: ");
        Serial.println(front_Door_State);
    }

    // Check the back door status and update only if the state has been changed
    if( digitalRead(backDoor) ^ back_Door_State){

        // Update back_Door_State
        back_Door_State = digitalRead(backDoor);

        // Publish updated back_Door_State
        String dataToSend = "{ \"Back_door_status\" : " + String(back_Door_State) + " }";  // Create json string
        mqttPublish("/HomeSecurity", dataToSend.c_str());    // Publish data

        // Print updated back_Door_State
        Serial.print("back_Door_State changed to: ");
        Serial.println(back_Door_State);
    }
}

// Intruder alarm function
void intruderAlarmFunction(){

    // Only operate if intruder alarm is enabled
    if(intruder_Alarm){

        // Check if doors are opened or if motion sensor detects movement
        if( front_Door_State | back_Door_State | (use_Motion_Sensor & motion_Detected) ){

            // If intruder_Event flag was not already set, set it and publish intruder event
            if(!intruder_Event){

                // Set intruder_Event flag
                intruder_Event = 1;

                // Publish intruder event
                String dataToSend = "{ \"Intruder_event\" : " + String(intruder_Event) + " }";  // Create json string
                mqttPublish("/HomeSecurity", dataToSend.c_str());    // Publish data

                // Record time intruder event was published
                last_Intruder_Message_Sent_Time = millis();

                // Print intruder event
                Serial.print("intruder_Event updated succesfully to: ");
                Serial.println(intruder_Event);
            }
        }

        // If intruder_Event is set and intruder_Acknowledged is not set, publish intruder event every 5 seconds
        if(intruder_Event & !intruder_Event_Acknowledged & ( (millis()-last_Intruder_Message_Sent_Time) > 5000 )){
            // Publish intruder event
            String dataToSend = "{ \"Intruder_event\" : " + String(intruder_Event) + " }";  // Create json string
            mqttPublish("/HomeSecurity", dataToSend.c_str());    // Publish data

            // Record time intruder event was published
            last_Intruder_Message_Sent_Time = millis();
        }
    }
}

// Callback function to enable/disable intruder alarm
void intruderAlarmCallback(unsigned int user_Option){

    // Update intruder_Alarm status
    if(user_Option == 0){
        intruder_Alarm = 0;   // Disable intruder_Alarm
        intruder_Event = 0; // Clear intruder_Event flag
        intruder_Event_Acknowledged = 0;    // Clear intruder_Event_Acknowledged flag
    }
    else if(user_Option == 1){
        intruder_Alarm = 1;   // Enable intruder_Alarm
    }

    // Publish newly set intruder_Alarm status
    String dataToSend = "{ \"Intruder_alarm\" : " + String(intruder_Alarm) + " }";  // Create json string
    mqttPublish("/HomeSecurity", dataToSend.c_str());    // Publish data

    // Print change succesfull message
    Serial.print("intruder_Alarm updated succesfully to: ");
    Serial.println(intruder_Alarm);
}

// Callback function to enable/disable use of motion sensor for intruder detection
void motionSensorForIntruderAlarmCallback(unsigned int user_Option){

    // Update use_Motion_Sensor status
    if(user_Option == 0){
        use_Motion_Sensor = 0;   // Disable motion sensor for intruder detection
    }
    else if(user_Option == 1){
        use_Motion_Sensor = 1;   // Enable motion sensor for intruder detection
    }

    // Publish newly set use_Motion_Sensor status
    String dataToSend = "{ \"Use_motion_sensor_for_intruder_alarm\" : " + String(use_Motion_Sensor) + " }";  // Create json string
    mqttPublish("/HomeSecurity", dataToSend.c_str());    // Publish data

    // Print change succesfull message
    Serial.print("use_Motion_Sensor updated succesfully to: ");
    Serial.println(use_Motion_Sensor);
}

// Callback function to set intruder event acknowledged flag
void acknowledgeIntruderEvent(unsigned int user_Option){

    // Set intruder_Event_Acknowledged flag if user_Option = 1
    if(user_Option == 1){
        intruder_Event_Acknowledged = 1;
    }

    // Print change succesfull message
    Serial.print("intruder_Event_Acknowledged updated succesfully to: ");
    Serial.println(intruder_Event_Acknowledged);
}

// Callback function to clear an intruder event and reset
void clearIntruderEvent(unsigned int user_Option){

    // Clear intruder_Event and intruder_Event_Acknowledged flags if user_Option = 1
    if(user_Option == 1){
        intruder_Event = 0;
        intruder_Event_Acknowledged = 0;
    }

    // Publish intruder event
    String dataToSend = "{ \"Intruder_event\" : " + String(intruder_Event) + " }";  // Create json string
    mqttPublish("/HomeSecurity", dataToSend.c_str());    // Publish data

    // Print intruder event
    Serial.print("intruder_Event cleared: ");
    Serial.println(intruder_Event);

    // Print change succesfull message
    Serial.print("intruder_Event_Acknowledged updated succesfully to: ");
    Serial.println(intruder_Event_Acknowledged);
}

// Initialise sensors
void initialiseSensorsAndDevices(){
    // Initialise motion detector:
    pinMode(pirPin, INPUT);     // Set motion detection sensor pin to input

    // Initialise auto lights:
    pinMode(autoLight, OUTPUT);    // Set auto light pin to output
    digitalWrite(autoLight, LOW);  // Initially set auto light to off

    // Initialise DHT11 sensor
    dht.begin();

    // Initialise servo
    window_Servo.attach(servo_Pin);  // attaches the servo pin to the servo object
    window_Servo.write(0);   // Set window to closed position initially

    // Initialise end stop switches for front and back doors
    pinMode(frontDoor, INPUT);     // Set front door switch to input
    pinMode(backDoor, INPUT);     // Set back door switch to input

}

// Publish all initial values to server
void initialDataPublish(){
    Serial.println("Initial default data value push");

    // Publish status of motion sensor
    String dataToSend = "{ \"Room\" : \"Bedroom\", \"Motion_detected\" : " + String(motion_Detected) + " }";  // Create json string
    mqttPublish("/MotionDetected", dataToSend.c_str());    // Publish data

    // Publish newly set motion_Detect_Timer
    dataToSend = "{ \"No_Motion_Detected_Delay\" : " + String(motion_Detect_Timer) + " }";  // Create json string
    mqttPublish("/MotionSensor", dataToSend.c_str());    // Publish data

    // Publish auto light operation mode to MQTT server
    dataToSend = "{ \"Room\" : \"Bedroom\", \"Auto_Light_Override\" : " + String(auto_Light_Override_Flag) + " }";  // Create json string
    mqttPublish("/AutoLight", dataToSend.c_str());    // Publish data

    // Publish status of auto light to MQTT server
    dataToSend = "{ \"Room\" : \"Bedroom\", \"Light_status\" : " + String(current_light_status) + " }";  // Create json string
    mqttPublish("/AutoLights", dataToSend.c_str());    // Publish data

    // Publish ambientLightThreshold as percentage
    dataToSend = "{ \"Ambient_Light_Threshold\" : " + String((ambientLightThreshold/4096)*100) + " }";  // Create json string
    mqttPublish("/AutoLight", dataToSend.c_str());    // Publish data

    // Publish delayed_Light_OFF_Timer
    dataToSend = "{ \"Light_Off_Delay\" : " + String(delayed_Light_OFF_Timer) + " }";  // Create json string
    mqttPublish("/AutoLight", dataToSend.c_str());    // Publish data

    // Publish temperature to MQTT server
    dataToSend = "{ \"Temperature\" : " + String(current_Temp) + " }";  // Create json string
    mqttPublish("/TemperatureSensor", dataToSend.c_str());    // Publish data

    // Publish target_Temperature
    dataToSend = "{ \"Target_Temperature\" : " + String(target_Temperature) + " }";  // Create json string
    mqttPublish("/TemperatureSensor", dataToSend.c_str());    // Publish data

    // Publish Automatic_Temperature_Management_Status
    dataToSend = "{ \"Automatic_Temperature_Management_Status\" : " + String(auto_Temp_Management_Enabled) + " }";  // Create json string
    mqttPublish("/TemperatureSensor", dataToSend.c_str());    // Publish data

    // Publish window status
    dataToSend = "{ \"Window_status\" : " + String(window_Current_State) + " }";  // Create json string
    mqttPublish("/Window", dataToSend.c_str());    // Publish data

    // Publish newly set window_Manual_Mode status
    dataToSend = "{ \"Window_manual_mode\" : " + String(window_Manual_Mode) + " }";  // Create json string
    mqttPublish("/Window", dataToSend.c_str());    // Publish data

    // Publish updated front_Door_State
    dataToSend = "{ \"Front_door_status\" : " + String(front_Door_State) + " }";  // Create json string
    mqttPublish("/HomeSecurity", dataToSend.c_str());    // Publish data

    // Publish updated back_Door_State
    dataToSend = "{ \"Back_door_status\" : " + String(back_Door_State) + " }";  // Create json string
    mqttPublish("/HomeSecurity", dataToSend.c_str());    // Publish data

    // Publish intruder_Alarm status
    dataToSend = "{ \"Intruder_alarm\" : " + String(intruder_Alarm) + " }";  // Create json string
    mqttPublish("/HomeSecurity", dataToSend.c_str());    // Publish data

    // Publish newly set use_Motion_Sensor status
    dataToSend = "{ \"Use_motion_sensor_for_intruder_alarm\" : " + String(use_Motion_Sensor) + " }";  // Create json string
    mqttPublish("/HomeSecurity", dataToSend.c_str());    // Publish data

    // Publish intruder event flag value
    dataToSend = "{ \"Intruder_event\" : " + String(intruder_Event) + " }";  // Create json string
    mqttPublish("/HomeSecurity", dataToSend.c_str());    // Publish data

    Serial.println("Finished initial data push"); Serial.println(); Serial.println();
}

// Blink onboardLED given number of times
void blinkLED(int times) {
    for (int i = 0; i < times; i++) {
        digitalWrite(onboardLED, HIGH);
        delay(200);
        digitalWrite (onboardLED, LOW);
        delay(200);
    }
}

// The receivedCallback() function will be invoked when this client receives data about the subscribed topic:
void receivedCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message received on topic:  ");
    Serial.print(topic);
    Serial.print("  Message reads:  ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
    blinkLED(2);

    // Convert topic to string so strings can be compared
    std::string topic_string = topic;

    // convert payload to string
    String payload_String = String((char *)payload);    // Convert to string
    //Serial.print("payload_string is: "); Serial.println(payload_String); Serial.println();

    // Convert string to unsigned int
    unsigned int data = (unsigned int) payload_String.toInt();  // Convert string to unsigned int
    Serial.print("data is: "); Serial.println(data); Serial.println();

    // Do something depending on received topic and message
    if( topic_string == motionDetectTopic ){
        motionDetectorCallback(data);
    }
    else if( topic_string == autoLightTopic ){
        manualLightControl(data);
    }
    else if( topic_string == lightDelayOffTopic){
        changeLightOffDelay(data);
    }
    else if ( topic_string == ambientLightThresholdTopic){
        changeAmbientLightThreshold(data);
    }
    else if( topic_string == targetTemperatureTopic){
        changeTargetTemperature(payload_String.toInt());
    }
    else if( topic_string == autoTempManagementControl){
        ControlAutoTemperatureManagement(data);
    }
    else if( topic_string == windowOperationModeTopic){
        changeWindowOperationMode(data);
    }
    else if( topic_string == manualWindowControlTopic){
        manualWindowControl(data);
    }
    else if( topic_string == intruderAlarmTopic){
        intruderAlarmCallback(data);
    }
    else if( topic_string == motionSensorForIntruderAlarmTopic){
        motionSensorForIntruderAlarmCallback(data);
    }
    else if( topic_string == acknowledgeIntruderEventTopic){
        acknowledgeIntruderEvent(data);
    }
    else if( topic_string == clearIntruderEventTopic){
        clearIntruderEvent(data);
    }
}

// The mqttConnect() function will attempt to connect to MQTT and subscribe to a topic feed:
void mqttConnect() {
    while (!mqttClient.connected()) {

        Serial.print("In mqttConnect(), connecting...  ");

        if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME.c_str(), MQTT_PASSWORD)) {
            Serial.println("...connected to mqtt server!");
            blinkLED(3);
            Serial.print("Subscribing to topic:  ");
            Serial.println(MQTT_SUBSCRIBE_TOPIC.c_str());

            // Subscribe topic with default QoS 0
            // Let's just subscribe to the same feed we are publishing to, to see if our message gets recorded.
            mqttClient.subscribe(MQTT_SUBSCRIBE_TOPIC.c_str());

        } else {
            Serial.println("...mqttConnect() failed, status code =");
            Serial.println(mqttClient.state());
            Serial.println("try again in 5 seconds...");
            delay(5000); // Wait 5 seconds before retrying
        }
    }
}

void setup() {
    // Initialise sensors
    initialiseSensorsAndDevices();
        
    // Main setup:
    pinMode(onboardLED, OUTPUT);
    Serial.begin(115200);
    Serial.println();
    Serial.println();
    Serial.println("Hello MQTT program");
    Serial.print("Attempting to connect to WiFi SSID:  ");
    Serial.println(WIFI_SSID);
    blinkLED(1);
    Serial.print("Connecting");
    
    // We start by connecting to a WiFi network
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi connected");
    blinkLED(2);
    Serial.print("IP address:  ");
    Serial.println(WiFi.localIP());

    Serial.println("Setting up MQTT...");
    
    // We need a certificate in order to do a __secure__ TLS/SSL connection to our server
    espClient.setCACert(CA_CERT);

    // Port 1883 is reserved with IANA for use with MQTT.
    // TCP/IP port 8883 is also registered, for using MQTT over SSL.
    // help url:  http://www.iotsharing.com/2017/08/how-to-use-esp32-mqtts-with-mqtts-mosquitto-broker-tls-ssl.html
    mqttClient.setServer(MQTT_SERVER, 8883); // Port 8883 for MQTT over SSL.

    // The receivedCallback() function will be invoked when this client receives the subscribed topic:
    mqttClient.setCallback(receivedCallback);

    mqttConnect();  // Check connection to mqtt broker. If no connection, try to connect.

    // Publish initial values to server
    initialDataPublish();
}

void loop() {
    mqttConnect();  // Check connection to mqtt broker. If no connection, try to connect.

    // this function will listen for incoming subscribed topic processes and invoke receivedCallback()
    mqttClient.loop();

    // we send a reading every 5 secs
    // we count until 5 secs reached to avoid blocking program (instead of using delay())
    long now = millis();
    if (now - lastMsgTimer > 5000) {
        lastMsgTimer = now;

        // just convert time stamp to a c-string and send as data:
        String dataToSend = String(millis()); // dataToSend could be a sensor reading instead
        Serial.println();
        //Serial.print("Publishing data:  ");
        //Serial.println(dataToSend);
        blinkLED(1);
        //mqttPublish("/TEST", dataToSend.c_str());    // Testing my mqtt publish function
    }

    // Monitor sensors:
    motionDetectionFunction();
    autoLightFunction();
    temperatureSensorFunction();
    autoWindow();
    checkDoors();
    intruderAlarmFunction();
}
