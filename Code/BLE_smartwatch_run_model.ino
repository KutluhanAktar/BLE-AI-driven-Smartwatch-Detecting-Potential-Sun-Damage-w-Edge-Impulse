         /////////////////////////////////////////////  
        //   BLE AI-driven Smartwatch Detecting    //
       //   Potential Sun Damage w/ Edge Impulse  //
      //             ---------------             //
     //               (XIAO BLE)                //           
    //             by Kutluhan Aktar           // 
   //                                         //
  /////////////////////////////////////////////

//
// Log UV & weather data on an SD card to train an Edge Impulse model. Then, run it to get informed of sun damage over BLE via an Android app.
//
// For more information:
// https://www.theamplituhedron.com/projects/BLE_AI_driven_Smartwatch_Detecting_Potential_Sun_Damage_w_Edge_Impulse
//
//
// Connections
// XIAO BLE :  
//                                Grove - UV Sensor
// A0  --------------------------- SIG
//                                BMP180 Barometric Pressure/Temperature/Altitude Sensor
// A4  --------------------------- SDA
// A5  --------------------------- SCL
//                                SSD1306 OLED Display (128x64)
// A4  --------------------------- SDA
// A5  --------------------------- SCL
//                                MicroSD Card Module (Built-in on the XIAO Expansion board)
// D10 --------------------------- MOSI
// D9  --------------------------- MISO
// D8  --------------------------- CLK (SCK)
// D2  --------------------------- CS  
//                                Button (Built-in on the XIAO Expansion board)
// D1  --------------------------- +
//                                Keyes 10mm RGB LED Module (140C05)
// D7  --------------------------- R
// D3  --------------------------- G
// D6  --------------------------- B  


// Include the required libraries:
#include <ArduinoBLE.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Include the Edge Impulse model converted to an Arduino library:
#include <BLE_Smartwatch_Detecting_Potential_Sun_Damage_inferencing.h>

// Define the required parameters to run an inference with the Edge Impulse model.
#define FREQUENCY_HZ        EI_CLASSIFIER_FREQUENCY
#define INTERVAL_MS         (1000 / (FREQUENCY_HZ + 1))

// Define the features array to classify one frame of data.
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
size_t feature_ix = 0;

// Define the threshold value for the model outputs (predictions).
float threshold = 0.60;

// Define the sun damage risk level (class) names and color codes:
String classes[] = {"Perilous", "Risky", "Tolerable"};
int color_codes[3][3] = {{255,0,0}, {255,255,0}, {0,255,0}};

// Create the BLE service:
BLEService BLE_smartwatch("19B10000-E8F2-537E-4F6C-D104768A1214");

// Create data characteristics and allow the remote device (central) to read and notify:
BLEFloatCharacteristic temperatureCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic altitudeCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic UVCharacteristic("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic pressureCharacteristic("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic classCharacteristic("19B10005-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

// Define the BMP180 Barometric Pressure/Temperature/Altitude Sensor.
Adafruit_BMP085 bmp;

// Define the Grove – UV Sensor pin.
#define UV_pin A0

// Define the 0.96 OLED display (SSD1306) on the XIAO Expansion board. 
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Define monochrome graphics:
static const unsigned char PROGMEM _error [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x01, 0x80, 0x01, 0x80,
0x06, 0x00, 0x00, 0x60, 0x0C, 0x00, 0x00, 0x30, 0x08, 0x01, 0x80, 0x10, 0x10, 0x03, 0xC0, 0x08,
0x30, 0x02, 0x40, 0x0C, 0x20, 0x02, 0x40, 0x04, 0x60, 0x02, 0x40, 0x06, 0x40, 0x02, 0x40, 0x02,
0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02,
0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x03, 0xC0, 0x02, 0x40, 0x01, 0x80, 0x02,
0x40, 0x00, 0x00, 0x02, 0x60, 0x00, 0x00, 0x06, 0x20, 0x01, 0x80, 0x04, 0x30, 0x03, 0xC0, 0x0C,
0x10, 0x03, 0xC0, 0x08, 0x08, 0x01, 0x80, 0x10, 0x0C, 0x00, 0x00, 0x30, 0x06, 0x00, 0x00, 0x60,
0x01, 0x80, 0x01, 0x80, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x3F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const unsigned char PROGMEM tolerable [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00,
0x00, 0x01, 0x80, 0x00, 0x04, 0x03, 0xC0, 0x20, 0x03, 0x00, 0x00, 0xC0, 0x03, 0xC7, 0xE3, 0xC0,
0x01, 0x9F, 0xF9, 0x80, 0x01, 0x3F, 0xFC, 0x80, 0x00, 0x7F, 0xFE, 0x00, 0x00, 0xFF, 0xFF, 0x00,
0x00, 0xFF, 0xFF, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x05, 0xFF, 0xFF, 0xA0, 0x3D, 0xFF, 0xFF, 0xBC,
0x7D, 0xFF, 0xFF, 0xBE, 0x0D, 0xFF, 0xFF, 0xB0, 0x01, 0xFF, 0xFF, 0x80, 0x00, 0xFF, 0xFF, 0x00,
0x00, 0xFF, 0xFF, 0x00, 0x00, 0x7F, 0xFE, 0x00, 0x01, 0x3F, 0xFC, 0x80, 0x01, 0x9F, 0xF9, 0x80,
0x03, 0xC7, 0xE3, 0xC0, 0x03, 0x80, 0x01, 0xC0, 0x06, 0x03, 0xC0, 0x60, 0x00, 0x03, 0xC0, 0x00,
0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const unsigned char PROGMEM risky [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x03, 0x80, 0x00, 0x02, 0x03, 0xC0, 0x00,
0x03, 0xC7, 0xE3, 0x80, 0x01, 0xE0, 0x07, 0x80, 0x01, 0xCF, 0xF3, 0x80, 0x01, 0x3F, 0xFC, 0x80,
0x00, 0x7F, 0xFE, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x1D, 0xFF, 0xFF, 0xB8, 0x7B, 0xFF, 0xFF, 0xDE,
0x3B, 0xFF, 0xFF, 0xDC, 0x1B, 0xFF, 0xFF, 0xD8, 0x0F, 0xFF, 0xFF, 0xF0, 0x07, 0xFF, 0xFF, 0xE0,
0x07, 0xFF, 0xFF, 0xE0, 0x1F, 0xFF, 0xFF, 0xF0, 0x1B, 0xFF, 0xFF, 0xD8, 0x3B, 0xFF, 0xFF, 0xDC,
0x7B, 0xFF, 0xFF, 0xDE, 0x0D, 0xFF, 0xFF, 0xB0, 0x00, 0xFF, 0xFF, 0x00, 0x01, 0x7F, 0xFE, 0x80,
0x01, 0x3F, 0xFD, 0x80, 0x01, 0xCF, 0xF3, 0x80, 0x01, 0xE0, 0x07, 0x80, 0x03, 0x87, 0xE1, 0x80,
0x00, 0x03, 0xC0, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const unsigned char PROGMEM perilous [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x11, 0x88, 0x00, 0x00, 0x19, 0x98, 0x00,
0x01, 0x19, 0x98, 0x80, 0x00, 0x9D, 0xBB, 0x00, 0x00, 0xE3, 0x87, 0x00, 0x08, 0xDF, 0xF2, 0x30,
0x07, 0x3F, 0xFC, 0xE0, 0x03, 0x7F, 0xFE, 0xC0, 0x02, 0xFF, 0xFF, 0x00, 0x7D, 0xFF, 0xFF, 0x3C,
0x1D, 0xFF, 0xFF, 0xB8, 0x05, 0xFF, 0xFF, 0xA0, 0x03, 0xFF, 0xFF, 0x80, 0x7F, 0xFF, 0xFF, 0xBE,
0x1F, 0xFF, 0xFF, 0xB8, 0x03, 0xFF, 0xFF, 0x80, 0x0D, 0xFF, 0xFF, 0xB0, 0x1D, 0xFF, 0xFF, 0xB8,
0x60, 0xFF, 0xFF, 0x04, 0x02, 0xFF, 0xFE, 0x40, 0x07, 0x7F, 0xFE, 0xE0, 0x0E, 0x3F, 0xF8, 0x70,
0x18, 0xEF, 0xE7, 0x10, 0x00, 0xD8, 0x13, 0x00, 0x01, 0x9D, 0xB9, 0x80, 0x01, 0x19, 0x98, 0x80,
0x00, 0x11, 0x98, 0x00, 0x00, 0x11, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Create an array including icons for labels (classes).
static const unsigned char PROGMEM *class_icons[] = {tolerable, risky, perilous};

// Define the RGB LED pins:
#define redPin     7
#define greenPin   3
#define bluePin    6

// Define the data holders:
float _temperature, _altitude, _real_altitude;
int UV_index, _pressure, _sea_level_pressure;
long timer;
int predicted_class = -1;
 
void setup(){
  Serial.begin(9600);
  // while(!Serial); // Uncomment for debugging.

  // Initialize the SSD1306 screen:
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(1000);

  display.clearDisplay();   
  display.setTextSize(2); 
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("BLE");
  display.println("Smartwatch");
  display.setTextSize(1);
  display.println("\n\nw/ Android");
  display.println("& Edge Impulse");
  display.display();

  // Check the BLE initialization status:
  while(!BLE.begin()){
    Serial.println("BLE initialization is failed!");
    err_msg();
  }
  Serial.println("\nBLE initialization is successful!\n");
  // Print this peripheral device's address information:
  Serial.print("MAC Address: "); Serial.println(BLE.address());
  Serial.print("Service UUID Address: "); Serial.println(BLE_smartwatch.uuid()); Serial.println();

  // Set the local name this peripheral advertises: 
  BLE.setLocalName("BLE UV Smartwatch");
  // Set the UUID for the service this peripheral advertises:
  BLE.setAdvertisedService(BLE_smartwatch);

  // Add the given data characteristics to the service:
  BLE_smartwatch.addCharacteristic(temperatureCharacteristic);
  BLE_smartwatch.addCharacteristic(altitudeCharacteristic);
  BLE_smartwatch.addCharacteristic(UVCharacteristic);
  BLE_smartwatch.addCharacteristic(pressureCharacteristic);
  BLE_smartwatch.addCharacteristic(classCharacteristic);

  // Add the service to the device:
  BLE.addService(BLE_smartwatch);

  // Assign event handlers for connected, disconnected devices to this peripheral:
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // Start advertising:
  BLE.advertise();
  Serial.println(("Bluetooth device active, waiting for connections..."));

  // Check the BMP180 Barometric Pressure/Temperature/Altitude Sensor connection status: 
  while(!bmp.begin()){
    Serial.println("BMP180 Barometric Pressure/Temperature/Altitude Sensor is not found!");
    err_msg();
    delay(1000);
  }
  Serial.println("\nBMP180 Barometric Pressure/Temperature/Altitude Sensor is connected successfully!\n");

  adjustColor(0,0,255);
  delay(5000);  
}
 
void loop(){
  get_UV_radiation();
  delay(100);
  collect_BMP180_data();
  delay(500);

  // Run inference:
  run_inference_to_make_predictions(10);

  // Every 30 seconds, advertise (transmit) the collected data and the predicted label (class) to the Android application over BLE.
  if(millis() - timer >= 30*1000){
    // If the Edge Impulse model predicted a label (class) successfully:
    if(predicted_class != -1){
      update_characteristics();
      // After updating characteristics, notify the user and print the predicted label (class) on the screen.
      display.clearDisplay();
      display.drawBitmap(48, 0, class_icons[predicted_class], 32, 32, SSD1306_WHITE);
      display.setTextSize(1); 
      display.setTextColor(SSD1306_WHITE);
      // Print:
      display.setCursor(0,40);
      display.println("BLE: Data Transmitted");
      String c = "Class: " + classes[predicted_class];
      int str_x = c.length() * 6;
      display.setCursor((SCREEN_WIDTH - str_x) / 2, 56);
      display.println(c);
      display.display();
      adjustColor(color_codes[predicted_class][0], color_codes[predicted_class][1], color_codes[predicted_class][2]);
      delay(5000);
      // Clear the predicted label (class).
      predicted_class = -1;
    }
    // Update the timer:
    timer = millis();
  }
  
  // Show the collected data on the screen.
  home_screen();

  // Poll for BLE events:
  BLE.poll();
}

void run_inference_to_make_predictions(int multiply){
  // Scale (normalize) data items depending on the given model:
  float scaled_UV_index = UV_index / 10;
  float scaled_temperature = _temperature / 100;
  float scaled_pressure = _pressure / 100000;
  float scaled_altitude = _altitude / 100;

  // Copy the scaled data items to the features buffer.
  // If required, multiply the scaled data items while copying them to the features buffer.
  for(int i=0; i<multiply; i++){  
    features[feature_ix++] = scaled_UV_index;
    features[feature_ix++] = scaled_temperature;
    features[feature_ix++] = scaled_pressure;
    features[feature_ix++] = scaled_altitude;
  }

  // Display the progress of copying data to the features buffer.
  Serial.print("Features Buffer Progress: "); Serial.print(feature_ix); Serial.print(" / "); Serial.println(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  
  // Run inference:
  if(feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE){    
    ei_impulse_result_t result;
    // Create a signal object from the features buffer (frame).
    signal_t signal;
    numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    // Run the classifier:
    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
    ei_printf("\nrun_classifier returned: %d\n", res);
    if(res != 0) return;

    // Print the inference timings on the serial monitor.
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n", 
        result.timing.dsp, result.timing.classification, result.timing.anomaly);

    // Obtain the prediction results for each label (class).
    for(size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++){
      // Print the prediction results on the serial monitor.
      ei_printf("%s:\t%.5f\n", result.classification[ix].label, result.classification[ix].value);
      // Get the predicted label (class).
      if(result.classification[ix].value >= threshold) predicted_class = ix;
    }
    Serial.print("\nPredicted Class: "); Serial.println(predicted_class);

    // Detect anomalies, if any:
    #if EI_CLASSIFIER_HAS_ANOMALY == 1
      ei_printf("Anomaly : \t%.3f\n", result.anomaly);
    #endif

    // Clear the features buffer (frame):
    feature_ix = 0;
  }
}

void update_characteristics(){
  // Update all data characteristics (floats):
  temperatureCharacteristic.writeValue(_temperature);
  altitudeCharacteristic.writeValue(_altitude);
  UVCharacteristic.writeValue(float(UV_index));
  pressureCharacteristic.writeValue(float(_pressure));
  classCharacteristic.writeValue(float(predicted_class));
  Serial.println("\n\nBLE: Data Characteristics Updated Successfully!\n");
}

void home_screen(){
  adjustColor(255,0,255);
  display.clearDisplay();   
  display.setTextSize(1); 
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,8);
  display.println("Estimations:");
  display.println("UV Index => " + String(UV_index));
  display.println("Temp. => " + String(_temperature) + " *C");
  display.println("Pressure => " + String(_pressure) + " Pa");
  display.println("Altitude => " + String(_altitude) + " m");
  display.display();  
}

void get_UV_radiation(){
  int sensorValue;
  long sum = 0;
  // Get the summation of the latest UV sensor measurements.
  for(int i=0;i<1024;i++){
    sensorValue = analogRead(UV_pin);
    sum+=sensorValue;
    delay(2);
  }
  // Obtain the average sensor measurement to remove the glitch.
  long avr_val = sum/1024;
  // Estimate the UV index value with this formula roughly.
  UV_index = (avr_val*1000/4.3-83)/21;
  UV_index = UV_index / 1000;
  Serial.print("\nEstimated UV index value: "); Serial.println(UV_index);
  delay(20);
}

void collect_BMP180_data(){
  _temperature = bmp.readTemperature();
  _pressure = bmp.readPressure();
  // Calculate altitude assuming 'standard' barometric pressure of 1013.25 millibars (101325 Pascals).
  _altitude = bmp.readAltitude();
  _sea_level_pressure = bmp.readSealevelPressure();
  // To get a more precise altitude measurement, use the current sea level pressure, which will vary with the weather conditions. 
  _real_altitude = bmp.readAltitude(101500);
  // Print the data generated by the BMP180 Barometric Pressure/Temperature/Altitude Sensor.
  Serial.print("Temperature => "); Serial.print(_temperature); Serial.println(" *C");
  Serial.print("Pressure => "); Serial.print(_pressure); Serial.println(" Pa");
  Serial.print("Altitude => "); Serial.print(_altitude); Serial.println(" meters");
  Serial.print("Pressure at sea level (calculated) => "); Serial.print(_sea_level_pressure); Serial.println(" Pa");
  Serial.print("Real Altitude => "); Serial.print(_real_altitude); Serial.println(" meters\n");
}

void err_msg(){
  // Show the error message on the SSD1306 screen.
  adjustColor(255, 0, 0);
  display.clearDisplay();   
  display.drawBitmap(48, 0, _error, 32, 32, SSD1306_WHITE);
  display.setTextSize(1); 
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,40); 
  display.println("Check the serial monitor to see the error!");
  display.display();  
}

void blePeripheralConnectHandler(BLEDevice central) {
  // Central connected event handler:
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // Central disconnected event handler:
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void adjustColor(int r, int g, int b){
  analogWrite(redPin, (255-r));
  analogWrite(greenPin, (255-g));
  analogWrite(bluePin, (255-b));
}

void ei_printf(const char *format, ...){
  static char print_buf[1024] = { 0 };
  va_list args;
  va_start(args, format);
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);
  if(r > 0){ Serial.write(print_buf); }
}
