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
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Define the BMP180 Barometric Pressure/Temperature/Altitude Sensor.
Adafruit_BMP085 bmp;

// Define the Grove – UV Sensor pin.
#define UV_pin A0

// Initialize the File class and define the chip select pin:
File myFile;
const int chip_select = 2;
// Define the CSV file name: 
const char* data_file = "UV_DATA.csv";

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
static const unsigned char PROGMEM sd [] = {
0x0F, 0xFF, 0xFF, 0xFE, 0x1F, 0xFF, 0xFF, 0xFF, 0x1F, 0xFE, 0x7C, 0xFF, 0x1B, 0x36, 0x6C, 0x9B,
0x19, 0x26, 0x4C, 0x93, 0x19, 0x26, 0x4C, 0x93, 0x19, 0x26, 0x4C, 0x93, 0x19, 0x26, 0x4C, 0x93,
0x19, 0x26, 0x4C, 0x93, 0x19, 0x26, 0x4C, 0x93, 0x19, 0x26, 0x4C, 0x93, 0x1F, 0xFF, 0xFF, 0xFF,
0x1F, 0xFF, 0xFF, 0xFF, 0x1F, 0xFF, 0xFF, 0xFF, 0x1F, 0xFF, 0xFF, 0xFF, 0x1F, 0xFF, 0xFF, 0xFF,
0x3F, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xC7, 0xFF, 0xFF, 0xF9, 0x41, 0xFF, 0x1F, 0xF9, 0xDD, 0xFF,
0x1F, 0xFC, 0xDD, 0xFF, 0x1F, 0xFE, 0x5D, 0xFF, 0x1F, 0xF8, 0x43, 0xFF, 0x1F, 0xFD, 0xFF, 0xFF,
0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE
};

// Define the integrated button pin on the XIAO Expansion board.
#define button 1
// Define the button state and the duration to utilize the integrated button in two different modes: long press and short press.
int button_state = 0;
#define DURATION 2000

// Define the RGB LED pins:
#define redPin     7
#define greenPin   3
#define bluePin    6

// Define the data holders:
int class_number = 0;
float _temperature, _altitude, _real_altitude;
int UV_index, _pressure, _sea_level_pressure;
long timer;
 
void setup(){
  Serial.begin(9600);
  // while(!Serial); // Uncomment for debugging.

  pinMode(button, INPUT_PULLUP);
  
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

  // Check the BMP180 Barometric Pressure/Temperature/Altitude Sensor connection status: 
  while(!bmp.begin()){
    Serial.println("BMP180 Barometric Pressure/Temperature/Altitude Sensor is not found!");
    err_msg();
    delay(1000);
  }
  Serial.println("\nBMP180 Barometric Pressure/Temperature/Altitude Sensor is connected successfully!\n");

  // Check the connection status between XIAO BLE and the SD card.
  if (!SD.begin(chip_select)){
    Serial.println("SD card initialization failed!\n");
    err_msg();
    while (1);
  }
  Serial.println("SD card is detected successfully!\n");
  adjustColor(0,0,255);
  delay(5000);  
}
 
void loop(){
  get_UV_radiation();
  delay(100);
  collect_BMP180_data();
  delay(500);

  // Show the collected data on the screen.
  home_screen();

  // Detect the long press and short press button modes:
  button_state = 0;
  if(!digitalRead(button)){
    adjustColor(255,255,255);
    timer = millis();
    button_state = 1;
    while((millis()-timer) <= DURATION){
      if(digitalRead(button)){
        button_state = 2;
        break;
      }
    }
  }
  
  if(button_state == 1){
    // Save the given data record to the given CSV file on the SD card when long-pressed.
    save_data_to_SD_Card(class_number);
  }else if(button_state == 2){
    // Change the class number when short-pressed.
    class_number++;
    if(class_number > 2) class_number = 0;
    Serial.println("Selected Class: " + String(class_number) + "\n");
  }

}
void save_data_to_SD_Card(int risk_level){
  // Open the given CSV file on the SD card in the WRITE file mode.
  // FILE MODES: WRITE, READ
  myFile = SD.open(data_file, FILE_WRITE);
  adjustColor(255,255,0);
  delay(1000);
  // If the given file is opened successfully:
  if(myFile){
    Serial.print("Writing to "); Serial.print(data_file); Serial.println("...");
    // Create the data record to be inserted as a new row: 
    String data_record = String(UV_index) + "," + String(_temperature)  + "," + String(_pressure)  + "," + String(_altitude) + "," + String(risk_level);
    // Append the data record:
    myFile.println(data_record);
    // Close the CSV file:
    myFile.close();
    Serial.println("Data saved successfully!\n");
    // Notify the user after appending the given data record successfully.
    adjustColor(0,255,0);
    display.clearDisplay(); 
    display.drawBitmap(48, 0, sd, 32, 44, SSD1306_WHITE);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);  
    display.setCursor(0,48); 
    display.println("Data saved to the SD card!");
    display.display();  
  }else{
    // If XIAO BLE cannot open the given CSV file successfully:
    Serial.println("XIAO BLE cannot open the given CSV file successfully!\n");
    err_msg();
  }
  // Exit and clear:
  delay(4000);
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
  display.println();
  display.println("Selected Class => " + String(class_number));
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
  Serial.print("Estimated UV index value: "); Serial.println(UV_index); Serial.println();
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

void adjustColor(int r, int g, int b){
  analogWrite(redPin, (255-r));
  analogWrite(greenPin, (255-g));
  analogWrite(bluePin, (255-b));
}
