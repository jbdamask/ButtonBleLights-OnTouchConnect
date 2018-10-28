/*********************************************************************
/*********************************************************
Author: John B Damask & Adafruit folks (via their demos)
Created: October 28, 2018
Purpose: Using button or Bluefruit to control NeoPixels. 
Note: Using RGB neopixels, a button and a Feather Bluefruit 32u4.
      The Feather can both read and write from BLE. The idea is that colors can come in from
      a central device (e.g. a Pi) or you can touch the button and send a color code (state)
      to the device. Note that I've adopted the payload format from Adafruit for sending Colors 
      (as reconstructed from packetParser.cpp). Basically it's !C<state><checksum>
      See https://github.com/jbdamask/ButtonBleLights-OnTouchConnect
Todo: Create classes for ble, touch and pixel functions.

*********************************************************************/
#include <Wire.h>
#include "FastLED.h"

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include <Adafruit_NeoPixel.h>

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
    PIN                       Which pin on the Arduino is connected to the NeoPixels?
    NUMPIXELS                 How many NeoPixels are attached to the Arduino?
    FRAMES_PER_SECOND         FastLed animation
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE     1
    #define PIN                     6
    #define NUMPIXELS               140    // Using NeoPixel ring. YMMV
    #define BRIGHTNESS              60
    #define MIN                     1
    #define MAX                     255
    #define NUMTOUCH                12
    /* DEVICE_NAME determines which PiHub it connects to.
     * If using my PiHubBleAwsIoT project, https://github.com/jbdamask/PiHubBleAwsIoT/blob/master/PiHub.cfg
     * set the name in PiHub.cfg
     */
    #define DEVICE_NAME             "AT+GAPDEVNAME=TouchLightsBle" 
    #define LED_TYPE     NEOPIXEL
    #define FRAMES_PER_SECOND 120
/*=========================================================================*/


/* ==========================================================================
 *  NEOPIXEL colors
 *  
 *  Set to true if using GRBW neopixels 
 *  (this code ignores white...but it will need to be passed if using GRBW)
 ---------------------------------------*/
bool neoPixelsWhite = false;
/*==========================================================================*/
int buttonPin = 10;
// Keeps track of the last pins touched
// so we know when buttons are 'released'
int buttonState;
int lastButtonState = LOW;  
/* We'll cycle through event states on button pushes */
uint8_t lastState = 0;
uint8_t currentState = 1;
uint8_t minState = 0;
uint8_t maxState = 11;
// Button debouncing
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 10;    // the debounce time; increase if the output flickers 
uint8_t output = 0;
uint8_t len = 0;
bool printOnceBle = false; // BLE initialization 
Adafruit_NeoPixel pixel;  // NeoPixel object
CRGB leds[NUMPIXELS];      // FastLed object
uint8_t gHue = 0; // rotating "base color" for FastLed

uint8_t state = 0;
bool isAnimationState = false;   // If true, each loop with call the same function. Useful for animations

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];
// the defined length of a color payload
//int colorLength = 6; // From days of sending RGB colors
int colorLength = 4;
uint16_t colLen = 3;
// Payload stuff
uint8_t xsum = 0;
uint8_t PAYLOAD_START = "!";
uint8_t COLOR_CODE = "C";
uint8_t BUTTON_CODE = "B";
uint8_t red;
uint8_t green;
uint8_t blue;

// the ble payload, set to max buffer size
uint8_t payload[21];

void setColors(uint8_t r, uint8_t g, uint8_t b){
  red = r; green = g; blue = b;
}

void wipe(){
  if(neoPixelsWhite){
    // My GRBW have twice the density of pixels and my GRB so they wipe faster
    colorWipe(pixel.Color(red, green, blue, 0),5);
  } else {
    colorWipe(pixel.Color(red, green, blue),10);
  }
}


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup()
{
  //while (!Serial);  // Uncomment when connected to computer. Comment out otherwise
  delay(500);
  Serial.begin(115200);
  Serial.println("Setting up");
  //pinMode(buttonPin, INPUT); // Set button
  pinMode(buttonPin, INPUT_PULLUP); // Set button and use internal pullup resistor (so you don't need to add a physical one)
  
  if (neoPixelsWhite) {
    pixel = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRBW + NEO_KHZ800);  
  }else{
    pixel = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800); 
  }

  // turn off neopixel
  pixel.begin(); // This initializes the NeoPixel library.
  for(uint8_t i=0; i<NUMPIXELS; i++) {
    pixel.setPixelColor(i, pixel.Color(0,0,0)); // off
  }
  pixel.show();

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  // Customize name
  ble.println(DEVICE_NAME);

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!
  Serial.println("All set, let's go!");
  /* Wait for connection */
  // We don't want to wait for the ble connection because we want touch to work locally, regardless
  /*while (! ble.isConnected()) {
      Serial.println("Waiting for bluetooth connection");
      delay(500);
  }*/

  /* FastLed */
  FastLED.addLeds<LED_TYPE,PIN>(leds,NUMPIXELS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

}

void bleSetupOnConnect(){
  if(ble.getMode() != 0) {
    Serial.println(F("***********************"));
    // Set Bluefruit to DATA mode
     Serial.println( F("Switching to DATA mode!") );
    ble.setMode(BLUEFRUIT_MODE_DATA);
    Serial.println(F("***********************"));
  }
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
 // Serial.println("In main loop");
  if(ble.isConnected()){
    if(!printOnceBle) {
      Serial.println("ble connected!");
      printOnceBle = true;
    }
    bleSetupOnConnect();
    // Check bluetooth input
    len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  }

  int reading = digitalRead(buttonPin);
  if(reading != lastButtonState){
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

   // Check for bluetooth input
 if(len != 0) {
  Serial.println("Bluetooth event detected!");
  //delay(2000);
  bl();
 } else if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    // if the button state has changed:
    if(reading != buttonState){
      buttonState = reading;
      if(buttonState == LOW){
        if (currentState < maxState)
        {
          currentState = currentState + 1;
        }else{
          currentState = minState;
        }
        Serial.print("Button pressed! New state is: ");
        Serial.println(currentState);
        packAndSend(); 
      }
    }
  }
  lastButtonState = reading;
  if(isAnimationState){ setLights(); }
  //delay(5);
}

void packAndSend()
{
  Serial.println("Sending data!");
  // Now package into a packetbuffer and write to Bluetooth
  payload[0] = 0x21;
  payload[1] = 0x42;
  payload[2] = currentState;

  xsum = 0;
  //for (uint8_t i=0; i<colLen; i++) {
  for (uint8_t i=0; i<colorLength-1; i++) {
    xsum += payload[i];
  }
  xsum = ~xsum;    
  payload[3] = xsum;  
  for(int i = 0; i < 4; i++){
    Serial.println(payload[i]);
  }
  ble.write(payload,colorLength);
  state = currentState;
  setLights();
}

void setLights(){

    switch (state){
      case 0:
        isAnimationState = true;
        FastLED.show();  
        // insert a delay to keep the framerate modest
        FastLED.delay(500/FRAMES_PER_SECOND); 
        // do some periodic updates
        EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
        confetti();
        break;        
      case 1:
        isAnimationState = false;
        setColors(0, 128, 128); // Blue green
        wipe();
        break;      
      case 2:
        isAnimationState = true;
        FastLED.show();  
        // insert a delay to keep the framerate modest
        FastLED.delay(500/FRAMES_PER_SECOND); 
        // do some periodic updates
        EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
        juggle();
        break;
      case 3:
        isAnimationState = true;      
        FastLED.show();  
        // insert a delay to keep the framerate modest
        FastLED.delay(500/FRAMES_PER_SECOND); 
        // do some periodic updates
        EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
        rainbowWithGlitter();     
        break;
      case 4: 
        isAnimationState = false;      
        setColors(0, 255, 255); // Cyan
        wipe();
        break;
      case 5: 
        isAnimationState = true;         
        slowRainbow(5);
        //isAnimationState = false;
        //setColors(0, 0, 255); // Blue
        //wipe();
        break;
      case 6:
        isAnimationState = true;
        FastLED.show();  
        // insert a delay to keep the framerate modest
        FastLED.delay(500/FRAMES_PER_SECOND); 
        // do some periodic updates
        EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
        sinelon();
        break;
      case 7:
        isAnimationState = false;      
        setColors(128, 0, 128); // Purple
        wipe();
        break;
      case 8:
        isAnimationState = true;
        FastLED.show();  
        // insert a delay to keep the framerate modest
        FastLED.delay(500/FRAMES_PER_SECOND); 
        // do some periodic updates
        EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
        bpm();
        break;
      case 9:
        isAnimationState = false;      
        setColors(192, 214, 228); // Grey / blue
        wipe();
        break;
      case 10:   
        isAnimationState = true;      
        //rainbow(5);
        FastLED.show();  
        // insert a delay to keep the framerate modest        
        FastLED.delay(1000/FRAMES_PER_SECOND); 
        // do some periodic updates
        EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
        rainbow();
        break;
      case 11:   // OFF
        isAnimationState = false;      
        setColors(0, 0, 0);
        wipe();
        break;
      default:
        isAnimationState = false;      
        setColors(0, 0, 0);
        wipe();
//       colorWipe(pixel.Color(0,0,0),5);
       break;  
    }

}

// Lights triggered by bluetooth
void bl(){
  /* Got a packet! */
//  printHex(packetbuffer, len);
  state = packetbuffer[2];
  setLights();
}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<pixel.numPixels(); i++) { 
    pixel.setPixelColor(i, c);
    pixel.setBrightness(BRIGHTNESS);
    pixel.show();
    delay(wait);
  }
}

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUMPIXELS, gHue, 7);
  Serial.println("FastLED.Rainbow called");
}

void rainbowWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUMPIXELS) ] += CRGB::White;
  }
}

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUMPIXELS, 10);
  int pos = random16(NUMPIXELS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUMPIXELS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16( i+7, 0, NUMPIXELS-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUMPIXELS, 20);
  int pos = beatsin16( 13, 0, NUMPIXELS-1 );
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUMPIXELS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void slowRainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel((i+j) & 255));
    }
    pixel.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return pixel.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return pixel.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixel.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

