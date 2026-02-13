// define pins and limits
//Pins
#define IRPin A0
#define mainLED 11  // pwm led pin
#define yloLED 13     // yellow led

//limits and variables
const int max_dist = 80;
const int min_dist = 10;
const float d2 = 42;
const float d1 = 14;
int reading;
float distance;
bool edges_indicator = false;
uint8_t brightness;  // 8 bits unisgned


void setup() {
  Serial.begin(9600);
  pinMode(mainLED, OUTPUT);
  pinMode(yloLED, OUTPUT);

  Serial.print("Sensor reading start:\n");
}

void loop() {
  // get data and convert using github equation
  reading = analogRead(IRPin);

  // account for false readings below 20, this uses the logic from SharpIR.cpp in the SharpIR ghithub library at https://github.com/qub1750ul/Arduino_SharpIR/blob/master/src/SharpIR.cpp
  // Sensor is GP2Y0A21YK0F, added a layer of protection in case of a very low reading bug
  if (reading > 20) {
    distance = 4800.0 / (reading - 20);
  } else {
    distance = max_dist + 1;   // force out-of-bounds
  }

  // account for the edge cases, keeping sensor 10cm from front of vehicle might prove useful...
  if(distance > 80){
    distance = 81;
    edges_indicator = true; // flash yellow led
  }
  else if(distance < 10){
    distance = 9;
    edges_indicator = true; // flash yellow led
  } 

  // brightness 
  if(distance < d1) brightness = 0;   // edge cases
  else if(distance > d2) brightness = 255;
  else{     // scale the brightness linearly from 14cm to 42cm
    brightness = 255 * (1 - (distance - d1)/(d2-d1)); 
    // if the brightness ever inverts for some reason, add failsafe to make sure brightness never goes past edges, as brightness is uint8_t
    // ex: brightness = -4 --> becomes brightness = 252
  }
  // note: some sites recommend using a filter to reduce noise in the sensor, ex: distance = prevDistance * 0.5 + newDistance * 0.5;
  //       presumably this prevents sudden jumps in brightness due to noise

  // control LED brightness for main LED
  analogWrite(mainLED, brightness);

  // serial show data
  Serial.print("Analog reading: ");
  Serial.print(reading);
  Serial.print(" | Distance: ");
  Serial.print(distance);
  Serial.print(" cm | Brightness: ");
  Serial.println(brightness);
  Serial.print(" /255");

  // yellow LED control, also delay of 1s between readings
  if(!edges_indicator)
  {
    // distance is within bounds
    delay(1000);
  }else{
    // flash led with T=1s when distance out of bounds
    digitalWrite(yloLED, HIGH);
    delay(500);
    digitalWrite(yloLED, LOW);
    edges_indicator = false;
    delay(500);
  }
}
