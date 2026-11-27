// # Editor     : roker
// # Date       : 05.03.2018
// # Product name: URM V5.0 ultrasonic sensor
// # Product SKU : SEN0001
// # Version     : 1.0
// # Description:
// # The Sketch for scanning 180 degree area 2-800cm detecting range
// # The sketch for using the URM37 PWM trigger pin mode from DFRobot
// #   and writes the values to the serialport
// # Connection:
// #       Vcc (Arduino)    -> Pin 1 VCC (URM V5.0)
// #       GND (Arduino)    -> Pin 2 GND (URM V5.0)
// #       Pin 5 (Arduino)  -> Pin 6 COMP/TRIG (URM V5.0)
// #       Pin A0 (Arduino) -> Pin 7 DAC (URM V5.0)
int URTRIG = 5;         // trigger pin
int sensorPin = A0;     // select the input pin for the potentiometer
int sensorValue = 0;    // variable to store the value coming from the sensor
unsigned int DistanceMeasured = 0;
void setup()
{
  //Serial initialization
  Serial.begin(9600);                        // Sets the baud rate to 9600
  pinMode(URTRIG, OUTPUT);                   // A low pull on pin COMP/TRIG
  digitalWrite(URTRIG, HIGH);                // Set to HIGH
  delay(500);
  Serial.println("Init the sensor");
}
void loop()
{
  digitalWrite(URTRIG, LOW);
  digitalWrite(URTRIG, HIGH);              
  delay(200);
  sensorValue = analogRead(sensorPin);
  
  sensorValue = sensorValue * 1.1; //  (sensorValue * 5000 / 1024 ) / 4.125 = sensorValue * 1.1 , calculate the voltage value ADC collected and divide it by 4.125mV /cm.
  Serial.println(sensorValue, 1);
}
