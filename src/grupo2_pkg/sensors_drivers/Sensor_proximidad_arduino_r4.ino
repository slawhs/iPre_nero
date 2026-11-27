// Sensor de proximidad URM37 V5.0 (DFRobot SEN0001) en Arduino UNO R4 WiFi.
// Modo disparo por COMP/TRIG, distancia leida de la salida analogica DAC.
//
// Conexion:
//   Vcc (Arduino)    -> Pin 1 VCC (URM V5.0)
//   GND (Arduino)    -> Pin 2 GND (URM V5.0)
//   Pin 5 (Arduino)  -> Pin 6 COMP/TRIG (URM V5.0)
//   Pin A0 (Arduino) -> Pin 7 DAC (URM V5.0)
//
// Salida serie (115200 baud): una linea por medida con la distancia en cm y un
// decimal, a 1000 / PERIODO_MS Hz. La lee grupo2_pkg/proximity_sensor_node.
//
// El DAC da 4.125 mV/cm. Con el ADC de 14 bits del R4 (0.31 mV por cuenta) la
// cuantizacion es ~0.07 cm, frente a ~1.2 cm con los 10 bits por defecto.
// Se lee justo antes del siguiente disparo: el DAC ya tiene la medida anterior.

const int URTRIG = 5;          // COMP/TRIG
const int SENSOR_PIN = A0;     // DAC
const unsigned long PERIODO_MS = 50;   // 20 Hz
const float MV_POR_CM = 4.125;
const float VREF_MV = 5000.0;
const float ADC_MAX = 16383.0; // 14 bits

void disparar() {
  digitalWrite(URTRIG, LOW);
  delayMicroseconds(10);
  digitalWrite(URTRIG, HIGH);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(14);
  pinMode(URTRIG, OUTPUT);
  digitalWrite(URTRIG, HIGH);
  delay(500);
  Serial.println("Init the sensor");
  disparar();
}

void loop() {
  static unsigned long t = millis();
  while (millis() - t < PERIODO_MS) {}   // ritmo fijo, sin deriva
  t += PERIODO_MS;

  float mv = analogRead(SENSOR_PIN) * VREF_MV / ADC_MAX;
  disparar();
  Serial.println(mv / MV_POR_CM, 1);
}
