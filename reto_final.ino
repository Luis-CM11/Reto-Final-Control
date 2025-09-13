#include <Wire.h>
#include <Servo.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <LiquidCrystal_I2C.h>

// -------- LCD 16x2 con backpack I²C --------
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Dirección 0x27 o 0x3F según tu módulo

// -------- IMU --------
BNO080 imu;

// -------- ESCs (motores) --------
Servo ESC1;  // Primer motor
Servo ESC2;  // Segundo motor
int escPin1 = 9;
int escPin2 = 10;

// -------- Potenciómetros --------
int potPinP   = A0;
int potPinI   = A1;
int potPinD   = A2;
int potPinRef = A3;

// -------- Botón de emergencia --------
const int estopPin = 2;
volatile bool emergency = false;

// -------- Variables PID --------
float Kp=5;
float Ki=1;
float Kd=1;
float Kp_min = Kp*0,   Kp_max = Kp*1.3;
float Ki_min = Ki*0,  Ki_max = Ki*1.3;
float Kd_min = Kd*0,   Kd_max = Kd*1.3;

float theta, theta_ref;
float e, e_prev = 0, e_int = 0;
float u = 0; // señal a los ESCs

float up = 0;
float ui = 0;
float ud = 0;

// -------- Tiempo de muestreo --------
const unsigned long Ts_ms = 50;
unsigned long tPrev = 0;

// -------- ISR de emergencia --------
void stopMotor() {
  emergency = true;
}

void setup() {
  Serial.begin(115200);

  // IMU
  Wire.begin();
  if (!imu.begin()) {
    Serial.println("No se detecta BNO080.");
    while (1);
  }
  imu.enableRotationVector(50);

  // ESCs - Inicializar ambos motores con microsegundos
  ESC1.attach(escPin1, 1000, 2000);
  ESC2.attach(escPin2, 1000, 2000);
  
  // Secuencia de armado MÁS LARGA para asegurar
  Serial.println("Armando ESCs... NO DESCONECTAR!");
  ESC1.writeMicroseconds(1000);
  ESC2.writeMicroseconds(1000);
  delay(5000);  // 5 segundos mínimo
  ESC1.writeMicroseconds(1500);
  ESC2.writeMicroseconds(1500);
  delay(2000);
  ESC1.writeMicroseconds(1000);
  ESC2.writeMicroseconds(1000);
  delay(2000);
  Serial.println("ESCs armados");

  // E-Stop
  pinMode(estopPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(estopPin), stopMotor, FALLING);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("PID Balance listo");
  delay(1500);
  lcd.clear();

  Serial.println("Sistema listo.");
}

void loop() {
  if (millis() - tPrev >= Ts_ms) {
    tPrev += Ts_ms;

    if (emergency) {
      ESC1.writeMicroseconds(1000);
      ESC2.writeMicroseconds(1000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("⚠ EMERGENCIA");
      lcd.setCursor(0, 1);
      lcd.print("Motores detenidos");
      Serial.println("⚠ EMERGENCIA: Motores detenidos");
      return;
    }

    // 1. Leer potenciómetros
    int adcP  = analogRead(potPinP);
    int adcI  = analogRead(potPinI);
    int adcD  = analogRead(potPinD);
    int adcSP = analogRead(potPinRef);

    Kp = Kp_min + (adcP / 1023.0) * (Kp_max - Kp_min);
    Ki = Ki_min + (adcI / 1023.0) * (Ki_max - Ki_min);
    Kd = Kd_min + (adcD / 1023.0) * (Kd_max - Kd_min);

    theta_ref = map(adcSP, 0, 1023, 0, 0); // referencia en grados

    // 2. Leer sensor
    if (imu.dataAvailable()) {
      theta = imu.getRoll() * 180.0 / PI; // convertir radianes a grados
    }

    // 3. PID discreto
    e = theta_ref - theta; // error en grados
    
    up = Kp * e;
    ui = Ki * (e + e_prev) * Ts_ms / 2000.0;
    ud = Kd * (e - e_prev) / (Ts_ms/1000.0);

    u = up + ui + ud;

    // 4. MAPEO DIRECTO A MICROSEGUNDOS (1000-2000μs) - SIN PWM
    int esc_signal_1 = map(u, -30, 30, 1330, 1275);  // Motor 1: 32 PWM = 1125μs, 100 PWM = 1375μs 

    // De PWM (64-90) a microsegundos (1000-2000μs)  
    int esc_signal_2 = map(u, -30, 30, 1265, 1325);  // Motor 2: 64 PWM = 1250μs, 90 PWM = 1350μs
    
    // Asegurar que estén dentro del rango válido
    esc_signal_1 = constrain(esc_signal_1, 1000, 2000);
    esc_signal_2 = constrain(esc_signal_2, 1000, 2000);

    // 5. Aplicar señales DIRECTAMENTE en microsegundos
    ESC1.writeMicroseconds(esc_signal_1);
    ESC2.writeMicroseconds(esc_signal_2);

    // 6. Mostrar en LCD los valores KP, KI, KD y Error
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("KP:");
    lcd.print(Kp, 1);
    lcd.print(" KI:");
    lcd.print(Ki, 1);
    
    lcd.setCursor(0, 1);
    lcd.print("KD:");
    lcd.print(Kd, 1);
    lcd.print(" e:");
    lcd.print(e, 1);

    // 7. Debug Serial - TODOS LOS PRINTS ORIGINALES
    Serial.print(" θ_ref=");
    Serial.print(theta_ref, 1);
    Serial.print(" θ=");
    Serial.print(theta, 2);
    Serial.print(" e=");
    Serial.print(e, 2);
    Serial.print(" Kp=");
    Serial.print(Kp, 2);
    Serial.print(" Ki=");
    Serial.print(Ki, 2);
    Serial.print(" Kd=");
    Serial.print(Kd, 2);
    Serial.print(" u=");
    Serial.print(u);
    Serial.print(" esc1=");
    Serial.print(esc_signal_1);
    Serial.print(" esc2=");
    Serial.println(esc_signal_2);

    e_prev = e;
  }
}