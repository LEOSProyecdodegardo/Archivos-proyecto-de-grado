/*######################################################################
# C CODE.
######################################################################
# Copyright (C) 2024. F.E.Segura-Quijano (FES) fsegura@uniandes.edu.co
#
# Este trabajo está licenciado bajo la Licencia:
# Creative Commons Atribución-NoComercial 4.0 Internacional.
# Para ver una copia de esta licencia, visita
# http://creativecommons.org/licenses/by-nc/4.0/ o envía una carta
# a Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
######################################################################

######################################################################
# Este código fue generado con asistencia de librerias de componentes
# Este código fue generado con asistencia de ChatGPT de OpenAI.
######################################################################*/

/*######################################################################
# LIBRARIES.
######################################################################*/
#define DEBUG // Variable to use debug. Comment to not use it.
#ifdef DEBUG
#endif

#include <Arduino.h>
#include <Wire.h>

#include <WiFi.h> // Para desactivar WiFi y reducir ruido

/*######################################################################*/
/* LIBRERIAS SENSOR MATRIZ DISTANCIA VL53L5CX */
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM
int imageResolution = 0;              // Used to pretty print output
int imageWidth = 0;                   // Used to pretty print output

/*######################################################################*/
/* LIBRERIAS BUTTON/PULSADOR */
#include <SparkFun_Qwiic_Button.h>
QwiicButton button;
// Define LED characteristics
uint8_t brightness = 250;  // The maximum brightness of the pulsing LED. Can be between 0 (min) and 255 (max)
uint16_t cycleTime = 1000; // The total time for the pulse to take. Set to a bigger number for a slower pulse, or a smaller number for a faster pulse
uint16_t offTime = 200;    // The total time to stay off between pulses. Set to 0 to be pulsing continuously.

/*######################################################################*/
/* LIBRERIAS OPENLOG */
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"
OpenLog myLog;            // Create instance
int ledPin = LED_BUILTIN; // Status LED connected to digital pin 13

/*######################################################################*/
/* LIBRERIAS LCD */
#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD
SerLCD lcd;         // Initialize the library with default I2C address 0x72

/*######################################################################*/
/* GPS SAM-M8Q */
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // Biblioteca para el GPS u-blox
SFE_UBLOX_GNSS gps;

#define GPS_BAUDRATE 9600
float latitude = 0.0;  // Para manejar valores decimales
float longitude = 0.0; // Para manejar valores decimales

// Pines UART para el GPS (ajustados a la configuración de la ESP32-S2 Thing Plus)
#define GPS_RX 33 // RX1 (GPIO33) del ESP32-S2 conectado a TX del GPS
#define GPS_TX 34 // TX1 (GPIO34) del ESP32-S2 conectado a RX del GPS

  // Variables para almacenar fecha y hora
  int year, month, day;
  int hour, minute, second;

/*######################################################################*/
/* VARIABLES GLOBALES */
int inicio = 0;
int distancemax = 0;
int distancemeas = 0;

byte SIV = 0;

unsigned long lastTime = 0;

String dateStamp;
String timeStamp;
String geoStamp;
String dataStamp;
String dataString;
String distanceString;

bool firstPressDetected = false;
bool waitingForSecondPress = false;
unsigned long lastPressTime = 0;
const unsigned long doublePressInterval = 500; // Intervalo de tiempo para doble pulsación (en ms)

/* VARIABLES GLOBALES PARA DETECCIÓN DE TARJETA SD */
bool sdCardPresent = true;     // Indica si la tarjeta SD está presente
unsigned long lastSdCheckTime = 0; // Tiempo de la última alerta de SD
const unsigned long sdAlertInterval = 5000; // Intervalo de alerta en milisegundos (5 segundos)

// Variables para medir el voltaje
int General = A0;
int Lectura_General = 0;
float Volt_General = 0.0;
const float VRef = 3.3; // Voltaje de referencia del ESP32-S2
const int Resolucion_ADC = 4095; // Resolución del ADC (12 bits)
const float Factor_Divisor = 2.0; // Divisor de voltaje con resistencias de 100kΩ
const float Calibracion_Factor = 0.87; // Factor de corrección manual
const int Num_Lecturas = 50; // Cantidad de lecturas para promediar

int Laser = 3;
int Red_Led = 37;

/*######################################################################
# SETUP
######################################################################*/
void setup() {
  resetI2CBus(); // Reinicia el bus I2C
  Wire.begin();  // Initialize I2C bus
  Serial.begin(115200);
  Serial.println(F("START SETUP: MasPasto"));
  delay(100);

  /*######################################################################*/
  /* LIBRERIAS SENSOR MATRIZ DISTANCIA VL53L5CX */
  Serial.println("START SENSOR VL53L5CX Imager: - 0");
  Wire.setClock(50000); // Sensor has max I2C freq of 400kHz
  Serial.println("START SENSOR VL53L5CX Imager: - 1");

  if (myImager.begin() == false){
    Serial.println(F("START SENSOR VL53L5CX Imager: Sensor not found - check your wiring. Freezing"));
    while (1);
    }

  myImager.setResolution(8 * 8);              // Enable all 64 pads
  imageResolution = myImager.getResolution(); // Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution);         // Calculate printing width
  myImager.startRanging();
  delay(100);

  //######################################################################*/
  /* LIBRERIA BUTTON/PULSADOR */
  // check if button will acknowledge over I2C
  if (button.begin() == false){
    Serial.println("START BUTTON: Device did not acknowledge! Freezing.");
    while (1);
    }

  button.LEDconfig(brightness, cycleTime, offTime);
  Serial.println("START BUTTON: Button acknowledged.");
  button.LEDoff(); // start with the LED off
  delay(100);

  //######################################################################*/
  /* LIBRERIA LCD */
  Serial.println("START LCD: LCS Started.");
  lcd.begin(Wire);
  lcd.clear();
  lcd.setCursor(0, 0); // Coloca el cursor en la columna 0, fila 0
  lcd.print("PROYECTO");
  lcd.println();
  lcd.setCursor(0, 1); // Coloca el cursor en la columna 0, fila 1
  lcd.print(" MasPasto v1.0");
  lcd.println();
  delay(100);

  //######################################################################
  /* GPS */
  Serial1.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX, GPS_TX); // Configurar Serial1 para el GPS

  Serial.println("Iniciando GPS SAM-M8Q...");

  // Iniciar la comunicación con el GPS SAM-M8Q
  if (!gps.begin(Serial1)){
    Serial.println("GPS SAM-M8Q no detectado. Verifica las conexiones.");
    while (true);
    }

  // Configurar el GPS para salida UBX y guardar configuración
  gps.setUART1Output(COM_TYPE_UBX); // Utilizar solo formato UBX
  gps.saveConfiguration();

  Serial.println("GPS SAM-M8Q inicializado correctamente.");

  //######################################################################/
  /* OPENLOG */
  pinMode(ledPin, OUTPUT);
  myLog.begin(); // Open connection to OpenLog (no pun intended)
  Serial.println("START OPENLOG: Write File Test");
  // Ajusta la hora a la zona horaria deseada, por ejemplo GMT-5 (para Colombia)
  hour = gps.getHour() - 5; // Ajusta la zona horaria
  if (hour < 0){
    hour += 24; // Ajusta si el resultado es una hora negativa
  }

  String fileName = "D_" + String(gps.getYear()) + "-" + String(gps.getMonth()) + "-" + String(gps.getDay()) + "T" + String(hour) + "-" + String(gps.getMinute()) + ".txt"; // FES

  // myLog.append("hola.txt");  // Crea un nuevo archivo con el nombre del timestamp
  myLog.append(fileName);
  myLog.println("MasPasto");
  Serial.println("START OPENLOG: Writing DATA");
  myLog.syncFile();
  Serial.println(F("FIN SETUP: MasPasto"));
  delay(100);

  //######################################################################/
  /* VOLTIMETRO */

  pinMode(Red_Led, OUTPUT);
  pinMode(Laser, OUTPUT);
  digitalWrite(Laser, HIGH);

  // Configuración del ADC
  analogReadResolution(12); // Forzamos la resolución a 12 bits
  analogSetAttenuation(ADC_11db); // Atenuación máxima para medir hasta ~3.9V

  // Desactivar WiFi para reducir interferencia
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

}








/*######################################################################
# LOOP.
######################################################################*/
void loop(void){
  const int numExecutions = 4; // Número de veces que quieres ejecutar DISTANCE_MEAS
  unsigned long currentTime = millis(); // Actualiza currentTime para el control de tiempo

  switch (inicio){

    // Case 0
    case 0:
      LCD_INFO_MIDIENDO_DMAX();
      Serial.println(F("C0======="));
      inicio = 1;
      break;

    // Case 1
    case 1:
      if (button.isPressed()){
        button.LEDconfig(brightness, cycleTime, offTime);
        while (button.isPressed()){
          delay(10); // Espera a que se suelte el botón
          button.LEDoff();
        }
        for (int i = 0; i < numExecutions; i++){
          do{
            distancemax = DISTANCE_DMAX();
          }
          while (distancemax == 0);
        }
        LCD_INFO_DATO_DMAX(distancemax);
        Serial.println(F("C1======="));
        inicio = 2;
      }
      break;

    // Case 2
    case 2:
      if (button.isPressed()){
        if (firstPressDetected){
          if ((currentTime - lastPressTime) <= doublePressInterval){
            // Doble pulsación detectada
            Serial.println("Double press detected!");
            inicio = 0; // Cambia a estado 0 para reiniciar el flujo
            firstPressDetected = false; // Reinicia el estado
            waitingForSecondPress = false;
            break;  // Sal del case 2
          }
        }

        else{
          // Detecta la primera pulsación
          firstPressDetected = true;
          waitingForSecondPress = true;
          lastPressTime = currentTime;
        }

        // Espera a que se libere el botón antes de continuar
        while (button.isPressed()){
          delay(10); // Pequeño retardo para evitar rebotes
        }
      }

      // Si estamos esperando una segunda pulsación y se acaba el tiempo, es una única pulsación
      if (waitingForSecondPress && (currentTime - lastPressTime) > doublePressInterval){
        Serial.println("Single press detected!");
        // Si el tiempo excede el intervalo, considera que fue una pulsación única
        button.LEDconfig(brightness, cycleTime, offTime);
        while (button.isPressed())
          delay(10);
        button.LEDoff();
        LCD_WAIT();
        for (int i = 0; i < numExecutions; i++){
          do{
            distancemeas = DISTANCE_MEAS();
          }
          while (distancemeas == 0); // Repite mientras distancemeas sea 0
        }
          // Llamada a la función GPS dentro del loop principal
        GPS();
        OPENLOG();
        LCD_INFO_DATO_MEAS(distancemeas);
        Serial.println(F("C2======="));

        firstPressDetected = false;
        waitingForSecondPress = false;
        inicio = 3; // Reinicia el flujo para evitar repetición continua
      }
      break;

    // Case 3
    case 3:
      delay(500);
      Serial.println(F("C3======="));
      inicio = 2;
      break;
  }
  




  // Voltímetro
  long suma_lecturas = 0;

  // Realizar múltiples lecturas para promediar
  for (int i = 0; i < Num_Lecturas; i++) {
    suma_lecturas += analogRead(General);
    delay(10); // Pequeña espera para estabilizar la lectura
  }

  // Calcular el promedio de las lecturas
  Lectura_General = suma_lecturas / Num_Lecturas;

  // Calibrar manualmente el voltaje medido
  float Voltaje_Medido = (VRef * Lectura_General / Resolucion_ADC) * Calibracion_Factor;

  // Calcular el voltaje tomando en cuenta el divisor de voltaje
  Volt_General = Voltaje_Medido * Factor_Divisor;

  // Indicadores seriales
  Serial.print("Lectura promedio: "); Serial.print(Lectura_General); Serial.print(", ");
  Serial.print("Voltaje General (Calculado): "); Serial.print(Volt_General, 2); Serial.println(" V");

  if (Volt_General <= 3.38){
    digitalWrite(Red_Led, HIGH);
  }
  else {
    digitalWrite(Red_Led, LOW);
  }
  



  // Llamada a la función de verificación de tarjeta SD
  //checkSdCardPresence();

  delay(200); // Controla la carga del bus I2C
}

/*######################################################################
# FUNCIONES.
######################################################################*/
void resetI2CBus(){
  Wire.end();
  delay(100); // Espera un momento
  Wire.begin();
  Wire.setClock(500000); // Configura nuevamente la frecuencia del bus I2C
}

void LCD_INFO_MIDIENDO_DMAX(void){
  Serial.println("FUNC: LCD_INFO_MIDIENDO_DMAX");
  lcd.clear();
  delay(10);
  lcd.setCursor(0, 0); // Coloca el cursor en la columna 0, fila 0
  lcd.print("Midiendo DMax...");
  lcd.println();
  lcd.setCursor(0, 1); // Coloca el cursor en la columna 0, fila 1
  lcd.print("Presione boton");
  lcd.println();
  delay(10);
}

void LCD_INFO_DATO_DMAX(int distance){
  Serial.println("FUNC: LCD_INFO_DATO_DMAX");
  lcd.clear();
  delay(10);
  lcd.setCursor(0, 0); // Coloca el cursor en la columna 0, fila 0
  lcd.print("D-Max:");
  lcd.print(distance / 10);
  lcd.print("cm");
  lcd.println();
  lcd.setCursor(0, 1); // Coloca el cursor en la columna 0, fila 1
  lcd.print("Presione boton");
  lcd.println();
  delay(10);
}

void LCD_INFO_DATO_MEAS(int distance){
  Serial.println("FUNC: LCD_MEAS");
  lcd.clear();
  delay(10);
  lcd.setCursor(0, 0); // Coloca el cursor en la columna 0, fila 0
  lcd.print("S:");
  lcd.print(String(SIV));
  lcd.print(" - ");
  lcd.print("D:");
  lcd.print(distance/10); //estaba dividido por 10
  lcd.print("cm");
  lcd.println();
  lcd.setCursor(0, 1); // Coloca el cursor en la columna 0, fila 1
  lcd.print(timeStamp);
  lcd.print(" - ");
  lcd.print("OK");
  lcd.println();
  delay(10);
}

void LCD_WAIT(void){
  Serial.println("FUNC: LCD_WAIT");
  lcd.clear();
  delay(10);
  lcd.setCursor(6, 1); // Coloca el cursor en la columna 0, fila 0
  lcd.print("wait...");
  lcd.println();
  delay(10);
}

bool getRangingDataWithRetries(VL53L5CX_ResultsData *data, int retries = 3){
  while (retries > 0){
    if (myImager.getRangingData(data)){
      return true;
    }
    retries--;
    delay(50); // Espera un poco antes de intentar de nuevo
  }
  return false;
}

int DISTANCE_DMAX(void) {
  int distance = 0;
  int distancetemp = 0;

  Serial.println("FUNC: DISTANCE_DMAX");
  if (myImager.isDataReady() == true) {
    if (getRangingDataWithRetries(&measurementData)) {
      for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth) {
        for (int x = imageWidth - 1; x >= 0; x--) {
          Serial.print("\t");
          distancetemp = measurementData.distance_mm[x + y];
          Serial.print(distancetemp);
          distance = distance + distancetemp;
        }
        Serial.println();
      }
      Serial.println();
    }
    distance = distance / (imageWidth * imageWidth);
  }
  Serial.print(F("Distance: "));
  Serial.println(distance);
  delay(500);
  return distance;
}


int DISTANCE_MEAS(void){
  int distance = 0;
  int distancetemp = 0;

  Serial.println("FUNC: DISTANCE_MEAS");
  if (myImager.isDataReady() == true){
    if (getRangingDataWithRetries(&measurementData)){
      distanceString = "";
      for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth){
        for (int x = imageWidth - 1; x >= 0; x--){
          Serial.print("\t");
          distancetemp = distancemax - measurementData.distance_mm[x + y];
          distanceString = distanceString + distancetemp + ",";
          Serial.print(distancetemp);
          distance = distance + distancetemp;
        }
        Serial.println();
      }
      Serial.println();
    }
    distance = distance / (imageWidth * imageWidth);
  }
  Serial.print(F("Distance: "));
  Serial.println(distance);
  delay(500);
  return distance;
}


void OPENLOG() {
    if (millis() - lastTime > 1000) {
        lastTime = millis(); // Actualiza el temporizador
        Serial.println("FUNC: OPENLOG");

        // Fecha y hora
        dateStamp = "Fecha:" + String(year) + "-" + String(month) + "-" + String(day);
        timeStamp = "Hora:" + String(hour) + ":" + String(minute) + ":" + String(second);

        // Posición (latitud y longitud con 8 decimales)
        char latBuffer[12]; // Buffer para latitud
        char lonBuffer[12]; // Buffer para longitud
        dtostrf(latitude, 12, 8, latBuffer); // 12: Ancho total, 8: Decimales
        dtostrf(longitude, 12, 8, lonBuffer);

        geoStamp = "Posición:" + String(latBuffer) + "," + String(lonBuffer);

        // Datos adicionales
        dataStamp = "Dmax, Dmeas:" + String(distancemax) + "," + String(distancemeas / 10);
        dataString = dateStamp + "," + timeStamp + "," + geoStamp + "," + dataStamp + "," + "64 Puntos:" + String(distanceString); 

        // Escribe en el log
        myLog.println(dataString);
        myLog.syncFile();
    }
}


void GPS() {
    // GPS
    Serial.println("FUNC: GPS INICIO");

    // Verificar si hay una señal GPS válida
    if (gps.getGnssFixOk()) {
        latitude = gps.getLatitude() / 10000000.0;
        longitude = gps.getLongitude() / 10000000.0;

        // Obtener fecha y hora del GPS
        year = gps.getYear();
        month = gps.getMonth();
        day = gps.getDay();

        // Ajuste de la hora UTC a UTC-5
        hour = gps.getHour() - 5;
        if (hour < 0) {
            hour += 24; // Ajuste para horas negativas
            day -= 1;   // Retrocede un día si se cruza la medianoche
        }

        minute = gps.getMinute();
        second = gps.getSecond();

        // Ajuste básico del día si cae al inicio del mes
        if (day == 0) {
            month -= 1;               // Retrocede al mes anterior
            if (month == 0) {         // Si el mes es 0, retrocede al año anterior
                month = 12;
                year -= 1;
            }
            // Determina el último día del mes anterior
            if (month == 2) {         // Febrero
                day = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) ? 29 : 28;
            } else if (month == 4 || month == 6 || month == 9 || month == 11) {
                day = 30;
            } else {
                day = 31;
            }
        }

        // Imprimir latitud y longitud
        Serial.print("Latitud: ");
        Serial.println(latitude, 7);
        Serial.print("Longitud: ");
        Serial.println(longitude, 7);

        // Imprimir fecha y hora
        Serial.printf("Fecha: %02d/%02d/%04d\n", day, month, year);
        Serial.printf("Hora: %02d:%02d:%02d (UTC-5)\n", hour, minute, second);
    } 
    else {
        Serial.println("No hay señal GPS válida.");
    }
    Serial.println("FUNC: GPS FIN");
    //delay(1000); // Espera un segundo antes de la siguiente lectura
}


//######################################################################//
/* FUNCIÓN PARA MOSTRAR ALERTA DE TARJETA SD NO DETECTADA */
void LCD_NO_SD_ALERT() {
  lcd.clear();
  delay(10);
  lcd.setCursor(0, 0);
  lcd.print("NO SD DETECTED");
  lcd.setCursor(0, 1);
  lcd.print("Insert SD Card");
  delay(2000); // Mostrar el mensaje durante 2 segundos
  lcd.clear(); // Limpiar la pantalla
}

/* FUNCIÓN PARA VERIFICAR LA PRESENCIA DE LA TARJETA SD */
void checkSdCardPresence() {
  // Verificar el estado de la tarjeta SD
  if (!myLog.begin()) { // Si la tarjeta SD no está presente
    if (sdCardPresent) { // Solo si antes se consideraba presente
      Serial.println("SD card removed."); // Mensaje en el monitor serial
      sdCardPresent = false;             // Actualiza el estado
      LCD_NO_SD_ALERT();                 // Mostrar mensaje en LCD
    }
  } 
  else { // Si la tarjeta SD está presente
    if (!sdCardPresent) { // Solo si antes se consideraba ausente
      Serial.println("SD card inserted."); // Mensaje en el monitor serial
      sdCardPresent = true;                // Actualiza el estado
      lcd.clear();                         // Limpiar la pantalla
      lcd.setCursor(0, 0);
      lcd.print("SD Card Detected");       // Opcional: mostrar mensaje de confirmación
      delay(1000);                         // Mostrar por un segundo
    }
  }
}