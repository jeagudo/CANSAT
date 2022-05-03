/*
Código Fuente de la estación de tierra del CANSAT: Sputnit-K
IES Suárez de Figueroa (Zafra(Badajoz)
-Pablo Ortíz Trigo
-Ildefonso Rayan Toro Hamidi
-Agustín Redondo Feria
-Álvaro Otero Matador
Contacto: jeagudo@educarex.es
*/
#include <SPI.h>
#include <LoRa.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <ADXL345.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "Adafruit_CCS811.h"



void fnc_mics4514_preheat(int _prepin)
{
  pinMode(_prepin, OUTPUT);
  digitalWrite(_prepin, 1);
  delay(10000);
  digitalWrite(_prepin, 0);
}

double fnc_mics4514(int _prepin, int _noxpin, int _redpin, int _result, double _temp)
{
  double vnox_value = analogRead(_noxpin);
  double VPIN_NO2 = vnox_value*5.0/1023.0;
  double R0_NO2 = 2200.0;
  double RS_NO2 = ((5*22000.0)/VPIN_NO2)-22000.0;
  double ppbNO2 = 0.0473538*(RS_NO2/R0_NO2)-60.4778+50.8654* pow( (RS_NO2/R0_NO2), 0.327898 );
  double ug_m3NO2 = ppbNO2*1.88;

  double vred_value = analogRead(_redpin);
  double VPIN_CO = vred_value*5.0/1023.0;
  double R0_CO = 750000.0;
  double RS_CO = ((5*47000.0)/VPIN_CO)-47000.0;
  double FactorT = 250.0/(3313.44/_temp+194.11-2.51*_temp);
  double RS_COT = RS_CO;
  if(_temp > -999) RS_COT = FactorT*RS_CO;
  double ppmCO = -75.6518+exp(-143.735*(RS_COT/R0_CO))+41.1398/(RS_COT/R0_CO)+27.3354*(RS_COT/R0_CO);
  double mg_m3CO = ppmCO*1.14;

  if(_result==0)return max(0.0,ppbNO2);
  if(_result==1 && (VPIN_CO >= 0.4))return max(0.0,ppmCO);
  if(_result==2)return max(0.0,ug_m3NO2);
  if(_result==3 && (VPIN_CO >= 0.4))return max(0.0,mg_m3CO);
  if(_result==4)return max(0.0,VPIN_NO2);
  if(_result==5)return max(0.0,VPIN_CO);
  return 0.0;
}
/***********************************************Declaración Variables**************************************/
Adafruit_CCS811 ccs;
TinyGPS gps;
SoftwareSerial softSerial(5, 4);
ADXL345 adxl;
SFE_BMP180 bmp180;
int sf=7; //Spreading Factor 6-12
long frec=433E6;  //Frecuencia 433.075 433.625 434.200  434.500 434.775
long bw=250E3; // Bandwith 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3
double PresionNivelMar = 1013.25; //presion sobre el nivel del mar en mbar
int counter = 0;
String clave="SDF_";//Clave de inicio del mensaje para descartar otros paquetes
int potenciometroPin = A2;    // Pin al que está conectado el potenciómetro
int potenciometro = 0;  // Variable para almancenar la frecuencia seleccionada, por defecto 433E6
long frecuencias[] = {433E6, 433.075E6, 433.625E6, 434.200E6, 434.500E6, 434.775E6, 410E6, 420E6, 430E6, 525E6};// Lista de frecuencias
int tiempoEnvio=2000; // Enviar mensaje cada 2 segundos
/****************************************Configuración incial************************************************/
void setup() {

  softSerial.begin(9600);
  Serial.begin(9600);
  while (!Serial);
 
  Serial.println("Sputnik-K CANSAT");
  Serial.print("Frecuencia: ");
  Serial.println(frec);
//Iniciamos LORA
  if (!LoRa.begin(frec)) {
    Serial.println("Fallo al iniciar LoRa");
    while (1);
  }
//Iniciamos BMP180
if (!bmp180.begin())
    Serial.println("Error al iniciar el BMP180");

//LORA  
  LoRa.setTxPower(20,PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSpreadingFactor(sf);           // ranges from 6-12,default 7 see API docs
  LoRa.setSignalBandwidth(bw);
  //LoRa.setGain(6);
//Acelerómetro
    adxl.powerOn();

    //set activity/ inactivity thresholds (0-255)
    adxl.setActivityThreshold(75); //62.5mg per increment
    adxl.setInactivityThreshold(75); //62.5mg per increment
    adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?

    //look of activity movement on this axes - 1 == on; 0 == off
    adxl.setActivityX(1);
    adxl.setActivityY(1);
    adxl.setActivityZ(1);

    //look of inactivity movement on this axes - 1 == on; 0 == off
    adxl.setInactivityX(1);
    adxl.setInactivityY(1);
    adxl.setInactivityZ(1);

    //look of tap movement on this axes - 1 == on; 0 == off
    adxl.setTapDetectionOnX(0);
    adxl.setTapDetectionOnY(0);
    adxl.setTapDetectionOnZ(1);

    //set values for what is a tap, and what is a double tap (0-255)
    adxl.setTapThreshold(50); //62.5mg per increment
    adxl.setTapDuration(15); //625us per increment
    adxl.setDoubleTapLatency(80); //1.25ms per increment
    adxl.setDoubleTapWindow(200); //1.25ms per increment

    //set values for what is considered freefall (0-255)
    adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
    adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment

    //setting all interrupts to take place on int pin 1
    //I had issues with int pin 2, was unable to reset it
    adxl.setInterruptMapping(ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN);
    adxl.setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN);
    adxl.setInterruptMapping(ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN);
    adxl.setInterruptMapping(ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN);
    adxl.setInterruptMapping(ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN);

    //register interrupt actions - 1 == on; 0 == off
    adxl.setInterrupt(ADXL345_INT_SINGLE_TAP_BIT, 1);
    adxl.setInterrupt(ADXL345_INT_DOUBLE_TAP_BIT, 1);
    adxl.setInterrupt(ADXL345_INT_FREE_FALL_BIT,  1);
    adxl.setInterrupt(ADXL345_INT_ACTIVITY_BIT,   1);
    adxl.setInterrupt(ADXL345_INT_INACTIVITY_BIT, 1);

  //Configuración CCS811  
  if(!ccs.begin())
    Serial.println("Fallo al iniciar CSS811.");

  // Wait for the sensor to be ready
  //while(!ccs.available());
  
//mics4514
  pinMode(2, OUTPUT);//PRE
  pinMode(A0, INPUT );//NOX
  pinMode(A1, INPUT);//RED
  //Serial.begin(9600);
  //Serial.flush();
  //while(Serial.available()>0)Serial.read();
  fnc_mics4514_preheat(2);
  
}

/*********************************************** Bucle principal****************************/
void loop() {
  char status; // Para sensor bmp180
  double T,P,A;//Temperatura, presion atmosférica, altutud relativa
  
  Serial.println("Enviando ");
// 2 Segundos entre cada envío  
  delay(tiempoEnvio);

  status = bmp180.startTemperature(); //Inicio de lectura de temperatura
  LoRa.beginPacket();
  // Lectura del sensor BMP180
  if (status != 0)
  {   
    delay(status); //Pausa para que finalice la lectura
    status = bmp180.getTemperature(T); //Obtener la temperatura
    if (status != 0)
    {
      status = bmp180.startPressure(3); //Inicio lectura de presión
      if (status != 0)
      {        
        delay(status); //Pausa para que finalice la lectura        
        status = bmp180.getPressure(P,T); //Obtener la presión
        if (status != 0)
        {                  
          A= bmp180.altitude(P,PresionNivelMar); //Calcular altura
          
          Serial.print(clave);
          Serial.print("Temperatura:");
          Serial.println(T);
          Serial.print("Presión:"); 
          Serial.println(P);
          Serial.print("Altitud:");
          Serial.println(A);  
          //Serial.println("------------");
          // Esto es lo que se envía
          LoRa.print(clave); //Clave de inicio de paquete para identificarlo
          LoRa.print("Temperatura:");
          LoRa.println(T);
          LoRa.print("Presión:"); 
          LoRa.println(P);
          LoRa.print("Altitud:");
          LoRa.println(A);

          
        }      
      }      
    }
  }
  //Boring accelerometer stuff
    /*int x, y, z;
    adxl.readXYZ(&x, &y, &z); //read the accelerometer values and store them in variables  x,y,z
    // Output x,y,z values
    Serial.print("values of X , Y , Z: ");
    Serial.print(x);
    Serial.print(" , ");
    Serial.print(y);
    Serial.print(" , ");
    Serial.println(z);*/
     
    double xyz[3];
    double ax, ay, az;
    adxl.getAcceleration(xyz);
    ax = xyz[0];
    ay = xyz[1];
    az = xyz[2];
    Serial.print("X:");
    Serial.println(ax);
    //Serial.println(" g");
    Serial.print("Y:");
    Serial.println(ay);
    //Serial.println(" g");
    Serial.print("Z:");
    Serial.println(az);
    //Serial.println(" g");
    //Serial.println("**********************");
    LoRa.print("X:");
    LoRa.println(ax);
    //Serial.println(" g");
    LoRa.print("Y:");
    LoRa.println(ay);
    //Serial.println(" g");
    LoRa.print("Z:");
    LoRa.println(az);
// GPS

   bool newData = false;
   unsigned long chars;
   unsigned short sentences, failed;
   
   // Intentar recibir secuencia durante un segundo
   for (unsigned long start = millis(); millis() - start < 1000;)
   {
      while (softSerial.available())
      {
         char c = softSerial.read();
         if (gps.encode(c)) // Nueva secuencia recibida
            newData = true;
      }
   }

   if (newData)
   {
      float flat, flon;
      unsigned long age;
      gps.f_get_position(&flat, &flon, &age);
      Serial.print("LAT: ");
      Serial.println(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
      Serial.print(" LON: ");
      Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
      Serial.print(" ALT: ");
      Serial.println(gps.f_altitude());
      LoRa.print("LAT: ");
      LoRa.println(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
      LoRa.print(" LON: ");
      LoRa.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
      LoRa.print(" ALT: ");
      LoRa.println(gps.f_altitude());  
      /*Serial.print(" COUR=");
      Serial.print(gps.f_course());
      Serial.print(" SPEED=");
      Serial.print(gps.f_speed_kmph());
      Serial.print(" SAT=");
      Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
      Serial.print(" PREC=");
      Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());*/
   }
//CCS811
  if(ccs.available()){
    if(!ccs.readData()){
      Serial.print("CO2: ");
      Serial.println(ccs.geteCO2());
      Serial.print("TVOC: ");
      Serial.println(ccs.getTVOC());
      LoRa.print("CO2: ");
      LoRa.println(ccs.geteCO2());
      LoRa.print("TVOC: ");
      LoRa.println(ccs.getTVOC());
    }
    else{
      Serial.println("ERROR CCS811!");
      while(1);
    }
  }
//mics4514
    Serial.print("NO2: ");
    Serial.println(fnc_mics4514(2,A0,A1,2,21));//NO2 ug/m3
    Serial.print("CO: ");
    Serial.println(fnc_mics4514(2,A0,A1,3,21));//C0 mg/m3
    LoRa.print("NO2: ");
    LoRa.println(fnc_mics4514(2,A0,A1,2,21));//NO2 ug/m3
    LoRa.print("CO: ");
    LoRa.println(fnc_mics4514(2,A0,A1,3,21));//C0 mg/m3
    //Serial.println(fnc_mics4514(2,A0,A1,0,21));//NO2 ppb
    //Serial.println(fnc_mics4514(2,A0,A1,1,21));//C0 ppm
    //Serial.println();

  Serial.println("----------------");
  LoRa.endPacket();
  
  //Si hay cambio en el potenciometro cambiamos la frecuencia
  potenciometro = analogRead(potenciometroPin); // Leemos Frecuencia
  if (frec!=frecuencias[map(potenciometro, 0, 1023, 0, 9)]){ 
    frec=frecuencias[map(potenciometro, 0, 1023, 0, 9)];// 10 frecuencias de 0 a 9
    LoRa.setFrequency(frec);
    Serial.print("Frecuencia ");
    Serial.println(frec);}
}
