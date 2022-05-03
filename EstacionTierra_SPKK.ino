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
/***********************************************Declaración Variables**************************************/
long frec=433E6; //Frecuencia actual
int sf=7; //Spreading Factor 6-12
long bw=250E3; // Bandwith 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3
int potenciometroPin = A2;    // Pin al que está conectado el potenciómetro
int potenciometro = 0;  // Variable para almancenar la frecuencia seleccionada, por defecto 433E6
long frecuencias[] = {433E6, 433.075E6, 433.625E6, 434.200E6, 434.500E6, 434.775E6, 410E6, 420E6, 430E6, 525E6};// Lista de frecuencias


/****************************************Configuración incial************************************************/
void setup() {
  Serial.begin(9600); //Iniciamos puerto serie
  while (!Serial);

  Serial.println("Estación Tierra Sputnik-K");

  if (!LoRa.begin(frec)) {
    Serial.println("Fallo al iniciar LoRa");
    while (1);
  }
  Serial.print("Frecuencia ");
  Serial.println(frec);
  //Configuramos LORA 
  LoRa.setTxPower(20,PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSpreadingFactor(sf);          
  LoRa.setSignalBandwidth(bw);
  //LoRa.setGain(6);
}

/*********************************************** Bucle principal****************************/
void loop() {
  int packetSize = LoRa.parsePacket();
  // Si recibimos paquete
  if (packetSize) {
    int cont=0;
    int valido=0;
    while (LoRa.available()) {
      char recibido=(char)LoRa.read();
      if (valido>3)
        Serial.print(recibido);
      if ((cont==0)&&(recibido=='S')) valido++;
      if ((cont==1)&&(recibido=='D')) valido++;
      if ((cont==2)&&(recibido=='F')) valido++;
      if ((cont==3)&&(recibido=='_')) valido++;
      cont++;
    }
  }
  //Si hay cambio en el potenciometro cambiamos la frecuencia
  potenciometro = analogRead(potenciometroPin); // Leemos Frecuencia
  if (frec!=frecuencias[map(potenciometro, 0, 1023, 0, 9)]){ 
    frec=frecuencias[map(potenciometro, 0, 1023, 0, 9)];// 10 frecuencias de 0 a 9
    LoRa.setFrequency(frec);
    Serial.print("Frecuencia ");
    Serial.println(frec);}
}
