#define HANDSHAKE 0xAA
#define ACK 0xDD

#include <Wire.h> // Arduino's I2C library
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <string.h>

#define TIME_TO_FINISH_MESSAGE 1000 //in milliseconds
#define SRF02_I2C_ADDRESS_1 byte((0xE4)>>1)
#define SRF02_I2C_ADDRESS_2 byte((0xE8)>>1)
#define OLED_I2C_ADDRESS 0x3D
#define SRF02_I2C_INIT_DELAY 100 // in milliseconds
#define SRF02_RANGING_DELAY 70 // milliseconds

// Define proper RST_PIN if required.
#define RST_PIN -1

// LCD05's command related definitions
#define COMMAND_REGISTER byte(0x00)
#define SOFTWARE_REVISION byte(0x00)
#define RANGE_HIGH_BYTE byte(2)
#define RANGE_LOW_BYTE byte(3)
#define AUTOTUNE_MINIMUM_HIGH_BYTE byte(4)
#define AUTOTUNE_MINIMUM_LOW_BYTE byte(5)

// SRF02's command codes
#define REAL_RANGING_MODE_INCHES    byte(80)
#define REAL_RANGING_MODE_CMS       byte(81)
#define REAL_RANGING_MODE_USECS     byte(82)
#define FAKE_RANGING_MODE_INCHES    byte(86)
#define FAKE_RANGING_MODE_CMS       byte(87)
#define FAKE_RANGING_MODE_USECS     byte(88)
#define TRANSMIT_8CYCLE_40KHZ_BURST byte(92)
#define FORCE_AUTOTUNE_RESTART      byte(96)
#define ADDRESS_CHANGE_1ST_SEQUENCE byte(160)
#define ADDRESS_CHANGE_3RD_SEQUENCE byte(165)
#define ADDRESS_CHANGE_2ND_SEQUENCE byte(170)

#define ONE_SHOT 0x11
#define ON 0x20
#define OFF 0x31
#define UNIT_INC 0x41
#define UNIT_CM 0x51
#define UNIT_MS 0x61
#define DELAY 0x70
#define STATUS 0x81
#define US 0x90
#define EXIT 0xA0
#define OLED_ON 0xB0
#define OLED_OFF 0XC0
#define OLED_SHOW_LAST_COMMAND 0XD0

inline void write_command(byte address,byte command)
{ 
  Wire.beginTransmission(address);
  Wire.write(COMMAND_REGISTER); 
  Wire.write(command); 
  Wire.endTransmission();
}

byte read_register(byte address,byte the_register)
{
  Wire.beginTransmission(address);
  Wire.write(the_register);
  Wire.endTransmission();
  
  // getting sure the SRF02 is not busy
  Wire.requestFrom(address,byte(1));
  while(!Wire.available()) { /* do nothing */ }
  return Wire.read();
}

constexpr const uint32_t serial_monitor_bauds=115200;
constexpr const uint32_t serial1_bauds=9600;

constexpr const uint32_t pseudo_period_ms=1000;

uint8_t led_state=LOW;

SSD1306AsciiWire oled;

void setup()
{
  //Serial.begin(9600);
  
  Serial.println("initializing Wire interface ...");
  Wire.begin();
  //CUIDADO
  //Wire.setClock(400000L);

  #if RST_PIN >= 0
    oled.begin(&Adafruit128x64, OLED_I2C_ADDRESS, RST_PIN);
  #else // RST_PIN >= 0
    oled.begin(&Adafruit128x64, OLED_I2C_ADDRESS);
  #endif // RST_PIN >= 0

  oled.setFont(System5x7);
  oled.clear();

  delay(SRF02_I2C_INIT_DELAY);  
   
  byte software_revision1=read_register(SRF02_I2C_ADDRESS_1,SOFTWARE_REVISION);
  byte software_revision2=read_register(SRF02_I2C_ADDRESS_2,SOFTWARE_REVISION);
  Serial.print("SFR02 ultrasonic range finder in address 0x");
  Serial.print(SRF02_I2C_ADDRESS_1,HEX); Serial.print("(0x");
  Serial.print(software_revision1,HEX); Serial.println(")");
  Serial.print("SFR02 ultrasonic range finder in address 0x");
  Serial.print(SRF02_I2C_ADDRESS_2,HEX); Serial.print("(0x");
  Serial.print(software_revision2,HEX); Serial.println(")");
  
  // Inicialización del puerto para el serial monitor 
  Serial.begin(serial_monitor_bauds);
  while (!Serial);

  // Inicialización del puerto de comunicaciones con el otro dispositivo MKR 
  Serial1.begin(serial1_bauds);
}

int usAddresses[2] = {SRF02_I2C_ADDRESS_1, SRF02_I2C_ADDRESS_2};
int usModes[2] = {REAL_RANGING_MODE_CMS, REAL_RANGING_MODE_CMS};
uint usDelays[2] = {1000, 1000};
int usStates[2] = {0, 0};
int usLastShot[2] = {0, 0};
int message[32];
int waitingForRead[2] = {0,0};
int usMinimumDelays[2] = {71,71};

int iM = 0;
uint usLastResult[2] = {0,0};
bool connected = false;
int numOfMessages = 0;
int startOfMessage = 0;
bool messageInProgress = false;
bool oled_on = false;
int oledMode = 0;
String lastCommandExecuted = "Ejecuta un comando";

void loop()
{
  if(millis()%1000 == 0){
    oled.clear();
    if(oled_on){
      if(connected){
        oled.println("Connection: ON");
      } else {
        oled.println("Connection: OFF");
      }
      for (int i = 0; i<2; i++){
        oled.print("US ");oled.print(usAddresses[i]*2);oled.print(": ");
        if(usStates[i] != 0){
          oled.print(usLastResult[i]);
          if(usModes[i] == REAL_RANGING_MODE_CMS) oled.println("cm");
          else if(usModes[i] == REAL_RANGING_MODE_INCHES) oled.println("inch");
          else if(usModes[i] == REAL_RANGING_MODE_USECS) oled.println("us");
        } else {
          oled.println("disconnected");
        }
      }
      oled.println();
      if(oledMode == 1){
        oled.println(lastCommandExecuted);
      }
    }
  }

  if((startOfMessage+TIME_TO_FINISH_MESSAGE < millis()) && messageInProgress){
    Serial.println("Mensaje recibido de forma incorrecta");
    iM = 0;
    numOfMessages = 0;
    messageInProgress = false;
    Serial1.write(0x07);
  }

  shootUS();
  for(int i = 0; i <2; i++){
    if(usStates[i] != 0 && (millis()>(usLastShot[i]+70)) && waitingForRead[i] == 1) readUS(i);
  }

  //Comunicación Serial1
  if(Serial1.available()>0) 
  {
    uint8_t data=Serial1.read();
    Serial.print("Leyendo Serial1: ");
    Serial.println(data);

    if(!connected){
      if(data == HANDSHAKE) {
          Serial1.write(ACK);
          Serial.println("Conexión establecida");
          connected = true;
          iM = 0;
      }
    }
    else {
      if(iM == 0){
        startOfMessage = millis();
        messageInProgress = true;
        numOfMessages = (data & 0x0F)+1;
      }
      if(iM == (numOfMessages-1)) {
        startOfMessage = millis();
        messageInProgress = true;

        message[iM] = data;
        iM++;
        int code = readMessage();
        if(code){
          Serial1.write(code);Serial.println("Ha sucedido un error al ejecutar el comando");
        }
      }
      else if (iM < numOfMessages-1){
        startOfMessage = millis();
        messageInProgress = true;

        message[iM] = data;
        iM++;
      }
    }
  }
}
int readMessage(){

  for(int i = 0; i<numOfMessages;i++){
    Serial.println(message[i]);
  }

  bool usIndexFound = false;
  bool usIndex = 0;
  if (numOfMessages>1){
    for(int i = 0; i < 2; i++){
      if(message[1] == (usAddresses[i]<<1)){
        usIndex = i;
        usIndexFound = true;
      }
    }
  }
  if((!usIndexFound) && numOfMessages > 1){ iM = 0; Serial.println("No se ha encontrado el dispositivo solicitado"); return 0x01;}
  
  if(message[0]>>4 == ONE_SHOT>>4){
    Serial.print("Activando One Shot del dispositivo: "); Serial.println(usAddresses[usIndex]);
    activateUS(usIndex, 1);
    String command = "us " + String(usAddresses[usIndex]*2) + "\none shot";
    restartMessage(command);
    return 0;
  }

  if(message[0]>>4 == ON>>4){

    int period = 0;
    for(int i = 2; i<numOfMessages; i++){
      
      period += message[i]<<8*(i-2);
    }
    if(period <= 70 || period <usMinimumDelays[usIndex]) {iM = 0; Serial.println("0x02 Periodo incorrecto");messageInProgress = false; return 0x02;}

    Serial.print("Activando el dispositivo: "); Serial.print(usAddresses[usIndex]);
    Serial.print("| con periodo: ");
    Serial.println(period);
    activateUS(usIndex, 2);
    setDelay(usIndex, period);
    String command = "us " + String(usAddresses[usIndex]*2) + "\non " + String(period);
    restartMessage(command);
    return 0;
  }
  if(message[0]>>4 == UNIT_INC>>4){
    Serial.print("Dispositivo: "); Serial.print(usAddresses[usIndex]); Serial.println(" en modo inches");
    setModeUS(usIndex, REAL_RANGING_MODE_INCHES);
    String command = "us " + String(usAddresses[usIndex]*2) + " inc";
    restartMessage(command);
    return 0;
  }
  if(message[0]>>4 == UNIT_CM>>4){
    Serial.print("Dispositivo: "); Serial.print(usAddresses[usIndex]*2); Serial.println(" en modo cm");
    setModeUS(usIndex, REAL_RANGING_MODE_CMS);
    String command = "us " + String(usAddresses[usIndex]*2) + " cm";
    restartMessage(command);
    return 0;
  }
  if(message[0]>>4 == UNIT_MS>>4){
    Serial.print("Dispositivo: "); Serial.print(usAddresses[usIndex]); Serial.println(" en modo ms");
    setModeUS(usIndex, REAL_RANGING_MODE_USECS);
    String command = "us " + String(usAddresses[usIndex]*2)+ " ms";
    restartMessage(command);
    return 0;
  }
  if(message[0]>>4 == OFF>>4){
    Serial.print("Apagando dispositivo: "); Serial.println(usAddresses[usIndex]);
    activateUS(usIndex, 0);
    String command = "us " + String(usAddresses[usIndex]*2) + " off";
    restartMessage(command);
    return 0;
  }
  if(message[0]>>4 == DELAY>>4){
    int period = 0;
    for(int i = 2; i<numOfMessages; i++){
      period += message[i]<<8*(i-2);
    }
    Serial.print("Configurando el periodo mínimo del dispositivo: "); Serial.print(usAddresses[usIndex]);
    Serial.print(" a ");
    Serial.print(period);
    Serial.println("ms");

    if(setMinimumDelay(usIndex,period)){Serial.println("Periodo establecido incorrecto");return 0x02;}
    String command = "us " + String(usAddresses[usIndex]*2) + "\ndelay " + String(period);
    restartMessage(command);
    return 0;
  }

  if(message[0]>>4 == STATUS>>4){
    Serial.print("Enviando estado del dispositivo: "); Serial.println(usAddresses[usIndex]);
    sendStatus(usIndex);

    String command = "status " + String(usAddresses[usIndex]*2);
    restartMessage(command);
    return 0;
  }
  if(message[0]>>4 == US>>4){
    Serial.println("Enviando direcciones de ultrasonidos disponibles");
    sendUS();
    restartMessage("us");
    return 0;
  }
  if(message[0]>>4 == EXIT>>4){
    connected = false;
    Serial.println("Desconectando");
    restartMessage("exit");
    return 0;
  }
  if(message[0]>>4 == OLED_ON>>4){
    Serial.println("Encendiendo pantalla OLED");
    oled_on = true;
    iM = 0;
    messageInProgress = false;
    return 0;
  }
  if(message[0]>>4 == OLED_OFF>>4){
    Serial.println("Apagando pantalla OLED");
    oled_on = false;
    iM = 0;
    messageInProgress = false;
    return 0;
  }
  if(message[0]>>4 == OLED_SHOW_LAST_COMMAND>>4){
    Serial.print("Modo show last command: ");
    if(oledMode == 1){
      Serial.println("OFF");
      oledMode = 0;
    } else {
      Serial.println("ON");
      oledMode = 1;
    }
    iM = 0;
    messageInProgress = false;
    return 0;
  }

  iM = 0;
  messageInProgress = false;
  return 0xE6;
}

void restartMessage(String texto){
  iM = 0;
  messageInProgress = false;
  lastCommandExecuted = texto;
}

int activateUS(int us, int state){
  Serial.println("activando ultrasonido");
  
  usStates[us] = state;

  return 0;
}

int setModeUS(int us, int mode){
  usModes[us] = mode;
  return 0;
}

int setDelay(int us, int delay){
  usDelays[us] = delay;
  return 0;
}

int setMinimumDelay(int us, int delay){
  if(delay < 70) return 1;
  usMinimumDelays[us] = delay;
  if(usDelays[us] < delay) usDelays[us] = delay;
  return 0;
}

int sendStatus(int us){

  int bytesDelay = 1;
  int delay = usMinimumDelays[us];
  while(delay > 255){
    delay = delay >> 8;
    bytesDelay++;
  }
  delay = usMinimumDelays[us];
  Serial1.write(0xF4 + bytesDelay);
  Serial1.write(usAddresses[us]<<1);
  
  //Forma de escribir byte a byte un entero desde parte menos significativa a más significativa
  //Ej al enviar 256 = 00000001 00000000 llegará 0 y luego 1
 
  Serial1.write(usModes[us]); 
  Serial1.write(usStates[us]); 

   while(delay > 255){
    Serial1.write(delay); 
    delay = delay >> 8;
  }
  Serial1.write(delay);
  return 0;
}

int sendUS(){
  Serial1.write(0xF2);
  Serial1.write(usAddresses[0]);
  Serial1.write(usAddresses[1]);
}

void shootUS(){
  for(int i = 0; i<2; i++){
    if(usStates[i] != 0 && ((usDelays[i]+usLastShot[i])<millis()))  {
      write_command(usAddresses[i], usModes[i]);
      usLastShot[i] = millis();
      waitingForRead[i] = 1;
    }
  }
}

void readUS(int us){
  byte high_byte_range=read_register(usAddresses[us],RANGE_HIGH_BYTE);
  byte low_byte_range=read_register(usAddresses[us],RANGE_LOW_BYTE);
  byte high_min=read_register(usAddresses[us],AUTOTUNE_MINIMUM_HIGH_BYTE);
  byte low_min=read_register(usAddresses[us],AUTOTUNE_MINIMUM_LOW_BYTE);
  
  int result = int((high_byte_range<<8) | low_byte_range);
  usLastResult[us] = result;
  Serial.print(usAddresses[us]);
  Serial.print(" medida: ");
  Serial.println(result);
  int bytesResult = 1;
  int resultByte = result;
  while(resultByte > 255){
    resultByte = resultByte >> 8;
    bytesResult++;
  }

  Serial1.write(0xF3 + bytesResult);
  Serial1.write(usAddresses[us]<<1);
  Serial1.write(usModes[us]);
  while(result > 255){
    Serial1.write(result);
    result = result >> 8;
  }
  Serial1.write(result);
  if(usStates[us] == 1) usStates[us] = 0;
  waitingForRead[us] = 0;
}
