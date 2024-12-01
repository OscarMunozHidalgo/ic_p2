#define HANDSHAKE 0xAA
#define ACK 0xDD
#define BYE 0x77

#include <Wire.h> // Arduino's I2C library
 
#define SRF02_I2C_ADDRESS_1 byte((0xE4)>>1)
#define SRF02_I2C_ADDRESS_2 byte((0xE8)>>1)
#define SRF02_I2C_INIT_DELAY 100 // in milliseconds
#define SRF02_RANGING_DELAY 70 // milliseconds

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

void setup()
{
  Serial.begin(9600);
  
  Serial.println("initializing Wire interface ...");
  Wire.begin();
  delay(SRF02_I2C_INIT_DELAY);  
   
  byte software_revision1=read_register(SRF02_I2C_ADDRESS_1,SOFTWARE_REVISION);
  byte software_revision2=read_register(SRF02_I2C_ADDRESS_2,SOFTWARE_REVISION);
  Serial.print("SFR02 ultrasonic range finder in address 0x");
  Serial.print(SRF02_I2C_ADDRESS_1,HEX); Serial.print("(0x");
  Serial.print(software_revision1,HEX); Serial.println(")");
  Serial.print("SFR02 ultrasonic range finder in address 0x");
  Serial.print(SRF02_I2C_ADDRESS_2,HEX); Serial.print("(0x");
  Serial.print(software_revision2,HEX); Serial.println(")");

  // Configuración del LED incluido en placa
  // Inicialmente apagado
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,led_state); led_state=(led_state+1)&0x01;
  
  // Inicialización del puerto para el serial monitor 
  Serial.begin(serial_monitor_bauds);
  while (!Serial);

  // Inicialización del puerto de comunicaciones con el otro dispositivo MKR 
  Serial1.begin(serial1_bauds);
}

int usAddresses[2] = {SRF02_I2C_ADDRESS_1, SRF02_I2C_ADDRESS_2};
int usModes[2] = {REAL_RANGING_MODE_CMS, REAL_RANGING_MODE_CMS};
int usDelays[2] = {70, 70};
int usStates[2] = {1, 1};
int usLastShot[2] = {0, 0};
int message[32];
int iM = 0;


void loop()
{
  //Envío I2C
  
  //Falta comprobar que el delay se cumpla
  for(int i = 0; i<2; i++){
    byte high_byte_range;
    byte low_byte_range;
    byte high_min;
    byte low_min;
    if(usStates[i] == 1 && ((millis()-usLastShot[i])>usDelays[i])){
      Serial.println("dentro del if");
      write_command(usAddresses[i], usModes[i]);
      usLastShot[i] = millis();
      usStates[i] = 0;
      
      Serial.println("Enviando 8");
      /*
      int result = 8;
      Serial1.write(result);
      Serial.println("Saliendo if");
      */
    }
    if(usStates[i] == 2 && ((millis()-usLastShot[i])>usDelays[i])){
      write_command(usAddresses[i], usModes[i]);
      usLastShot[i] = millis();
    }
  }

  //Incluir delay de mínimo 70 ms

  //Comunicación Serial1
  /*
  uint32_t last_ms=millis();
  while(millis()-last_ms<pseudo_period_ms) 
  { 
    //Serial1.write(15);
    if(Serial1.available()>0) 
    {
      uint8_t data=Serial1.read();
      if(data == HANDSHAKE) {
        Serial.println("Handshake recibido"); Serial.println("enviando ACK");
        Serial1.write(ACK);
        iM = 0;
        break;
      }
      if(iM < 32){
        message[iM] = data;
        iM++;
      }
      if(data == BYE) {
        Serial.println("A mimir");
        readMessage();
        break;
      }
      /*if(data%2 != 0 && iM >1) {
        Serial.println("A mimir");
        readMessage();
        break;
      }
      if(data == 0x12){
        Serial.println("One shot");
      }
      Serial.print(data);
      break;
    }
    
  }

  if(millis()-last_ms<pseudo_period_ms) delay(pseudo_period_ms-(millis()-last_ms));
  else Serial.println("<-- received: TIMEOUT!!"); 

  Serial.println("*******************************************************"); 

  digitalWrite(LED_BUILTIN,led_state); led_state=(led_state+1)&0x01;
  */
}
void readMessage(){
  Serial.println("hello your computer has virus");
  for(int i = 0; i<iM;i++){
    Serial.println(message[i]);
  }
  if(message[0] == 0x12){
    activateUS(message[1], 1);
  }
}
int activateUS(int us, int state){
  Serial.println("activando ultrasonido");
  int index = 0;
  for(int i = 0; i < 2; i++){
    if(us == usAddresses[i]){
      index = i;
      break;
    }
  }
  usStates[index] = state;
  return 0;
}

int changeDelay(int us, int delay){
  usDelays[us] = delay;
  return 0;
}

void intToStr(int N, char *str) {
    int i = 0;
  
    // Save the copy of the number for sign
    int sign = N;

    // If the number is negative, make it positive
    if (N < 0)
        N = -N;

    // Extract digits from the number and add them to the
    // string
    while (N > 0) {
      
        // Convert integer digit to character and store
      	// it in the str
        str[i++] = N % 10 + '0';
      	N /= 10;
    } 

    // If the number was negative, add a minus sign to the
    // string
    if (sign < 0) {
        str[i++] = '-';
    }

    // Null-terminate the string
    str[i] = '\0';

    // Reverse the string to get the correct order
    for (int j = 0, k = i - 1; j < k; j++, k--) {
        char temp = str[j];
        str[j] = str[k];
        str[k] = temp;
    }
}
