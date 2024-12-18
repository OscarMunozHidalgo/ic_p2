#include <Arduino.h>

constexpr const uint32_t serial_monitor_bauds = 115200;
constexpr const uint32_t serial1_bauds = 9600;
constexpr const uint32_t pseudo_period_ms = 1000;

#define HANDSHAKE 0xAA
#define ACK 0xDD //221
#define MAX_WORDS 4
#define BYE 0x77
#define pseudo_period_ms 1000

#define ONE_SHOT 0x11
#define ON 0x21
#define OFF 0x31
#define UNIT_INC 0x41
#define UNIT_CM 0x51
#define UNIT_MS 0x61
#define DELAY 0x71
#define STATUS 0x81
#define US 0x90
#define EXIT 0xA0
#define OLED_ON 0xB0
#define OLED_OFF 0xC0
#define OLED_SHOW_LAST_COMMAND 0xD0

uint8_t handshakeEnviado = 0;
uint32_t last_ms = 0;
uint8_t processingCommand = 0;

void setup() {
  Serial.begin(serial_monitor_bauds);
  while (!Serial);

  Serial1.begin(serial1_bauds); // Serial1.write para escribir y .read para leer, siempre dentro de un serial1.available>0
}

void sendHandshake() {
  Serial1.write(HANDSHAKE);
}

void sendBye() {
  Serial1.write(BYE);
}

void command_help() {
  Serial.println("\nLista de comandos:");
  Serial.println("help: Muestra la lista de comandos y su sintaxis");
  Serial.println("us: Muestra la relación de sensores de ultrasonidos disponibles en el dispositivo sensor.");
  Serial.println("us <srf02> one-shot: El sensor con dirección <srf02> se programa para producir un disparo único.");
  Serial.println("us <srf02> on <period_ms>: El sensor con dirección <srf02> se programa para producir periódicamente disparos cada <period_ms> milisegundos.");
  Serial.println("us <srf02> off: El sensor con dirección <srf02> se apaga si este estaba disparando periódicamente.");
  Serial.println("us <srf02> unit {inc | cm | ms}: Modifica la unidad de medida devuelta por el sensor <srf02>.");
  Serial.println("us <srf02> delay <ms>: Establece el retardo mínimo que debe haber entre dos disparos consecutivos del sensor <srf02>.");
  Serial.println("us <srf02> status: Proporciona información de configuración del sensor <srf02>.");
  Serial.println("oled on: Enciende la pantalla OLED.");
  Serial.println("oled off: Apaga la pantalla OLED.");
  Serial.println("oled show: Alterna entre mostrar o no el último comando ejecutado en la pantalla OLED.");
  Serial.println("exit: Cierra la conexión con el dispositivo esclavo.");
}

void command_us() {
  if (!handshakeEnviado) {
    if (!establecerComunicacion()) { Serial.println("Ha ocurrido un fallo al establecer la comunicación"); return; }
  }
  Serial1.write(US); //Comando us
  Serial.println("Enviando us...");
}

int establecerComunicacion() {
    if (!handshakeEnviado) { 
    sendHandshake(); 
    handshakeEnviado = 1; 
    Serial.println("Estableciendo comunicación...");
    uint32_t last_ms=millis(); 
    while(millis()-last_ms<pseudo_period_ms) {
      if (Serial1.available()>0) {
        uint8_t data=Serial1.read();
        if (data != ACK) { Serial.println("Error al enviar el comando"); return 0; }
        else return 1;
      }
    }
  }
}

void command_one_shot(String *words) {
  if (!handshakeEnviado) {
    if (!establecerComunicacion()) { Serial.println("Ha ocurrido un fallo al establecer la comunicación"); return; }
  }
  Serial1.write(ONE_SHOT); //Comando one-shot
  unsigned long address = strtoul(words[1].c_str(), NULL, 16);
  Serial1.write(address);
  Serial.println("Enviando one-shot...");
}

void command_on_period(String *words, uint8_t *bytes, uint8_t byteArraySize) {
  if (!handshakeEnviado) {
    if (!establecerComunicacion()) { Serial.println("Ha ocurrido un fallo al establecer la comunicación"); return; }
  }
  Serial1.write(ON + byteArraySize); //Comando on
  unsigned long address = strtoul(words[1].c_str(), NULL, 16);
  Serial1.write(address);
  for (int i=0; i<byteArraySize; i++) Serial1.write(bytes[i]);
  Serial.println("Enviando on period...");
}

void command_off(String *words) {
  if (!handshakeEnviado) {
    if (!establecerComunicacion()) { Serial.println("Ha ocurrido un fallo al establecer la comunicación"); return; }
  }
  Serial1.write(OFF); //Comando off
  unsigned long address = strtoul(words[1].c_str(), NULL, 16);
  Serial1.write(address);
  Serial.println("Apagando sensor...");
  return;
}

void command_unit(String *words) {
  if (!handshakeEnviado) {
    if (!establecerComunicacion()) { Serial.println("Ha ocurrido un fallo al establecer la comunicación"); return; }
  }
  if (words[3] == "cm") Serial1.write(UNIT_CM);
      else if (words[3] == "inc") Serial1.write(UNIT_INC);
      else if (words[3] == "ms") Serial1.write(UNIT_MS);
      else {
        Serial.println("Unidad no reconocida. Use 'help' para ver una lista detallada de los comandos.");
        return;
  }
  unsigned long address = strtoul(words[1].c_str(), NULL, 16);
  Serial1.write(address);
  Serial.println("Cambiando de unidades...");
}

void command_delay(String *words, uint8_t *bytes, uint8_t byteArraySize) {
  if (!handshakeEnviado) {
    if (!establecerComunicacion()) { Serial.println("Ha ocurrido un fallo al establecer la comunicación"); return; }
  }
  Serial1.write(DELAY + byteArraySize); //Comando on
  unsigned long address = strtoul(words[1].c_str(), NULL, 16);
  Serial1.write(address);
  for (int i=0; i<byteArraySize; i++) Serial1.write(bytes[i]);
  Serial.println("Cambiando delay...");
  return;
}

void command_status(String *words) {
  if (!handshakeEnviado) {
    if (!establecerComunicacion()) { Serial.println("Ha ocurrido un fallo al establecer la comunicación"); return; }
  }
  Serial1.write(STATUS); //Comando status
  unsigned long address = strtoul(words[1].c_str(), NULL, 16);
  Serial1.write(address);
  Serial.println("Enviando status...");
  return;
}

void command_oled_on() {
  if (!handshakeEnviado) {
    if (!establecerComunicacion()) { Serial.println("Ha ocurrido un fallo al establecer la comunicación"); return; }
  }
  Serial1.write(OLED_ON); //Comando oled on
  Serial.println("Encendiendo pantalla OLED...");
  return;
}

void command_oled_off() {
  if (!handshakeEnviado) {
    if (!establecerComunicacion()) { Serial.println("Ha ocurrido un fallo al establecer la comunicación"); return; }
  }
  Serial1.write(OLED_OFF); //Comando oled on
  Serial.println("Apagando pantalla OLED...");
  return;
}

void command_oled_show() {
  if (!handshakeEnviado) {
    if (!establecerComunicacion()) { Serial.println("Ha ocurrido un fallo al establecer la comunicación"); return; }
  }
  Serial1.write(OLED_SHOW_LAST_COMMAND); //Comando oled on
  Serial.println("Mostrando último comando en la pantalla OLED...");
  return;
}

int validateSRF02Dir(String word) {
  if (word.indexOf("0") != 0 || word.indexOf("x") != 1 || word.length() != 4) return 0;
  return 1;
}

void splitBytes(int period, uint8_t *byteArray, uint8_t byteArraySize) {
  for (int i = 0; i < byteArraySize; i++) {
    byteArray[i] = (period >> (8 * i)) & 0xFF;
  }
}

void validateCommand(String *words, int wordCount) {
  if (!validateSRF02Dir(words[1])) { Serial.println("Error: Dirección del sensor incorrecta. Es de la forma 0x.."); return; }

  if (words[2] == "one-shot") { command_one_shot(words); return; }

  else if (words[2] == "on") {
      int period = words[3].toInt();
      if (period == 0) { Serial.println("El periodo no es correcto."); return; } 
      if (period>10000) { Serial.println("El periodo máximo es de 10.000ms"); return; }

      float log2Numero = log(period) / log(2); 
      uint8_t byteArraySize = (int)ceil(log2Numero/8);

      uint8_t bytes[byteArraySize];
      splitBytes(period, bytes, byteArraySize);
      command_on_period(words, bytes, byteArraySize);
      return;
  }
  else if (words[2] == "off") command_off(words);

  else if (words[2] == "unit") command_unit(words);
  else if (words[2] == "delay") {
      int delay = words[3].toInt();
      if (delay == 0) { Serial.println("El delay no es correcto."); return; } 
      if (delay>10000) { Serial.println("El delay máximo es de 10000ms."); return; }

      float log2Numero = log(delay) / log(2); 
      uint8_t byteArraySize = (int)ceil(log2Numero/8);

      uint8_t bytes[byteArraySize];
      splitBytes(delay, bytes, byteArraySize);
      command_delay(words, bytes, byteArraySize);
      return;
  }
  else if (words[2] == "status") command_status(words);

  else unrecognizedCommand();
}

void splitCommand(String command) {
  String words[MAX_WORDS];
  int wordCount = 0;

  int startIndex = 0;
  int spaceIndex = 0;

  while ((spaceIndex = command.indexOf(' ', startIndex)) != -1 && wordCount < MAX_WORDS) {
    words[wordCount++] = command.substring(startIndex, spaceIndex);
    startIndex = spaceIndex + 1;
  }

  if (startIndex < command.length() && wordCount < MAX_WORDS) {
    words[wordCount++] = command.substring(startIndex);
  }

  validateCommand(words, wordCount);
}

void us_commandFamily(String command) {
  if (command.length() == 2) command_us();
  else splitCommand(command);
}

void unrecognizedCommand() {
  Serial.println("Error: Comando no reconocido. Use 'help' para ver una lista de los comandos y su sintaxis.");
}

void command_exit() {
  Serial.println("Desconectando...");
  Serial1.write(EXIT);
  handshakeEnviado = 0;
}

void oled_commandFamily(String command) {
  int indexOfSpace = command.indexOf(" ");
  if (indexOfSpace<0) return;
  String comType = command.substring(indexOfSpace + 1);
  if (comType == "on") command_oled_on();
  else if (comType == "off") command_oled_off();
  else if (comType == "show") command_oled_show();
  else unrecognizedCommand();
}

void checkFirstWord(String command) {
  command.trim();
  int index = command.indexOf(" ");
  String firstWord = index == -1 ? command : command.substring(0, index);

  if (firstWord == "help") command_help();
  else if (firstWord == "us") us_commandFamily(command);
  else if (firstWord == "oled") oled_commandFamily(command);
  else if (firstWord == "exit") command_exit();
  else unrecognizedCommand();
}

void readCommand() {
  String inputCommand = Serial.readStringUntil('\n');
  inputCommand.trim();
  if (inputCommand.length() == 0) {
    Serial.println("Error: Comando vacío. Use 'help' para ver una lista de los comandos y su sintaxis.");
    return;
  }

  checkFirstWord(inputCommand);
}

void manageErrors(uint8_t response) {
  switch (response) {
    case 0x01:
      Serial.println("Dirección del dispositivo de ultrasonidos no encontrada. Use 'us' para obtener información sobre los dispositivos.");
      break;
    case 0x02:
      Serial.println("El periodo introducido no es válido.");
      break;
    case 0x06:
      Serial.println("Error fatal. Cerrando conexión.");
      exit(1);
    case 0x07:
      Serial.println("TIMEOUT");
      break;
  }
}

uint8_t timeout = 1;
void readData() {  
  
  //0xFx si es correcto, 0x0x si es error
  uint8_t responseHeader = Serial1.read();
  uint8_t responseType = responseHeader>>4;
  if (responseType == 0) { manageErrors(responseHeader); return; }

  uint8_t numElements = responseHeader & 0x0F;
  uint8_t values[numElements];
  uint32_t time = millis();

  uint8_t indice = 0;
  while (indice < (numElements-1) && (millis() - time < pseudo_period_ms)) {
    if (Serial1.available()>0) { 
      uint8_t byte = Serial1.read();
      values[indice++] = byte;
      timeout = 0;
    }
  }
  if (timeout) {Serial.println("TIMEOUT"); return; }
  if (responseType == 0x0F) formatResponseMedida(numElements, values);
  else if (responseType == 0x0E) formatResponseStatus(numElements, values);
  else if (responseType == 0x0D) formatResponseUS(numElements, values);
}

String getUnit(uint8_t byteUnit) {
  if (byteUnit == 80) return "inc";
  if (byteUnit == 81) return "cm";
  if (byteUnit == 82) return "ms";
}

void formatResponseStatus(uint8_t numElements, uint8_t* values) {
    Serial.print("Sensor: ");
    Serial.println(String(values[0], HEX));
    Serial.print("Unidades: ");
    Serial.println(getUnit(values[1]));
    Serial.print("Estado: ");
    Serial.println(values[2]);
    Serial.print("Delay mínimo: ");
    int delayMinimo = 0;
    for (int i=3; i<(numElements-1); i++) delayMinimo += values[i]<<8*(i-3); 
    Serial.println(delayMinimo);
}

void formatResponseUS(uint8_t numElements, uint8_t* values) {
    for (int i=0; i<(numElements-1); i++) {
      String message = "Sensor "+ String(i+1) + " con dirección: " + String(values[i], HEX);
      Serial.println(message);
    }
}

void formatResponseMedida(uint8_t numElements, uint8_t* values) {
  int medida = 0;
  for (int i=2; i<(numElements-1); i++) medida += values[i]<<8*(i-2); 
  Serial.print("Sensor "); Serial.print(String(values[0], HEX)); Serial.print(": "); Serial.print(medida); Serial.println(getUnit(values[1]));
}

void loop() {
  if (Serial.available() > 0) readCommand();

  if (Serial1.available() > 0) readData();
}

