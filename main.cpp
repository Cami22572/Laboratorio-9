// Pines para los potenciómetros
const int pot1Pin = 34;  // Potenciómetro 1
const int pot2Pin = 35;  // Potenciómetro 2

// Pines para los botones
const int button1Pin = 4;  // Botón 1
const int button2Pin = 5;  // Botón 2

volatile bool readPot1 = false;  // Variables para controlar la lectura
volatile bool readPot2 = false;

void IRAM_ATTR onButton1Press() {
  readPot1 = true;  // Señal de leer el potenciómetro 1
}

void IRAM_ATTR onButton2Press() {
  readPot2 = true;  // Señal de leer el potenciómetro 2
}

void setup() {
  Serial.begin(115200);  // Inicializar UART para comunicación

  // Configuración de pines de botones
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);

  // Adjuntar interrupciones a los botones
  attachInterrupt(digitalPinToInterrupt(button1Pin), onButton1Press, FALLING);
  attachInterrupt(digitalPinToInterrupt(button2Pin), onButton2Press, FALLING);

  // Configuración de pines ADC
  pinMode(pot1Pin, INPUT);
  pinMode(pot2Pin, INPUT);
}

void loop() {
  if (readPot1) {
    int pot1Value = analogRead(pot1Pin);  // Leer potenciómetro 1
    Serial.write((pot1Value >> 8) & 0xFF);  // Enviar byte alto
    Serial.write(pot1Value & 0xFF);  // Enviar byte bajo
    readPot1 = false;  // Reiniciar bandera
  }

  if (readPot2) {
    int pot2Value = analogRead(pot2Pin);  // Leer potenciómetro 2
    Serial.write((pot2Value >> 8) & 0xFF);  // Enviar byte alto
    Serial.write(pot2Value & 0xFF);  // Enviar byte bajo
    readPot2 = false;  // Reiniciar bandera
  }

  delay(100);  // Pequeño retraso para evitar saturación
}
