#include <Arduino.h>

constexpr uint32_t UART_BAUD = 115200;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);  // USB debug
  Serial1.begin(UART_BAUD); // Teensy RX1=pin 0, TX1=pin 1

  delay(300);
  Serial.println("[ECHO] Teensy UART echo pret");
  Serial1.println("ECHO_READY");
}

void loop()
{
  while (Serial1.available() > 0)
  {
    int incoming = Serial1.read();
    if (incoming < 0)
    {
      break;
    }

    char c = static_cast<char>(incoming);
    Serial1.write(c);    // echo vers RPi
    Serial.write(c);     // trace USB

    digitalWrite(LED_BUILTIN, HIGH);
    delay(5);
    digitalWrite(LED_BUILTIN, LOW);
  }

  while (Serial.available() > 0)
  {
    int incoming = Serial.read();
    if (incoming < 0)
    {
      break;
    }
    Serial1.write(static_cast<char>(incoming));
  }
}
