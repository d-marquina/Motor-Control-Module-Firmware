#include <Arduino.h>
#include "electricui.h"
#include "AS5600.h"

//=================================================
// Declaración de Funciones
//=================================================
void serial_rx_handler();
void serial_write(uint8_t *data, uint16_t len);

//=================================================
// Variables
//=================================================
uint8_t blink_enable = 1;
uint8_t led_state = 0;
uint16_t glow_time = 200;
uint32_t led_timer = 0;
char device_id[] = "MCM_00";

uint16_t angulo = 0;

//=================================================
// Objetos
//=================================================
eui_interface_t serial_comms = EUI_INTERFACE(&serial_write);
eui_message_t tracked_variables[] =
    {
        EUI_UINT8("led_blink", blink_enable),
        EUI_UINT8("led_state", led_state),
        EUI_UINT16("lit_time", glow_time),
        EUI_UINT16("MCM_angle", angulo),
};

AS5600 as5600;

//=================================================
// Inicialización
//=================================================
void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  eui_setup_interface(&serial_comms);
  EUI_TRACK(tracked_variables);
  eui_setup_identifier(device_id, 24);

  Wire.begin(17, 16); // SDA, SCL
  as5600.begin(4);
  as5600.setDirection(AS5600_CLOCK_WISE);

  led_timer = millis();
}

//=================================================
// Lazo principal
//=================================================
void loop()
{
  serial_rx_handler(); // check for new inbound data
  if (blink_enable)
  {
    // Check if the LED has been on for the configured duration
    if (millis() - led_timer >= glow_time)
    {
      led_state = !led_state; // invert led state
      led_timer = millis();
    }
  }
  digitalWrite(LED_BUILTIN, led_state);

  angulo = as5600.readAngle();
}

//=================================================
// Definición de Funciones
//=================================================
void serial_rx_handler()
{
  // While we have data, we will pass those bytes to the ElectricUI parser
  while (Serial.available() > 0)
  {
    eui_parse(Serial.read(), &serial_comms); // Ingest a byte
  }
}

void serial_write(uint8_t *data, uint16_t len)
{
  Serial.write(data, len);
}