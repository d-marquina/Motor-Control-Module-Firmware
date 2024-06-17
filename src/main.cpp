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

uint8_t en_pin = 27;
uint8_t dir_pin = 26;
uint8_t pul_pin = 25;
const int ledc_channel = 0;
bool mot_dir = false;
int16_t mot_speed = 0;
int16_t mot_abs_speed = 0;
uint8_t test_flag = 0;
uint8_t old_test_flag = 0;

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
        EUI_UINT8("MCM_test_flag", test_flag),
        EUI_INT16("MCM_motor_speed", mot_speed),
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

  pinMode(en_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(pul_pin, OUTPUT);
  digitalWrite(en_pin, LOW);   // Enabled by default
  digitalWrite(dir_pin, HIGH); // Depends on connection
  ledcSetup(ledc_channel, 0, 8);
  ledcAttachPin(pul_pin, ledc_channel);

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

  if (test_flag != old_test_flag)
  {
    if (test_flag > 0)
    {
      ledcWriteTone(ledc_channel, mot_speed);
    }
    else
    {
      ledcWriteTone(ledc_channel, 0);
    }
    old_test_flag = test_flag;
  }
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