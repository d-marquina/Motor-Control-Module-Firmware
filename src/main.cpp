/* espressif32 @ 6.7.0 */
#include <Arduino.h>
#include "electricui.h"
#include "AS5600.h"

//=================================================
// Declaración de Funciones
//=================================================
void serial_rx_handler();
void serial_write(uint8_t *data, uint16_t len);
void control_loop_task(void *pvParameters);

//=================================================
// Declaración de Estructuras
//=================================================
typedef struct
{
  uint32_t timestamp;
  float data;
} sensor_timeseries;

typedef struct
{
  uint8_t begin_flag;
  uint8_t end_flag;
  uint32_t trigger_signal;
  uint32_t refresh_rate;
} settings_struct;

//=================================================
// Variables
//=================================================
uint8_t blink_enable = 1;
uint8_t led_state = 0;
uint16_t glow_time = 200;
uint32_t led_timer = 0;//*/
char device_id[] = "MCM_00";

uint16_t angulo = 0;
sensor_timeseries angle_sensor = {0, 0};

uint8_t en_pin = 27;
uint8_t dir_pin = 26;
uint8_t pul_pin = 25;
const int ledc_channel = 0;
bool mot_dir = false;
int16_t mot_speed = 0;
int16_t mot_abs_speed = 0;
uint8_t test_flag = 0;
uint8_t old_test_flag = 0;

settings_struct module_settings = {false, true, 0, 10};
uint8_t test_cl_enable = false;
uint32_t tracker_loop = 0;

//=================================================
// Objetos
//=================================================
eui_interface_t serial_comms = EUI_INTERFACE(&serial_write);
eui_message_t tracked_variables[] =
    {
        EUI_UINT8("led_blink", blink_enable),
        EUI_UINT8("led_state", led_state),
        EUI_UINT16("lit_time", glow_time),//*/
        EUI_UINT16("MCM_angle", angulo),
        EUI_UINT8("MCM_test_flag", test_flag),
        EUI_INT16("MCM_motor_speed", mot_speed),
        EUI_CUSTOM_RO("angle_sensor", angle_sensor),
        EUI_CUSTOM("settings", module_settings),
};

AS5600 as5600;

TaskHandle_t control_loop_task_handle;

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

  xTaskCreatePinnedToCore(
      control_loop_task,         /* Task function. */
      "ControlLoopTask",         /* name of task. */
      10000,                     /* Stack size of task */
      NULL,                      /* parameter of the task */
      1,                         /* priority of the task */
      &control_loop_task_handle, /* Task handle to keep track of created task */
      0);                        /* pin task to core 0 */

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
  digitalWrite(LED_BUILTIN, led_state);//*/

  if (module_settings.begin_flag)
  {
    module_settings.begin_flag = false;
    module_settings.end_flag = false;
    test_cl_enable = true;
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

void control_loop_task(void *pvParameters)
{
  const TickType_t taskPeriod = 10; // ms
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    // Control Loop here
    angulo = as5600.readAngle();
    angle_sensor.timestamp = millis();
    angle_sensor.data = as5600.readAngle() * 0.088; // To degrees

    if (test_cl_enable){
      //my_sin_signal.data = sin_signal;
      module_settings.trigger_signal += 1;

      if (module_settings.trigger_signal >= module_settings.refresh_rate)
      {
        module_settings.trigger_signal = 0;
      }

      tracker_loop += 1;
      if (tracker_loop >= 900)
      {
        tracker_loop = 0;
        module_settings.end_flag = true;
        test_cl_enable = false;
      }
    }

    if (test_flag > 0)
    {
      ledcWriteTone(ledc_channel, mot_speed);
    }
    else
    {
      ledcWriteTone(ledc_channel, 0);
    }

    eui_send_tracked("angle_sensor");
    eui_send_tracked("settings");

    vTaskDelayUntil(&xLastWakeTime, taskPeriod);
  }
}