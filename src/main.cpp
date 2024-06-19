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
uint16_t glow_time = 1000;
uint32_t led_timer = 0; //*/
char device_id[] = "MCM_00";

float MCM_angle = 0;
sensor_timeseries angle_sensor = {0, 0};

uint8_t en_pin = 27;
uint8_t dir_pin = 26;
uint8_t pul_pin = 25;
const int ledc_channel = 0;
uint8_t MCM_en_mot = 0;
uint8_t old_MCM_en_mot = 0;
bool mot_dir = false;
int16_t MCM_mot_sp = 0;
int16_t mot_speed = 0;
int16_t mot_abs_speed = 0;

settings_struct module_settings = {false, true, 0, 10};
uint8_t MCM_pid_mode = false;
uint8_t old_MCM_pid_mode = false;
float MCM_set_pt = 180;
uint32_t tracker_loop = 0;

//=================================================
// Objetos
//=================================================
eui_interface_t serial_comms = EUI_INTERFACE(&serial_write);
eui_message_t tracked_variables[] =
    {
        EUI_UINT8("led_blink", blink_enable),
        EUI_UINT8("led_state", led_state),
        EUI_UINT16("lit_time", glow_time), //*/
        EUI_FLOAT("MCM_angle", MCM_angle),
        EUI_UINT8("MCM_en_mot", MCM_en_mot),
        EUI_INT16("MCM_mot_sp", MCM_mot_sp),
        EUI_UINT8("MCM_pid_mode", MCM_pid_mode),
        EUI_FLOAT("MCM_set_pt", MCM_set_pt),
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
  digitalWrite(en_pin, HIGH);  // Disabled by default
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
  digitalWrite(LED_BUILTIN, led_state); //*/

  if (MCM_en_mot != old_MCM_en_mot)
  {
    if (MCM_en_mot == 1)
    {
      digitalWrite(en_pin, LOW); // Enabled with LOW
    }
    else if (MCM_en_mot == 0)
    {
      digitalWrite(en_pin, HIGH); // Disabled with HIGH
      MCM_mot_sp = 0;
      ledcWriteTone(ledc_channel, MCM_mot_sp);
      eui_send_tracked("MCM_mot_sp");
    }
    old_MCM_en_mot = MCM_en_mot;
  }

  if (MCM_pid_mode != old_MCM_pid_mode)
  {
    // Reset Motor
    MCM_mot_sp = 0;
    ledcWriteTone(ledc_channel, MCM_mot_sp);
    eui_send_tracked("MCM_mot_sp");
    // Reset Set Point
    MCM_set_pt = 180;
    eui_send_tracked("MCM_set_pt");
    // Reset recording data
    module_settings.begin_flag = false;
    module_settings.end_flag = true;
    module_settings.trigger_signal = 0;
    // Update backup variable
    old_MCM_pid_mode = MCM_pid_mode;
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
    MCM_angle = as5600.readAngle() * 0.088; // To degrees
    angle_sensor.timestamp = millis();
    angle_sensor.data = as5600.readAngle() * 0.088; // To degrees

    if (MCM_pid_mode)
    {
      //--------------------
      // PID code here
      //--------------------

      if (module_settings.begin_flag)
      {
        module_settings.trigger_signal += 1;
        if (module_settings.trigger_signal >= module_settings.refresh_rate)
        {
          module_settings.trigger_signal = 0;
        }

        tracker_loop += 10; // Ts in ms, same as taskPeriod
        if (tracker_loop >= 10000)
        {
          tracker_loop = 0;
          module_settings.begin_flag = false;
          module_settings.end_flag = true;
        }
      }
    }
    else
    {
      ledcWriteTone(ledc_channel, MCM_mot_sp);
    }

    eui_send_tracked("angle_sensor");
    eui_send_tracked("settings");

    vTaskDelayUntil(&xLastWakeTime, taskPeriod);
  }
}