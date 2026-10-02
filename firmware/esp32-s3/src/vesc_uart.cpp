#include "vesc_uart.h"

#include <Arduino.h>
#include <VescUart.h>

#include <new>
#include <string.h>

#include "esp32_config.h"
#include "esp_log.h"

#define VESC_TASK_STACK_SIZE 4096
#define VESC_TASK_PRIORITY 6

static const char *TAG = "vesc_uart";

static bool rpm_has_opposite_direction(int32_t current, int32_t requested)
{
    return (current > 0 && requested < 0) || (current < 0 && requested > 0);
}

static void store_telemetry(vesc_uart_t *vesc, const VescUart &driver)
{
    xSemaphoreTake(vesc->lock, portMAX_DELAY);
    vesc->measured_rpm = driver.data.rpm;
    vesc->motor_current = driver.data.avgMotorCurrent;
    vesc->input_current = driver.data.avgInputCurrent;
    vesc->duty_cycle = driver.data.dutyCycleNow;
    vesc->input_voltage = driver.data.inpVoltage;
    vesc->amp_hours = driver.data.ampHours;
    vesc->amp_hours_charged = driver.data.ampHoursCharged;
    vesc->watt_hours = driver.data.wattHours;
    vesc->watt_hours_charged = driver.data.wattHoursCharged;
    vesc->temp_mosfet = driver.data.tempMosfet;
    vesc->temp_motor = driver.data.tempMotor;
    vesc->pid_position = driver.data.pidPos;
    vesc->tachometer = (int32_t)driver.data.tachometer;
    vesc->tachometer_abs = (int32_t)driver.data.tachometerAbs;
    vesc->controller_id = driver.data.id;
    vesc->fault_code = (uint8_t)driver.data.error;
    vesc->last_telemetry_tick = xTaskGetTickCount();
    vesc->has_received_telemetry = true;
    if (++vesc->telemetry_sequence == 0) {
        ++vesc->telemetry_sequence;
    }
    xSemaphoreGive(vesc->lock);
}

static void vesc_control_task(void *arg)
{
    vesc_uart_t *vesc = static_cast<vesc_uart_t *>(arg);
    VescUart *driver = static_cast<VescUart *>(vesc->driver);
    const TickType_t command_timeout = pdMS_TO_TICKS(VESC_COMMAND_TIMEOUT_MS);
    const TickType_t neutral_time = pdMS_TO_TICKS(VESC_DIRECTION_CHANGE_NEUTRAL_MS);
    const TickType_t command_keepalive = pdMS_TO_TICKS(VESC_COMMAND_KEEPALIVE_MS);
    const TickType_t telemetry_interval = pdMS_TO_TICKS(VESC_TELEMETRY_INTERVAL_MS);
    TickType_t last_command_send = 0;
    TickType_t last_telemetry_request = xTaskGetTickCount();
    int32_t last_sent_rpm = 0;
    bool last_sent_brake = true;
    bool has_sent_command = false;

    while (true) {
        TickType_t now = xTaskGetTickCount();
        int32_t output_rpm = 0;

        /* Read fault telemetry before deciding the next actuator command. This
           prevents one stale RPM keepalive from being sent after a newly
           reported undervoltage or other controller fault. */
        if (now - last_telemetry_request >= telemetry_interval) {
            last_telemetry_request = now;
            if (driver->getVescValues()) {
                store_telemetry(vesc, *driver);
            }
        }

        xSemaphoreTake(vesc->lock, portMAX_DELAY);

        bool command_fresh = vesc->has_received_command &&
                             (now - vesc->last_command_tick <= command_timeout);
        /* A VESC fault is an actuator-level safety condition. Do not rely on
           the Jetson control loop to notice it before removing torque: brake
           locally as soon as telemetry reports any nonzero fault. */
        bool brake_active = !command_fresh || vesc->brake_requested ||
                            (vesc->has_received_telemetry && vesc->fault_code != 0);
        int32_t target_rpm = brake_active ? 0 : vesc->requested_rpm;

        if (target_rpm == 0) {
            vesc->active_rpm = 0;
            vesc->direction_change_pending = false;
        } else if (vesc->direction_change_pending) {
            vesc->active_rpm = 0;
            if (now - vesc->direction_change_tick >= neutral_time) {
                vesc->direction_change_pending = false;
                vesc->active_rpm = target_rpm;
            }
        } else if (rpm_has_opposite_direction(vesc->active_rpm, target_rpm)) {
            vesc->active_rpm = 0;
            vesc->direction_change_pending = true;
            vesc->direction_change_tick = now;
            ESP_LOGI(TAG, "Neutral pause before direction change");
        } else {
            vesc->active_rpm = target_rpm;
        }

        output_rpm = vesc->active_rpm;
        xSemaphoreGive(vesc->lock);

        bool output_changed = !has_sent_command ||
                              brake_active != last_sent_brake ||
                              (!brake_active && output_rpm != last_sent_rpm);
        if (output_changed || now - last_command_send >= command_keepalive) {
            if (brake_active) {
                // VESC_BRAKE_CURRENT_A was never defined in any commit (deployed value
                // unknown); hold zero speed like the pre-handoff firmware did.
                driver->setRPM(0.0f);
            } else {
                driver->setRPM((float)output_rpm);
            }
            last_sent_brake = brake_active;
            last_sent_rpm = output_rpm;
            last_command_send = xTaskGetTickCount();
            has_sent_command = true;
        }

        vTaskDelay(pdMS_TO_TICKS(VESC_SEND_INTERVAL_MS));
    }
}

extern "C" esp_err_t vesc_uart_init(vesc_uart_t *vesc)
{
    if (vesc == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(vesc, 0, sizeof(*vesc));
    vesc->lock = xSemaphoreCreateMutex();
    if (vesc->lock == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    initArduino();

    HardwareSerial *serial = new (std::nothrow) HardwareSerial((uint8_t)VESC_UART_PORT);
    VescUart *driver = new (std::nothrow) VescUart(VESC_TELEMETRY_TIMEOUT_MS);
    if (serial == nullptr || driver == nullptr) {
        delete serial;
        delete driver;
        vSemaphoreDelete(vesc->lock);
        vesc->lock = nullptr;
        return ESP_ERR_NO_MEM;
    }

    serial->begin(VESC_UART_BAUD_RATE, SERIAL_8N1, VESC_UART_RX_IO, VESC_UART_TX_IO);
    driver->setSerialPort(serial);
    vesc->serial_port = serial;
    vesc->driver = driver;

    driver->setRPM(0.0f);
    vTaskDelay(pdMS_TO_TICKS(5));
    driver->setRPM(0.0f);

    if (xTaskCreate(vesc_control_task,
                    "vesc_control",
                    VESC_TASK_STACK_SIZE,
                    vesc,
                    VESC_TASK_PRIORITY,
                    &vesc->task) != pdPASS) {
        serial->end();
        delete driver;
        delete serial;
        vesc->driver = nullptr;
        vesc->serial_port = nullptr;
        vSemaphoreDelete(vesc->lock);
        vesc->lock = nullptr;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG,
             "SolidGeek/VescUart ready on UART%d TX=%d RX=%d at %d baud",
             (int)VESC_UART_PORT,
             VESC_UART_TX_IO,
             VESC_UART_RX_IO,
             VESC_UART_BAUD_RATE);
    return ESP_OK;
}

extern "C" esp_err_t vesc_uart_set_drive(vesc_uart_t *vesc, int32_t rpm, bool brake)
{
    if (vesc == nullptr || vesc->lock == nullptr ||
        rpm < -VESC_MAX_ABS_RPM || rpm > VESC_MAX_ABS_RPM) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(vesc->lock, portMAX_DELAY);
    vesc->requested_rpm = brake ? 0 : rpm;
    vesc->brake_requested = brake;
    vesc->last_command_tick = xTaskGetTickCount();
    vesc->has_received_command = true;
    if (rpm == 0) {
        vesc->active_rpm = 0;
        vesc->direction_change_pending = false;
    }
    xSemaphoreGive(vesc->lock);
    return ESP_OK;
}

extern "C" esp_err_t vesc_uart_set_target_rpm(vesc_uart_t *vesc, int32_t rpm)
{
    return vesc_uart_set_drive(vesc, rpm, false);
}

extern "C" esp_err_t vesc_uart_get_snapshot(vesc_uart_t *vesc, vesc_uart_snapshot_t *snapshot)
{
    if (vesc == nullptr || vesc->lock == nullptr || snapshot == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(vesc->lock, portMAX_DELAY);
    TickType_t now = xTaskGetTickCount();
    snapshot->requested_rpm = vesc->requested_rpm;
    snapshot->active_rpm = vesc->active_rpm;
    snapshot->command_fresh = vesc->has_received_command &&
                              (now - vesc->last_command_tick <= pdMS_TO_TICKS(VESC_COMMAND_TIMEOUT_MS));
    snapshot->direction_change_pending = vesc->direction_change_pending;
    snapshot->brake_active = !snapshot->command_fresh || vesc->brake_requested ||
                             (vesc->has_received_telemetry && vesc->fault_code != 0);
    snapshot->telemetry_fresh = vesc->has_received_telemetry &&
                                (now - vesc->last_telemetry_tick <= pdMS_TO_TICKS(VESC_TELEMETRY_STALE_MS));
    snapshot->telemetry_sequence = vesc->telemetry_sequence;
    if (vesc->has_received_telemetry) {
        uint64_t age_ms = (uint64_t)(now - vesc->last_telemetry_tick) * 1000ULL /
                          (uint64_t)configTICK_RATE_HZ;
        snapshot->telemetry_age_ms = age_ms > UINT32_MAX ? UINT32_MAX : (uint32_t)age_ms;
    } else {
        snapshot->telemetry_age_ms = UINT32_MAX;
    }
    snapshot->measured_rpm = vesc->measured_rpm;
    snapshot->motor_current = vesc->motor_current;
    snapshot->input_current = vesc->input_current;
    snapshot->duty_cycle = vesc->duty_cycle;
    snapshot->input_voltage = vesc->input_voltage;
    snapshot->amp_hours = vesc->amp_hours;
    snapshot->amp_hours_charged = vesc->amp_hours_charged;
    snapshot->watt_hours = vesc->watt_hours;
    snapshot->watt_hours_charged = vesc->watt_hours_charged;
    snapshot->temp_mosfet = vesc->temp_mosfet;
    snapshot->temp_motor = vesc->temp_motor;
    snapshot->pid_position = vesc->pid_position;
    snapshot->tachometer = vesc->tachometer;
    snapshot->tachometer_abs = vesc->tachometer_abs;
    snapshot->controller_id = vesc->controller_id;
    snapshot->fault_code = vesc->fault_code;
    xSemaphoreGive(vesc->lock);
    return ESP_OK;
}
