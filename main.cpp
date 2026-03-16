#include <cstdio>    //includes standard i/o functions   
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"   //includes the LEDC PWM hardware driver
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

// CONSTANTS 
static constexpr uint8_t MOTOR_COUNT = 3;
static constexpr uint32_t PWM_FREQ_HZ = 20000;   //20 KHz
static constexpr ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_12_BIT;
static constexpr uint32_t MAX_DUTY = 4095;      // 2^12 - 1
static constexpr const char* TAG = "MOTOR_DRIVER";

// MOTOR PINS (Cytron Drivers) 
struct MotorPins {
    static constexpr gpio_num_t MOTOR1_PWM = GPIO_NUM_13;
    static constexpr gpio_num_t MOTOR1_DIR = GPIO_NUM_12;
    static constexpr gpio_num_t MOTOR2_PWM = GPIO_NUM_14;
    static constexpr gpio_num_t MOTOR2_DIR = GPIO_NUM_27;
    static constexpr gpio_num_t MOTOR3_PWM = GPIO_NUM_26;
    static constexpr gpio_num_t MOTOR3_DIR = GPIO_NUM_25;
};

// PWM CONFIGURATION 
struct PWMConfig {        //defines a c++ structure that groups the related constants 
    static constexpr ledc_timer_t TIMER = LEDC_TIMER_0;
    static constexpr ledc_mode_t SPEED_MODE = LEDC_LOW_SPEED_MODE;
    static constexpr ledc_channel_t CHANNEL1 = LEDC_CHANNEL_0;
    static constexpr ledc_channel_t CHANNEL2 = LEDC_CHANNEL_1;
    static constexpr ledc_channel_t CHANNEL3 = LEDC_CHANNEL_2;
};

// MOTOR CLASS 
class CytronMotor {
private:
    const gpio_num_t pwm_gpio;
    const gpio_num_t dir_gpio;
    const ledc_channel_t pwm_channel;
    float current_speed;
    
public:
    // Constructor
    CytronMotor(gpio_num_t pwm, gpio_num_t dir, ledc_channel_t channel) 
        : pwm_gpio(pwm), dir_gpio(dir), pwm_channel(channel), current_speed(0.0f) {}
    
    // Initialize the motor (PWM and direction pins)
    esp_err_t init() {
        esp_err_t err;
        
        // Configure PWM channel
        ledc_channel_config_t channel_conf = {
            .gpio_num = static_cast<int>(pwm_gpio),
            .speed_mode = PWMConfig::SPEED_MODE,
            .channel = pwm_channel,
            .timer_sel = PWMConfig::TIMER,
            .duty = 0,
            .hpoint = 0,
            .flags = {.output_invert = 0}
        };
        
        err = ledc_channel_config(&channel_conf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure PWM channel for GPIO %d", pwm_gpio);
            return err;
        }
        
        // Configure direction pin
        gpio_config_t dir_conf = {
            .pin_bit_mask = 1ULL << dir_gpio,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        
        err = gpio_config(&dir_conf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure direction pin %d", dir_gpio);
            return err;
        }
        
        // Initialize direction to LOW (stop)
        gpio_set_level(dir_gpio, 0);
        
        ESP_LOGI(TAG, "Motor initialized: PWM=%d, DIR=%d, Channel=%d", 
                 pwm_gpio, dir_gpio, pwm_channel);
        
        return ESP_OK;
    }
    
    // Set motor speed (-1.0 to 1.0)
    esp_err_t setSpeed(float speed) {
        // Clamp speed
        if (speed > 1.0f) speed = 1.0f;
        if (speed < -1.0f) speed = -1.0f;
        
        current_speed = speed;
        
        // Set direction
        gpio_set_level(dir_gpio, (speed >= 0) ? 1 : 0);
        
        // Calculate and set PWM duty
        uint32_t duty = static_cast<uint32_t>((speed >= 0 ? speed : -speed) * MAX_DUTY);
        
        esp_err_t err = ledc_set_duty(PWMConfig::SPEED_MODE, pwm_channel, duty);
        if (err != ESP_OK) return err;
        
        err = ledc_update_duty(PWMConfig::SPEED_MODE, pwm_channel);
        if (err != ESP_OK) return err;
        
        ESP_LOGD(TAG, "Motor ch%d: speed=%.2f, duty=%lu", pwm_channel, speed, duty);
        
        return ESP_OK;
    }
    
    // Stop motor
    esp_err_t stop() {
        return setSpeed(0.0f);
    }
    
    // Get current speed
    float getSpeed() const {
        return current_speed;
    }
    
    // Get PWM channel
    ledc_channel_t getChannel() const {
        return pwm_channel;
    }
};

// MOTOR CONTROLLER CLASS 
class MotorController {
private:
    static constexpr size_t MAX_MOTORS = 3;
    CytronMotor* motors[MAX_MOTORS];
    size_t motor_count;
    
public:
    MotorController() : motor_count(0) {
        // Initialize timer once for all motors
        initTimer();
    }
    
    ~MotorController() {
        for (size_t i = 0; i < motor_count; i++) {
            delete motors[i];
        }
    }
    
    // Initialize shared PWM timer
    esp_err_t initTimer() {
        ledc_timer_config_t timer_conf = {
            .speed_mode = PWMConfig::SPEED_MODE,
            .timer_num = PWMConfig::TIMER,
            .freq_hz = PWM_FREQ_HZ,
            .duty_resolution = PWM_RESOLUTION,
            .clk_cfg = LEDC_AUTO_CLK
        };
        
        esp_err_t err = ledc_timer_config(&timer_conf);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "PWM timer initialized: %lu Hz, 12-bit", PWM_FREQ_HZ);
        } else {
            ESP_LOGE(TAG, "Failed to initialize PWM timer");
        }
        
        return err;
    }
    
    // Add a motor to the controller
    esp_err_t addMotor(gpio_num_t pwm, gpio_num_t dir, ledc_channel_t channel) {
        if (motor_count >= MAX_MOTORS) {
            ESP_LOGE(TAG, "Cannot add more than %u motors", MAX_MOTORS);
            return ESP_ERR_NO_MEM;
        }
        
        motors[motor_count] = new CytronMotor(pwm, dir, channel);
        esp_err_t err = motors[motor_count]->init();
        
        if (err == ESP_OK) {
            motor_count++;
            ESP_LOGI(TAG, "Motor added. Total motors: %u", motor_count);
        }
        
        return err;
    }
    
    // Set motor speed by index (0-based)
    esp_err_t setMotorSpeed(size_t index, float speed) {
        if (index >= motor_count) {
            ESP_LOGE(TAG, "Motor index %u out of range", index);
            return ESP_ERR_INVALID_ARG;
        }
        
        return motors[index]->setSpeed(speed);
    }
    
    // Set motor speed by channel (for convenience)
    esp_err_t setMotorSpeedByChannel(ledc_channel_t channel, float speed) {
        for (size_t i = 0; i < motor_count; i++) {
            if (motors[i]->getChannel() == channel) {
                return motors[i]->setSpeed(speed);
            }
        }
        
        ESP_LOGE(TAG, "No motor found with channel %d", channel);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Stop all motors
    void stopAll() {
        ESP_LOGI(TAG, "Stopping all motors");
        for (size_t i = 0; i < motor_count; i++) {
            motors[i]->stop();
        }
    }
    
    // Ramp motor speed smoothly
    esp_err_t rampMotor(size_t index, float target_speed, uint32_t duration_ms) {
        if (index >= motor_count) {
            return ESP_ERR_INVALID_ARG;
        }
        
        float current = motors[index]->getSpeed();
        constexpr int steps = 20;
        float step = (target_speed - current) / steps;
        
        for (int i = 0; i < steps; i++) {
            current += step;
            motors[index]->setSpeed(current);
            vTaskDelay(pdMS_TO_TICKS(duration_ms / steps));
        }
        
        // Ensure exact target speed
        return motors[index]->setSpeed(target_speed);
    }
    
    // Print status of all motors
    void printStatus() const {
        printf("\n=== Motor Status ===\n");
        for (size_t i = 0; i < motor_count; i++) {
            printf("Motor %zu (Ch%d): speed = %.2f\n", 
                   i + 1, motors[i]->getChannel(), motors[i]->getSpeed());
        }
        printf("===================\n");
    }
};

// ========== TEST TASK ==========
void motorTestTask(void* pvParameters) {
    auto* controller = static_cast<MotorController*>(pvParameters);
    
    ESP_LOGI(TAG, "Motor test task started");
    
    while (1) {
        ESP_LOGI(TAG, "\n=== TEST SEQUENCE START ===");
        
        // Test 1: Individual motors forward
        for (int i = 0; i < 3; i++) {
            ESP_LOGI(TAG, "Motor %d forward 50%%", i + 1);
            controller->setMotorSpeed(i, 0.5f);
            vTaskDelay(pdMS_TO_TICKS(2000));
            controller->setMotorSpeed(i, 0.0f);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        // Test 2: Individual motors reverse
        for (int i = 0; i < 3; i++) {
            ESP_LOGI(TAG, "Motor %d reverse 50%%", i + 1);
            controller->setMotorSpeed(i, -0.5f);
            vTaskDelay(pdMS_TO_TICKS(2000));
            controller->setMotorSpeed(i, 0.0f);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        // Test 3: Simultaneous different speeds
        ESP_LOGI(TAG, "Simultaneous different speeds");
        controller->setMotorSpeed(0, 0.5f);   // Motor 1: 50% forward
        controller->setMotorSpeed(1, -0.3f);  // Motor 2: 30% reverse
        controller->setMotorSpeed(2, 0.8f);   // Motor 3: 80% forward
        controller->printStatus();
        vTaskDelay(pdMS_TO_TICKS(3000));
        controller->stopAll();
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Test 4: Speed ramping
        ESP_LOGI(TAG, "Speed ramping test");
        for (int i = 0; i < 3; i++) {
            ESP_LOGI(TAG, "Ramping motor %d", i + 1);
            controller->rampMotor(i, 0.7f, 2000);
            vTaskDelay(pdMS_TO_TICKS(500));
            controller->rampMotor(i, -0.5f, 2000);
            vTaskDelay(pdMS_TO_TICKS(500));
            controller->setMotorSpeed(i, 0.0f);
        }
        
        ESP_LOGI(TAG, "=== TEST SEQUENCE COMPLETE ===\n");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

// ========== APPLICATION MAIN ==========
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Cytron Motor Driver Starting...");
    
    // Create motor controller
    auto* controller = new MotorController();
    
    // Add motors using pin definitions
    controller->addMotor(MotorPins::MOTOR1_PWM, 
                         MotorPins::MOTOR1_DIR, 
                         PWMConfig::CHANNEL1);
    
    controller->addMotor(MotorPins::MOTOR2_PWM, 
                         MotorPins::MOTOR2_DIR, 
                         PWMConfig::CHANNEL2);
    
    controller->addMotor(MotorPins::MOTOR3_PWM, 
                         MotorPins::MOTOR3_DIR, 
                         PWMConfig::CHANNEL3);
    
    // Create test task
    xTaskCreate(motorTestTask, "motor_test", 4096, controller, 5, nullptr);
    
    ESP_LOGI(TAG, "System ready. Motor test running...");
}