/**
 * ═══════════════════════════════════════════════════════════════════════════
 *  AVIM – ESP32 Firmware  (solo LEDs)
 *  micro-ROS  ←── USB Serial ──→ Raspberry Pi 4B
 *
 *  Motores, servo y ultrasonidos → ahora en la Raspberry Pi
 *  LEDs → siguen en el ESP32
 *
 *  Pinout ESP32:
 *    GPIO 14 → LED blinker izquierdo
 *    GPIO 15 → LED blinker derecho
 *    GPIO 25 → LED señal izquierda
 *    GPIO 26 → LED señal derecha
 *
 *  Tópicos suscritos:
 *    /lights/blinkers      (std_msgs/Bool) → parpadeo ambos blinkers
 *    /lights/left_signal   (std_msgs/Bool) → señal izquierda fija
 *    /lights/right_signal  (std_msgs/Bool) → señal derecha fija
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

// ── Pines de LEDs ─────────────────────────────────────────────────────────
#define LED_LEFT_BLINKER   14
#define LED_RIGHT_BLINKER  15
#define LED_LEFT_SIGNAL    25
#define LED_RIGHT_SIGNAL   26

// ── micro-ROS ─────────────────────────────────────────────────────────────
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rclc_executor_t executor;

rcl_subscription_t sub_blinkers;
rcl_subscription_t sub_left_signal;
rcl_subscription_t sub_right_signal;

std_msgs__msg__Bool msg_blinkers;
std_msgs__msg__Bool msg_left_signal;
std_msgs__msg__Bool msg_right_signal;

// ── Estado de blinkers ────────────────────────────────────────────────────
bool          blinkers_active = false;
unsigned long last_blink_ms   = 0;
bool          blink_state     = false;

// ── Macro de error ────────────────────────────────────────────────────────
#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if (temp_rc != RCL_RET_OK) { error_loop(); } \
}

void error_loop() {
    // Parpadeo rápido = error micro-ROS
    while (true) {
        digitalWrite(LED_LEFT_BLINKER, !digitalRead(LED_LEFT_BLINKER));
        delay(100);
    }
}

// ── Callbacks ─────────────────────────────────────────────────────────────
void cb_blinkers(const void* msg_in) {
    const std_msgs__msg__Bool* m = (const std_msgs__msg__Bool*)msg_in;
    blinkers_active = m->data;
    if (!blinkers_active) {
        digitalWrite(LED_LEFT_BLINKER,  LOW);
        digitalWrite(LED_RIGHT_BLINKER, LOW);
    }
}

void cb_left_signal(const void* msg_in) {
    const std_msgs__msg__Bool* m = (const std_msgs__msg__Bool*)msg_in;
    digitalWrite(LED_LEFT_SIGNAL, m->data ? HIGH : LOW);
}

void cb_right_signal(const void* msg_in) {
    const std_msgs__msg__Bool* m = (const std_msgs__msg__Bool*)msg_in;
    digitalWrite(LED_RIGHT_SIGNAL, m->data ? HIGH : LOW);
}

// ── Blinkers no bloqueante ────────────────────────────────────────────────
void update_blinkers(unsigned long now_ms) {
    if (!blinkers_active) return;
    if (now_ms - last_blink_ms >= 500) {
        blink_state   = !blink_state;
        last_blink_ms = now_ms;
        digitalWrite(LED_LEFT_BLINKER,  blink_state ? HIGH : LOW);
        digitalWrite(LED_RIGHT_BLINKER, blink_state ? HIGH : LOW);
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────
void setup() {
    // Configurar pines de LEDs
    pinMode(LED_LEFT_BLINKER,  OUTPUT);
    pinMode(LED_RIGHT_BLINKER, OUTPUT);
    pinMode(LED_LEFT_SIGNAL,   OUTPUT);
    pinMode(LED_RIGHT_SIGNAL,  OUTPUT);

    // Apagar todos al inicio
    digitalWrite(LED_LEFT_BLINKER,  LOW);
    digitalWrite(LED_RIGHT_BLINKER, LOW);
    digitalWrite(LED_LEFT_SIGNAL,   LOW);
    digitalWrite(LED_RIGHT_SIGNAL,  LOW);

    // micro-ROS
    set_microros_transports();
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "avim_esp32_lights", "", &support));

    // Subscriptores
    RCCHECK(rclc_subscription_init_default(&sub_blinkers, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/lights/blinkers"));

    RCCHECK(rclc_subscription_init_default(&sub_left_signal, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/lights/left_signal"));

    RCCHECK(rclc_subscription_init_default(&sub_right_signal, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/lights/right_signal"));

    // Executor con 3 subscriptores
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

    RCCHECK(rclc_executor_add_subscription(&executor, &sub_blinkers,
        &msg_blinkers, &cb_blinkers, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(&executor, &sub_left_signal,
        &msg_left_signal, &cb_left_signal, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(&executor, &sub_right_signal,
        &msg_right_signal, &cb_right_signal, ON_NEW_DATA));
}

// ── Loop ──────────────────────────────────────────────────────────────────
void loop() {
    unsigned long now_ms = millis();

    // Procesar mensajes de ROS 2
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    // Actualizar blinkers (no bloqueante)
    update_blinkers(now_ms);
}
