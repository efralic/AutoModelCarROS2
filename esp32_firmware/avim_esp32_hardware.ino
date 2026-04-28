/**
 * ═══════════════════════════════════════════════════════════════════════════
 *  AVIM – Hardware Interface Firmware para ESP32
 *  micro-ROS  (reemplaza motor_driver.py + lights_controller.py +
 *              ultrasonic_sensors.py que corrían en la Raspberry Pi 4B)
 *
 *  Comunicación: ESP32 ←── USB Serial ──→ Raspberry Pi 4B
 *                         (micro-ROS Agent corre en la Pi)
 *
 *  Plataforma: ESP32 (testeado en ESP32-DevKitC)
 *  Framework : Arduino + micro_ros_arduino
 *
 *  Instalación en Arduino IDE:
 *    1. Añadir URL: https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/main/extras/package_index.json
 *    2. Instalar: "micro_ros_arduino"
 *    3. Instalar librería: "ESP32Servo" (para el servo)
 *
 *  En la Raspberry Pi (una sola vez):
 *    sudo apt install ros-humble-micro-ros-agent
 *    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
 *
 *  Pinout ESP32:
 *   ── Motor Module I2C ──
 *    GPIO 21 → SDA
 *    GPIO 22 → SCL
 *
 *   ── Servo de dirección ──
 *    GPIO 13 → Signal
 *
 *   ── LEDs ──
 *    GPIO 14 → LED blinker izquierdo
 *    GPIO 15 → LED blinker derecho
 *    GPIO 25 → LED señal izquierda
 *    GPIO 26 → LED señal derecha   (en Pi era GPIO 8, no disponible en ESP32)
 *
 *   ── Ultrasonidos HC-SR04 ──
 *    front:  TRIG=5   ECHO=6  (nota: GPIO 6 es flash en algunos ESP32,
 *                               usa GPIO 34/35 si hay conflicto)
 *    back:   TRIG=16  ECHO=17
 *    left:   TRIG=18  ECHO=19
 *    right:  TRIG=23  ECHO=32
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/range.h>
#include <rosidl_runtime_c/string_functions.h>

#include <Wire.h>
#include <ESP32Servo.h>

// ═══════════════════════════════════════════════════════════════════════════
//  CONFIGURACIÓN DE PINES
// ═══════════════════════════════════════════════════════════════════════════

// I2C Motor Module
#define I2C_SDA  21
#define I2C_SCL  22

// Servo
#define SERVO_PIN           13
#define SERVO_MIN_PULSE_US  1222   // 65°
#define SERVO_MAX_PULSE_US  1778   // 115°
#define SERVO_CENTER_US     1500   // 90°

// LEDs
#define LED_LEFT_BLINKER   14
#define LED_RIGHT_BLINKER  15
#define LED_LEFT_SIGNAL    25
#define LED_RIGHT_SIGNAL   26

// Ultrasonidos (TRIG, ECHO)
#define US_FRONT_TRIG  5
#define US_FRONT_ECHO  33   // GPIO 34/35 son input-only, usar 33
#define US_BACK_TRIG   16
#define US_BACK_ECHO   17
#define US_LEFT_TRIG   18
#define US_LEFT_ECHO   19
#define US_RIGHT_TRIG  23
#define US_RIGHT_ECHO  32

// ═══════════════════════════════════════════════════════════════════════════
//  REGISTROS DEL MÓDULO MOTOR I2C (Encoder Motor Module)
// ═══════════════════════════════════════════════════════════════════════════
#define MOTOR_I2C_ADDR              0x34
#define MOTOR_TYPE_ADDR             0x14
#define MOTOR_ENCODER_POLARITY_ADDR 0x15
#define MOTOR_FIXED_SPEED_ADDR      0x33
#define MOTOR_TYPE_JGB37            3

// ═══════════════════════════════════════════════════════════════════════════
//  MICRO-ROS – Nodos, suscriptores, publicadores
// ═══════════════════════════════════════════════════════════════════════════

// ── Soporte de micro-ROS ──────────────────────────────────────────────────
rclc_support_t   support;
rcl_allocator_t  allocator;
rcl_node_t       node;
rclc_executor_t  executor;

// ── Subscriptores ─────────────────────────────────────────────────────────
rcl_subscription_t sub_speed;
rcl_subscription_t sub_steering;
rcl_subscription_t sub_blinkers;
rcl_subscription_t sub_left_signal;
rcl_subscription_t sub_right_signal;
rcl_subscription_t sub_mode;
rcl_subscription_t sub_emergency;

// ── Publicadores ──────────────────────────────────────────────────────────
rcl_publisher_t pub_us_front;
rcl_publisher_t pub_us_back;
rcl_publisher_t pub_us_left;
rcl_publisher_t pub_us_right;

// ── Mensajes de entrada ───────────────────────────────────────────────────
std_msgs__msg__Int16 msg_speed;
std_msgs__msg__Int16 msg_steering;
std_msgs__msg__Bool  msg_blinkers;
std_msgs__msg__Bool  msg_left_signal;
std_msgs__msg__Bool  msg_right_signal;
std_msgs__msg__String msg_mode;

// ── Mensajes de salida (rango) ────────────────────────────────────────────
sensor_msgs__msg__Range msg_range_front;
sensor_msgs__msg__Range msg_range_back;
sensor_msgs__msg__Range msg_range_left;
sensor_msgs__msg__Range msg_range_right;

// ═══════════════════════════════════════════════════════════════════════════
//  ESTADO DEL VEHÍCULO
// ═══════════════════════════════════════════════════════════════════════════
Servo          servo;
bool           autonomous_mode  = true;
bool           emergency_stop   = false;
bool           blinkers_active  = false;
unsigned long  last_blink_ms    = 0;
bool           blink_state      = false;

// ── Macro de comprobación de errores micro-ROS ────────────────────────────
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if(temp_rc != RCL_RET_OK){ error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

void error_loop() {
    // Parpadeo rápido = error micro-ROS
    while (true) {
        digitalWrite(LED_LEFT_BLINKER, !digitalRead(LED_LEFT_BLINKER));
        delay(100);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  HARDWARE: MOTOR MODULE I2C
// ═══════════════════════════════════════════════════════════════════════════

void setup_motor_module() {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.beginTransmission(MOTOR_I2C_ADDR);
    Wire.write(MOTOR_TYPE_ADDR);
    Wire.write(MOTOR_TYPE_JGB37);
    Wire.endTransmission();
    delay(50);

    Wire.beginTransmission(MOTOR_I2C_ADDR);
    Wire.write(MOTOR_ENCODER_POLARITY_ADDR);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(50);
}

/**
 * Escribe velocidades a los 4 canales del módulo motor.
 * Valores: -100 a 100. Se convierte a complemento a 2 (byte sin signo).
 * m1/m2 = motores derecho/izquierdo. m3/m4 sin uso (= 0).
 */
void write_motor_speeds(int m1, int m2, int m3 = 0, int m4 = 0) {
    auto to_u8 = [](int v) -> uint8_t {
        v = constrain(v, -100, 100);
        return (uint8_t)(v & 0xFF);
    };
    Wire.beginTransmission(MOTOR_I2C_ADDR);
    Wire.write(MOTOR_FIXED_SPEED_ADDR);
    Wire.write(to_u8(m1));
    Wire.write(to_u8(m2));
    Wire.write(to_u8(m3));
    Wire.write(to_u8(m4));
    Wire.endTransmission();
}

/**
 * Mapea velocidad del Master (-500..500) → motor (-100..100).
 * Negativo = adelante, positivo = reversa.
 */
void set_motor_speed(int speed) {
    if (speed == 0) { write_motor_speeds(0, 0); return; }
    int motor_val = constrain((int)((speed / 500.0f) * 100), -100, 100);
    int m = -motor_val;   // negativo master = adelante = positivo motor
    write_motor_speeds(m, m);
}

/**
 * Mapea ángulo (65-115°) → pulso servo (1222-1778 µs).
 */
void set_servo_angle(int angle) {
    angle = constrain(angle, 65, 115);
    int pulse = map(angle, 65, 115, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
    servo.writeMicroseconds(pulse);
}

// ═══════════════════════════════════════════════════════════════════════════
//  HARDWARE: ULTRASONIDOS HC-SR04
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Mide distancia en metros con HC-SR04.
 * Retorna -1.0 si hay timeout o fuera de rango (2 cm – 4 m).
 */
float measure_distance(uint8_t trig, uint8_t echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    long duration = pulseIn(echo, HIGH, 25000);  // timeout 25 ms ≈ 4.25 m
    if (duration == 0) return -1.0f;

    float distance_m = (duration * 0.000343f) / 2.0f;
    return (distance_m >= 0.02f && distance_m <= 4.0f) ? distance_m : -1.0f;
}

void fill_range_msg(sensor_msgs__msg__Range* msg, const char* frame_id,
                    float distance, rcl_clock_t* clock)
{
    // Header timestamp
    rcl_time_point_value_t now;
    rcl_clock_get_now(clock, &now);
    msg->header.stamp.sec     = (int32_t)(now / 1000000000LL);
    msg->header.stamp.nanosec = (uint32_t)(now % 1000000000LL);
    rosidl_runtime_c__String__assign(&msg->header.frame_id, frame_id);

    msg->radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    msg->field_of_view  = 0.26f;   // ~15°
    msg->min_range      = 0.02f;
    msg->max_range      = 4.0f;
    msg->range          = (distance > 0.0f) ? distance : msg->max_range;
}

// ═══════════════════════════════════════════════════════════════════════════
//  HARDWARE: LEDs / LUCES
// ═══════════════════════════════════════════════════════════════════════════

void update_blinkers(unsigned long now_ms) {
    if (!blinkers_active) {
        digitalWrite(LED_LEFT_BLINKER,  LOW);
        digitalWrite(LED_RIGHT_BLINKER, LOW);
        return;
    }
    if (now_ms - last_blink_ms >= 500) {
        blink_state      = !blink_state;
        last_blink_ms    = now_ms;
        digitalWrite(LED_LEFT_BLINKER,  blink_state ? HIGH : LOW);
        digitalWrite(LED_RIGHT_BLINKER, blink_state ? HIGH : LOW);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  CALLBACKS DE SUBSCRIPCIÓN (equivalentes a los callbacks ROS 1/2 en Python)
// ═══════════════════════════════════════════════════════════════════════════

void cb_speed(const void* msg_in) {
    if (!autonomous_mode || emergency_stop) return;
    const std_msgs__msg__Int16* m = (const std_msgs__msg__Int16*)msg_in;
    set_motor_speed(m->data);
}

void cb_steering(const void* msg_in) {
    if (!autonomous_mode || emergency_stop) return;
    const std_msgs__msg__Int16* m = (const std_msgs__msg__Int16*)msg_in;
    set_servo_angle(m->data);
}

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

void cb_mode(const void* msg_in) {
    const std_msgs__msg__String* m = (const std_msgs__msg__String*)msg_in;
    // Comparar string del modo
    if (strcmp(m->data.data, "AUTONOMOUS") == 0) {
        autonomous_mode = true;
    } else if (strcmp(m->data.data, "TELEOP") == 0) {
        autonomous_mode = false;
    }
}

void cb_emergency(const void* msg_in) {
    const std_msgs__msg__Bool* m = (const std_msgs__msg__Bool*)msg_in;
    emergency_stop = m->data;
    if (emergency_stop) {
        write_motor_speeds(0, 0);
        servo.writeMicroseconds(SERVO_CENTER_US);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    // ── GPIO ──────────────────────────────────────────────────────────────
    pinMode(LED_LEFT_BLINKER,  OUTPUT);
    pinMode(LED_RIGHT_BLINKER, OUTPUT);
    pinMode(LED_LEFT_SIGNAL,   OUTPUT);
    pinMode(LED_RIGHT_SIGNAL,  OUTPUT);

    pinMode(US_FRONT_TRIG, OUTPUT); pinMode(US_FRONT_ECHO, INPUT);
    pinMode(US_BACK_TRIG,  OUTPUT); pinMode(US_BACK_ECHO,  INPUT);
    pinMode(US_LEFT_TRIG,  OUTPUT); pinMode(US_LEFT_ECHO,  INPUT);
    pinMode(US_RIGHT_TRIG, OUTPUT); pinMode(US_RIGHT_ECHO, INPUT);

    // ── Servo ─────────────────────────────────────────────────────────────
    ESP32PWM::allocateTimer(0);
    servo.setPeriodHertz(50);
    servo.attach(SERVO_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
    servo.writeMicroseconds(SERVO_CENTER_US);

    // ── Motor Module I2C ──────────────────────────────────────────────────
    setup_motor_module();

    // ── micro-ROS: comunicación serial con la Raspberry Pi ────────────────
    set_microros_transports();   // Serial a 115200 bps

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Crear el nodo micro-ROS (aparece como nodo ROS 2 en la red de la Pi)
    RCCHECK(rclc_node_init_default(&node, "avim_esp32_hardware", "", &support));

    // ── Crear subscriptores ───────────────────────────────────────────────
    RCCHECK(rclc_subscription_init_default(&sub_speed, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
        "/AutoModelMini/manual_control/speed"));

    RCCHECK(rclc_subscription_init_default(&sub_steering, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
        "/AutoModelMini/manual_control/steering"));

    RCCHECK(rclc_subscription_init_default(&sub_blinkers, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/lights/blinkers"));

    RCCHECK(rclc_subscription_init_default(&sub_left_signal, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/lights/left_signal"));

    RCCHECK(rclc_subscription_init_default(&sub_right_signal, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/lights/right_signal"));

    RCCHECK(rclc_subscription_init_default(&sub_mode, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/vehicle_mode"));

    RCCHECK(rclc_subscription_init_default(&sub_emergency, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/emergency_stop"));

    // ── Crear publicadores ────────────────────────────────────────────────
    RCCHECK(rclc_publisher_init_default(&pub_us_front, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "/sensors/ultrasonic/front"));

    RCCHECK(rclc_publisher_init_default(&pub_us_back, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "/sensors/ultrasonic/back"));

    RCCHECK(rclc_publisher_init_default(&pub_us_left, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "/sensors/ultrasonic/left"));

    RCCHECK(rclc_publisher_init_default(&pub_us_right, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "/sensors/ultrasonic/right"));

    // ── Executor: registrar 7 subscriptores ──────────────────────────────
    RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));

    RCCHECK(rclc_executor_add_subscription(&executor, &sub_speed,
        &msg_speed,        &cb_speed,        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_steering,
        &msg_steering,     &cb_steering,     ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_blinkers,
        &msg_blinkers,     &cb_blinkers,     ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_left_signal,
        &msg_left_signal,  &cb_left_signal,  ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_right_signal,
        &msg_right_signal, &cb_right_signal, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_mode,
        &msg_mode,         &cb_mode,         ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_emergency,
        &msg_emergency,    &cb_emergency,    ON_NEW_DATA));

    // Inicializar buffer del string de modo
    char mode_buf[16];
    msg_mode.data.data     = mode_buf;
    msg_mode.data.capacity = sizeof(mode_buf);
    msg_mode.data.size     = 0;
}

// Declarar msg_emergency aquí (fuera de scope por RCCHECK)
std_msgs__msg__Bool msg_emergency;

// ═══════════════════════════════════════════════════════════════════════════
//  LOOP PRINCIPAL
// ═══════════════════════════════════════════════════════════════════════════

static unsigned long last_us_publish_ms = 0;
static rcl_clock_t*  ros_clock          = nullptr;

void loop() {
    unsigned long now_ms = millis();

    // ── Procesar mensajes entrantes de la Pi (callbacks) ──────────────────
    // Equivale a rclpy.spin_once() en Python
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    // ── Actualizar blinkers (no bloqueante, reemplaza threading.Thread) ───
    update_blinkers(now_ms);

    // ── Publicar ultrasonidos a 10 Hz ─────────────────────────────────────
    if (now_ms - last_us_publish_ms >= 100) {
        last_us_publish_ms = now_ms;

        float d_front = measure_distance(US_FRONT_TRIG, US_FRONT_ECHO);
        float d_back  = measure_distance(US_BACK_TRIG,  US_BACK_ECHO);
        float d_left  = measure_distance(US_LEFT_TRIG,  US_LEFT_ECHO);
        float d_right = measure_distance(US_RIGHT_TRIG, US_RIGHT_ECHO);

        // Obtener el reloj ROS para el timestamp (una sola vez)
        if (ros_clock == nullptr) {
            ros_clock = (rcl_clock_t*)malloc(sizeof(rcl_clock_t));
            rcl_ros_clock_init(ros_clock, &allocator);
        }

        fill_range_msg(&msg_range_front, "ultrasonic_front", d_front, ros_clock);
        fill_range_msg(&msg_range_back,  "ultrasonic_back",  d_back,  ros_clock);
        fill_range_msg(&msg_range_left,  "ultrasonic_left",  d_left,  ros_clock);
        fill_range_msg(&msg_range_right, "ultrasonic_right", d_right, ros_clock);

        RCSOFTCHECK(rcl_publish(&pub_us_front, &msg_range_front, NULL));
        RCSOFTCHECK(rcl_publish(&pub_us_back,  &msg_range_back,  NULL));
        RCSOFTCHECK(rcl_publish(&pub_us_left,  &msg_range_left,  NULL));
        RCSOFTCHECK(rcl_publish(&pub_us_right, &msg_range_right, NULL));
    }
}
