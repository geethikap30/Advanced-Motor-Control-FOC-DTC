#include "main.h"
#include "math.h"

#define PI 3.14159265358979f
#define PPR 1024
#define Ts 0.001f // Control loop interval (1 ms)

float Ia, Ib;                // Phase currents from ADC
float I_alpha, I_beta;      // Clarke transformation outputs
float Id, Iq;               // Park transformation outputs
float Vd, Vq;               // Controller outputs
float Valpha, Vbeta;        // Inverse Park outputs
float theta_electric;       // Rotor angle
float speed_meas, speed_ref = 1000; // Speed in RPM
float e_speed, integral_speed = 0;

// PID constants
float Kp = 0.3, Ki = 0.01;

// Initialization (called once in main)
void System_Initialization() {
    // System clocks, ADC, Timers, UART, etc.
}

// Get rotor angle from encoder
float Get_Rotor_Angle() {
    uint32_t encoder_count = __HAL_TIM_GET_COUNTER(&htim2);
    return ((float)encoder_count / PPR) * 2.0f * PI;
}

// Clarke Transformation
void Clarke(float Ia, float Ib, float* alpha, float* beta) {
    *alpha = Ia;
    *beta = (Ia + 2 * Ib) / sqrt(3.0f);
}

// Park Transformation
void Park(float alpha, float beta, float theta, float* d, float* q) {
    *d =  alpha * cos(theta) + beta * sin(theta);
    *q = -alpha * sin(theta) + beta * cos(theta);
}

// Inverse Park Transformation
void InvPark(float d, float q, float theta, float* alpha, float* beta) {
    *alpha = d * cos(theta) - q * sin(theta);
    *beta  = d * sin(theta) + q * cos(theta);
}

// PID Controller for Iq (Torque control)
void PID_Speed_Control(float speed_meas, float* Vq_out) {
    e_speed = speed_ref - speed_meas;
    integral_speed += e_speed * Ts;
    *Vq_out = Kp * e_speed + Ki * integral_speed;
}

// Space Vector Modulation output (simplified)
void Apply_SVM(float alpha, float beta) {
    // Map to PWM values and write to TIM1 CH1, CH2, CH3
}

// Main control loop (called every 1 ms)
void FOC_Control_Loop() {
    // 1. Read ADC current values
    Ia = Read_ADC_Channel(1);
    Ib = Read_ADC_Channel(2);

    // 2. Estimate rotor angle
    theta_electric = Get_Rotor_Angle();

    // 3. Clarke Transformation
    Clarke(Ia, Ib, &I_alpha, &I_beta);

    // 4. Park Transformation
    Park(I_alpha, I_beta, theta_electric, &Id, &Iq);

    // 5. Speed Control (PID on Iq)
    PID_Speed_Control(speed_meas, &Vq);
    Vd = 0;  // Flux component not used

    // 6. Inverse Park Transformation
    InvPark(Vd, Vq, theta_electric, &Valpha, &Vbeta);

    // 7. Space Vector Modulation
    Apply_SVM(Valpha, Vbeta);
}
