#include "config.h"

void init_serial() {
    MCSerial.begin(MC_SERIAL_BAUD_RATE);
}

void init_motor(MotorConfig m)
{
    pinMode(m.DIR_P, OUTPUT);
    pinMode(m.PWM_P, OUTPUT);
}

void init_stepper(StepperConfig cfg, Stepper* stepper)
{
    pinMode(cfg.PWM1_P, OUTPUT);
    pinMode(cfg.PWM2_P, OUTPUT);
    stepper->setSpeed(cfg.speed);
}

void write_motor(MotorConfig m, uint8_t pwm, MotorDir dir)
{
    digitalWrite(m.DIR_P, dir);
    analogWrite(min(m.PWM_P), pwm);
}

void write_serial_motor(MotorConfig m, int8_t power) {
    m.st->motor(m.motor, power);
}

void stepper_step(StepperConfig s, Stepper* stepper, StepperDir dir) {
    stepper->step(s.step_size * dir);
}

void stepper_on(StepperConfig s) {
    digitalWrite(s.PWM1_P, HIGH);
    digitalWrite(s.PWM2_P, HIGH);
}

void stepper_off(StepperConfig s) {
    digitalWrite(s.PWM1_P, LOW);
    digitalWrite(s.PWM2_P, LOW);
}

void stop_motor(MotorConfig m) {
    analogWrite(m.PWM_P,0);
}