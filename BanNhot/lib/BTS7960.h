/*
* File: BTS7960.h
* Author: HOANG QUOC BINH
* Date: 30/03/2023
* Description: This is library for BTS7960 control motor via PCF8574
*/
#pragma once
#include "Arduino.h"
#include <stdio.h>
#include <stdint.h>


class BTS7960
{
private:
    /* data */
    uint8_t _EN;
    uint8_t _L_PWM;
    uint8_t _R_PWM;
public:
    BTS7960(uint8_t EN, uint8_t L_PWM, uint8_t R_PWM);
    void TurnLeft(Adafruit_PCF8574 &pcf, uint8_t pwm);
	void TurnRight(Adafruit_PCF8574 &pcf, uint8_t pwm);
	void Stop(Adafruit_PCF8574 &pcf);
};

BTS7960::BTS7960(uint8_t EN, uint8_t L_PWM, uint8_t R_PWM){
    this->_L_PWM = L_PWM;
    this->_R_PWM = R_PWM;
    this->_EN = EN;
    pinMode(_EN, OUTPUT);
}
void BTS7960::TurnLeft(Adafruit_PCF8574 &pcf, uint8_t pwm){
    analogWrite(_EN, pwm);
    pcf.digitalWrite(_R_PWM, 0);
    delayMicroseconds(100);
    pcf.digitalWrite(_L_PWM, 1);
}

void BTS7960::TurnRight(Adafruit_PCF8574 &pcf, uint8_t pwm){
    analogWrite(_EN, pwm);
    pcf.digitalWrite(_R_PWM, 1);
    delayMicroseconds(100);
    pcf.digitalWrite(_L_PWM, 0);
}

void BTS7960::Stop(Adafruit_PCF8574 &pcf){
	analogWrite(_EN, 0);
    pcf.digitalWrite(_R_PWM, HIGH);
    pcf.digitalWrite(_L_PWM, LOW);
}