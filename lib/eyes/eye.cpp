#include <Arduino.h>
#include <Servo.h>

#include "eye.h"

/*!
 *    @brief  Instantiates a new  class
 */
Eye::Eye(void) {}

/*!
 *    @brief  Instantiates a new class and run begin
 */
Eye::Eye(int horizontal_pin, int vertical_pin, int lid_pin) {
    begin(horizontal_pin, vertical_pin, lid_pin);
}

void Eye::begin (int horizontal_pin, int vertical_pin, int lid_pin) {
    horizontal_servo.attach(horizontal_pin);
    vertical_servo.attach(horizontal_pin);
    lid_servo.attach(lid_pin);
}

void Eye::setLimitsHorizontal (int min_pos, int max_pos) {
    horizontal_min_pos = min_pos;
    horizontal_max_pos = max_pos;
}

void Eye::setLimitsVertical (int min_pos, int max_pos) {
    vertical_min_pos = min_pos;
    vertical_max_pos = max_pos;
}

void Eye::setLimitsLid (int min_pos, int max_pos) {
    lid_min_pos = min_pos;
    lid_max_pos = max_pos;
}

void Eye::blink() {
    blink(1000);
}

void Eye::blink(int delay_ms) {
    open();
    delay(delay_ms);
    close();
}

void Eye::open() {
    lid_servo.write(lid_min_pos);
}

void Eye::close() {
    lid_servo.write(lid_max_pos);
}

void Eye::lookAt (int hpos, int vpos) {
    lookAtHorizontal(hpos);
    lookAtVertical(vpos);
}

void Eye::lookAtHorizontal (int pos) {
    horizontal_servo.write(pos);
}

void Eye::lookAtVertical (int pos) {
    vertical_servo.write(pos);
}