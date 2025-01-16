#include <Arduino.h>
#include <Servo.h>

#include "eye.h"
#include "eyes.h"

/*!
 *    @brief  Instantiates a new  class
 */
Eyes::Eyes(void) {}

/*!
 *    @brief  Instantiates a new class and run begin
 */
Eyes::Eyes(Eye left, Eye right) {
    begin(left, right);
}

void Eyes::begin(Eye left, Eye right) {
    eye_left = left;
    eye_right = right;
}

void Eyes::blink() {
    blink(1000);
}

void Eyes::blink(int delay_ms) {
    open();
    delay(delay_ms);
    close();
}

void Eyes::open() {
    eye_left.open();
    eye_right.open();
}

void Eyes::close() {
    eye_left.close();
    eye_right.close();
}

void Eyes::lookAt (int hpos, int vpos) {
    lookAtHorizontal(hpos);
    lookAtVertical(vpos);
}

void Eyes::lookAtHorizontal (int pos) {
    eye_left.lookAtHorizontal(pos);
    eye_right.lookAtHorizontal(pos);
}

void Eyes::lookAtVertical (int pos) {
    eye_left.lookAtVertical(pos);
    eye_right.lookAtVertical(pos);
}

void Eyes::lidAt (int pos) {
    eye_left.lidAt(pos);
    eye_right.lidAt(pos);
}

int Eyes::getHorizontalPosition () { 
    // since the positions are relative this should be the same for both eyes
    return eye_left.getHorizontalPosition();
}

int Eyes::getVerticalPosition () { 
    return eye_left.getVerticalPosition();
}

int Eyes::getEyelidPosition () { 
    return eye_left.getEyelidPosition();
}