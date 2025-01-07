#ifndef _IMMUTEC_EYE
#define _IMMUTEC_EYE

#include <Servo.h>

class Eye {
    private:
        Servo horizontal_servo;
        int horizontal_min_pos = 0;
        int horizontal_max_pos = 180;

        Servo vertical_servo;
        int vertical_min_pos = 0;
        int vertical_max_pos = 180;

        Servo lid_servo;
        int lid_min_pos = 40;
        int lid_max_pos = 150;
    public:
        Eye();
        Eye(int, int, int);
        void begin(int, int, int);

        void setLimitsHorizontal (int, int);
        void setLimitsVertical (int, int);
        void setLimitsLid (int, int);

        void blink();
        void blink(int);
        void open();
        void close();
        
        void lookAt(int, int);
        void lookAtHorizontal(int);
        void lookAtVertical(int);
};

#endif
