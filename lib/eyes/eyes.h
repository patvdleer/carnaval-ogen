#ifndef _IMMUTEC_EYES
#define _IMMUTEC_EYES

#include "eye.h"

class Eyes {
    private:
        Eye eye_left;
        Eye eye_right;
    public:
        Eyes();
        Eyes(Eye, Eye);

        void begin(Eye, Eye);

        void blink();
        void blink(int);
        void open();
        void close();
        
        void lookAt(int, int);
        void lookAtHorizontal(int);
        void lookAtVertical(int);
};

#endif
