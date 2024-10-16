#ifndef SLOWTICKER_H
#define SLOWTICKER_H

#include "Module.h"
#include "SoftTimer.h"
#include "FreeRTOS.h"
#include "task.h"

class SlowTicker : public Module {
public:
    SlowTicker()
        : timer("SlowTickerTimer", 1000, true, this, &SlowTicker::timerCallback),
          taskHandle(nullptr)
    {
    }

    void on_module_loaded(void);
    void on_idle(void*);
    void start();

private:
    SoftTimer timer;
    TaskHandle_t taskHandle;

    void timerCallback();
};

#endif
