#include "SlowTicker.h"
#include "Kernel.h"

void SlowTicker::on_module_loaded() {
    taskHandle = xTaskGetCurrentTaskHandle();
    this->register_for_event(ON_IDLE);
}

void SlowTicker::start() {
    timer.start();
}

void SlowTicker::timerCallback() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(taskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void SlowTicker::on_idle(void*) {
    if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
        THEKERNEL.call_event(ON_SECOND_TICK);
    }
}
