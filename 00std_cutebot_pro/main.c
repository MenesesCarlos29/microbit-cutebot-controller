#include "modules/ex8_tilt_tx.h"
#include "modules/ex8_tilt_rx.h"

// Pick ONE of these two before flashing:
//   - run_tilt_tx() on the controller (manette) micro:bit
//   - run_tilt_rx() on the receiver (cutebot) micro:bit
int main(void) {
    //run_tilt_tx();
    run_tilt_rx();
    while(1);
}