//#include "modules/ex2_full_speed.h"
//#include "modules/ex3_speed_up.h"
#include "modules/ex8_tilt_tx.h"
#include "modules/ex8_tilt_rx.h"

// Pick ONE of these two before flashing:
//   - run_tilt_tx() on the controller (manette) micro:bit
//   - run_tilt_rx() on the receiver (cutebot) micro:bit
int main(void) {
    //run_full_speed();
    //run_gradual_speed();
    run_tilt_tx();
    //run_tilt_rx();
    while(1);
}