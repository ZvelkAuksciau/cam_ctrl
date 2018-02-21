#include <ch.h>
#include <hal.h>
#include <math.h>

#include "node.hpp"

/*
 * standard 9600 baud serial config.
 */
static const SerialConfig serialCfg = {
  9600,
  0,
  0,
  0
};


static THD_WORKING_AREA(waThread1, 128);
void Thread1(void) {
  chRegSetThreadName("blinker");

  while(1) {
    palClearPad(GPIOB, GPIOB_LED);
    chThdSleepMilliseconds(500);
    palSetPad(GPIOB, GPIOB_LED);
    chThdSleepMilliseconds(500);
  }
}

static Node::uavcanNodeThread canNode;

int main(void) {
  halInit();
  chSysInit();
  sdStart(&SD1, &serialCfg);

  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, (tfunc_t)Thread1, NULL);

  canNode.start(NORMALPRIO);

  while(1) {
    chThdSleepMilliseconds(500);
  }
}

