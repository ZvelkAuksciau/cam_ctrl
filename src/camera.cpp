#include <ch.hpp>
#include <hal.h>
#include <camera.hpp>

using namespace chibios_rt;

extern uint8_t g_boardstatus;

bool camera_on = false;


/*
 * Camera control thread
 */
class CameraThread : public BaseStaticThread<128> {
protected:
    virtual msg_t main(void) {
        setName("camera");
        while (TRUE) {
            if(!palReadPad(GPIOA, GPIOA_ON_OFF)){
                g_boardstatus |= TURNING_CAMERA_OFF;
            }
            if(g_boardstatus & TAKING_PICTURE){
                if(camera_on){
                    palSetPad(GPIOB, GPIOB_CAM_TRIG_CTRL);
                    chThdSleepMilliseconds(200);
                    palClearPad(GPIOB, GPIOB_CAM_TRIG_CTRL);
                }
                g_boardstatus &= ~TAKING_PICTURE;
            }
            else if(g_boardstatus & TURNING_CAMERA_OFF){
                if(camera_on){
                    turn_camera_vcc(true);
                    chThdSleepMilliseconds(100);
                    palSetPad(GPIOC, GPIOC_CAM_POWER_BUT);
                    chThdSleepMilliseconds(200);
                    palClearPad(GPIOC, GPIOC_CAM_POWER_BUT);
                    chThdSleepMilliseconds(3000);
                    turn_camera_vcc(false);
                    camera_on = false;
                    g_boardstatus &= ~TURNING_CAMERA_OFF;
                }
                else g_boardstatus &= ~TURNING_CAMERA_OFF;
            }
            else if(g_boardstatus & TURNING_CAMERA_ON){
                if(!camera_on){
                    turn_camera_vcc(false);
                    chThdSleepMilliseconds(500);
                    turn_camera_vcc(true);
                    chThdSleepMilliseconds(1000);
                    palSetPad(GPIOC, GPIOC_CAM_POWER_BUT);
                    chThdSleepMilliseconds(2000);
                    palClearPad(GPIOC, GPIOC_CAM_POWER_BUT);
                    camera_on = true;
                    g_boardstatus &= ~TURNING_CAMERA_ON;
                }
                else g_boardstatus &= ~TURNING_CAMERA_ON;
            }
            chThdSleepMilliseconds(10);
        }
    }
public:
    CameraThread(void) : BaseStaticThread<128>() {
    }
};
static CameraThread camera;

void take_picture() {
    g_boardstatus |= TAKING_PICTURE;
}

void turn_on_camera() {
    g_boardstatus |= TURNING_CAMERA_ON;
}

void turn_off_camera() {
    g_boardstatus |= TURNING_CAMERA_OFF;
}

void turn_camera_vcc(bool on) {
    if(on) {
        palSetPad(GPIOA, GPIOA_CAM_VCC_EN);
    }
    else {
        palClearPad(GPIOA, GPIOA_CAM_VCC_EN);
    }

}

void init_camera(){
    camera.start(NORMALPRIO);
}



