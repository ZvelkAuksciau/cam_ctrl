#include <ch.hpp>
#include <hal.h>
#include <mavlink.h>
#include <camera.hpp>

using namespace chibios_rt;

uint8_t g_boardstatus = 0;
mavlink_system_t mavlink_system;

#define SHORT_BLINK 100
#define LONG_BLINK  400
#define DELAY_SEQ   500

/*
 * Led sequences
 * BOOTING_UP -          long long
 * TAKING_PICTURE -     long
 * TURNING_CAMERA_ON -  short short
 * TURNING_CAMERA_OFF - short long
 */
struct led_status{
    uint16_t time_ms;
};
const uint16_t boot_up[] = {LONG_BLINK, LONG_BLINK};
const uint16_t taking_picture[] = {SHORT_BLINK, SHORT_BLINK};
const uint16_t turning_camera_on[] = {LONG_BLINK, SHORT_BLINK};
const uint16_t turning_camera_off[] = {SHORT_BLINK, LONG_BLINK};

void play_led_sequence(const uint16_t *led){
    uint8_t lenght = sizeof(led)/sizeof(const uint16_t);
    for(int i = 0; i<lenght; i++){
        palSetPad(GPIOB, GPIOB_LED);
        chThdSleepMilliseconds(led[i]);
        palClearPad(GPIOB, GPIOB_LED);
        chThdSleepMilliseconds(led[i]);
    }
    chThdSleepMilliseconds(DELAY_SEQ);
}

/*
 * Blinker thread
 */
class BlinkerThread : public BaseStaticThread<128> {
protected:
    virtual msg_t main(void) {
        setName("blinker");
        while (TRUE) {
            if(g_boardstatus == 0){
                chThdSleepMilliseconds(10); //Fast cheking for any changes
                continue;
            }
            if(g_boardstatus & TAKING_PICTURE)
                play_led_sequence(taking_picture);
            if(g_boardstatus & BOOTING_UP)
                play_led_sequence(boot_up);
            if(g_boardstatus & TURNING_CAMERA_ON)
                play_led_sequence(turning_camera_on);
            if(g_boardstatus & TURNING_CAMERA_OFF)
                play_led_sequence(turning_camera_off);

        }
        return 0;
    }
public:
    BlinkerThread(void) : BaseStaticThread<128>() {
    }
};
static BlinkerThread blinker;


class UARThread : public BaseStaticThread<2000> {
protected:
    virtual msg_t main(void) {
        static mavlink_message_t msg;
        static mavlink_status_t status;
        uint8_t mavBuff[64];
        uint8_t bufS[MAVLINK_MAX_PACKET_LEN];

        while (true) {
            uint8_t bytesRead = chnReadTimeout(&SD1, mavBuff, 64, MS2ST(5));
            uint8_t i;
            for (i = 0; i < bytesRead; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, mavBuff[i], &msg,
                        &status)) {
                    switch (msg.msgid) {
                    case MAVLINK_MSG_ID_COMMAND_LONG: {
                        mavlink_command_long_t pack;
                        mavlink_msg_command_long_decode(&msg, &pack);
                        switch (pack.command) {
                        case MAV_CMD_DO_DIGICAM_CONTROL:
                            if(pack.param5 == 1) take_picture(); //TODO: check if it is for us
                            if(pack.param1 == 1) turn_on_camera();
                            if(pack.param1 == 2) turn_off_camera();
                        }
                        mavlink_message_t msgs;
                        mavlink_msg_command_ack_pack(mavlink_system.sysid,
                                mavlink_system.compid, &msgs, pack.command,
                                MAV_CMD_ACK_OK);
                        uint16_t len = mavlink_msg_to_send_buffer(bufS, &msgs);
                        chnWrite(&SD1, bufS, len);
                        break;
                    }
                    case MAVLINK_MSG_ID_HEARTBEAT: {
                        mavlink_heartbeat_t pack;
                        mavlink_msg_heartbeat_decode(&msg, &pack);
                        break;
                    }
                    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                        //mavlink_param_value_t pack;
                        mavlink_message_t msgs;
                        uint16_t len;
                        mavlink_msg_param_value_pack(mavlink_system.sysid,
                                mavlink_system.compid, &msgs, "CAMERA_SX230\0",
                                1.0f, MAV_PARAM_TYPE_UINT8, 1, 0);
                        len = mavlink_msg_to_send_buffer(bufS, &msgs);
                        chnWrite(&SD1, bufS, len);
                        mavlink_msg_param_value_pack(mavlink_system.sysid,
                                mavlink_system.compid, &msgs, "CAMERA_NDVI\0",
                                1.0f, MAV_PARAM_TYPE_UINT8, 1, 1);
                        len = mavlink_msg_to_send_buffer(bufS, &msgs);
                        chnWrite(&SD1, bufS, len);
                        mavlink_msg_param_value_pack(mavlink_system.sysid,
                                mavlink_system.compid, &msgs, "MNF_20151115\0",
                                1.0f, MAV_PARAM_TYPE_UINT8, 1, 2);
                        len = mavlink_msg_to_send_buffer(bufS, &msgs);
                        chnWrite(&SD1, bufS, len);
                    }
                    }
                }
            }
            BaseThread::sleep((MS2ST(5)));
        }
        /* This point may be reached if shut down is requested. */
        return 0;
    }
};

static UARThread serial_t;

/*
 * Application entry point.
 */
int main(void) {

    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
    halInit();
    chSysInit();

    g_boardstatus |= BOOTING_UP;

    init_camera();
    /*
     * Starts the blinker thread.
     */
    blinker.start(NORMALPRIO-10);

    sdStart(&SD1, NULL); //UART1 for mavlink comms (38400)

    mavlink_system.sysid = 1;
    mavlink_system.compid = MAV_COMP_ID_CAMERA;

    // Define the system type, in this case an airplane
    uint8_t system_type = MAV_TYPE_GIMBAL;
    uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

    uint8_t system_mode = MAV_MODE_AUTO_ARMED; ///< Booting up
    uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
    uint8_t system_state = MAV_STATE_ACTIVE; ///< System ready for flight

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    serial_t.start(HIGHPRIO);

    g_boardstatus &= ~BOOTING_UP;

    turn_on_camera();

    while (TRUE) {
        chnWrite(&SD1, buf, len);
        BaseThread::sleep((MS2ST(1000)));

    }
}
