/**
 * Crazyflie Sender - Based strictly on your verified 51DK code
 */
#include "board.h"
#include "radio.h"
#include "leds.h"
#include "sctimer.h"

#include "cf_crazyflie.h"
#include "cf_multiranger.h"
#include "cf_pm.h"
#include "cf_systick.h"
#include "cf_api_commander_high_level.h"
//=========================== defines =========================================

#define LENGTH_PACKET   125+LENGTH_CRC
#define LEN_PKT_TO_SEND 20+LENGTH_CRC
#define CHANNEL         26  
#define TIMER_PERIOD    (0xffff>>4)
#define ID              0x99
#define PACKET_TYPE_MULTIRANGER 0x01

#define STARTUP_DELAY_MS 3000
#define HEARTBEAT_INTERVAL_MS 500  // Send control command every 500ms to reset watchdog and prevent STM32 reboot
#define CRAZYFLIE_HANDLE_INTERVAL_MS 5  // Call crazyflieHandle() every 5ms to maintain syslink heartbeat

int last_heartbeat_tick = 0;
int last_crazyflie_handle_tick = 0;

#define PACKET_TYPE_COMMAND 0x02
#define PACKET_TYPE_PROXIMITY 0x03  // Proximity status packet
//=========================== variables =======================================

int tick;
bool enHighLevel = false;

enum {
    APP_FLAG_START_FRAME = 0x01,
    APP_FLAG_END_FRAME   = 0x02,
    APP_FLAG_TIMER       = 0x04,
};

typedef enum {
    APP_STATE_TX         = 0x01,
    APP_STATE_RX         = 0x02,
} app_state_t;

typedef struct {
    uint8_t              flags;
    app_state_t          state;
    uint8_t              packet[LENGTH_PACKET];
    uint8_t              packet_len;
} app_vars_t;

app_vars_t app_vars;

// Data packet structure definition
typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  drone_id;
    uint16_t dist_front;
    uint16_t dist_back;
    uint16_t dist_left;
    uint16_t dist_right;
    uint16_t dist_up;
} broadcast_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t  type;    // Packet type
    uint8_t  drone_id;// ID
    uint8_t  proximity_flags; // Bitfield: bit0=front, bit1=back, bit2=left, bit3=right, bit4=up, bit5=down
} command_payload_t;

// Proximity status bit definitions
#define PROXIMITY_FRONT_BIT  (1 << 0)
#define PROXIMITY_BACK_BIT   (1 << 1)
#define PROXIMITY_LEFT_BIT   (1 << 2)
#define PROXIMITY_RIGHT_BIT  (1 << 3)
#define PROXIMITY_UP_BIT     (1 << 4)
#define PROXIMITY_DOWN_BIT   (1 << 5)

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);
void     cb_timer(void);

//=========================== main ============================================

int mote_main(void) {
    memset(&app_vars,0,sizeof(app_vars_t));

    board_init();
    crazyflieInit(); 
    
    // Initialize Radio callbacks
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    sctimer_set_callback(cb_timer);
    sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);
    sctimer_enable();

    // Initial Radio configuration
    radio_rfOn();
    radio_setFrequency(CHANNEL, FREQ_RX);
    radio_rxEnable();
    app_vars.state = APP_STATE_RX;
    app_vars.flags |= APP_FLAG_TIMER;

    while (1) {
        tick = systickGetTick();

        if (tick - last_crazyflie_handle_tick >= CRAZYFLIE_HANDLE_INTERVAL_MS) {
            crazyflieHandle();
            last_crazyflie_handle_tick = tick;
        }

        if (!enHighLevel && tick > STARTUP_DELAY_MS) {

            high_level_enable();
            mutiranger_init();
            radio_init(); 
            radio_setStartFrameCb(cb_startFrame); 
            radio_setEndFrameCb(cb_endFrame);
            radio_rfOn();            
            enHighLevel = true; // Lock to prevent re-entry
        }

        if (enHighLevel) {
            if (tick - last_heartbeat_tick >= HEARTBEAT_INTERVAL_MS) {
                high_level_stop(); 
                last_heartbeat_tick = tick;
            }
        }

        while (app_vars.flags) {
            if (app_vars.flags & APP_FLAG_START_FRAME) {
                app_vars.flags &= ~APP_FLAG_START_FRAME;
            }
            
            if (app_vars.flags & APP_FLAG_END_FRAME) {
                if (app_vars.state == APP_STATE_TX) {
                    radio_rxEnable();
                    radio_rxNow();
                    app_vars.state = APP_STATE_RX;
                }
                app_vars.flags &= ~APP_FLAG_END_FRAME;
            }

            if (app_vars.flags & APP_FLAG_TIMER) {
                if (app_vars.state == APP_STATE_RX) {
                    if (enHighLevel) {
                        radio_rfOff();
                        command_payload_t *pData = (command_payload_t *)app_vars.packet;
                        pData->type     = PACKET_TYPE_COMMAND; // 0x02
                        pData->drone_id = 1;
                        
                        // Check proximity status for all directions and set corresponding bits
                        pData->proximity_flags = 0;
                        if (mutiranger_front_isClose()) {
                            pData->proximity_flags |= PROXIMITY_FRONT_BIT;
                        }
                        if (mutiranger_back_isClose()) {
                            pData->proximity_flags |= PROXIMITY_BACK_BIT;
                        }
                        if (mutiranger_left_isClose()) {
                            pData->proximity_flags |= PROXIMITY_LEFT_BIT;
                        }
                        if (mutiranger_right_isClose()) {
                            pData->proximity_flags |= PROXIMITY_RIGHT_BIT;
                        }
                        if (mutiranger_up_isClose()) {
                            pData->proximity_flags |= PROXIMITY_UP_BIT;
                        }
                        if (mutiranger_down_isClose()) {
                            pData->proximity_flags |= PROXIMITY_DOWN_BIT;
                        }

                        if (pData->proximity_flags != 0) {
                            leds_all_on();
                        } else {
                            leds_all_off();
                        }

                        radio_setFrequency(CHANNEL, FREQ_TX);
                        radio_loadPacket((uint8_t*)pData, sizeof(command_payload_t));
                        radio_txEnable();
                        radio_txNow();
                        app_vars.state = APP_STATE_TX;
                    }
                }
                app_vars.flags &= ~APP_FLAG_TIMER;
            }
        }
    }
}

//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
    app_vars.flags |= APP_FLAG_START_FRAME;
}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {
    app_vars.flags |= APP_FLAG_END_FRAME;
}

void cb_timer(void) {
    app_vars.flags |= APP_FLAG_TIMER;
    sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);
}