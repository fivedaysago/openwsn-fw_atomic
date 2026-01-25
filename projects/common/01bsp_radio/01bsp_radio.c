/**
 * Crazyflie Receiver - receive the proximity data from the other Crazyflie
 */

//nrf
#include "board.h"
#include "radio.h"
#include "leds.h"
#include "sctimer.h"
//cf
#include "cf_crazyflie.h"
#include "cf_pm.h"
#include "cf_systick.h"
#include "cf_api_commander_high_level.h"
#include "single_status_led.h"
//=========================== defines =========================================

#define LENGTH_PACKET   125+LENGTH_CRC
#define LEN_PKT_TO_SEND 20+LENGTH_CRC
#define CHANNEL         26  
#define TIMER_PERIOD    (0xffff>>4)
#define ID              0x99

#define PACKET_TYPE_MULTIRANGER 0x01 // multiranger packet type
#define PACKET_TYPE_COMMAND 0x02 // command packet type
#define PACKET_TYPE_SYNC 0x03  // sync packet type

#define STARTUP_DELAY_MS 3000
#define HEARTBEAT_INTERVAL_MS 500 // heartbeat interval
#define CRAZYFLIE_HANDLE_INTERVAL_MS 5 // crazyflie handle interval
#define PROXIMITY_FRONT_BIT  (1 << 0)
#define PROXIMITY_BACK_BIT   (1 << 1) // proximity back bit
#define PROXIMITY_LEFT_BIT   (1 << 2) // proximity left bit
#define PROXIMITY_RIGHT_BIT  (1 << 3) // proximity right bit
#define PROXIMITY_UP_BIT     (1 << 4) // proximity up bit
#define PROXIMITY_DOWN_BIT   (1 << 5) // proximity down bit

//=========================== variables =======================================

int tick;
bool enHighLevel = false;
int last_heartbeat_tick = 0;
int last_crazyflie_handle_tick = 0;

typedef enum {
   SYNC_STATE_IDLE = 0,
   SYNC_STATE_WAIT_TAKEOFF,
   SYNC_STATE_TAKEOFF,
   SYNC_STATE_FLYING,
   SYNC_STATE_STOP
} sync_flight_state_t;

typedef struct {
   sync_flight_state_t state;
   int state_start_tick;
   bool sync_received;
   uint8_t last_sequence;
   int last_action_tick;
} sync_flight_t;

sync_flight_t sync_flight = {SYNC_STATE_IDLE, 0, false, 0, 0};

#define SYNC_TAKEOFF_DELAY_MS   10000 // sync takeoff delay
#define TAKEOFF_HEIGHT          0.5f // takeoff height
#define TAKEOFF_DURATION        1.0f // takeoff duration
#define FLYING_DURATION_MS      30000 // flying duration
#define MOVE_DISTANCE           0.2f // move distance
#define MOVE_DURATION           1.0f // move duration
#define ACTION_COOLDOWN_MS      500 // action cooldown
#define YAW                     0.0f // yaw

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
   uint8_t  type;
   uint8_t  drone_id;
   uint8_t  proximity_flags;
} command_payload_t;

typedef struct __attribute__((packed)) {
   uint8_t  type;
   uint8_t  sequence;
   uint8_t  proximity_flags;
} sync_payload_t;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);
void     cb_timer(void);

//=========================== main ============================================

int mote_main(void) {
   memset(&app_vars, 0, sizeof(app_vars_t));

    board_init();
    crazyflieInit(); 

    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    sctimer_set_callback(cb_timer);
   sctimer_setCompare(sctimer_readCounter() + TIMER_PERIOD);
    sctimer_enable();

    radio_rfOn();
    radio_setFrequency(CHANNEL, FREQ_RX);
    radio_rxEnable();
   radio_rxNow();
    app_vars.state = APP_STATE_RX;
    app_vars.flags |= APP_FLAG_TIMER;

    while (1) {
        tick = systickGetTick();

        if (tick - last_crazyflie_handle_tick >= CRAZYFLIE_HANDLE_INTERVAL_MS) {
            crazyflieHandle();
            last_crazyflie_handle_tick = tick;
        }

       if (app_vars.flags) {
           crazyflieHandle();
       }

        if (!enHighLevel && tick > STARTUP_DELAY_MS) {
            high_level_enable();
            radio_init(); 
            radio_setStartFrameCb(cb_startFrame); 
            radio_setEndFrameCb(cb_endFrame);
            radio_rfOn();           
            radio_setFrequency(CHANNEL, FREQ_RX);
            radio_rxEnable();
            radio_rxNow();
            enHighLevel = true;
        }

        if (enHighLevel) {
            if (tick - last_heartbeat_tick >= HEARTBEAT_INTERVAL_MS) {
                if (!sync_flight.sync_received || 
                    sync_flight.state == SYNC_STATE_IDLE || 
                    sync_flight.state == SYNC_STATE_WAIT_TAKEOFF || 
                    sync_flight.state == SYNC_STATE_STOP) {
                    high_level_stop();
                }
                last_heartbeat_tick = tick;
           }
           
           if (sync_flight.sync_received) {
               int state_elapsed = tick - sync_flight.state_start_tick;
               
               switch (sync_flight.state) {
                   case SYNC_STATE_IDLE:
                       if (sync_flight.sync_received) {
                           sync_flight.state = SYNC_STATE_WAIT_TAKEOFF;
                           sync_flight.state_start_tick = tick;
                       }
                       break;
                       
                   case SYNC_STATE_WAIT_TAKEOFF:
                       if (state_elapsed >= SYNC_TAKEOFF_DELAY_MS) {
                           //high_level_takeoff(TAKEOFF_HEIGHT, TAKEOFF_DURATION, YAW);
                           leds_error_off();
                           for (volatile uint8_t i = 0; i < 10; i++) {
                               leds_error_toggle();
                               for (volatile uint32_t j = 0; j < 0x1ffff; j++);
                               if (i % 2 == 0) {
                                   crazyflieHandle();
                               }
                           }
                           sync_flight.state = SYNC_STATE_TAKEOFF;
                           sync_flight.state_start_tick = systickGetTick();
                       }
                       break;
                       
                   case SYNC_STATE_TAKEOFF:
                       if (state_elapsed >= (int)(TAKEOFF_DURATION * 1000)) {
                           sync_flight.state = SYNC_STATE_FLYING;
                           sync_flight.state_start_tick = tick;
                       }
                       break;
                       
                   case SYNC_STATE_FLYING:
                       if (state_elapsed >= FLYING_DURATION_MS) {
                           //high_level_stop();
                           sync_flight.state = SYNC_STATE_STOP;
                           sync_flight.state_start_tick = tick;
                       }
                       break;
                       
                   case SYNC_STATE_STOP:
                       break;
               }
           }
       }

        while (app_vars.flags) {
            if (app_vars.flags & APP_FLAG_START_FRAME) {
                app_vars.flags &= ~APP_FLAG_START_FRAME;
               crazyflieHandle();
            }

            if (app_vars.flags & APP_FLAG_END_FRAME) {
                if (app_vars.state == APP_STATE_TX) {
                    radio_rxEnable();
                    radio_rxNow();
                    app_vars.state = APP_STATE_RX;
               } else if (app_vars.state == APP_STATE_RX) {
                   if (enHighLevel) {
                       uint8_t rx_buffer[128];
                       uint8_t rx_len;
                       int8_t rssi;
                       uint8_t lqi;
                       uint8_t crc_valid;

                       radio_getReceivedFrame(rx_buffer, &rx_len, sizeof(rx_buffer),
                                             &rssi, &lqi, &crc_valid);

                       if (crc_valid != 0 && rx_len >= 1) {
                           uint8_t packet_type = rx_buffer[0];
                           uint8_t proximity_flags = 0;
                           
                           if (packet_type == PACKET_TYPE_COMMAND && rx_len >= sizeof(command_payload_t)) {
                               command_payload_t *pData = (command_payload_t *)rx_buffer;
                               proximity_flags = pData->proximity_flags;
                           }
                           else if (packet_type == PACKET_TYPE_SYNC && rx_len >= sizeof(sync_payload_t)) {
                               sync_payload_t *pSync = (sync_payload_t *)rx_buffer;
                               proximity_flags = pSync->proximity_flags;
                               
                               if (!sync_flight.sync_received || pSync->sequence != sync_flight.last_sequence) {
                                   sync_flight.sync_received = true;
                                   sync_flight.last_sequence = pSync->sequence;
                                   sync_flight.state = SYNC_STATE_IDLE;
                                   sync_flight.state_start_tick = tick;
                                   sync_flight.last_action_tick = 0;
                               }
                           }
                           
                           if (proximity_flags != 0 && sync_flight.state == SYNC_STATE_FLYING) {
                               if (tick - sync_flight.last_action_tick >= ACTION_COOLDOWN_MS) {
                                   sync_flight.last_action_tick = tick;
                                   
                                //    if (proximity_flags & PROXIMITY_RIGHT_BIT) {
                                //        //high_level_goto(0.0f, MOVE_DISTANCE, 0.0f, YAW, MOVE_DURATION, true);
                                //        //status_led_set(LED_ON);
                                //     }
                                //    else if (proximity_flags & PROXIMITY_LEFT_BIT) {
                                //        //high_level_goto(0.0f, -MOVE_DISTANCE, 0.0f, YAW, MOVE_DURATION, true);
                                //        status_led_set(LED_ON);
                                //     }
                                if (proximity_flags & PROXIMITY_LEFT_BIT) {
                                    status_led_set(LED_ON);
                                }
                               }
                           }
                       }
                   }
               }

               radio_rxEnable();
               radio_rxNow();
               
               app_vars.flags &= ~APP_FLAG_END_FRAME;
           }

           if (app_vars.flags & APP_FLAG_TIMER) {
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
   sctimer_setCompare(sctimer_readCounter() + TIMER_PERIOD);
}
