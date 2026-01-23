/**
 * Crazyflie Sender - Based  51DK code
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
#define HEARTBEAT_INTERVAL_MS 500  // Send control command every 500ms to feed watchdog, ensure STM32 won't restart
#define CRAZYFLIE_HANDLE_INTERVAL_MS 5  // Call crazyflieHandle() every 5ms to maintain syslink heartbeat

int last_heartbeat_tick = 0;
int last_crazyflie_handle_tick = 0;
int last_sync_tick = 0;      // Time of last sync flag transmission
uint8_t sync_sequence = 0;   // Sync sequence number


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
    bool sync_triggered;        
    int last_action_tick;       
} sync_flight_t;

sync_flight_t sync_flight = {SYNC_STATE_IDLE, 0, false, 0};

#define SYNC_TAKEOFF_DELAY_MS   10000  // Delay 10 seconds after sync before takeoff
#define TAKEOFF_HEIGHT          0.5f   // Takeoff height (meters)
#define TAKEOFF_DURATION        2.0f   // Takeoff duration (seconds)
#define FLYING_DURATION_MS      30000  // Flight duration (30 seconds)
#define MOVE_DISTANCE           0.3f   // Move distance (meters)
#define MOVE_DURATION           1.0f   // Move duration (seconds)
#define ACTION_COOLDOWN_MS      500    // Action cooldown time (debouncing, 500ms)
#define YAW                     0.0f   // Yaw angle (radians)

#define PACKET_TYPE_COMMAND 0x02
#define PACKET_TYPE_SYNC 0x03  // Sync flag packet
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

// Define packet structures
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
    uint8_t  proximity_flags; // Bit field: bit0=front, bit1=back, bit2=left, bit3=right, bit4=up, bit5=down
} command_payload_t;

// Sync flag packet structure (includes sync flag and multiranger data)
typedef struct __attribute__((packed)) {
    uint8_t  type;           
    uint8_t  sequence;      
    uint8_t  proximity_flags; 
} sync_payload_t;

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

        // Initialization process (executed only once)
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
            // Heartbeat: only send stop command in non-flying state (to avoid overriding takeoff and flight commands)
            if (tick - last_heartbeat_tick >= HEARTBEAT_INTERVAL_MS) {
                if (!sync_flight.sync_triggered || 
                    sync_flight.state == SYNC_STATE_IDLE || 
                    sync_flight.state == SYNC_STATE_WAIT_TAKEOFF || 
                    sync_flight.state == SYNC_STATE_STOP) {
                    high_level_stop(); // Send valid control packet to feed watchdog, reset STM32 anti-shutdown countdown
                }
                last_heartbeat_tick = tick;
            }
            
            // Process synchronized flight sequence state machine (master)
            if (sync_flight.sync_triggered) {
                int state_elapsed = tick - sync_flight.state_start_tick;
                
                switch (sync_flight.state) {
                    case SYNC_STATE_IDLE:
                        // Trigger sync, enter wait for takeoff state
                        if (sync_flight.sync_triggered) {
                            sync_flight.state = SYNC_STATE_WAIT_TAKEOFF;
                            sync_flight.state_start_tick = tick;
                        }
                        break;
                        
                    case SYNC_STATE_WAIT_TAKEOFF:
                        // Wait 10 seconds then takeoff
                        if (state_elapsed >= SYNC_TAKEOFF_DELAY_MS) {
                            high_level_takeoff(TAKEOFF_HEIGHT, TAKEOFF_DURATION, YAW);
                            sync_flight.state = SYNC_STATE_TAKEOFF;
                            sync_flight.state_start_tick = tick;
                        }
                        break;
                        
                    case SYNC_STATE_TAKEOFF:
                        // After takeoff completes, enter flying state (test mode: shortened time)
                        if (state_elapsed >= (int)(TAKEOFF_DURATION * 1000)) {
                            sync_flight.state = SYNC_STATE_FLYING;
                            sync_flight.state_start_tick = tick;
                        }
                        break;
                        
                    case SYNC_STATE_FLYING:

                        if (state_elapsed >= FLYING_DURATION_MS) {
                            high_level_stop();
                            sync_flight.state = SYNC_STATE_STOP;
                            sync_flight.state_start_tick = tick;
                        }
                        // Note: multiranger data processing is in the timer packet transmission location
                        break;
                        
                    case SYNC_STATE_STOP:
                        // Stop state
                        break;
                }
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

                        bool send_sync = false;
                        
                        // Example 1: periodically send sync flag (5 seconds after startup, send continuously for 2 seconds to ensure receiver can receive)
                        if (tick > 5000 && tick < 7000) {
                            // During continuous sending period, send sync packet every time
                            send_sync = true;
                            
                            // Only increment sequence number and initialize state machine on first trigger
                            if (last_sync_tick == 0) {
                                sync_sequence++;
                                last_sync_tick = tick;
                                
                                // Master also triggers its own synchronized flight sequence (only once)
                                if (!sync_flight.sync_triggered) {
                                    sync_flight.sync_triggered = true;
                                    sync_flight.state = SYNC_STATE_IDLE;
                                    sync_flight.state_start_tick = tick;
                                    sync_flight.last_action_tick = 0;
                                }
                            }
                        }
                        // Stop continuous sending (after 2 seconds)
                        else if (tick >= 7000 && last_sync_tick > 0 && last_sync_tick < 7000) {
                            last_sync_tick = tick; // Mark that sending has stopped
                        }
                        // Example 2: 

                        if (send_sync) {
                            // Send sync flag packet (also includes multiranger data)
                            sync_payload_t *pSync = (sync_payload_t *)app_vars.packet;
                            pSync->type = PACKET_TYPE_SYNC;
                            pSync->sequence = sync_sequence;
                            
                            // Also includes multiranger proximity status data
                            pSync->proximity_flags = 0;
                            if (mutiranger_front_isClose()) {
                                pSync->proximity_flags |= PROXIMITY_FRONT_BIT;
                            }
                            if (mutiranger_back_isClose()) {
                                pSync->proximity_flags |= PROXIMITY_BACK_BIT;
                            }
                            if (mutiranger_left_isClose()) {
                                pSync->proximity_flags |= PROXIMITY_LEFT_BIT;
                            }
                            if (mutiranger_right_isClose()) {
                                pSync->proximity_flags |= PROXIMITY_RIGHT_BIT;
                            }
                            if (mutiranger_up_isClose()) {
                                pSync->proximity_flags |= PROXIMITY_UP_BIT;
                            }
                            if (mutiranger_down_isClose()) {
                                pSync->proximity_flags |= PROXIMITY_DOWN_BIT;
                            }
                            
                            radio_setFrequency(CHANNEL, FREQ_TX);
                            radio_loadPacket((uint8_t*)pSync, sizeof(sync_payload_t));
                            radio_txEnable();
                            radio_txNow();
                            app_vars.state = APP_STATE_TX;
                        } else {
                            // Normally send proximity status packet
                            command_payload_t *pData = (command_payload_t *)app_vars.packet;
                            pData->type     = PACKET_TYPE_COMMAND; // 0x02
                            pData->drone_id = 1;
                            
                            // Check proximity status of all directions and set corresponding bits
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
                            
                            // In flying state, execute obstacle avoidance actions based on multiranger data (master, only handle left-right directions)
                            if (sync_flight.state == SYNC_STATE_FLYING) {
                                // Debouncing: limit action frequency
                                if (tick - sync_flight.last_action_tick >= ACTION_COOLDOWN_MS) {
                                    sync_flight.last_action_tick = tick;
                                    if (pData->proximity_flags & PROXIMITY_RIGHT_BIT) {
                                        high_level_goto(0.0f, -MOVE_DISTANCE, 0.0f, YAW, MOVE_DURATION, true);
                                    }
                                    // Obstacle on left → move right (positive Y direction)
                                    else if (pData->proximity_flags & PROXIMITY_LEFT_BIT) {
                                        high_level_goto(0.0f, MOVE_DISTANCE, 0.0f, YAW, MOVE_DURATION, true);
                                    }
                                }
                            }

                            radio_setFrequency(CHANNEL, FREQ_TX);
                            radio_loadPacket((uint8_t*)pData, sizeof(command_payload_t));
                            radio_txEnable();
                            radio_txNow();
                            app_vars.state = APP_STATE_TX;
                        }
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