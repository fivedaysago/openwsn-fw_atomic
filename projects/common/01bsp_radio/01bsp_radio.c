/**
 * Crazyflie Receiver - Receive proximity data from other drones
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
#define PACKET_TYPE_COMMAND 0x02
#define PACKET_TYPE_SYNC 0x03  // Sync flag packet

#define STARTUP_DELAY_MS 3000
#define HEARTBEAT_INTERVAL_MS 500  // Send control command every 500ms to feed watchdog, ensure STM32 won't restart
#define CRAZYFLIE_HANDLE_INTERVAL_MS 5  // Call crazyflieHandle() every 5ms to maintain syslink heartbeat

// Proximity status bit definitions
#define PROXIMITY_FRONT_BIT  (1 << 0)
#define PROXIMITY_BACK_BIT   (1 << 1)
#define PROXIMITY_LEFT_BIT   (1 << 2)
#define PROXIMITY_RIGHT_BIT  (1 << 3)
#define PROXIMITY_UP_BIT     (1 << 4)
#define PROXIMITY_DOWN_BIT   (1 << 5)

//=========================== variables =======================================

int tick;
bool enHighLevel = false;
int last_heartbeat_tick = 0;
int last_crazyflie_handle_tick = 0;

// Synchronized flight sequence state machine
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


#define SYNC_TAKEOFF_DELAY_MS   10000  // Delay 10 seconds after sync before takeoff
#define TAKEOFF_HEIGHT          0.5f   // Takeoff height (meters)
#define TAKEOFF_DURATION        2.0f   // Takeoff duration (seconds)
#define FLYING_DURATION_MS      30000  // Flight duration (30 seconds)
#define MOVE_DISTANCE           0.3f   // Move distance (meters)
#define MOVE_DURATION           1.0f   // Move duration (seconds)
#define ACTION_COOLDOWN_MS      500    // Action cooldown time (debouncing, 500ms)
#define YAW                     0.0f   // Yaw angle (radians)

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
   uint8_t  type;           // Packet type
   uint8_t  drone_id;       // Drone ID
   uint8_t  proximity_flags; // Bit field: bit0=front, bit1=back, bit2=left, bit3=right, bit4=up, bit5=down
} command_payload_t;


typedef struct __attribute__((packed)) {
   uint8_t  type;           // Packet type = PACKET_TYPE_SYNC
   uint8_t  sequence;      // Sequence number (optional, used to distinguish different sync points)
   uint8_t  proximity_flags; // Bit field: bit0=front, bit1=back, bit2=left, bit3=right, bit4=up, bit5=down
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

    // Initialize Radio callbacks
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    sctimer_set_callback(cb_timer);
   sctimer_setCompare(sctimer_readCounter() + TIMER_PERIOD);
    sctimer_enable();

    // Initial Radio configuration
    radio_rfOn();
    radio_setFrequency(CHANNEL, FREQ_RX);
    radio_rxEnable();
   radio_rxNow();  // Start receiving immediately
    app_vars.state = APP_STATE_RX;
    app_vars.flags |= APP_FLAG_TIMER;

    while (1) {
        tick = systickGetTick();


        if (tick - last_crazyflie_handle_tick >= CRAZYFLIE_HANDLE_INTERVAL_MS) {
            crazyflieHandle();
            last_crazyflie_handle_tick = tick;
        }

       // Also call in flag processing to ensure timely response
       if (app_vars.flags) {
           crazyflieHandle();
       }

        // Initialization process (executed only once)
        if (!enHighLevel && tick > STARTUP_DELAY_MS) {
            high_level_enable();
            mutiranger_init();
            radio_init(); 
            radio_setStartFrameCb(cb_startFrame); 
            radio_setEndFrameCb(cb_endFrame);
            radio_rfOn();           
           radio_setFrequency(CHANNEL, FREQ_RX);
           radio_rxEnable();
           radio_rxNow();  // Restart receiving
            enHighLevel = true; // Lock to prevent re-entry
        }

        if (enHighLevel) {
            // Heartbeat: only send stop command in non-flying state (to avoid overriding takeoff and flight commands)
            if (tick - last_heartbeat_tick >= HEARTBEAT_INTERVAL_MS) {
                // Only send stop as heartbeat in IDLE, WAIT_TAKEOFF, or STOP states
                // In TAKEOFF and FLYING states, flight commands themselves maintain communication, no stop needed
                if (!sync_flight.sync_received || 
                    sync_flight.state == SYNC_STATE_IDLE || 
                    sync_flight.state == SYNC_STATE_WAIT_TAKEOFF || 
                    sync_flight.state == SYNC_STATE_STOP) {
                    high_level_stop(); // Send valid control packet to feed watchdog, reset STM32 anti-shutdown countdown
                }
                last_heartbeat_tick = tick;
           }
           
           // Process synchronized flight sequence state machine
           if (sync_flight.sync_received) {
               int state_elapsed = tick - sync_flight.state_start_tick;
               
               switch (sync_flight.state) {
                   case SYNC_STATE_IDLE:
                       // Received sync flag, enter wait for takeoff state
                       if (sync_flight.sync_received) {
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
                       // Note: In FLYING state, multiranger data processing is in the packet reception location
                       break;
                       
                   case SYNC_STATE_STOP:
                       // Stop state, can add other logic here
                       // If reset is needed, can set sync_flight.sync_received = false;
                       break;
               }
           }
       }

       // Process flags
        while (app_vars.flags) {
            if (app_vars.flags & APP_FLAG_START_FRAME) {
                app_vars.flags &= ~APP_FLAG_START_FRAME;
               crazyflieHandle(); // Maintain communication
            }

            if (app_vars.flags & APP_FLAG_END_FRAME) {
                if (app_vars.state == APP_STATE_TX) {
                    radio_rxEnable();
                    radio_rxNow();
                    app_vars.state = APP_STATE_RX;
               } else if (app_vars.state == APP_STATE_RX) {
                   // Receiver: process received packets
                   if (enHighLevel) {
                       uint8_t rx_buffer[128];
                       uint8_t rx_len;
                       int8_t rssi;
                       uint8_t lqi;
                       uint8_t crc_valid;  // Use uint8_t instead of bool to match function declaration

                       // Read received packet
                       radio_getReceivedFrame(rx_buffer, &rx_len, sizeof(rx_buffer),
                                             &rssi, &lqi, &crc_valid);

                       // Check CRC and packet length
                       if (crc_valid != 0 && rx_len >= 1) {
                           uint8_t packet_type = rx_buffer[0];
                           
                           // Process proximity status packet
                           if (packet_type == PACKET_TYPE_COMMAND && rx_len >= sizeof(command_payload_t)) {
                               command_payload_t *pData = (command_payload_t *)rx_buffer;
                               uint8_t flags = pData->proximity_flags;

                               // In flying state, execute obstacle avoidance actions based on multiranger data (only handle left-right directions)
                               if (sync_flight.state == SYNC_STATE_FLYING) {
                                   // Debouncing: limit action frequency
                                   if (tick - sync_flight.last_action_tick >= ACTION_COOLDOWN_MS) {
                                       sync_flight.last_action_tick = tick;
                                       
                                       if (flags & PROXIMITY_RIGHT_BIT) {
                                           high_level_goto(0.0f, -MOVE_DISTANCE, 0.0f, YAW, MOVE_DURATION, true);
                                       }
                                       // Obstacle on left → move right (positive Y direction)
                                       else if (flags & PROXIMITY_LEFT_BIT) {
                                           high_level_goto(0.0f, MOVE_DISTANCE, 0.0f, YAW, MOVE_DURATION, true);
                                       }
                                   }
                               }
                           }
                           // Process sync flag packet (also includes multiranger data)
                           else if (packet_type == PACKET_TYPE_SYNC && rx_len >= sizeof(sync_payload_t)) {
                               sync_payload_t *pSync = (sync_payload_t *)rx_buffer;
                               
                               // Process multiranger proximity status data
                               uint8_t flags = pSync->proximity_flags;

                               // In flying state, execute obstacle avoidance actions based on multiranger data (only handle left-right directions)
                               if (sync_flight.state == SYNC_STATE_FLYING) {
                                   // Debouncing: limit action frequency
                                   if (tick - sync_flight.last_action_tick >= ACTION_COOLDOWN_MS) {
                                       sync_flight.last_action_tick = tick;
                                       

                                       if (flags & PROXIMITY_RIGHT_BIT) {
                                           high_level_goto(0.0f, -MOVE_DISTANCE, 0.0f, YAW, MOVE_DURATION, true);
                                       }
                                       // Obstacle on left → move right (positive Y direction)
                                       else if (flags & PROXIMITY_LEFT_BIT) {
                                           high_level_goto(0.0f, MOVE_DISTANCE, 0.0f, YAW, MOVE_DURATION, true);
                                       }
                                   }
                               }
                               
                               // Received sync flag, trigger local flight sequence
                               if (!sync_flight.sync_received || pSync->sequence != sync_flight.last_sequence) {
                                   sync_flight.sync_received = true;
                                   sync_flight.last_sequence = pSync->sequence;
                                   sync_flight.state = SYNC_STATE_IDLE;  // Reset state machine
                                   sync_flight.state_start_tick = tick;
                                   sync_flight.last_action_tick = 0;     // Reset action time
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
               // Receiver doesn't need periodic transmission, only needs to maintain reception state
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
