/**
 * Crazyflie Receiver - Receive proximity status data from other drones
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
#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"

//=========================== defines =========================================

#define LENGTH_PACKET   125+LENGTH_CRC
#define LEN_PKT_TO_SEND 20+LENGTH_CRC
#define CHANNEL         26
#define TIMER_PERIOD    (0xffff>>4)
#define ID              0x99

#define PACKET_TYPE_MULTIRANGER 0x01
#define PACKET_TYPE_COMMAND 0x02

#define STARTUP_DELAY_MS 3000
#define HEARTBEAT_INTERVAL_MS 500  
#define CRAZYFLIE_HANDLE_INTERVAL_MS 5  

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

// Define packet structure
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
    uint8_t  type;            // Packet type
    uint8_t  drone_id;        // Drone ID
    uint8_t  proximity_flags; // Bitfield: bit0=front, bit1=back, bit2=left, bit3=right, bit4=up, bit5=down
} command_payload_t;

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

        if (app_vars.flags) {
            crazyflieHandle();
        }

        if (!enHighLevel && tick > STARTUP_DELAY_MS) {
            high_level_enable();
            mutiranger_init();
            radio_init();
            radio_setStartFrameCb(cb_startFrame);
            radio_setEndFrameCb(cb_endFrame);
            radio_rfOn();
            radio_setFrequency(CHANNEL, FREQ_RX);
            radio_rxEnable();
            radio_rxNow();  // Restart reception
            enHighLevel = true; // Lock to prevent re-entry
        }

        if (enHighLevel) {
            if (tick - last_heartbeat_tick >= HEARTBEAT_INTERVAL_MS) {
                high_level_stop(); 
                last_heartbeat_tick = tick;
            }
        }

        // Handle flags
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
                    // Receiver: Handle received packets
                    if (enHighLevel) {
                        uint8_t rx_buffer[128];
                        uint8_t rx_len;
                        int8_t rssi;
                        uint8_t lqi;
                        uint8_t crc_valid;  n

                        // Read received packet
                        radio_getReceivedFrame(rx_buffer, &rx_len, sizeof(rx_buffer),
                                              &rssi, &lqi, &crc_valid);

                        // command_payload_t structure: type(1) + drone_id(1) + proximity_flags(1) = 3 bytes
                        if (crc_valid != 0) {
                            // Print raw packet info (for debugging)
                            SEGGER_RTT_printf(0, "[RX] Raw: Len=%d Expected:%d\n", rx_len, sizeof(command_payload_t));
                            
                            if (rx_len >= sizeof(command_payload_t)) {
                                command_payload_t *pData = (command_payload_t *)rx_buffer;

                                // Print raw packet content (for debugging)
                                SEGGER_RTT_printf(0, "[RX] Bytes: ");
                                for (int i = 0; i < (rx_len < 8 ? rx_len : 8); i++) {
                                    SEGGER_RTT_printf(0, "%02X ", rx_buffer[i]);
                                }
                                SEGGER_RTT_printf(0, "\n");

                                // Check packet type
                                if (pData->type == PACKET_TYPE_COMMAND) {
                                    uint8_t flags = pData->proximity_flags;

                                    // Print received data
                                    SEGGER_RTT_printf(0, "[RX] DroneID:%d Flags:0x%02X RSSI:%d LQI:%d\n",
                                                      pData->drone_id, flags, rssi, lqi);

                                    // Print proximity status for each direction
                                    SEGGER_RTT_printf(0, "  Front:%s Back:%s Left:%s Right:%s Up:%s Down:%s\n",
                                                      (flags & PROXIMITY_FRONT_BIT) ? "CLOSE" : "OK  ",
                                                      (flags & PROXIMITY_BACK_BIT)  ? "CLOSE" : "OK  ",
                                                      (flags & PROXIMITY_LEFT_BIT)  ? "CLOSE" : "OK  ",
                                                      (flags & PROXIMITY_RIGHT_BIT) ? "CLOSE" : "OK  ",
                                                      (flags & PROXIMITY_UP_BIT)    ? "CLOSE" : "OK  ",
                                                      (flags & PROXIMITY_DOWN_BIT)  ? "CLOSE" : "OK  ");

                                    // Execute corresponding action based on received proximity status
                                    if (flags & PROXIMITY_FRONT_BIT) {
                                        // Front proximity - can execute corresponding action, e.g., stop moving forward
                                        // TODO: Add specific action
                                    }

                                    if (flags & PROXIMITY_BACK_BIT) {
                                        // Back proximity - can execute corresponding action
                                        // TODO: Add specific action
                                    }

                                    if (flags & PROXIMITY_LEFT_BIT) {
                                        // Left proximity - can execute corresponding action
                                        // TODO: Add specific action
                                    }

                                    if (flags & PROXIMITY_RIGHT_BIT) {
                                        // Right proximity - can execute corresponding action
                                        // TODO: Add specific action
                                    }

                                    if (flags & PROXIMITY_UP_BIT) {
                                        // Up proximity - Turn on LEDs as warning
                                        leds_all_on();
                                    }

                                    if (flags & PROXIMITY_DOWN_BIT) {
                                        // Down proximity - can execute corresponding action
                                        // TODO: Add specific action
                                    }

                                    // If no proximity in any direction, turn off LEDs
                                    if (flags == 0) {
                                        leds_all_off();
                                    }
                                } else {
                                    SEGGER_RTT_printf(0, "[RX] Unknown packet type: 0x%02X\n", pData->type);
                                }
                            } else {
                                // Packet length insufficient
                                SEGGER_RTT_printf(0, "[RX] Packet too short! Len:%d Need:%d\n",
                                                  rx_len, sizeof(command_payload_t));
                            }
                        } else {
                            // CRC Error
                            SEGGER_RTT_printf(0, "[RX] CRC Error! Len:%d\n", rx_len);
                        }
                    }
                }

                // Immediately re-enable reception after handling packet (Critical!)
                radio_rxEnable();
                radio_rxNow();
                
                app_vars.flags &= ~APP_FLAG_END_FRAME;
            }

            if (app_vars.flags & APP_FLAG_TIMER) {
                // Receiver does not need timed transmission, just maintain receive state
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