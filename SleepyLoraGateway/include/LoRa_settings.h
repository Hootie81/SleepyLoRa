// Heltec - SX126x pin configuration
int PIN_LORA_RESET = 12;
int PIN_LORA_BUSY = 13;
int PIN_LORA_DIO_1 = 14;
int PIN_LORA_NSS = 8;
int PIN_LORA_SCLK = 9;
int PIN_LORA_MISO = 11;
int PIN_LORA_MOSI = 10;
int RADIO_TXEN = -1;
int RADIO_RXEN = -1;

/** Number of retries if CAD shows busy */
#define CAD_RETRY 20
#define LORA_TIMEOUT 20000

// Define LoRa parameters
// #define RF_FREQUENCY 915000000 // Hz
// Use config.rf_frequency instead of hardcoded define
#define TX_OUTPUT_POWER 22      // dBm
#define LORA_BANDWIDTH 0        // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1       // [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8 // Increased for Blinds: 60ms sleep + 30ms Rx + buffer
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 3000
#define TX_TIMEOUT_VALUE 3000

#define BUFFER_SIZE 512 // Define the payload size here

#define TIME_SYNC_RETRIES 3
#define TIME_SYNC_RATE_LIMIT 5  //s  timeout after setting clock