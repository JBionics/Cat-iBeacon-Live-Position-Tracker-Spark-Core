#include "SparkIntervalTimer/SparkIntervalTimer.h"

// Timing Parameters - affects power consumption
#define DEEP_SLEEP_TIME_SECONDS (50)
#define PUBLISH_EVERY_X_RUNS (10)

#define SCAN_TIME_ALLOWED_SECONDS (10)

// these are safe guards using hardware timer interrupts for if the
// regular sleep mechanism fails for some reason
#define AWAKE_TIME_ALLOWED_SECONDS (15)
#define PUBLISH_TIME_ALLOWED_SECONDS (20)

// Eeprom Slots
#define SLOT_RUN_NUM (0)
#define SLOT_NUM_READINGS (1)
#define SLOT_CURRENT_SECTOR (2)
#define SLOT_SECTOR_POS (3)


#define FLASH_ADDR (0x80000)
#define SECTOR_SIZE (0x1000) // 4 kB
#define BLE_READING_BUFFER_SIZE (16)

#define JBDEBUG (0)

// BLE Defines
#define BUILD_UINT16(loByte, hiByte) ((uint16_t)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))
#define DEVICE_INITIALIZED 0x600
#define DEVICE_DISCOVERY_DONE 0x601
#define DEVICE_INFORMATION 0x60D

// Globals
static uint8_t buf[64];
static char szInfo[63];
static IntervalTimer sleepTimer;
static unsigned long start;
static uint8_t runNum;

void setup() {
    start = millis();
    
    runNum = EEPROM.read(SLOT_RUN_NUM);
    EEPROM.write(SLOT_RUN_NUM, runNum + 1);
    
    // Every X runs, don't go to sleep, so we can publish the results
    if (runNum % PUBLISH_EVERY_X_RUNS != 0) {
        Spark.sleep(1000); // don't wake up!
    } 
    
    sleepTimer.begin(sleepCallback, AWAKE_TIME_ALLOWED_SECONDS * 1000 * 2, hmSec);
    
    sFLASH_Init();
    
    Serial1.begin(57600);
    
    // Pull RST on BLE module low so module powers up
    pinMode(D0, OUTPUT);
    digitalWrite(D0, LOW);
    delay(100); //JBTODO: needed?
    
    #if JBDEBUG
    Spark.publish("Setup!", "Hello World");
    delay(1000);
    publishCachedData();
    #endif
    
    // Initialize ble mini
    hci_init();    
}

void loop(){ 
    if (Serial1.available()){
        ble_event_process();
    }
    if ((millis() - start) > (SCAN_TIME_ALLOWED_SECONDS * 1000L)) {
        publishAndSleep();
    }
}

static uint8_t timesCalled = 0;
// Callback for Timer 1
void sleepCallback(void) {
    ++timesCalled;
    // Timer is called immediately on setup, so need to wait for the second call
    if (timesCalled > 1) {
        Spark.sleep(SLEEP_MODE_DEEP, DEEP_SLEEP_TIME_SECONDS);
    }
}

void publishAndSleep()
{
    if (runNum % PUBLISH_EVERY_X_RUNS == 0 && Spark.connected()) {
        publishCachedData();
    }
    Spark.sleep(SLEEP_MODE_DEEP, DEEP_SLEEP_TIME_SECONDS);
}

void publishCachedData()
{
    uint8_t numReadings = EEPROM.read(SLOT_NUM_READINGS);
    //if (numReadings == 0) return; // Uncomment to skip publishing if no readings

    // Update sleep timer backup
    // We go to sleep when finished publishing ... but if we get stuck, this should kick in instead
    sleepTimer.resetPeriod_SIT(PUBLISH_TIME_ALLOWED_SECONDS * 1000 * 2, hmSec);

    // Clear readings
    EEPROM.write(SLOT_NUM_READINGS, 0);

    uint8_t sectorPos = EEPROM.read(SLOT_SECTOR_POS);
    uint8_t currentSector = EEPROM.read(SLOT_CURRENT_SECTOR);
    
    uint8_t buffer[BLE_READING_BUFFER_SIZE*4];
    
    // 256 slots (8 bytes each), using all 256 sectors in total in this circular buffer
    uint32_t addr = (FLASH_ADDR + (currentSector * SECTOR_SIZE) + (sectorPos * BLE_READING_BUFFER_SIZE) - (numReadings * BLE_READING_BUFFER_SIZE)) % 0x200000; // mod end of external flash
    
    sprintf(szInfo, "Run: %d NumReadings: %d addr %x", runNum, numReadings, addr);
    Spark.publish("device_readings", szInfo);

    uint8_t eventsSent = 0;
    
    for (uint8_t i = 0; i < numReadings; i += 4) {
        if (eventsSent++ % 3 == 0) delay(1500); // delay every 3 msgs
        sFLASH_ReadBuffer(buffer, addr, sizeof(buffer));
        // String containing: beacon id (2 digit) - run number (1 digit) - tx power (3 digit) - rssi (3 digit) - seconds into run (1 digit)
        // Get 4 readings into 1 publish event
        // Burst rate is 4 per second with a max of 60 per minute
        sprintf(szInfo, "%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d", 
            buffer[0]%100,buffer[1]%10,buffer[2],buffer[3],buffer[4]%10,buffer[16]%100,buffer[17]%10,buffer[18],buffer[19],buffer[20]%10,
            buffer[32]%100,buffer[33]%10,buffer[34],buffer[35],buffer[36]%10,buffer[48]%100,buffer[49]%10,buffer[50],buffer[51],buffer[52]%10); 
        addr += BLE_READING_BUFFER_SIZE * 4;
        addr = addr % 0x200000;
        Spark.publish("beacon", szInfo);
    }
    delay(1000);
}

// BLE Event Processing
byte ble_event_process(){
    uint8_t type, event_code, data_len, status1;
    uint16_t event;
    bool timedOut;
    
    type = serialReadWithTimeout(timedOut);
    if (timedOut) return 0x00;
    
    event_code = serialReadWithTimeout(timedOut);
    if (timedOut) return 0x00;
    
    data_len = serialReadWithTimeout(timedOut);
    if (timedOut) return 0x00;
  
    for (int i = 0; i < data_len; i++) {
        buf[i] = serialReadWithTimeout(timedOut);
        if (timedOut) return 0x00;
    }
    
    event = BUILD_UINT16(buf[0], buf[1]);
    status1 = buf[2];
    
    switch(event){
        case DEVICE_INITIALIZED:{
            //Serial.write("DEVICE_INITIALIZED\n");
            #if JBDEBUG
            delay(1000);
            sprintf(szInfo, "%d Discovery Start! millis %ld seconds %ld", runNum, millis(), millis() / 1000L);            
            Spark.publish("time", szInfo);            
            #endif
            hci_start_discovery();
            break;
        }
        case DEVICE_DISCOVERY_DONE:{
            //Serial.write("DEVICE_DISCOVERY_DONE\n");
            #if JBDEBUG
            delay(1000);
            sprintf(szInfo, "%d Discovery Done, sleeping - millis %ld seconds %ld", runNum, millis(), millis() / 1000L);            
            Spark.publish("time", szInfo);   
            #endif
            //publishAndSleep();
            break;
        }
        case DEVICE_INFORMATION:{
            // Get RSSI and Measured Power
            uint8_t rssi = buf[11];
            uint8_t txpower = buf[42];
            uint8_t beaconID = buf[38];
            uint8_t seconds = (millis() / 1000L);
            
            if (beaconID == 0xFF) break; // filter weird beacon readings
            
            logData(beaconID, runNum, txpower, rssi, seconds);
            break;
        }
        default:
        //Serial.write("unkown cmd\n");

        break;
    }
}

void freshStart()
{
    EEPROM.write(SLOT_RUN_NUM, 0);
    EEPROM.write(SLOT_NUM_READINGS, 0);
    EEPROM.write(SLOT_CURRENT_SECTOR, 0);
    EEPROM.write(SLOT_SECTOR_POS, 0);    
}

void logData(uint8_t id, uint8_t run, uint8_t tx, uint8_t rssi, uint8_t seconds)
{
    // Update number of readings
    uint8_t numReadings = EEPROM.read(SLOT_NUM_READINGS);
    EEPROM.write(SLOT_NUM_READINGS, numReadings + 1);
    
    uint8_t sectorPos = EEPROM.read(SLOT_SECTOR_POS);
    uint8_t currentSector = EEPROM.read(SLOT_CURRENT_SECTOR);
    
    if (sectorPos == 0) {
        sFLASH_EraseSector(FLASH_ADDR + currentSector * SECTOR_SIZE);
        delay(20);
    }
    
    uint8_t buffer[BLE_READING_BUFFER_SIZE];
    buffer[0] = id;
    buffer[1] = run;
    buffer[2] = tx;
    buffer[3] = rssi;
    buffer[4] = seconds;
    
    // 255 slots (16 bytes each), using 255 sectors in total in this circular buffer
    sFLASH_WriteBuffer(buffer, FLASH_ADDR + (currentSector * SECTOR_SIZE) + (sectorPos * BLE_READING_BUFFER_SIZE), BLE_READING_BUFFER_SIZE);
    
    // Update sectorPos and currentSector if we wrap around
    ++sectorPos;
    EEPROM.write(SLOT_SECTOR_POS, sectorPos);
    if (sectorPos == 0) {
        EEPROM.write(SLOT_CURRENT_SECTOR, currentSector + 1);
    }
}

uint8_t serialReadWithTimeout(bool& timedOut) 
{
    timedOut = false;
    uint8_t timeoutMs = 100;
    unsigned long start = millis();
    while(!Serial1.available()) {
        if (millis() - start > timeoutMs) {
            timedOut = true;
            return 0x00;
        }
    }
    return Serial1.read();
}

// BLE Stuff ...
#define GAP_PROFILE_CENTRAL           0x08
#define KEYLEN                        16

static uint8_t gapCentralRoleTaskId = 0;
static uint8_t  gapCentralRoleIRK[KEYLEN] = {0};
static uint8_t  gapCentralRoleSRK[KEYLEN] = {0};
static uint32_t gapCentralRoleSignCounter = 1;
static uint8_t  gapCentralRoleMaxScanRes = 5;


int hci_init()
{
    return GAP_DeviceInit(gapCentralRoleTaskId, GAP_PROFILE_CENTRAL, gapCentralRoleMaxScanRes, gapCentralRoleIRK, gapCentralRoleSRK, &gapCentralRoleSignCounter);
}

int hci_start_discovery(){
    return GAP_DeviceDiscoveryRequest();
}

// Send initialize HCI command
int GAP_DeviceInit(uint8_t taskID, uint8_t profileRole, uint8_t maxScanResponses, uint8_t *pIRK, uint8_t *pSRK, uint32_t *pSignCounter){
    uint8_t len = 0;
    
    buf[len++] = 0x01;                  // -Type    : 0x01 (Command)
    buf[len++] = 0x00;                  // -Opcode  : 0xFE00 (GAP_DeviceInit)
    buf[len++] = 0xFE;
  
    buf[len++] = 0x26;                  // -Data Length
    buf[len++] = profileRole;           //  Profile Role
    buf[len++] = maxScanResponses;      //  MaxScanRsps
    memcpy(&buf[len], pIRK, 16);        //  IRK
    len += 16;
    memcpy(&buf[len], pSRK, 16);        //  SRK
    len += 16;
    memcpy(&buf[len], pSignCounter, 4); //  SignCounter
    len += 4;

    Serial1.write(buf, len);

    return 1;
}

// Send start discovery request
int GAP_DeviceDiscoveryRequest(){
    uint8_t len = 0;
    
    buf[len++] = 0x01;                 // -Type    : 0x01 (Command)
    buf[len++] = 0x04;                 // -Opcode  : 0xFE04 (GAP_DeviceDiscoveryRequest)
    buf[len++] = 0xFE;
        
    buf[len++] = 0x03;                 // -Data Length
    buf[len++] = 0x03;                 //  Mode
    buf[len++] = 0x01;                 //  ActiveScan
    buf[len++] = 0x00;                 //  WhiteList
  
    Serial1.write(buf, len);
  
    return 1;
}