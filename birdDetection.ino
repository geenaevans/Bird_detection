#include "VideoStream.h"
#include "StreamIO.h"
#include "NNObjectDetection.h"
#include "VideoStreamOverlay.h"
#include "ObjectClassList.h"
#include "BLEDevice.h"

// ================= CAMERA DEFINES =================
#define CHANNEL     0
#define CHANNELNN   3

#define NNWIDTH   576
#define NNHEIGHT  320

// ================= BLE UUIDs (UART SERVICE) =======
#define UART_SERVICE_UUID      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define STRING_BUF_SIZE 100

// ================= CAMERA OBJECTS =================
VideoSetting config(VIDEO_VGA, 10, VIDEO_RGB, 0);
VideoSetting configNN(NNWIDTH, NNHEIGHT, 10, VIDEO_RGB, 0);

NNObjectDetection ObjDet;
StreamIO videoStreamerNN(1, 1);

// ================= BLE OBJECTS ====================
BLEService UartService(UART_SERVICE_UUID);
BLECharacteristic Tx(CHARACTERISTIC_UUID_TX);

BLEAdvertData advdata;
BLEAdvertData scndata;

bool bird_sent = false;

// ================= SETUP ==========================
void setup()
{
    Serial.begin(115200);

    // -------- BLE SETUP --------
    advdata.addFlags(GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED);
    advdata.addCompleteName("Bird_Detector");
    scndata.addCompleteServices(BLEUUID(UART_SERVICE_UUID));

    Tx.setNotifyProperty(true);
    Tx.setBufferLen(STRING_BUF_SIZE);

    UartService.addCharacteristic(Tx);

    BLE.init();
    BLE.configAdvert()->setAdvData(advdata);
    BLE.configAdvert()->setScanRspData(scndata);
    BLE.configServer(1);
    BLE.addService(UartService);
    BLE.beginPeripheral();

    Serial.println("BLE UART Sender started");

    // -------- CAMERA SETUP --------
    Camera.configVideoChannel(CHANNEL, config);
    Camera.configVideoChannel(CHANNELNN, configNN);
    Camera.videoInit();

    ObjDet.configVideo(configNN);
    ObjDet.modelSelect(OBJECT_DETECTION, DEFAULT_YOLOV4TINY, NA_MODEL, NA_MODEL);
    ObjDet.begin();

    videoStreamerNN.registerInput(Camera.getStream(CHANNELNN));
    videoStreamerNN.registerOutput(ObjDet);
    videoStreamerNN.begin();

    Camera.channelBegin(CHANNEL);
    Camera.channelBegin(CHANNELNN);

    OSD.configVideo(CHANNEL, config);
    OSD.begin();

    Serial.println("Bird detection started");
}

// ================= LOOP ===========================
void loop()
{
    std::vector<ObjectDetectionResult> results = ObjDet.getResult();
    bool bird_detected_now = false;

    int count = ObjDet.getResultCount();

    if (count > 0) {
        for (int i = 0; i < count; i++) {

            ObjectDetectionResult item = results[i];
            int obj_type = item.type();
            const char* label = itemList[obj_type].objectName;

            Serial.print("Detected: ");
            Serial.print(label);
            Serial.print("  Score: ");
            Serial.println(item.score());

            if (strcmp(label, "bird") == 0 && item.score() > 40) {
                bird_detected_now = true;
            }
        }
    }

    // -------- SEND BLE MESSAGE --------
    if (bird_detected_now && !bird_sent) {

        if (BLE.connected(0)) {
            Tx.writeString("BIRD_DETECTED");
            Tx.notify(0);

            Serial.println("Bird detected -> BLE message sent");
            bird_sent = true;
        }
    }

    // Reset when bird disappears
    if (!bird_detected_now) {
        bird_sent = false;
    }

    delay(100);
}