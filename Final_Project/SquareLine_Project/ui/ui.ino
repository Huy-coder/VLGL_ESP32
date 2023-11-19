#include <lvgl.h>
#include "ui.h"
#include <Arduino_GFX_Library.h>
#include <BLEDevice.h>

/*Don't forget to set Sketchbook location in File/Preferencesto the path of your UI project (the parent foder of this INO file)*/

#define GFX_BL DF_GFX_BL // default backlight pin, you may replace DF_GFX_BL to actual backlight pin

/* More dev device declaration: https://github.com/moononournation/Arduino_GFX/wiki/Dev-Device-Declaration */
/* More data bus class: https://github.com/moononournation/Arduino_GFX/wiki/Data-Bus-Class */
/* More display class: https://github.com/moononournation/Arduino_GFX/wiki/Display-Class */

#define GFX_BL 44
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
  GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
  40 /* DE */, 41 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
  45 /* R0 */, 48 /* R1 */, 47 /* R2 */, 21 /* R3 */, 14 /* R4 */,
  5 /* G0 */, 6 /* G1 */, 7 /* G2 */, 15 /* G3 */, 16 /* G4 */, 4 /* G5 */,
  8 /* B0 */, 3 /* B1 */, 46 /* B2 */, 9 /* B3 */, 1 /* B4 */
);

// Uncomment for ST7262 IPS LCD 800x480
Arduino_RPi_DPI_RGBPanel *gfx = new Arduino_RPi_DPI_RGBPanel(
  bus,
  480 /* width */, 0 /* hsync_polarity */, 8 /* hsync_front_porch */, 4 /* hsync_pulse_width */, 8 /* hsync_back_porch */,
  272 /* height */, 0 /* vsync_polarity */, 8 /* vsync_front_porch */, 4 /* vsync_pulse_width */, 8 /* vsync_back_porch */,
  1 /* pclk_active_neg */, 16000000 /* prefer_speed */, true /* auto_flush */);

/*******************************************************************************
   End of Arduino_GFX setting
 ******************************************************************************/

/*******************************************************************************
   End of Arduino_GFX setting
 ******************************************************************************/

#include "touch.h"

/*Change to your screen resolution*/
static uint16_t screenWidth;
static uint16_t screenHeight;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;

// The remote service we wish to connect to.
static BLEUUID serviceUUID("0000FFE0-0000-1000-8000-00805F9B34FB");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("0000FFE1-0000-1000-8000-00805F9B34FB");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

unsigned long previous_connect = 0;
bool logic_command = 0;

extern int directionButton;
extern int flag;
int valueInt;
#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
  Serial.printf(buf);
  Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
  uint32_t w = ( area->x2 - area->x1 + 1 );
  uint32_t h = ( area->y2 - area->y1 + 1 );

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready( disp );
}

TaskHandle_t Task1;
TaskHandle_t Task2;

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}

class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
    }

    void onDisconnect(BLEClient* pclient) {
      connected = false;
      Serial.printf("onDisconnect\n");
    }
};

bool connectToServer() {
  static const char* TAG = "connectToServer";
  Serial.printf("Forming a connection to %s\n", myDevice->getAddress().toString());

  BLEClient*  pClient  = BLEDevice::createClient();
  Serial.printf("- Created client\n");
  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.printf("- Connected to server\n");
  pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.printf("Failed to find our service UUID: %s\n", serviceUUID.toString());
    pClient->disconnect();
    return false;
  }
  Serial.printf("- Found our service\n");

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.printf("Failed to find our characteristic UUID: %s\n", charUUID.toString());
    pClient->disconnect();
    return false;
  }
  Serial.printf("- Found our service\n");

  // Read the value of the characteristic.
  if (pRemoteCharacteristic->canRead()) {
    std::string value = pRemoteCharacteristic->readValue();
    Serial.printf("The characteristic value was: %s\n", value.c_str());
  }

  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(notifyCallback);

  connected = true;
  return true;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    /**
        Called for each advertising BLE server.
    */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      static const char* TAG = "MyAdvertisedDeviceCallbacks->onResult";
      Serial.print("BLE Advertised Device found: ");
      Serial.println(advertisedDevice.toString().c_str());

      // We have found a device, let us now see if it contains the service we are looking for.
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

        BLEDevice::getScan()->stop();
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
        doConnect = true;
        doScan = true;

      } // Found our server
    } // onResult
}; // MyAdvertisedDeviceCallbacks

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  static const char* TAG = "notifyCallback";
  Serial.printf("Notify callback for characteristic %s of data length %d data: %s\n", pBLERemoteCharacteristic->getUUID().toString(), length, pData);
  String myString = (String) static_cast<unsigned>(pData[0]);
  Serial.printf("Hello %s\n",myString);
  valueInt = myString.toInt();
}

void setup()
{
  Serial.begin(115200); /* pepare for possible serial debug */
  String LVGL_Arduino = "! ";
  LVGL_Arduino = String("LVGL Version: V") + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
  Serial.println(LVGL_Arduino);

#ifdef GFX_PWD
  pinMode(GFX_PWD, OUTPUT);
  digitalWrite(GFX_PWD, HIGH);
#endif

  touch_init(gfx->width(), gfx->height());
  gfx->begin();
  gfx->fillScreen(BLACK);

#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
#endif

  lv_init();

  screenWidth = gfx->width();
  screenHeight = gfx->height();

  Serial.printf("Free PSRAM: %d\n", ESP.getPsramSize() - ESP.getFreePsram());
  disp_draw_buf = (lv_color_t *)ps_malloc(sizeof(lv_color_t) * screenWidth * 10);
  Serial.printf("Free PSRAM after used: %d\n", ESP.getPsramSize() - ESP.getFreePsram());

  if (!disp_draw_buf)
  {
    Serial.printf("LVGL disp_draw_buf allocate failed!");
  }
  else
  {
    lv_disp_draw_buf_init( &draw_buf, disp_draw_buf, NULL, screenWidth * 10 );

    /*Initialize the display*/
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );

    ui_init();

    Serial.printf("Starting Init BLE Client application...\n");
    BLEDevice::init("");
    Serial.printf("Starting Starting BLE Client application...");
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(5, false);
    Serial.printf("done\n");
    xTaskCreatePinnedToCore(BLE_Task, "Task1", 10000, NULL, 1, &Task1, 1);  delay(500);
    xTaskCreatePinnedToCore(LVGL_Task, "Task2", 10000, NULL, 1, &Task2, 0);  delay(500);
  }
}
void BLE_Task( void * parameter )
{
  Serial.print("BLE_Task is running on core ");
  Serial.println(xPortGetCoreID());
  /*BEGIN TASK 1*/
  while (1) {
    if (doConnect == false && connected == false)
    {
      doConnect == true;
    }
    if (doConnect == true) {
      if (connectToServer()) {
        Serial.printf("Connected to the BLE Server.\n");
      } else {

        Serial.printf("Failed connect to the server; nothin more will do.\n");
      }
      doConnect = false;
    }

    if (connected) {
      String value = String(directionButton);
      if (flag == 1) {
        Serial.printf("flag = %d\n", flag);
        Serial.printf("Setting new characteristic value to \"%s\"\n", value);
        pRemoteCharacteristic->writeValue(value.c_str(), value.length());
        flag = 0;
      }
      checkCondition(valueInt);
    } else if (doScan) {
      BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
    }
    /*END TASK 1*/
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void LVGL_Task( void * parameter )
{
  Serial.print("LVGL_Task is running on core ");
  Serial.println(xPortGetCoreID());
  while (1) {
    /*BEGIN TASK 2*/
    lv_timer_handler(); /* let the GUI do its work */
    vTaskDelay(10 / portTICK_PERIOD_MS);
    /*END TASK 2*/
  }
}

void checkCondition(int value)
{
  switch (value) {
    case 49:
      Serial.println("Case 1");
      _ui_flag_modify( ui_Finger1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
      delay(1000);
      _ui_flag_modify( ui_Finger1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
      break;
    case 50:
      Serial.println("Case 2");
      _ui_flag_modify( ui_Finger2, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
      delay(1000);
      _ui_flag_modify( ui_Finger2, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
      break;
    case 51:
      Serial.println("Case 3");
      _ui_flag_modify( ui_Finger3, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
      delay(1000);
      _ui_flag_modify( ui_Finger3, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
      break;
    case 52:
      Serial.println("Case 4");
      _ui_flag_modify( ui_Finger4, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
      delay(1000);
      _ui_flag_modify( ui_Finger4, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
      break;
    default:
      break;
  }
  valueInt = -1;
}

void loop()
{
  //NOT USE
}
