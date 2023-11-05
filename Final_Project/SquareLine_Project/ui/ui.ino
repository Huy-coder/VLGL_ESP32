
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

//class MyClientCallback : public BLEClientCallbacks {
//    void onConnect(BLEClient* pclient) {
//    }
//
//    void onDisconnect(BLEClient* pclient) {
//      connected = false;
//      Serial.println("onDisconnect");
//    }
//};
//
//bool connectToServer() {
//  Serial.print("Forming a connection to ");
//  Serial.println(myDevice->getAddress().toString().c_str());
//
//  BLEClient*  pClient  = BLEDevice::createClient();
//  Serial.println(" - Created client");
//
//  pClient->setClientCallbacks(new MyClientCallback());
//
//  // Connect to the remove BLE Server.
//  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
//  Serial.println(" - Connected to server");
//  pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
//
//  // Obtain a reference to the service we are after in the remote BLE server.
//  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
//  if (pRemoteService == nullptr) {
//    Serial.print("Failed to find our service UUID: ");
//    Serial.println(serviceUUID.toString().c_str());
//    pClient->disconnect();
//    return false;
//  }
//  Serial.println(" - Found our service");
//
//
//  // Obtain a reference to the characteristic in the service of the remote BLE server.
//  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
//  if (pRemoteCharacteristic == nullptr) {
//    Serial.print("Failed to find our characteristic UUID: ");
//    Serial.println(charUUID.toString().c_str());
//    pClient->disconnect();
//    return false;
//  }
//  Serial.println(" - Found our characteristic");
//
//  // Read the value of the characteristic.
//  if (pRemoteCharacteristic->canRead()) {
//    std::string value = pRemoteCharacteristic->readValue();
//    Serial.print("The characteristic value was: ");
//    Serial.println(value.c_str());
//  }
//
//  if (pRemoteCharacteristic->canNotify())
//    pRemoteCharacteristic->registerForNotify(notifyCallback);
//
//  connected = true;
//  return true;
//}
//
//class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
//    /**
//        Called for each advertising BLE server.
//    */
//    void onResult(BLEAdvertisedDevice advertisedDevice) {
//      Serial.print("BLE Advertised Device found: ");
//      Serial.println(advertisedDevice.toString().c_str());
//
//      // We have found a device, let us now see if it contains the service we are looking for.
//      if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
//
//        BLEDevice::getScan()->stop();
//        myDevice = new BLEAdvertisedDevice(advertisedDevice);
//        doConnect = true;
//        doScan = true;
//
//      } // Found our server
//    } // onResult
//}; // MyAdvertisedDeviceCallbacks
//
//static void notifyCallback(
//  BLERemoteCharacteristic* pBLERemoteCharacteristic,
//  uint8_t* pData,
//  size_t length,
//  bool isNotify) {
//  Serial.print("Notify callback for characteristic ");
//  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
//  Serial.print(" of data length ");
//  Serial.println(length);
//  Serial.print("data: ");
//  Serial.write(pData, length);
//  Serial.println();
//}

void setup()
{
  Serial.begin(115200); /* pepare for possible serial debug */
  
  String LVGL_Arduino = "Hello Arduino! ";
  LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

  Serial.println( LVGL_Arduino );
  Serial.println( "I am LVGL_Arduino" );
  //BLEDevice::init("");
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
#ifdef ESP32
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * 10, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#else
  disp_draw_buf = (lv_color_t *)malloc(sizeof(lv_color_t) * screenWidth * 10);
#endif
  if (!disp_draw_buf)
  {
    Serial.println("LVGL disp_draw_buf allocate failed!");
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
    Serial.println("Starting Arduino BLE Client application...");
    
//
//    BLEScan* pBLEScan = BLEDevice::getScan();
//    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
//    pBLEScan->setInterval(1349);
//    pBLEScan->setWindow(449);
//    pBLEScan->setActiveScan(true);
//    pBLEScan->start(5, false);

    Serial.println( "Setup done" );
  }
}

void loop()
{
  lv_timer_handler(); /* let the GUI do its work */
//  if (doConnect == true) {
//    if (connectToServer()) {
//      Serial.println("We are now connected to the BLE Server.");
//    } else {
//      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
//    }
//    doConnect = false;
//  }
//
//  if (connected) {
//    String newValue = "Y";
//    Serial.println("Setting new characteristic value to \"" + newValue + "\"");
//
//    // Set the characteristic's value to be the array of bytes that is actually a string.
//    pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
//    delay(1000);
//    newValue = "N";
//    Serial.println("Setting new characteristic value to \"" + newValue + "\"");
//
//    // Set the characteristic's value to be the array of bytes that is actually a string.
//    pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
//  } else if (doScan) {
//    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
//  }
  delay(5);
}
