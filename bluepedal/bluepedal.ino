/*************************************************************************
This code is provided as-is and I make no representations or warranties
of any kind concerning the code, express, implied, statutory or otherwise,
including without limitation warranties of title, merchantability, fitness
for a particular purpose, non infringement, or the absence of latent or
other defects, accuracy, or the present or absence of errors, whether or
not discoverable, all to the greatest extent permissible under applicable
law.

This code is licensed under CC0 -- meaning that effectively it's in the
public domain and you can use it without any restrictions whatsoever. For
details, see the accompanying LICENSE.md.

Of course, that does not go for the AdaFruit and Arduino libraries that
it uses; see their respective LICENSE files for those.

However, I would not mind an attribution if you reuse this code somewhere.

-- Pol Van Aubel <dev@qwfp.nl>, 2019-05-02
*************************************************************************/


#include <bluefruit.h>

BLEDis bledis;
BLEHidAdafruit blehid;



/*
 * Defines for correct functioning of battery level read.
 */
#define PIN_VDIV (PIN_A6)
#define VBAT_MV_PER_LSB (0.87890625F)  // Assuming 3.6V range and 12-bit ADC resolution
#define VBAT_DIVIDER (0.5F)            // 100kOhm + 100kOhm resistors = 100 / (100+100)
#define VBAT_DIVIDER_COMP (2)          // Reciprocal of the above



/*
 * Input pins:
 * 6: This pedal.
 * Viewed facing the connection pins on the TRS connectors:
 * 9: right channel on right connector.
 * 10: left channel on right connector.
 * 11: right channel on left connector.
 * 12: left channel on left connector.
 */
byte pedals[] = {6, 9, 10, 11, 12};

#define DEBOUNCE_TIME (3)  // Debounce time in ms
#define BOUNCE_COUNT (5)   // Number of bounces during DEBOUNCE_TIME considered a button press.

typedef void (* InterruptPedalInput) (void);
typedef void (* TimerCallback) (TimerHandle_t);
InterruptPedalInput pedalISRs[] = {int0, int1, int2, int3, int4};
TimerCallback timercallbacks[] = {tcb0, tcb1, tcb2, tcb3, tcb4};

#define NUMPEDALS (5)
SoftwareTimer timers[NUMPEDALS];
volatile int intreports[NUMPEDALS];
TaskHandle_t keytasks[NUMPEDALS];



/*
 * Static keycode values used for task creation. Passed by reference, so not possible
 * to put in the setup functions because those are out of scope by the time the task
 * gets through its initialization.
 */
static uint8_t pageup = HID_KEY_PAGE_UP;
static uint8_t pagedown = HID_KEY_PAGE_DOWN;


void setup(void) {
  setupSerial();
  setupBattery();

  setupBluetooth();

  setupInputs();

  teardownSerial();
  suspendLoop();
}



void setupBattery(void) {
  /*
   * Set analog read resolution to 12 bits and 3.6V reference voltage
   * for accurate VDIV measurement. 
   */
  // Figure out how to en/disable the ADC.
  analogReadResolution(12);
  analogReference(AR_INTERNAL);
}


void teardownBattery(void) {
  //nrf_adc_disable();
}



void setupSerial(void) {
  /*
   * Unless you actually need USB Serial for anything other than debug reports,
   * don't bother waiting for it. I wait here once for 100ms to give the device
   * a chance to settle before continuing.
   * 
   * On the nRF52840's native USB, if(Serial) correctly
   * reflects the opened / closed state of the USB Serial's file descriptor on the
   * host. So just wrap debugging in if (Serial) commands and you're golden.
   * 
   * Also, because we're running with a suspended loop, for proper operation
   * the Serial needs to be flushed after every print you actually care about.
   * Otherwise, output is buffered for a long time.
   */
  //Serial.begin(115200);
  //while ( !Serial ) (
  //  delay(10);
  //}
  if (!Serial) {
    delay(1000);
  }

  if (Serial) {
    Serial.println("BluePedal52");
    Serial.println("--------------------------------");
  
    Serial.println();
    Serial.println("Pair your device then open an");
    Serial.println("app that accepts keyboard input");
    Serial.flush();
  }
}


void teardownSerial(void) {
  /*
   * On the nRF52840, Serial.end() is a noop.
   */
  //Serial.end();
}



void setupInputs(void) {
  TaskHandle_t pgupHandle = NULL, pgdnHandle = NULL;
  if (xTaskCreate(taskKeypress, "PageUp", configMINIMAL_STACK_SIZE, (void*) &pageup, TASK_PRIO_LOW, &pgupHandle) != pdPASS) {
    if (Serial) {
      Serial.println("Unable to create task for PageUp! Panic!");
      Serial.flush();
    }
  }
  if (xTaskCreate(taskKeypress, "PageDown", configMINIMAL_STACK_SIZE, (void*) &pagedown, TASK_PRIO_LOW, &pgdnHandle) != pdPASS) {
    if (Serial) {
      Serial.println("Unable to create task for PageDown! Panic!");
      Serial.flush();
    }
  }
  if (Serial) {
    Serial.println("Tasks have now been created.");
    Serial.flush();
  }

  keytasks[0] = pgdnHandle;
  keytasks[1] = pgupHandle;
  keytasks[2] = pgdnHandle;
  keytasks[3] = pgupHandle;
  keytasks[4] = pgdnHandle;
  
  for (byte i = 0; i < NUMPEDALS; ++i) {
    pinMode(pedals[i], INPUT_PULLUP);
    timers[i].begin(DEBOUNCE_TIME, timercallbacks[i], false);
    attachInterrupt(digitalPinToInterrupt(pedals[i]), pedalISRs[i], FALLING);
  }
}



void setupBluetooth(void) {
  Bluefruit.begin();
  Bluefruit.setTxPower(-20);  // Supported values in bluefruit.h. Power consumption estimates in chip datasheet.
  Bluefruit.setName("BluePedal");

  // Turn off Blue LED to minimize power consumption.
  Bluefruit.autoConnLed(false);

  // Configure and start Device Information Service
  bledis.setManufacturer("Pol Van Aubel");
  bledis.setModel("BluePedal v0.1");
  bledis.begin();

  // Configure and start Human Interface Device
  // min and max connection intervals default to 11.25ms and 15ms.
  // TODO figure out how these affect power consumption and performance.
  blehid.begin();
  // Set connection intervals (min, max) as min*1.25 and max*1.25 ms. Defaulted to (9, 12)
  //Bluefruit.Periph.setConnInterval(9, 12);

  // Set callback for keyboard status LED
  blehid.setKeyboardLedCallback(set_keyboard_led);

  // Set up and start advertising
  startAdv();
  
  // TODO figure out what other callbacks might be useful.
  // TODO stop advertising after 30 seconds?
  
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
}


void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);
  
  // Include BLE HID service
  Bluefruit.Advertising.addService(blehid);

  // There is enough room for the dev name in the advertising packet
  Bluefruit.Advertising.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 2056);  // in units of 0.625 ms: 20ms fast advertising, 1285ms slow advertising.
  //Bluefruit.Advertising.setInterval(3200, 3200); // in units of 0.625 ms: 2 seconds each. Not recommended by Apple.
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds. TODO should this be changed?
}



float readVBat(void) {
  int raw;
  float vdiv, vbat;
  raw = analogRead(PIN_VDIV);
  vdiv = raw * VBAT_MV_PER_LSB;
  vbat = vdiv * VBAT_DIVIDER_COMP;
  return vbat;
}



void interrupt_service_routine(byte isrnum) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
  ++intreports[isrnum];

  /*
   * Reset the timer every interrupt, so that it only expires
   * 3 mSec after the last interrupt fired.
   */
  if (timers[isrnum].resetFromISR(&xHigherPriorityTaskWoken) != pdPASS) {
    /* The reset did not execute successfully. Figure out what to do. */
    // TODO Serial printing in a non-deferred ISR is a horrible idea.
    if (Serial) {
      Serial.print("Reset in interrupt ");
      Serial.print(isrnum);
      Serial.println(" failed!");
      Serial.flush();
    }
  }
  
  if (xHigherPriorityTaskWoken != pdFALSE) {
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

static void int0(void) { interrupt_service_routine(0); }
static void int1(void) { interrupt_service_routine(1); }
static void int2(void) { interrupt_service_routine(2); }
static void int3(void) { interrupt_service_routine(3); }
static void int4(void) { interrupt_service_routine(4); }



static void timer_callback(byte callbacknum) {
  /*
   * Abuse the bouncyness of the buttons to figure out whether it's a button press
   * or crosstalk. A button press should result in more than 10 interrupts in 3ms.
   */
  if (intreports[callbacknum] >= BOUNCE_COUNT) {
    /*
     * Start the task that actually sends the keypresses to the master.
     */
     vTaskResume(keytasks[callbacknum]);
  }
  intreports[callbacknum] = 0;
}

static void tcb0(TimerHandle_t xTimer) { timer_callback(0); }
static void tcb1(TimerHandle_t xTimer) { timer_callback(1); }
static void tcb2(TimerHandle_t xTimer) { timer_callback(2); }
static void tcb3(TimerHandle_t xTimer) { timer_callback(3); }
static void tcb4(TimerHandle_t xTimer) { timer_callback(4); }



void taskKeypress(void* key) {
  uint8_t keyreport[] = {*((uint8_t*)key), 0, 0, 0, 0, 0};
  static uint8_t releaseall_arr[] = {0, 0, 0, 0, 0, 0};
  
  if (Serial) {
    Serial.print("Created task for keycode ");
    Serial.println(keyreport[0]);
    Serial.flush();
  }
  
  // Tasks must run as an infinite loop, unless they delete themselves
  // by using vTaskDelete before returning.
  while(1) {
    // Suspend this task.
    // Done at the top of the loop because it seems that
    // the task is run once upon creation.
    vTaskSuspend(NULL);
    
    blehid.keyboardReport(0, keyreport);
    blehid.keyboardReport(0, releaseall_arr);

    if (Serial) {
      Serial.print("Emitting keycode ");
      Serial.print(keyreport[0]);
      Serial.println(" over BT.");
      Serial.flush();
    } else {
      digitalToggle(LED_BLUE);
    }
  }
}


/**
 * Callback invoked when received Set LED from central.
 * Must be set previously with setKeyboardLedCallback()
 *
 * The LED bit map is as follows: (also defined by KEYBOARD_LED_* )
 *    Kana (4) | Compose (3) | ScrollLock (2) | CapsLock (1) | Numlock (0)
 */
void set_keyboard_led(uint16_t conn_handle, uint8_t led_bitmap)
{
  (void) conn_handle;
  
  // light up Red Led if any bits is set
  if ( led_bitmap ) {
    ledOn( LED_RED );
  } else {
    ledOff( LED_RED );
  }
}



void rtos_idle_callback(void)
{
  // Don't call any  FreeRTOS blocking API()
  // Perform background task(s) here

  // In freertos tickless idle mode, it may not be a good idea
  // to explicitly power the device down here -- or at least
  // to call waitForEvent(). See
  // https://www.freertos.org/low-power-tickless-rtos.html
  //
  // Figure out the device's power behaviour using the red LED.
  digitalToggle( LED_RED );
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  //waitForEvent();
}



// Look mom, no loops!
void loop(void) { };
//  if (Serial) {
//    Serial.println(readVBat());
//  }
