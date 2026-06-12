//anchor #4 setup


// be sure to edit anchor_addr and select the previously calibrated anchor delay
// my naming convention is anchors 1, 2, 3, ... have the lowest order byte of the MAC address set to 81, 82, 83, ...

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

#define I2C_SDA 4
#define I2C_SCL 5
// leftmost two bytes below will become the "short address"
char anchor_addr[] = "86:00:5B:D5:A9:9A:E2:9C"; //#4

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//calibrated Antenna Delay setting for this anchor
uint16_t Adelay = 16556;



// previously determined calibration results for antenna delay
// #1 16630
// #2 16610
// #3 16607
// #4 16580

// calibration distance
float dist_m = 2; //meters

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 21;   // spi select pin


Adafruit_SSD1306 display(128, 64, &Wire, -1);

void setup()
{
  Serial.begin(115200);
  delay(1000); //wait for serial monitor to connect
  Serial.println("Anchor config and start");
  Serial.print("Antenna delay ");
  Serial.println(Adelay);
  Serial.print("Calibration distance ");
  Serial.println(dist_m);

   /* OLED */
    Wire.begin(I2C_SDA, I2C_SCL);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println("SSD1306 failed");
        while (1);
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("Anchor 3");

    display.setTextSize(1);
    display.setCursor(0, 40);
    display.println(anchor_addr);

    display.display();

  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  // set antenna delay for anchors only. Tag is default (16384)
  DW1000.setAntennaDelay(Adelay);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  //start the module as an anchor, do not assign random short address
  DW1000Ranging.startAsAnchor(anchor_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_SHORTDATA_FAST_LOWPOWER);
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_FAST_LOWPOWER);
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_SHORTDATA_FAST_ACCURACY);
  // DW1000Ranging.startAsAnchor(anchor_addr, DW1000.MODE_LONGDATA_FAST_ACCURACY);
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_RANGE_ACCURACY);
}

void loop()
{
  DW1000Ranging.loop();
}

void newRange()
{
  // static const int WINDOW_SIZE = 20;
  // static float readings[WINDOW_SIZE];
  // static int index = 0;
  // static int count = 0;

  float dist = DW1000Ranging.getDistantDevice()->getRange();

  // // Store new reading
  // readings[index] = dist;
  // index = (index + 1) % WINDOW_SIZE;

  // if (count < WINDOW_SIZE)
  //   count++;

  // // Calculate statistics
  // float sum = 0;
  // float minVal = readings[0];
  // float maxVal = readings[0];

  // for (int i = 0; i < count; i++)
  // {
  //   sum += readings[i];

  //   if (readings[i] < minVal)
  //     minVal = readings[i];

  //   if (readings[i] > maxVal)
  //     maxVal = readings[i];
  // }

  // float smartAvg;

  // if (count > 2)
  //   smartAvg = (sum - minVal - maxVal) / (count - 2);
  // else
  //   smartAvg = sum / count;

  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print(", Current: ");
  Serial.println(dist, 3);

//   Serial.print(" m, Avg20: ");
//   Serial.print(smartAvg, 3);

//   Serial.print(" m, Min: ");
//   Serial.print(minVal, 3);

//   Serial.print(" m, Max: ");
//   Serial.print(maxVal, 3);

//   Serial.println(" m");
}

void newDevice(DW1000Device *device)
{
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
  Serial.print("Delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}
