# RaceboxMAX AIO

## Hardware

| Part | Notes |
|---|---|
| (1) RaceBox Micro | The instrument. Accepts 3.5–16 V, draws ~25 mA recording |
| (1) Seeed XIAO ESP32-C6 | 21 x 17.5 mm, BLE 5.3, USB-C |
| (1) 2.42" SSD1309 OLED | 128 x 64, I2C mode, address 0x3C |
| (1) Adafruit MAX17048 | LiPo fuel gauge, I2C, address 0x36 |
| (1) Adafruit PowerBoost 1000C | 1 A charger, 5.2 V boost, load sharing |
| (1) Protected 18650 | 3400 mAh. Protected cells run ~69–70 mm, check holder fit |
| (3) 3 x 6 mm tactile switches | Up, down, select |
| (1) SPST toggle | Master power, wired to the PowerBoost EN pin |

No fuses and no external resistors. The protected cell's own PCB opens a
short in tens of microseconds; A pptc takes tens of milliseconds and would
never win that race. The I2C pull ups are already on both breakouts.

## Wiring

The 18650 passes through the MAX17048's two parallel JST connectors into
the PowerBoost battery input, so the gauge sees the cell without being in
the way. The PowerBoost 5.2 V rail feeds the XIAO's VBUS pin and the
RaceBox in parallel. Everything on I2C runs from the XIAO's 3V3 pin so the
logic levels match the C6's 3.3 V GPIO.

| Signal | XIAO pin | GPIO |
|---|---|---|
| I2C SDA | D4 | 22 |
| I2C SCL | D5 | 23 |
| Display RES | D3 | 21 |
| Button UP | D0 | 0 |
| Button DOWN | D1 | 1 |
| Button SELECT | D2 | 2 |

The XIAO's own BAT solder pads stay empty. The PowerBoost1000 is the charger, so having two charger ICs on one cell
will fight each other.


## Power

| Load | On the 5.2 V rail |
|---|---|
| XIAO ESP32-C6 | ~70 mA |
| SSD1309 | ~50 mA depending on lit pixels |
| RaceBox Micro | ~25 mA |


The switch grounds the PowerBoost EN pin, which is active high. Closed
means off, which is backwards from what you expect, and worth labelling on
the enclosure. It also has to be on for the cell to charge.

### Ground

Ground is one net but three separate buses

The I2C peripherals ground at the XIAO rather than at the PowerBoost. The
C6 pulls roughly 300 mA in short bursts during radio activity, and that
current has to return somewhere. Referencing the display and gauge to the
same point as their bus master makes any resulting offset common mode
instead of putting it across SDA and SCL.

## Interface

Up, down, and select. 600 ms long-press on select backs out from anywhere.

A status bar is drawn across the top of every screen and no page can write
into it. It shows fix type and satellite count on the left, battery percent
and discharge rate in percent per hour on the right. The battery text is
right-aligned by measured width, and the left label is suppressed if it
would not fit, so the two can never collide.

### Live page

Speed in MPH as the headline number, with lateral and longitudinal G,
heading and compass point, altitude, and horizontal accuracy underneath.

### Track page

Pick a Track and configuration — Pre-loaded with Thunderhill East 3 mile, West 2 mile, or the 5 mile
combined — and the screen reports one of four states: no fix, not at track
with distance, no gate set, or armed. Once running it shows current lap,
last lap, best lap, and draws a checkered flag on a new best.

Lap timing works by segment intersection. Each 25 Hz sample is projected
into local metres and tested against the gate line; when the path between
two consecutive samples crosses it, the exact crossing instant is
interpolated using the RaceBox's own iTOW timestamp rather than millis().
At 100 mph you cover 1.8 m between samples, so the interpolation is the
difference between hundredth-second resolution and plus or minus 0.04 s.

Gate coordinates ship empty. Open a track, press up, and the firmware
captures a 30 m gate perpendicular to your current heading and prints the
line to serial ready to paste into the tracks table:

```
GATE T-Hill East 3mi
  {39.5372100, -122.3310400, 39.5371800, -122.3309900, true}
```

The center coordinates in the table are approximate and used only for the
proximity check with a 3 km radius. They are not accurate enough to be
gates.

### Drag page

Standing start auto-arms after one second stationary and triggers on
movement — no button press at the line. It captures 60 ft, 0–60 mph, and
eighth and quarter mile with trap speeds, integrating distance from speed
against iTOW deltas.

A separate 60–0 mode arms above 62 mph, starts measuring when you drop
through 60, and reports braking distance in feet.

There is no rollout correction, so these read 0.2 to 0.3 s slower than a
dragstrip slip, which starts the clock after the car rolls a foot.

### Options page

A battery page showing state of charge, cell voltage, charge rate in
percent per hour, hours remaining, and the gauge's IC version, chip ID and
alert flags.


### Libraries

- U8g2 by oliver
- Adafruit MAX1704X
- NimBLE-Arduino, **2.x**

NimBLE 1.4.x will not compile on Arduino core 3.x. It was built against
ESP-IDF 4.4 and fails on a missing `esp_coexist_internal.h`. The C6 only
exists on core 3.x, so 2.x is the only option.

If the display comes up scrambled or vertically offset, change the U8g2
constructor from `NONAME0` to `NONAME2`. Most 2.42" SSD1309 modules also
ship jumpered for SPI and need solder jumpers moved before I2C will
enumerate at all. Run an I2C scanner first and confirm both 0x36 and 0x3C
respond.

## Enclosure

Printed in PETG and made up of 7 parts.
Screen face, Enclosure, Enclosure lid, ESP32C6 tray, MAX17048 tray, Powerboost1000 Tray.

Do not print in anything carbon-filled. The XIAO's BLE antenna is inside
the box and carbon fibre is conductive enough to kill it.

The RaceBox mounts on top of the case, outside, because its patch antenna
needs a clear view of the sky and the display needs to face the driver.
Those are 90 degrees apart and there is no arrangement that satisfies both
in one sealed box. Metal below a patch antenna is harmless; metal above or
beside it is not, which is why the 18650 lives in the bottom compartment.

## What itdoes not do

It does not record anything. No SD card, no filesystem, no NVS. Best lap
and best drag times live in RAM and clear on power cycle. Captured gates do
the same, which is why they print to serial as well.

Downloading the RaceBox's internal recordings over BLE is possible and
documented in their protocol. The device streams History Data messages
after an unlock, and the lock state resets on every connection. It is not
implemented here because there is nowhere to put two hours of 25 Hz data
without adding storage.
