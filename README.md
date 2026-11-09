# Soil Buddy RE

Soil moisture and water-level monitor for an ESP32 board, with a phone web app (Web Bluetooth) and an optional ESP32-CAM view.

- **App** (`index.html`): installable web app. Shows the soil moisture index, the tank water level in cm, pump status, alerts, a terminal and the camera.
- **Firmware** (`firmware/SoilBuddyRE/SoilBuddyRE.ino`): v3.3 sketch for ESP32-WROOM. It reads the sensors, runs the pump and talks to the app over Bluetooth LE (Dabble Terminal protocol).

## Repo layout

```
index.html                    the whole app (one file)
sw.js                         service worker (offline cache, notification buttons)
manifest.webmanifest          install info
icon-*.png, apple-touch-icon.png, badge-96.png, logo-*.svg
firmware/SoilBuddyRE/         Arduino sketch (folder name must match the .ino name)
.nojekyll                     tells GitHub Pages to serve files as they are
```

## Put the app online (GitHub Pages)

1. Create a repository and upload **everything in this folder** (drag the files and the `firmware` folder into "Add file > Upload files"). `index.html` must sit at the top level.
2. Open **Settings > Pages**. Under "Build and deployment" pick **Deploy from a branch**, branch **main**, folder **/ (root)**, then Save.
3. After a minute the app is at `https://<your-username>.github.io/<repo-name>/`.
4. Open that link in **Chrome or Edge** (Android, Windows, Mac, ChromeOS). Web Bluetooth needs HTTPS, which Pages provides. iPhone and Safari cannot use Web Bluetooth.
5. On Android, use **Setup > Install app** or the browser menu, Add to Home screen.

When you change a file, upload it again to the same place. The service worker is network first, so a refreshed page shows the new version. Bump the cache name in `sw.js` (`sb-re-9`) if an old copy sticks.

## Flash the board

1. Install the **DabbleESP32** library in the Arduino IDE and add ESP32 board support.
2. Open `firmware/SoilBuddyRE/SoilBuddyRE.ino`, pick an ESP32 Dev Module style board (ESP32-WROOM) and upload.
3. The board advertises over **Bluetooth LE** as "Soil Buddy RE". It does not show in the phone's normal Bluetooth list. Use the app's Connect button, the Dabble app or nRF Connect.
4. The board allows one phone at a time. Close the Dabble app before connecting from the web app.

Pins: soil sensor AO on GPIO35, HW-038 water sensor on GPIO34, pump through GPIO26 > 330 ohm > TIP120 base. The pump is OFF at boot and AUTO starts disabled.

## Water level in cm

The tank is about **4 cm high and 15 cm long**. The sketch turns the HW-038 ADC reading into a level in cm with a two-point map:

```
level_cm = (adc - WATER_ADC_AT_EMPTY) / (WATER_ADC_AT_FULL - WATER_ADC_AT_EMPTY) * 4.0
```

The defaults (0 and 4095) are only a rough guess. To calibrate:

1. Send `water` with the sensor dry and note `HW-038 ADC`. Put it in `WATER_ADC_AT_EMPTY`.
2. Fill the tank to the 4 cm mark, send `water` again, and put that ADC in `WATER_ADC_AT_FULL`.
3. Re-upload the sketch.

If the sensor reads lower when wetter, swap the two numbers and set `WATER_ADC_INCREASES_WITH_LEVEL` to `false` (that flag only feeds the low-water guard).

The AUTO low-water guard still uses a fixed raw ADC cutoff (`WATER_LOW_ADC_THRESHOLD = 500`, clears at 750). Check what depth that is on your tank and make sure the pump intake is submerged above it.

## Commands (Terminal tab, Dabble Terminal or serial)

`help`, `soil`, `water` (or `cm`), `status`, `chat on` / `chat off`, `pump test`, `pump off`, `pump auto`, `threshold`

- `pump test` runs the pump for 30 s and **skips the water guard**. Only run it with the pump submerged and you beside it.
- `pump off` stops the pump and turns AUTO off.
- The app polls with the short command `s`. The board answers with one line:
  `#S,soilIndex,soilAdc,waterCm,waterAdc,mode,pump,demand,guard`
  (mode 0 = off, 1 = auto, 2 = timed test; pump, demand and guard are 1 or 0).

## Known limits

- Soil % is a relative ADC index, not a calibrated moisture value. Water cm is an estimate from a resistive sensor, which is not truly linear with depth.
- The soil thresholds (0 and 50), the 1 s pulse and the 500 ADC cutoff are pilot settings. Validate them on your own plant and soil.
- Alerts only work while the app is open and connected. Closing it disconnects Bluetooth.
- The camera tab expects an ESP32-CAM stream at `http://<ip>:81/stream`. A page served over HTTPS may be blocked from loading an `http://` stream. If so, open the stream link shown under the camera field, or run the app from `http://localhost`.
- Web Bluetooth works in Chrome and Edge only (not iPhone, not Safari).
