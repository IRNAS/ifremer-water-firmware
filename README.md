# smartparks-rhino/lion-tracker-firmware
Smartparks Rhino and Lion GPS tracker firmware repository. This firmware is the first generation solution for multiple LoraWAN devices. The implementation is as straightforward as possible to ensure simple operation, however not suitable for very complex device operation task which is implemented int eh second generation in a separate repository.

WARNING: This documentation may not be up to date with the actual firwmare release.

# Requirements:
 * Modified Arduino Core for STM32L0
 * vscode or similar compile tool (Arduino IDE may break things)
 * this repository
 * TheThingsNetwork or other similar LoraWAN network server solution

# Operation
This device is a power-efficeint GPS tracker sending data via LoraWAN. There are three key messages being sent by the device:
 * `settings - port 3` - current device settings - only upon boot, change or request
 * `status - port 2 ` - (legacy, not enabled in latest version anymore) device status message being sent with the specified `status_interval` in minutes - recommended to do this every 24h
 * `gps - port 1` - gps location - gps packet is set based on the `settings` and various intervals defined there.
 * `status+gps - port 12` - device status message being sent with the specified `status_interval` in minutes - recommended to do this every 24h, combined with last gps position. Particularly useful if gps fix interval is longer then the status interval.
 * `gps log` - 

## GPS data
The device reports GPS position with various mechanisms, these only expain fields related to GPS
* `status - port 12 `
  * `decoded.lat` - Latitude position
  * `decoded.lon` - Longtitude position
  * `decoded.gps_resend` - number status packets sent since last updated GPS position
  * `decoded.gps_on_time_total` - cumulative time in seconds since device reboot. This best correlated to the power consumption of the device.
  * `decoded.gps_time` - linux epoch time when the last GPS position has been acquired
* `gps - port 1`
  * `decoded.lat` - Latitude position
  * `decoded.lon` - Longtitude position
  * `decoded.alt` - Altitude above sea level
  * `decoded.satellites` - number of satellites acquired in the fix, expect >3 for 3D fix and >4 for 3D fix.
  * `decoded.hdop` - horizontal dillution of precision - expect <5 for normal operation, see https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)
  * `decoded.time_to_fix` - time to fix of the device, depends on configuration, signal and how ofthen the fix is acquired. <5s is expected if fix every 5min, <15s is expected for fix every hour, <60s for cold fix
  * `decoded.epe` - estimated position error, this is configured as a target for every gps fix and the result show here is what happened actually. Normally in meters or feet. 50 is a good default, 100 will be very inaccurate fix, 10 is very accurate fix. Best advice is to keep this at 50, and set `gps_min_fix_time` to larger value, say 30 and the fix will be very good, epe is then expected to be about <10 in hotfix conditions.
  * `decoded.snr` - best satellite SNR value, good functioning device is in 40-50 range, <30 is critical, check device orientation and close by objects.
  * `decoded.motion` - high if the gps position has been triggered by movement, subject to configuration
  * `decoded.gps_time` - linux epoch time when the last GPS position has been acquired
* `gps log - port 11 ` - 5 positions per packet of this info
  * `decoded.lat` - Latitude position
  * `decoded.lon` - Longtitude position
  * `decoded.gps_time` - linux epoch time when the last GPS position has been acquired

## GPS acquisition logic
The GPS system is configured by defining the `gps_periodic_interval` - time between fixes periodically and `gps_triggered_interval` - time between fixes if device is in active mode based on `gps_triggered_threshold` and `gps_triggered_duration`.
* `gps_periodic_interval` - gps periodic fix interval in minutes, 0 disables it - 240 is a good default
* `gps_triggered_interval` - gps triggered interval in minutes, 0 disables it - 0 is a good default
* `gps_triggered_threshold` - the amount of G-force of the movement to trigger a fix, assume 10 as a safe default
* `gps_triggered_duration` - the time in seconds that the accelerometer needs to detect motion before it triggers a fix, assume 10 as a safe default
* `gps_cold_fix_timeout` - cold fix timeout in seconds - 120 is a good default
* `gps_hot_fix_timeout` - hot fix timeout in seconds - 30 is a good default
* `gps_min_fix_time` - minimal fix time - 5-15 is a good default
* `gps_min_ehpe` - minimal ehpe to be achieved - 50 is a good default
* `gps_hot_fix_retry`(0-255)- number of times a hot fix is retried before failing to cold-fix - 5 is a good default
* `gps_cold_fix_retry`(0-255) - number of time a cold fix is retried before failing the gps module. `gps_settings.hot_fix` needs to be enabled for this settings to be used - 2 is a good default, 255 is a special value indicating the cold fix never times out.
* `gps_fail_retry` - number of times gps system is retried before putting it in failed state, only 0 can be used currently
* `gps_settings` -
  *  `bit 0` - 3d fix enabled - 1 is agood default
  *  `bit 1` - linear backoff upon fail (based on interval time) - 1 is a good default
  *  `bit 2` - hot fix enabled - 1 is a good default
  *  `bit 3` - fully resolved required - 1 is a good default
* `gps_accel_z_threshold` - accelerometer threshold for z value, such that gps does not trigger on wrong orientation. Typically GPS up is a negative accel Z value, good option to set this to is -500, 0 to disable this check

Theory of operation is as follows:
* GPS is initialized when first used, either by periodic or triggered interval, both can be used at the same time. GPS starts in the cold fix mode and will try to get the fix for the `gps_cold_fix_timeout` duration, if successful and hot-fix is enabled in general settings, then it will try all consequent fixes for the `gps_hot_fix_timeout` duration. If a fix fails to be acquired for `gps_hot_fix_retry` number of times then it reverts to cold fix for the `gps_cold_fix_timeout` duration for `gps_cold_fix_retry` times. When that is exhausted the gps goes to `gps_fail_retry`, which can currently only be 0, so practically the GPS will be disabled upon the reset of the device.

`gps_settings` allows for 3D fix to be enabled, linear backoff upon a failure of a cold or hot fix, meaning tha the defined `gps_periodic_interval` or `gps_triggered_interval`will be multiplied by the number of fails that have occured. This sonserves the battery by allowing the device to get out of a bad spot. Hot fix should alwazy be enabled unles very clearly needed otherwise, likewise for fully resolved.

`gps_min_ehpe` is the principal factor to configure how fast the fix can be acquired and how good it is. A general value of 50 is a good starting point, 100 makes the fix fast but very inaccurate, under 20 the fix times get very very long and drain the battery. Leave at 50 unless you know what you are doing. `gps_min_fix_time` forces the fixes not to be too short, acquiring a bit more of almanach. If you can afford battery wise, 15s is a good default

`gps_accel_z_threshold` is the value set as accelerometer z value above which gps can trigger, meaning the tracker is pointing to the sky. Negative hangles are handled as well such that the more negative number then the threshold, then trigger occurs.

## GPS error logic
GPS errors are shown by 3 different varaibles in status message:
 * `gps_periodic_error` - issued when `gps_cold_fix_retry` times the gps has failed toa cquire a fix. Subject to `gps_fail_retry` the system may try self-reset the GPS system.
 * `gps_triggered_error` - absolutely same as above
 * `gps_fix_error` - issued for every fix attempt, which fails due to tiemout hot or cold.


## LoraWAN comissioning/provisioning
The device povisioning is done with a separate sketch that uploads the keys to the EEPROM. Then the device gets the actual firmware. This is implemented such that the same firmware build can be distributed publicly and does not contain any key information.

If the keys are incorrect or not provisioned on the device, then the device fill revert to the fallback provisioning specified in the firwmare. Note this is nto secure and you can modify it per project, so no devices should be ever using this. If you see a device responding as fallback you can try to remotely reset it and that is as much as you can do, otherwise you need to reprogram.

Default fallback:
 * `devAddr` = "26011D63"
 * `nwkSKey` = "9518E9E68D1476BC3386409B76476208"
 * `appSKey` = "7972E2A484F76EF7B579D641D0BFEBD5"

## LoraWAN data
See `decoder.js` which is rather human-readable for description of values being sent.

## LoraWAN commands
There are a number of single-byte commands specified for critical device control mechanisms. WARNING: Commands need to be sent unfonfirmed as they may reset the device, loosing the reply.

Implemented commands to be sent as single byte to port 99:
 * `0xaa` - Sent the current settings
 * `0xab` - Triggers system reset
 * `0xde` - Resets LoraWAN stored settings in EEPROM and forces a re-join
 * `0x11` - Request the device GPS position log - 5 positions per packet, all available positions in sequence
 * `0xf1` - Clear GPS position log - use if necessary, but log simply stores last 100 positions and overwrites itself

GPS commands are a separate option on port 91:
 * `0xcc` - Request the GPS to be active based on the downlink command, required additional parameters
   * `duration` - configures how long in minutes should this mode be active
   * `interval` - configures how often in minutes should the unit report position
   * `example binary packet to port 91`: `0xCC 0x05 0x00 0x01 0x00` - send GPS every 1 minute for 5 minutes

## Scripts

## Calibration process

Calibration process is needed to calibrate the magnetometer, gyroscope and accelerometer sensor.
Entering into calibration process is only possible 5 seconds after startup, after that is disabled.
To turn on the device put in batteries and press *battery activate* button.
Green LED will blink once.

To enter calibration sequence afterwards follow steps below:

1. Press and hold button 1 until LED starts blinking rapidly, then release. Led will turn off.
2. Press either button 1 or once
3. When the LED turns on, start moving the unit in figure 8 shape for 1 minute and 30 seconds until the LED turns off
4. Put the device on the flat surface.
5. Press button 1 once and wait 10 seconds.
6. Led will then blink 3 times, which means that calibration procedure is complete.

Calibration values are saved into devices flash and are not erased at reset.
