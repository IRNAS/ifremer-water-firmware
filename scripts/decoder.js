//note some values need to be tuned to the hardware in here, make sure to do so
function get_num(x, min, max, precision, round) {

  var range = max - min;
  var new_range = (Math.pow(2, precision) - 1) / range;
  var back_x = x / new_range;

  if (back_x === 0) {
    back_x = min;
  }
  else if (back_x === (max - min)) {
    back_x = max;
  }
  else {
    back_x += min;
  }
  return Math.round(back_x * Math.pow(10, round)) / Math.pow(10, round);
}

function Decoder(bytes) {

  var decoded = {};
  var cnt = 0;
  var resetCause_dict = {
    0: "POWERON",
    1: "EXTERNAL",
    2: "SOFTWARE",
    3: "WATCHDOG",
    4: "FIREWALL",
    5: "OTHER",
    6: "STANDBY"
  };


  // settings
  if (port === 3) {
    decoded.system_status_interval = (bytes[1] << 8) | bytes[0];
    decoded.system_functions = {};//bytes[2];
    decoded.system_functions.gps_periodic = ((bytes[2] >> 0) & 0x01) ? 1 : 0;
    decoded.system_functions.gps_triggered = ((bytes[2] >> 1) & 0x01) ? 1 : 0;
    decoded.system_functions.gps_hot_fix = ((bytes[2] >> 2) & 0x01) ? 1 : 0;
    decoded.system_functions.accelerometer_enabled = ((bytes[2] >> 3) & 0x01) ? 1 : 0;
    decoded.system_functions.light_enabled = ((bytes[2] >> 4) & 0x01) ? 1 : 0;
    decoded.system_functions.temperature_enabled = ((bytes[2] >> 5) & 0x01) ? 1 : 0;
    decoded.system_functions.humidity_enabled = ((bytes[2] >> 6) & 0x01) ? 1 : 0;
    decoded.system_functions.charging_enabled = ((bytes[2] >> 7) & 0x01) ? 1 : 0;

    decoded.lorawan_datarate_adr = {};//bytes[3];
    decoded.lorawan_datarate_adr.datarate = bytes[3] & 0x0f;
    decoded.lorawan_datarate_adr.confirmed_uplink = ((bytes[3] >> 6) & 0x01) ? 1 : 0;
    decoded.lorawan_datarate_adr.adr = ((bytes[3] >> 7) & 0x01) ? 1 : 0;

    decoded.gps_periodic_interval = (bytes[5] << 8) | bytes[4];
    decoded.gps_triggered_interval = (bytes[7] << 8) | bytes[6];
    decoded.gps_triggered_threshold = bytes[8];
    decoded.gps_triggered_duration = bytes[9];
    decoded.gps_cold_fix_timeout = (bytes[11] << 8) | bytes[10];
    decoded.gps_hot_fix_timeout = (bytes[13] << 8) | bytes[12];
    decoded.gps_min_fix_time = bytes[14];
    decoded.gps_min_ehpe = bytes[15];
    decoded.gps_hot_fix_retry = bytes[16];
    decoded.gps_cold_fix_retry = bytes[17];
    decoded.gps_fail_retry = bytes[18];
    decoded.gps_settings = {};//bytes[19];
    decoded.gps_settings.d3_fix = ((bytes[19] >> 0) & 0x01) ? 1 : 0;
    decoded.gps_settings.fail_backoff = ((bytes[19] >> 1) & 0x01) ? 1 : 0;
    decoded.gps_settings.hot_fix = ((bytes[19] >> 2) & 0x01) ? 1 : 0;
    decoded.gps_settings.fully_resolved = ((bytes[19] >> 3) & 0x01) ? 1 : 0;
    decoded.system_voltage_interval = bytes[20];
    decoded.gps_charge_min = bytes[21]*10+2500;
    decoded.system_charge_min = bytes[22]*10+2500;
    decoded.system_charge_max = bytes[23]*10+2500;
    decoded.system_input_charge_min = (bytes[25] << 8) | bytes[24];
    decoded.pulse_threshold = bytes[26];
    decoded.pulse_on_timeout = bytes[27];
    decoded.pulse_min_interval = (bytes[29] << 8) | bytes[28];
    decoded.gps_accel_z_threshold = ((bytes[31] << 8) | bytes[30])-2000;
    decoded.fw_version = (bytes[33] << 8) | bytes[32];
    raw_magnetic_declination = ((bytes[35] << 8) | bytes[34]);
    decoded.magnetic_declination = (raw_magnetic_declination - 90) / 100;

  }
  else if (port === 12) {
    decoded.resetCause = resetCause_dict[bytes[0]&0x07];
    decoded.system_state_timeout = bytes[0]>>3;
    decoded.battery = bytes[1]*10+2500; // result in mV
    decoded.temperature = get_num(bytes[2], -20, 80, 8, 1);
    decoded.system_functions_errors = {};//bytes[5];
    decoded.system_functions_errors.gps_periodic_error = ((bytes[3] >> 0) & 0x01) ? 1 : 0;
    decoded.system_functions_errors.gps_triggered_error = ((bytes[3] >> 1) & 0x01) ? 1 : 0;
    decoded.system_functions_errors.gps_fix_error = ((bytes[3] >> 2) & 0x01) ? 1 : 0;
    decoded.system_functions_errors.accelerometer_error = ((bytes[3] >> 3) & 0x01) ? 1 : 0;
    decoded.system_functions_errors.light_error = ((bytes[3] >> 4) & 0x01) ? 1 : 0;
    decoded.system_functions_errors.charging_status = (bytes[3] >> 5) & 0x07;
    decoded.lat = ((bytes[4] << 16) >>> 0) + ((bytes[5] << 8) >>> 0) + bytes[6];
    decoded.lon = ((bytes[7] << 16) >>> 0) + ((bytes[8] << 8) >>> 0) + bytes[9];
    if(decoded.lat!==0 && decoded.lon!==0){
      decoded.lat = (decoded.lat / 16777215.0 * 180) - 90;
      decoded.lon = (decoded.lon / 16777215.0 * 360) - 180;
      decoded.lat = Math.round(decoded.lat*100000)/100000;
      decoded.lon = Math.round(decoded.lon*100000)/100000;
    }
    decoded.gps_resend = bytes[10];
    decoded.accelx = get_num(bytes[11], -2000, 2000, 8, 1);
    decoded.accely = get_num(bytes[12], -2000, 2000, 8, 1);
    decoded.accelz = get_num(bytes[13], -2000, 2000, 8, 1);
    decoded.battery_low = (bytes[15] << 8) | bytes[14]; // result in mV
    decoded.gps_on_time_total = (bytes[17] << 8) | bytes[16];
    decoded.gps_time = bytes[18] | (bytes[19] << 8) | (bytes[20] << 16) | (bytes[21] << 24);
    var d= new Date(decoded.gps_time*1000);
    decoded.gps_time_decoded = d.toLocaleString();
    decoded.pulse_counter = bytes[22];
    decoded.pulse_energy = (bytes[23]<<4) | (bytes[24] | (bytes[25] << 8)>>12);
    decoded.pulse_voltage = (bytes[24] | (bytes[25] << 8)) & 0x0fff;
    decoded.voltage_fence_v = decoded.pulse_voltage * 8;

    decoded.ctzn_temp    = get_num(bytes[26], 0, 40, 8, 1);
    conductivity_ctz     = (bytes[28] << 8) | bytes[27];
    salinity             = (bytes[30] << 8) | bytes[29];
    conductivity_no_comp = (bytes[32] << 8) | bytes[31];

    decoded.optod_temp = get_num(bytes[33], 0, 40, 8, 1);
    oxygen_sat = (bytes[35] << 8) | bytes[34];
    oxygen_mgL = (bytes[37] << 8) | bytes[36];
    oxygen_ppm = (bytes[39] << 8) | bytes[38];

    decoded.conductivity_ctz     = get_num(conductivity_ctz,     0, 100, 16, 2);
    decoded.salinity             = get_num(salinity,             5,  60, 16, 2);
    decoded.conductivity_no_comp = get_num(conductivity_no_comp, 0, 100, 16, 2);

    decoded.oxygen_sat = get_num(oxygen_sat, 0, 200, 16, 2);
    decoded.oxygen_mgL = get_num(oxygen_mgL, 5,  20, 16, 2);
    decoded.oxygen_ppm = get_num(oxygen_ppm, 0,  20, 16, 2);

    decoded.bme_temp        = get_num(bytes[40], -40,  85, 8, 1);
    decoded.bme_pressure    = get_num(bytes[41], 300,1100, 8, 1);
    decoded.bme_humid       = get_num(bytes[42],   0, 100, 8, 1);

    decoded.significant_wh  = ((bytes[44] << 8) | bytes[43]) / 1000;
    decoded.average_wh      = ((bytes[46] << 8) | bytes[45]) / 1000;
    decoded.average_period  = ((bytes[48] << 8) | bytes[47]) / 1000;
  }
  else if (port === 1) {
    decoded.lat = ((bytes[cnt++] << 16) >>> 0) + ((bytes[cnt++] << 8) >>> 0) + bytes[cnt++];
    decoded.lon = ((bytes[cnt++] << 16) >>> 0) + ((bytes[cnt++] << 8) >>> 0) + bytes[cnt++];
    if(decoded.lat!==0 && decoded.lon!==0){
      decoded.lat = (decoded.lat / 16777215.0 * 180) - 90;
      decoded.lon = (decoded.lon / 16777215.0 * 360) - 180;
      decoded.lat = Math.round(decoded.lat*100000)/100000;
      decoded.lon = Math.round(decoded.lon*100000)/100000;
    }
    decoded.alt = bytes[cnt++] | (bytes[cnt++] << 8);
    decoded.satellites = (bytes[cnt] >> 4);
    decoded.hdop = (bytes[cnt++] & 0x0f);
    decoded.time_to_fix = bytes[cnt++];
    decoded.epe = bytes[cnt++];
    decoded.snr = bytes[cnt++];
    decoded.lux = bytes[cnt++];
    decoded.motion = bytes[cnt++];
    decoded.time = bytes[cnt++] | (bytes[cnt++] << 8) | (bytes[cnt++] << 16) | (bytes[cnt++] << 24);
    var d= new Date(decoded.time*1000);
    decoded.time_decoded = d.toLocaleString();
  }
  else if (port === 11) {
    var locations=[];
    for(i = 0; i < 5; i++){
      var location={}
      location.lat = ((bytes[cnt++] << 16) >>> 0) + ((bytes[cnt++] << 8) >>> 0) + bytes[cnt++];
      location.lon = ((bytes[cnt++] << 16) >>> 0) + ((bytes[cnt++] << 8) >>> 0) + bytes[cnt++];
      if(location.lat!==0 && location.lon!==0){
        location.lat = (location.lat / 16777215.0 * 180) - 90;
        location.lon = (location.lon / 16777215.0 * 360) - 180;
        location.lat = Math.round(location.lat*100000)/100000;
        location.lon = Math.round(location.lon*100000)/100000;
      }
      location.time = bytes[cnt++] | (bytes[cnt++] << 8) | (bytes[cnt++] << 16) | (bytes[cnt++] << 24);
      var d= new Date(location.time*1000);
      location.time_decoded = d.toLocaleString();
      locations.push(location);
    }
    decoded.locations=JSON.stringify(locations);
  }
  else if (port === 30) {
    var vswr=[];
    for(i = 0; i < bytes.length; i++){
      var value=(bytes[i]);
      vswr.push(value);
    }
    decoded.vswr=vswr;
  }

  return decoded;
}
